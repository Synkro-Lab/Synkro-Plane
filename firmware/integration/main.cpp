// ============================================================
//  Piloto automático para avión RC (alerones, elevador, timón)  -  v2
//  Raspberry Pi Pico (RP2040) - Pico SDK - C++
//
//  NÚCLEO 1 (crítico): i-BUS, selección de modo, failsafe, ESC, servos, foto, watchdog.
//  NÚCLEO 0 (cálculo): MPU6050/6500, BMP280, DHT22, actitud, PID, telemetría hacia la ESP32-CAM.
//  Comunicación entre núcleos: estructuras protegidas con contador de secuencia.
//
//  Enlace con la ESP32-CAM (UART1): la Pico manda una línea CSV por iteración (100 Hz) y mensajes
//  que empiezan con '#'. La ESP32 manda comandos de 2 caracteres: "!c" recalibrar, "!i" resumen.
// ============================================================
#include <stdio.h>
#include <stdarg.h>
#include <math.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/uart.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "hardware/dma.h"
#include "hardware/watchdog.h"
#include "hardware/sync.h"
#include "hardware/clocks.h"
#include "config.h"

#define IBUS_UART uart0
#define TLM_UART  uart1
#define I2C_PORT  i2c0
#define FW_VERSION "v2"

enum Mode : uint8_t { MODE_MANUAL = 0, MODE_STAB = 1, MODE_AUTO = 2, MODE_FAILSAFE = 3 };

static const float DEG2RAD = 0.017453293f;
static const float RAD2DEG = 57.29578f;

static inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
static inline int   clampi(int v, int lo, int hi)       { return v < lo ? lo : (v > hi ? hi : v); }

// ------------------------------------------------------------
//  Intercambio entre núcleos (un solo escritor por estructura)
// ------------------------------------------------------------
template <typename T>
struct SeqLock {
    volatile uint32_t seq = 0;
    T data{};
    void write(const T& v) { seq = seq + 1; __dmb(); data = v; __dmb(); seq = seq + 1; }
    T read() const {
        T c{};
        for (int i = 0; i < 20; i++) {
            uint32_t s1 = seq; __dmb();
            c = data;          __dmb();
            if (!(s1 & 1) && s1 == seq) break;
        }
        return c;
    }
};

// núcleo 1 -> núcleo 0
struct RadioShared {
    uint8_t  mode;
    float    roll, pitch, yaw;      // sticks -1..1 (pitch + = nariz arriba)
    uint16_t ch[7];                 // CH1..CH7 en µs
    uint16_t out_us[5];             // salidas: alerón izq., alerón der., elevador, timón, ESC
    uint16_t thr;                   // gas pedido por la radio (µs)
    uint8_t  flags;                 // bit0 = failsafe activo, bit1 = armado
    uint32_t photo;                 // cuenta acumulada de peticiones de foto
};
// núcleo 0 -> núcleo 1
struct CtrlShared  { uint8_t mode; bool valid; float roll, pitch, yaw, flap; uint32_t t_us; };

static SeqLock<RadioShared> g_radio;
static SeqLock<CtrlShared>  g_ctrl;
static bool g_wd_reboot = false;

// ------------------------------------------------------------
//  Salidas PWM (servos y ESC) - 1 tick = 1 µs
// ------------------------------------------------------------
static const uint SERVO_PIN[4] = {PIN_SERVO_AIL_L, PIN_SERVO_AIL_R, PIN_SERVO_ELEV, PIN_SERVO_RUD};

static void pwm_out_init(uint pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(pin);
    pwm_config c = pwm_get_default_config();
    pwm_config_set_clkdiv(&c, (float)clock_get_hz(clk_sys) / 1000000.0f);
    pwm_config_set_wrap(&c, 1000000 / PWM_HZ - 1);
    pwm_init(slice, &c, true);
}
static inline void pwm_us(uint pin, int us) { pwm_set_gpio_level(pin, (uint16_t)us); }

// defl en [-1, 1] -> microsegundos con centro, recorrido, sentido y límites. Devuelve los µs escritos.
static int write_servo(int idx, float defl) {
    int us = SERVO_CENTER_US[idx] + (int)(SERVO_DIR[idx] * clampf(defl, -1.0f, 1.0f) * SERVO_RANGE_US[idx]);
    us = clampi(us, SERVO_MIN_US, SERVO_MAX_US);
    pwm_us(SERVO_PIN[idx], us);
    return us;
}

// ------------------------------------------------------------
//  i-BUS: paquetes de 32 bytes, 115200 8N1
//  [0x20][0x40][14 canales x 2 bytes LE][checksum 2 bytes LE = 0xFFFF - suma de los 30 anteriores]
// ------------------------------------------------------------
static uint16_t g_ch[14];
static uint32_t g_last_pkt_us = 0;
static bool     g_have_pkt = false;      // ya llegó al menos un paquete válido

static void ibus_init() {
    uart_init(IBUS_UART, 115200);
    gpio_set_function(PIN_IBUS_RX, GPIO_FUNC_UART);
    uart_set_format(IBUS_UART, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(IBUS_UART, true);
}

static void ibus_poll() {
    static uint8_t buf[32];
    static uint8_t idx = 0;
    static uint32_t last_byte_us = 0;
    while (uart_is_readable(IBUS_UART)) {
        uint32_t now = time_us_32();
        if (now - last_byte_us > 2000) idx = 0;      // hueco entre paquetes: resincroniza
        last_byte_us = now;
        uint8_t b = uart_getc(IBUS_UART);
        if (idx == 0 && b != 0x20) continue;
        if (idx == 1 && b != 0x40) { idx = 0; continue; }
        buf[idx++] = b;
        if (idx == 32) {
            idx = 0;
            uint16_t sum = 0xFFFF;
            for (int i = 0; i < 30; i++) sum -= buf[i];
            uint16_t chk = buf[30] | (buf[31] << 8);
            if (sum != chk) continue;
            uint16_t tmp[14];
            for (int i = 0; i < 14; i++) tmp[i] = buf[2 + 2 * i] | (buf[3 + 2 * i] << 8);
            bool ok = true;
            for (int i = 0; i < 6; i++) if (tmp[i] < 800 || tmp[i] > 2200) ok = false;   // CH1-6 deben ser sensatos
            if (!ok) continue;
            memcpy(g_ch, tmp, sizeof(g_ch));
            g_last_pkt_us = now;
            g_have_pkt = true;
        }
    }
}

static inline float stick_norm(uint16_t us) { return clampf(((float)us - 1500.0f) / 500.0f, -1.0f, 1.0f); }
static inline float deadband(float v) { return fabsf(v) < STICK_DEADBAND ? 0.0f : v; }

// ------------------------------------------------------------
//  NÚCLEO 1: modo, failsafe y actuadores (cada 10 ms)
// ------------------------------------------------------------
static void control_step(uint32_t now) {
    static bool     fs_active = false;
    static uint32_t fs_ok_since = 0;
    static bool     armed = false;
    static bool     photo_prev = true;          // true: no dispara si arrancas con el switch activo
    static uint32_t photo_cnt = 0, last_photo = 0;

    // --- Failsafe: sin paquetes, o bandera de la radio (CH7) ---
    bool link_lost = (now - g_last_pkt_us) > FAILSAFE_TIMEOUT_US;
    bool fs_flag   = (CH_FS > 0) && (g_ch[CH_FS - 1] > 1700);
    if (link_lost || fs_flag) {
        fs_active = true; fs_ok_since = 0;
    } else if (fs_active) {
        if (fs_ok_since == 0) fs_ok_since = now;
        else if (now - fs_ok_since > FAILSAFE_RECOVER_US) fs_active = false;
    }

    // --- Armado del gas: el motor no responde hasta ver el gas abajo una vez ---
    uint16_t thr = g_ch[CH_THR - 1];
    if (g_wd_reboot) armed = true;                              // reinicio en vuelo: conserva el gas
    if (!armed && g_have_pkt && !fs_active && thr < 1100) armed = true;

    // --- Modo ---
    uint16_t m = g_ch[CH_MODE - 1];
    Mode mode;
    if (fs_active)       mode = MODE_FAILSAFE;
    else if (m < 1250)   mode = MODE_MANUAL;
    else if (m < 1750)   mode = MODE_STAB;
    else                 mode = MODE_AUTO;

    float s_roll  = STICK_SIGN_ROLL  * stick_norm(g_ch[CH_AIL - 1]);
    float s_pitch = STICK_SIGN_PITCH * stick_norm(g_ch[CH_ELE - 1]);
    float s_yaw   = STICK_SIGN_YAW   * stick_norm(g_ch[CH_RUD - 1]);

    // --- Órdenes calculadas por el núcleo 0 (solo se usan si son frescas y del modo actual) ---
    CtrlShared c = g_ctrl.read();
    bool fresh = c.valid && ((now - c.t_us) < CTRL_STALE_US) && (c.mode == mode);

    float roll, pitch, yaw, flap = 0.0f;
    switch (mode) {
        case MODE_MANUAL:
            roll = s_roll; pitch = s_pitch; yaw = s_yaw;
            break;
        case MODE_STAB:
        case MODE_AUTO:
            if (fresh) { roll = c.roll; pitch = c.pitch; yaw = c.yaw; }
            else       { roll = s_roll; pitch = s_pitch; yaw = s_yaw; }   // IMU caída: pasa a manual
            break;
        default:  // MODE_FAILSAFE
            if (fresh) { roll = c.roll; pitch = c.pitch; yaw = c.yaw; flap = c.flap; }
            else       { roll = FS_OPEN_ROLL; pitch = FS_OPEN_PITCH; yaw = FS_OPEN_YAW; flap = FS_FLAP; }
            break;
    }

    // --- Mezcla de alerones (deflexión positiva = hacia abajo) y salidas ---
    // Alabeo a la derecha: alerón derecho sube, izquierdo baja. Los flaps bajan ambos.
    RadioShared rs{};
    rs.out_us[0] = (uint16_t)write_servo(0, +roll + flap);
    rs.out_us[1] = (uint16_t)write_servo(1, -roll + flap);
    rs.out_us[2] = (uint16_t)write_servo(2, pitch);
    rs.out_us[3] = (uint16_t)write_servo(3, yaw);

    int thr_us = (armed && mode != MODE_FAILSAFE) ? clampi(thr, 1000, 2000) : 1000;
    pwm_us(PIN_ESC, thr_us);
    rs.out_us[4] = (uint16_t)thr_us;

    // --- Foto: flanco del switch o temporizador en autónomo. La ESP32 ve subir la cuenta por el enlace serie ---
    bool want = g_ch[CH_PHOTO - 1] > 1700;
    bool trigger = want && !photo_prev;
    photo_prev = want;
    if (mode == MODE_AUTO && (now - last_photo) > (uint32_t)PHOTO_INTERVAL_S * 1000000u) trigger = true;
    if (trigger) { photo_cnt++; last_photo = now; }

    // --- Publica el estado para el núcleo 0 ---
    rs.mode = mode; rs.roll = s_roll; rs.pitch = s_pitch; rs.yaw = s_yaw;
    for (int i = 0; i < 7; i++) rs.ch[i] = g_ch[i];
    rs.thr = thr;
    rs.flags = (fs_active ? 1 : 0) | (armed ? 2 : 0);
    rs.photo = photo_cnt;
    g_radio.write(rs);
}

static void core1_main() {
    for (int i = 0; i < 14; i++) g_ch[i] = 1500;          // valores neutros hasta que llegue el primer paquete
    g_ch[CH_THR - 1] = 1000; g_ch[CH_MODE - 1] = 1000; g_ch[CH_PHOTO - 1] = 1000;
    if (CH_FS > 0) g_ch[CH_FS - 1] = 1000;
    ibus_init();
    g_last_pkt_us = time_us_32();
    uint32_t next = time_us_32() + 10000;
    while (true) {
        ibus_poll();
        watchdog_update();          // solo el núcleo 1 alimenta el watchdog
        uint32_t now = time_us_32();
        if ((int32_t)(now - next) >= 0) { next += 10000; control_step(now); }
    }
}

// ------------------------------------------------------------
//  Enlace serie con la ESP32-CAM (núcleo 0): TX por DMA con buffer circular, RX de comandos
// ------------------------------------------------------------
#define TLM_RING 4096
static uint8_t  tlm_ring[TLM_RING];
static uint32_t tlm_head = 0, tlm_tail = 0, tlm_dma_len = 0, tlm_drop = 0;
static int      tlm_dma = -1;

static void tlm_init() {
    uart_init(TLM_UART, TLM_BAUD);
    gpio_set_function(PIN_TLM_TX, GPIO_FUNC_UART);
    gpio_set_function(PIN_TLM_RX, GPIO_FUNC_UART);
    uart_set_format(TLM_UART, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(TLM_UART, true);
    tlm_dma = dma_claim_unused_channel(true);
    dma_channel_config c = dma_channel_get_default_config(tlm_dma);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, uart_get_dreq(TLM_UART, true));
    dma_channel_configure(tlm_dma, &c, &uart_get_hw(TLM_UART)->dr, tlm_ring, 0, false);
}

// Copia al buffer circular; si no cabe, descarta (nunca bloquea el lazo de control)
static void tlm_write(const char* s, int n) {
    uint32_t used = (tlm_head + TLM_RING - tlm_tail) % TLM_RING;
    if ((uint32_t)n >= TLM_RING - used - 1) { tlm_drop++; return; }
    for (int i = 0; i < n; i++) { tlm_ring[tlm_head] = (uint8_t)s[i]; tlm_head = (tlm_head + 1) % TLM_RING; }
}

// Lanza el siguiente tramo por DMA cuando el anterior terminó
static void tlm_pump() {
    if (dma_channel_is_busy(tlm_dma)) return;
    if (tlm_dma_len) { tlm_tail = (tlm_tail + tlm_dma_len) % TLM_RING; tlm_dma_len = 0; }
    if (tlm_head == tlm_tail) return;
    uint32_t len = (tlm_head > tlm_tail) ? (tlm_head - tlm_tail) : (TLM_RING - tlm_tail);
    if (len > 512) len = 512;
    tlm_dma_len = len;
    dma_channel_set_read_addr(tlm_dma, &tlm_ring[tlm_tail], false);
    dma_channel_set_trans_count(tlm_dma, len, true);
}

static void tlm_msg(const char* fmt, ...) {
    char t[200];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(t, sizeof(t), fmt, ap);
    va_end(ap);
    if (n <= 0) return;
    if (n >= (int)sizeof(t)) n = sizeof(t) - 1;
    tlm_write(t, n);
}

// Constructor de líneas CSV (escribe "nan" para valores no válidos)
struct LineBuf {
    char b[256];
    int  n = 0;
    void f(float v, int prec) {
        if (n > (int)sizeof(b) - 24) return;
        if (isfinite(v)) n += snprintf(b + n, sizeof(b) - n, "%.*f,", prec, (double)v);
        else             n += snprintf(b + n, sizeof(b) - n, "nan,");
    }
    void u(unsigned long v) { if (n > (int)sizeof(b) - 24) return; n += snprintf(b + n, sizeof(b) - n, "%lu,", v); }
    void i(long v)          { if (n > (int)sizeof(b) - 24) return; n += snprintf(b + n, sizeof(b) - n, "%ld,", v); }
    void end() { if (n > 0 && b[n - 1] == ',') b[n - 1] = '\n'; else if (n < (int)sizeof(b)) b[n++] = '\n'; }
};

// Comandos de la ESP32: '!' + letra
static bool g_cmd_recal = false, g_cmd_info = false;
static void poll_commands() {
    static bool bang = false;
    while (uart_is_readable(TLM_UART)) {
        char b = (char)uart_getc(TLM_UART);
        if (bang) {
            bang = false;
            if (b == 'c') g_cmd_recal = true;
            else if (b == 'i') g_cmd_info = true;
        } else if (b == '!') {
            bang = true;
        }
    }
}

// ------------------------------------------------------------
//  I2C
// ------------------------------------------------------------
static bool i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t val) {
    uint8_t b[2] = {reg, val};
    return i2c_write_timeout_us(I2C_PORT, addr, b, 2, false, 3000) == 2;
}
static bool i2c_read_regs(uint8_t addr, uint8_t reg, uint8_t* buf, size_t n) {
    if (i2c_write_timeout_us(I2C_PORT, addr, &reg, 1, true, 3000) != 1) return false;
    return i2c_read_timeout_us(I2C_PORT, addr, buf, n, false, 6000) == (int)n;
}

static void i2c_setup_pins() {
    gpio_set_function(PIN_I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_I2C_SDA);
    gpio_pull_up(PIN_I2C_SCL);
}

// Si un esclavo se quedó sujetando SDA, 9 pulsos de reloj y un STOP lo liberan
static void i2c_bus_clear() {
    i2c_deinit(I2C_PORT);
    gpio_set_function(PIN_I2C_SCL, GPIO_FUNC_SIO); gpio_set_dir(PIN_I2C_SCL, GPIO_OUT); gpio_put(PIN_I2C_SCL, 1);
    gpio_set_function(PIN_I2C_SDA, GPIO_FUNC_SIO); gpio_set_dir(PIN_I2C_SDA, GPIO_IN);
    for (int i = 0; i < 9; i++) { gpio_put(PIN_I2C_SCL, 0); sleep_us(5); gpio_put(PIN_I2C_SCL, 1); sleep_us(5); }
    gpio_set_dir(PIN_I2C_SDA, GPIO_OUT); gpio_put(PIN_I2C_SDA, 0); sleep_us(5);
    gpio_put(PIN_I2C_SCL, 1); sleep_us(5);
    gpio_put(PIN_I2C_SDA, 1); sleep_us(5);
    i2c_init(I2C_PORT, I2C_BAUD);
    i2c_setup_pins();
}

// ------------------------------------------------------------
//  MPU6050 / MPU6500 (acepta ambos: muchos módulos "6050" traen 6500)
// ------------------------------------------------------------
static uint8_t g_mpu_addr = 0x68, g_who = 0;
struct ImuSample { float a[3]; float g[3]; int32_t raw_sum; bool zero; };   // a en g, g en °/s, ejes del avión

static bool mpu_detect() {
    const uint8_t addrs[2] = {0x68, 0x69};
    for (int i = 0; i < 2; i++) {
        uint8_t id = 0;
        if (i2c_read_regs(addrs[i], 0x75, &id, 1)) { g_mpu_addr = addrs[i]; g_who = id; return true; }
    }
    return false;
}

// Escribe la configuración sin reset (se usa también para refrescarla en vuelo)
static bool mpu_apply_cfg() {
    bool ok = true;
    ok &= i2c_write_reg(g_mpu_addr, 0x6B, 0x01);                 // despierta, reloj del giroscopio X
    ok &= i2c_write_reg(g_mpu_addr, 0x6C, 0x00);                 // ningún eje en standby
    ok &= i2c_write_reg(g_mpu_addr, 0x1A, MPU_DLPF);
    ok &= i2c_write_reg(g_mpu_addr, 0x1B, MPU_GYRO_FS_BITS);
    ok &= i2c_write_reg(g_mpu_addr, 0x1C, MPU_ACC_FS_BITS);
    return ok;
}

static bool mpu_reset_and_config() {
    bool ok = i2c_write_reg(g_mpu_addr, 0x6B, 0x80);             // reset
    sleep_ms(150);
    ok &= i2c_write_reg(g_mpu_addr, 0x6B, 0x01);
    sleep_ms(20);
    ok &= mpu_apply_cfg();
    sleep_ms(100);
    return ok;
}

// 1 = conserva su configuración, 0 = cambió (se reinició), -1 = error de bus
static int mpu_cfg_state() {
    uint8_t c[3], p = 0;
    if (!i2c_read_regs(g_mpu_addr, 0x1A, c, 3) || !i2c_read_regs(g_mpu_addr, 0x6B, &p, 1)) return -1;
    return (c[0] == MPU_DLPF && c[1] == MPU_GYRO_FS_BITS && c[2] == MPU_ACC_FS_BITS && p == 0x01) ? 1 : 0;
}

static bool mpu_read(ImuSample& o) {
    uint8_t b[14];
    if (!i2c_read_regs(g_mpu_addr, 0x3B, b, 14)) return false;
    int16_t v[7];
    for (int i = 0; i < 7; i++) v[i] = (int16_t)((b[2 * i] << 8) | b[2 * i + 1]);
    float a_raw[3] = {v[0] / MPU_ACC_LSB_PER_G, v[1] / MPU_ACC_LSB_PER_G, v[2] / MPU_ACC_LSB_PER_G};
    float g_raw[3] = {v[4] / MPU_GYRO_LSB_PER_DPS, v[5] / MPU_GYRO_LSB_PER_DPS, v[6] / MPU_GYRO_LSB_PER_DPS};
    for (int i = 0; i < 3; i++) {
        o.a[i] = IMU_SIGN[i] * a_raw[IMU_AXIS[i]];
        o.g[i] = IMU_SIGN[i] * g_raw[IMU_AXIS[i]];
    }
    o.raw_sum = (int32_t)v[4] + v[5] + v[6] + v[0] + v[1] + v[2];
    o.zero = (v[0] == 0 && v[2] == 0);              // ax y az exactamente en cero = lectura muerta
    return true;
}

// Con el módulo quieto: |a| ~ 1 g y ax/az no pueden estar en cero exacto
static bool mpu_selftest(float& mag) {
    float sx = 0, sy = 0, sz = 0;
    int ok = 0, zeros = 0;
    for (int i = 0; i < 30; i++) {
        ImuSample s;
        if (mpu_read(s)) { ok++; sx += s.a[0]; sy += s.a[1]; sz += s.a[2]; if (s.zero) zeros++; }
        sleep_ms(10);
    }
    if (ok < 25) { mag = NAN; return false; }
    sx /= ok; sy /= ok; sz /= ok;
    mag = sqrtf(sx * sx + sy * sy + sz * sz);
    return (mag > 0.85f && mag < 1.15f && zeros < ok / 2);
}

static bool mpu_start(float& mag) {
    for (int intento = 1; intento <= 5; intento++) {
        bool w = mpu_reset_and_config();
        bool v = mpu_selftest(mag);
#if DEBUG_PRINT
        printf("Config MPU, intento %d: escrituras %s, |a| = %.2f g -> %s\n", intento, w ? "OK" : "con error", mag, v ? "OK" : "NO valida");
#endif
        if (w && v) return true;
    }
    return false;
}

static void mpu_recover() {
    i2c_bus_clear();
    mpu_reset_and_config();
}

// ------------------------------------------------------------
//  BMP280 / BME280 (presión -> altitud relativa)
// ------------------------------------------------------------
struct Bmp {
    uint8_t addr = 0;
    uint16_t T1 = 0; int16_t T2 = 0, T3 = 0;
    uint16_t P1 = 0; int16_t P2 = 0, P3 = 0, P4 = 0, P5 = 0, P6 = 0, P7 = 0, P8 = 0, P9 = 0;
    double p0 = 0; int p0_n = 0; double p0_sum = 0;
};

static bool bmp_init(Bmp& b) {
    const uint8_t addrs[2] = {0x76, 0x77};
    for (int i = 0; i < 2 && !b.addr; i++) {
        uint8_t id = 0;
        if (i2c_read_regs(addrs[i], 0xD0, &id, 1) && (id == 0x58 || id == 0x60)) b.addr = addrs[i];
    }
    if (!b.addr) return false;
    uint8_t c[24];
    if (!i2c_read_regs(b.addr, 0x88, c, 24)) { b.addr = 0; return false; }
    auto u16 = [&](int i) { return (uint16_t)(c[i] | (c[i + 1] << 8)); };
    auto s16 = [&](int i) { return (int16_t)(c[i] | (c[i + 1] << 8)); };
    b.T1 = u16(0);  b.T2 = s16(2);  b.T3 = s16(4);
    b.P1 = u16(6);  b.P2 = s16(8);  b.P3 = s16(10); b.P4 = s16(12); b.P5 = s16(14);
    b.P6 = s16(16); b.P7 = s16(18); b.P8 = s16(20); b.P9 = s16(22);
    bool ok = i2c_write_reg(b.addr, 0xF5, 0x08);   // filtro IIR x4
    ok &= i2c_write_reg(b.addr, 0xF4, 0x2F);       // T x1, P x4, modo normal
    return ok;
}

static bool bmp_read(const Bmp& b, double& pa) {
    uint8_t d[6];
    if (!i2c_read_regs(b.addr, 0xF7, d, 6)) return false;
    int32_t adc_p = ((int32_t)d[0] << 12) | ((int32_t)d[1] << 4) | (d[2] >> 4);
    int32_t adc_t = ((int32_t)d[3] << 12) | ((int32_t)d[4] << 4) | (d[5] >> 4);
    double v1 = (adc_t / 16384.0 - b.T1 / 1024.0) * b.T2;
    double x = adc_t / 131072.0 - b.T1 / 8192.0;
    double v2 = x * x * b.T3;
    double t_fine = v1 + v2;
    v1 = t_fine / 2.0 - 64000.0;
    v2 = v1 * v1 * b.P6 / 32768.0;
    v2 = v2 + v1 * b.P5 * 2.0;
    v2 = v2 / 4.0 + b.P4 * 65536.0;
    v1 = (b.P3 * v1 * v1 / 524288.0 + b.P2 * v1) / 524288.0;
    v1 = (1.0 + v1 / 32768.0) * b.P1;
    if (v1 == 0.0) return false;
    double p = 1048576.0 - adc_p;
    p = (p - v2 / 4096.0) * 6250.0 / v1;
    v1 = b.P9 * p * p / 2147483648.0;
    v2 = p * b.P8 / 32768.0;
    pa = p + (v1 + v2 + b.P7) / 16.0;
    return true;
}

// ------------------------------------------------------------
//  DHT22 (solo para registro; es lento y bloquea ~7 ms)
// ------------------------------------------------------------
static int wait_level(bool level, uint32_t timeout_us) {
    uint32_t t0 = time_us_32();
    while (gpio_get(PIN_DHT22) != level) if (time_us_32() - t0 > timeout_us) return -1;
    return (int)(time_us_32() - t0);
}

static bool dht22_read(float& hum, float& temp) {
    uint8_t d[5] = {0, 0, 0, 0, 0};
    gpio_set_dir(PIN_DHT22, GPIO_OUT);
    gpio_put(PIN_DHT22, 0);
    sleep_ms(2);
    uint32_t irq = save_and_disable_interrupts();
    gpio_put(PIN_DHT22, 1);
    busy_wait_us(30);
    gpio_set_dir(PIN_DHT22, GPIO_IN);
    bool ok = wait_level(0, 100) >= 0 && wait_level(1, 100) >= 0 && wait_level(0, 100) >= 0;
    for (int i = 0; ok && i < 40; i++) {
        if (wait_level(1, 100) < 0) { ok = false; break; }
        int t = wait_level(0, 120);
        if (t < 0) { ok = false; break; }
        d[i / 8] = (uint8_t)((d[i / 8] << 1) | (t > 40 ? 1 : 0));
    }
    restore_interrupts(irq);
    if (!ok || ((d[0] + d[1] + d[2] + d[3]) & 0xFF) != d[4]) return false;
    hum  = ((d[0] << 8) | d[1]) / 10.0f;
    temp = (((d[2] & 0x7F) << 8) | d[3]) / 10.0f;
    if (d[2] & 0x80) temp = -temp;
    return true;
}

// ------------------------------------------------------------
//  Actitud: filtro complementario (X adelante, Y derecha, Z abajo)
//  roll + = ala derecha abajo; pitch + = nariz arriba
// ------------------------------------------------------------
struct Att { float roll = 0, pitch = 0; };   // grados

static float wrap_pi(float a) { while (a > M_PI) a -= 2 * M_PI; while (a < -M_PI) a += 2 * M_PI; return a; }

static void att_from_accel(const float a[3], float& roll_deg, float& pitch_deg) {
    roll_deg  = atan2f(-a[1], -a[2]) * RAD2DEG;
    pitch_deg = atan2f(a[0], sqrtf(a[1] * a[1] + a[2] * a[2])) * RAD2DEG;
}

static void att_update(Att& att, const float a[3], const float g[3], float dt) {
    float phi = att.roll * DEG2RAD, th = att.pitch * DEG2RAD;
    float p = g[0] * DEG2RAD, q = g[1] * DEG2RAD, r = g[2] * DEG2RAD;
    float ct = cosf(th); if (fabsf(ct) < 0.1f) ct = 0.1f;
    float dphi = p + (q * sinf(phi) + r * cosf(phi)) * (sinf(th) / ct);
    float dth  = q * cosf(phi) - r * sinf(phi);
    float phi_g = phi + dphi * dt, th_g = th + dth * dt;

    float norm = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    float k = (fabsf(norm - 1.0f) < ACC_TRUST_BAND) ? ACC_GAIN : 0.0f;   // acelerómetro solo si no hay aceleraciones fuertes
    float phi_a = atan2f(-a[1], -a[2]);
    float th_a  = atan2f(a[0], sqrtf(a[1] * a[1] + a[2] * a[2]));
    phi = phi_g + k * wrap_pi(phi_a - phi_g);
    th  = th_g  + k * (th_a - th_g);
    att.roll = phi * RAD2DEG;
    att.pitch = th * RAD2DEG;
}

// ------------------------------------------------------------
//  PID en cascada: ángulo (P) -> velocidad angular (PI + D opcional sobre la medición)
// ------------------------------------------------------------
struct Axis {
    float kang, rate_max, kp, ki, kd, ilim;
    float integ = 0, prev_rate = 0, d_f = 0;
    void reset() { integ = 0; d_f = 0; }
    float run(float ang_sp, float ang, float rate, float dt, bool integrate) {
        float rate_sp = clampf(kang * (ang_sp - ang), -rate_max, rate_max);
        float e = rate_sp - rate;
        float d_raw = -(rate - prev_rate) / dt;
        prev_rate = rate;
        d_f += 0.2f * (d_raw - d_f);
        float u = kp * e + integ + kd * d_f;
        if (integrate && fabsf(u) < 1.0f) integ = clampf(integ + ki * e * dt, -ilim, ilim);   // anti-windup
        return clampf(u, -1.0f, 1.0f);
    }
};

// ------------------------------------------------------------
//  Calibración del gyro (avión quieto ~2 s por intento). Devuelve el sesgo y la actitud inicial.
// ------------------------------------------------------------
static bool calibrate_gyro(float bias[3], Att& att, int max_attempts) {
    for (int attempt = 0; attempt < max_attempts; attempt++) {
        double gs[3] = {0, 0, 0}, as[3] = {0, 0, 0};
        float gmin[3] = {1e9f, 1e9f, 1e9f}, gmax[3] = {-1e9f, -1e9f, -1e9f};
        const int N = 200;
        for (int i = 0; i < N; i++) {
            ImuSample s;
            if (!mpu_read(s)) return false;
            for (int k = 0; k < 3; k++) {
                gs[k] += s.g[k]; as[k] += s.a[k];
                if (s.g[k] < gmin[k]) gmin[k] = s.g[k];
                if (s.g[k] > gmax[k]) gmax[k] = s.g[k];
            }
            tlm_pump();
            sleep_ms(10);
        }
        bool quiet = true;
        for (int k = 0; k < 3; k++) if ((gmax[k] - gmin[k]) > 6.0f) quiet = false;   // se movió: repetir
        float a[3] = {(float)(as[0] / N), (float)(as[1] / N), (float)(as[2] / N)};
        float an = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
        if (an < 0.8f || an > 1.2f) quiet = false;                                    // lectura de gravedad rara
        if (!quiet) continue;
        for (int k = 0; k < 3; k++) bias[k] = (float)(gs[k] / N);
        att_from_accel(a, att.roll, att.pitch);
        return true;
    }
    return false;
}

// ------------------------------------------------------------
//  Resumen del arranque (se reenvía a la ESP32 cuando lo pide con "!i")
// ------------------------------------------------------------
static char g_info[640];
static int  g_info_len = 0;

static void build_info(bool cfg_ok, float mag, const float bias[3], bool calibrated, bool bmp_ok) {
    int n = snprintf(g_info, sizeof(g_info),
        "#I Autopilot " FW_VERSION " | telemetria %d Hz a %d baud\n"
        "#I MPU dir=0x%02X WHO_AM_I=0x%02X config=%s |a|=%.2f g I2C=%d kHz\n"
        "#I Escalas: acc +-16 g (%.0f LSB/g) gyro +-1000 dps (%.1f LSB/dps) DLPF=0x%02X\n"
        "#I Gyro sesgo %.2f %.2f %.2f dps calibrada=%d | BMP=%d | reinicio_wd=%d\n",
        TLM_HZ, TLM_BAUD,
        g_mpu_addr, g_who, cfg_ok ? "OK" : "NO verificada", (double)mag, I2C_BAUD / 1000,
        (double)MPU_ACC_LSB_PER_G, (double)MPU_GYRO_LSB_PER_DPS, MPU_DLPF,
        (double)bias[0], (double)bias[1], (double)bias[2], calibrated, bmp_ok, g_wd_reboot);
    if (n < 0) n = 0;
    if (n >= (int)sizeof(g_info)) n = sizeof(g_info) - 1;
    g_info_len = n;
}

// ------------------------------------------------------------
//  NÚCLEO 0: sensores + control + telemetría
// ------------------------------------------------------------
int main() {
    // 1) Salidas seguras antes que nada: ESC en 1000 µs (arma el ESC) y servos al centro.
    g_wd_reboot = watchdog_caused_reboot();
    const uint all_pins[5] = {PIN_SERVO_AIL_L, PIN_SERVO_AIL_R, PIN_SERVO_ELEV, PIN_SERVO_RUD, PIN_ESC};
    for (int i = 0; i < 5; i++) pwm_out_init(all_pins[i]);
    pwm_us(PIN_ESC, 1000);
    for (int i = 0; i < 4; i++) write_servo(i, 0.0f);
    gpio_init(PIN_DHT22); gpio_set_dir(PIN_DHT22, GPIO_IN); gpio_pull_up(PIN_DHT22);

#if DEBUG_PRINT
    stdio_init_all();
#endif

    // 2) Watchdog y núcleo 1: desde aquí el modo manual ya funciona aunque falle todo lo demás.
    watchdog_enable(WATCHDOG_MS, true);
    multicore_launch_core1(core1_main);

    // 3) Enlace con la ESP32 y sensores
    tlm_init();
    i2c_init(I2C_PORT, I2C_BAUD);
    i2c_setup_pins();

    Att att;
    float gbias[3] = {0, 0, 0};
    float mag = NAN;
    bool imu_present = mpu_detect();
    bool cfg_ok = false, calibrated = false;
    if (imu_present) {
        if (g_wd_reboot) cfg_ok = mpu_reset_and_config();       // en vuelo no se puede hacer el autotest ni calibrar
        else             cfg_ok = mpu_start(mag);
    }
    if (cfg_ok && !g_wd_reboot) calibrated = calibrate_gyro(gbias, att, 5);
    Bmp bmp;
    bool bmp_ok = bmp_init(bmp);
    build_info(cfg_ok, mag, gbias, calibrated, bmp_ok);
    tlm_write(g_info, g_info_len);
#if DEBUG_PRINT
    printf("IMU=%d cfg=%d calibrada=%d BMP=%d wd_reboot=%d WHO=0x%02X\n", imu_present, cfg_ok, calibrated, bmp_ok, g_wd_reboot, g_who);
#endif

    Axis roll_pid  {ROLL_KANG,  ROLL_RATE_MAX,  ROLL_KP,  ROLL_KI,  ROLL_KD,  ROLL_ILIM};
    Axis pitch_pid {PITCH_KANG, PITCH_RATE_MAX, PITCH_KP, PITCH_KI, PITCH_KD, PITCH_ILIM};

    bool healthy = imu_present && cfg_ok;
    int fail_cnt = 0, good_cnt = 0, frozen_cnt = 0;
    int32_t last_raw_sum = 0x7FFFFFFF;
    float last_a[3] = {0, 0, 0}, last_g[3] = {0, 0, 0};
    float alt = 0, hum = NAN, temp = NAN;
    float cmd_r = 0, cmd_p = 0, cmd_y = 0;
    uint32_t iter = 0, ev = 0, rc = 0, last_recover_us = 0;
    uint8_t prev_mode = 255;

    const uint32_t period = 1000000 / LOOP_HZ;
    const uint32_t tlm_div = (LOOP_HZ / TLM_HZ) > 0 ? (LOOP_HZ / TLM_HZ) : 1;
    uint32_t t_prev = time_us_32();
    uint32_t next = t_prev + period;

    while (true) {
        while ((int32_t)(time_us_32() - next) < 0) { tlm_pump(); tight_loop_contents(); }
        uint32_t now = time_us_32();
        uint32_t dt_us = now - t_prev;
        float dt = clampf(dt_us * 1e-6f, 0.001f, 0.05f);
        t_prev = now; next += period;
        iter++;

        poll_commands();
        RadioShared r = g_radio.read();

        // --- Comandos de la ESP32 ---
        if (g_cmd_info) { g_cmd_info = false; tlm_write(g_info, g_info_len); }
        if (g_cmd_recal) {
            g_cmd_recal = false;
            if (imu_present && cfg_ok && r.mode == MODE_MANUAL && r.thr < 1100) {
                tlm_msg("#N Recalibrando giroscopio (~2 s): avion quieto\n");
                calibrated = false;
                bool ok = calibrate_gyro(gbias, att, 2);
                calibrated = ok;
                tlm_msg("#N Calibracion %s\n", ok ? "OK" : "FALLO (se movio el avion)");
                build_info(cfg_ok, mag, gbias, calibrated, bmp_ok);
                tlm_write(g_info, g_info_len);
                roll_pid.reset(); pitch_pid.reset();
                now = time_us_32(); t_prev = now; next = now + period;   // retoma el ritmo tras el bloqueo
            } else {
                tlm_msg("#N Recalibracion rechazada: solo en tierra (modo manual y gas abajo)\n");
            }
        }

        // --- Verificación de la configuración del MPU (cada 100 ms) ---
        bool skip_read = false;
        if (imu_present && cfg_ok && (iter % MPU_CHECK_ITER) == 0) {
            int st = mpu_cfg_state();
            if (st == 0) {                                        // el MPU perdió su configuración: se reescribe
                ev++;
                mpu_apply_cfg();
                healthy = false; good_cnt = 0; skip_read = true;
                tlm_msg("#E t=%lu MPU perdio la configuracion (ev=%lu), reconfigurado\n", (unsigned long)(now / 1000), (unsigned long)ev);
            }
        }

        // --- IMU ---
        ImuSample s;
        bool got = imu_present && cfg_ok && !skip_read && mpu_read(s);
        if (got) {
            if (s.raw_sum == last_raw_sum) frozen_cnt++; else frozen_cnt = 0;   // datos congelados = falla
            last_raw_sum = s.raw_sum;
            if (frozen_cnt > 30 || s.zero) got = false;
        }
        if (got) {
            for (int k = 0; k < 3; k++) { last_a[k] = s.a[k]; last_g[k] = s.g[k] - gbias[k]; }
            att_update(att, s.a, last_g, dt);
            fail_cnt = 0; if (good_cnt < 1000) good_cnt++;
            if (!healthy && good_cnt > 50) healthy = true;
        } else if (!skip_read) {
            good_cnt = 0;
            if (++fail_cnt > 25) healthy = false;
            // Racha de fallos: libera el bus y vuelve a configurar el MPU (como en tu logger v4)
            if (imu_present && fail_cnt >= 20 && (now - last_recover_us) > 500000) {
                last_recover_us = now;
                rc++;
                tlm_msg("#E t=%lu MPU sin datos validos: recuperando bus (rc=%lu)\n", (unsigned long)(now / 1000), (unsigned long)rc);
                mpu_recover();
                fail_cnt = 0; frozen_cnt = 0;
                now = time_us_32(); t_prev = now; next = now + period;
            }
        }

        // --- Barómetro (25 Hz) y DHT22 (cada 3 s) ---
        if (bmp_ok && (iter % 4) == 0) {
            double pa;
            if (bmp_read(bmp, pa)) {
                if (bmp.p0_n < 20) { bmp.p0_sum += pa; if (++bmp.p0_n == 20) bmp.p0 = bmp.p0_sum / 20.0; }
                else { float h = (float)(44330.0 * (1.0 - pow(pa / bmp.p0, 0.1903))); alt += 0.2f * (h - alt); }
            }
        }
        if ((iter % (LOOP_HZ * 3)) == 0) { float h, t; if (dht22_read(h, t)) { hum = h; temp = t; } }

        // --- Control ---
        CtrlShared c{};
        c.mode = r.mode; c.t_us = now; c.valid = false;
        bool can = calibrated && healthy;

        if (r.mode != prev_mode) { roll_pid.reset(); pitch_pid.reset(); prev_mode = r.mode; }

        if (r.mode != MODE_MANUAL && can) {
            float roll  = att.roll  - ROLL_OFFSET_DEG;
            float pitch = att.pitch - PITCH_OFFSET_DEG;
            float sr = deadband(r.roll), sp = deadband(r.pitch);
            float roll_sp = 0, pitch_sp = 0, flap = 0;

            if (r.mode == MODE_STAB) {
                roll_sp = sr * MAX_ROLL_DEG;  pitch_sp = sp * MAX_PITCH_DEG;
            } else if (r.mode == MODE_AUTO) {
                roll_sp = 0;  pitch_sp = AUTO_PITCH_DEG;
                if (fabsf(sr) > STICK_OVERRIDE) roll_sp  = sr * MAX_ROLL_DEG;     // el piloto retoma el eje
                if (fabsf(sp) > STICK_OVERRIDE) pitch_sp = sp * MAX_PITCH_DEG;
            } else {  // failsafe con IMU sana: viraje suave + flaps
                roll_sp = FS_BANK_DEG; pitch_sp = FS_PITCH_DEG; flap = FS_FLAP;
            }

            float roll_cmd  = roll_pid.run(roll_sp, roll, last_g[0], dt, true);
            float pitch_cmd = pitch_pid.run(pitch_sp, pitch, last_g[1], dt, true);
            float pilot_yaw = (r.mode == MODE_FAILSAFE) ? 0.0f : deadband(r.yaw);
            float yaw_cmd   = clampf(pilot_yaw - YAW_DAMP * last_g[2] + RUD_MIX * roll_cmd, -1.0f, 1.0f);

            c.valid = true; c.roll = roll_cmd; c.pitch = pitch_cmd; c.yaw = yaw_cmd; c.flap = flap;
            cmd_r = roll_cmd; cmd_p = pitch_cmd; cmd_y = yaw_cmd;
        } else {
            roll_pid.reset(); pitch_pid.reset();
            cmd_r = cmd_p = cmd_y = 0;
        }
        g_ctrl.write(c);

        // --- Telemetría hacia la ESP32 (misma estructura de columnas que el logger v4 + las del piloto) ---
        if ((iter % tlm_div) == 0) {
            uint32_t hs = (healthy ? 1u : 0u) | (calibrated ? 2u : 0u) | (bmp_ok ? 4u : 0u)
                        | ((r.flags & 1) ? 8u : 0u) | ((r.flags & 2) ? 16u : 0u) | (imu_present ? 32u : 0u);
            LineBuf L;
            L.u(now / 1000);
            L.f(got ? last_a[0] : NAN, 3); L.f(got ? last_a[1] : NAN, 3); L.f(got ? last_a[2] : NAN, 3);
            L.f(got ? last_g[0] : NAN, 2); L.f(got ? last_g[1] : NAN, 2); L.f(got ? last_g[2] : NAN, 2);
            L.f(hum, 1); L.f(temp, 1);
            L.u(ev); L.u(rc);
            L.u(r.mode);
            L.f(att.roll, 1); L.f(att.pitch, 1); L.f(alt, 1);
            for (int i = 0; i < 7; i++) L.u(r.ch[i]);
            for (int i = 0; i < 5; i++) L.u(r.out_us[i]);
            L.i((long)(cmd_r * 1000.0f)); L.i((long)(cmd_p * 1000.0f)); L.i((long)(cmd_y * 1000.0f));
            L.u(hs); L.u(r.photo); L.u(dt_us);
            L.end();
            tlm_write(L.b, L.n);
        }
        tlm_pump();

#if DEBUG_PRINT
        if ((iter % (LOOP_HZ / 5)) == 0)
            printf("modo=%d roll=%.1f pitch=%.1f alt=%.1f T=%.1f H=%.0f sano=%d cal=%d ev=%lu rc=%lu drop=%lu\n",
                   r.mode, att.roll, att.pitch, alt, temp, hum, healthy, calibrated,
                   (unsigned long)ev, (unsigned long)rc, (unsigned long)tlm_drop);
#endif
    }
}
