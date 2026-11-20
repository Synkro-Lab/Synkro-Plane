#pragma once
// ============================================================
//  CONFIGURACIÓN DEL PILOTO AUTOMÁTICO (Raspberry Pi Pico)
//  Todo lo que normalmente tendrás que tocar está en este archivo.
// ============================================================

// ---------- Pines (numeración GP) ----------
#define PIN_IBUS_RX      1    // UART0 RX  <- señal i-BUS (puerto "i-BUS SERVO" del FS-iA6B)
#define PIN_I2C_SDA      4    // I2C0 SDA  -> MPU6050 + BMP280
#define PIN_I2C_SCL      5    // I2C0 SCL
#define PIN_DHT22        6
#define PIN_SERVO_AIL_L  10
#define PIN_SERVO_AIL_R  11
#define PIN_SERVO_ELEV   12
#define PIN_SERVO_RUD    13
#define PIN_ESC          16
#define PIN_TLM_TX       8    // UART1 TX -> GPIO13 (RX2) de la ESP32-CAM
#define PIN_TLM_RX       9    // UART1 RX <- GPIO4  (TX2) de la ESP32-CAM

// ---------- Canales de la radio (1 = CH1 ... ) ----------
#define CH_AIL    1
#define CH_ELE    2
#define CH_THR    3
#define CH_RUD    4
#define CH_MODE   5   // SwC: abajo-arriba = manual / estabilizado / autónomo
#define CH_PHOTO  6   // SwB: foto al activarlo
#define CH_FS     7   // Bandera de failsafe (0 = desactivada, solo se usa el timeout)

// Signos de los sticks (Modo 2 de la FlySky). Positivo = alabeo derecha, nariz arriba, guiñada derecha.
// Palanca de elevador adelante da CH2 alto, que significa nariz abajo, por eso es -1.
#define STICK_SIGN_ROLL   (+1)
#define STICK_SIGN_PITCH  (-1)
#define STICK_SIGN_YAW    (+1)

// ---------- Servos ----------
// Orden de los arreglos: 0 = alerón izq., 1 = alerón der., 2 = elevador, 3 = timón
#define PWM_HZ 50                        // 50 Hz analógicos; 200-300 Hz si TODOS tus servos son digitales
static const int SERVO_CENTER_US[4] = {1500, 1500, 1500, 1500};   // punto neutro + trim
static const int SERVO_RANGE_US[4]  = { 400,  400,  400,  400};   // recorrido por lado (limita según tu mecánica)
static const int SERVO_MIN_US = 1000;
static const int SERVO_MAX_US = 2000;
// Sentido de cada servo. Alerones: +1 significa que un pulso mayor baja la superficie.
// Elevador: +1 pulso mayor sube la superficie. Timón: +1 pulso mayor lo manda a la derecha.
static const int SERVO_DIR[4] = {+1, +1, +1, +1};

// ---------- Sensores ----------
// Escalas validadas en tu logger (log_002 y log_005: |a| = 1.01 g en reposo)
#define MPU_ACC_FS_BITS       0x18      // ±16 g
#define MPU_ACC_LSB_PER_G     2048.0f
#define MPU_GYRO_FS_BITS      0x10      // ±1000 dps
#define MPU_GYRO_LSB_PER_DPS  32.8f
#define MPU_DLPF              0x03      // filtro ~44 Hz (0x02 = ~92 Hz, como tu logger)
#define MPU_CHECK_ITER        10        // cada cuántas iteraciones (100 ms) se comprueba que el MPU conserve su configuración
#define I2C_BAUD              100000    // 100 kHz como tu logger; sube a 400000 solo si el log sale limpio

// Remapeo de ejes del MPU a ejes del avión: X adelante, Y ala derecha, Z abajo.
// IMU_AXIS[i] dice qué eje del chip va a cada eje del avión (0=X, 1=Y, 2=Z); IMU_SIGN lo invierte si hace falta.
// Con el chip plano, cara de componentes hacia arriba, X del chip hacia la nariz: Y chip = izquierda, Z chip = arriba.
static const int   IMU_AXIS[3] = {0, 1, 2};
static const float IMU_SIGN[3] = {+1.0f, -1.0f, -1.0f};
// Lo que marca el sensor cuando el avión vuela nivelado (por montaje). Ajústalo en pruebas.
#define ROLL_OFFSET_DEG   0.0f
#define PITCH_OFFSET_DEG  0.0f

// ---------- Estimación de actitud ----------
#define LOOP_HZ          100
#define ACC_GAIN         0.02f    // peso del acelerómetro en el filtro complementario (alpha = 0.98)
#define ACC_TRUST_BAND   0.20f    // solo se usa el acelerómetro si |a| está dentro de 1g ± este valor

// ---------- Control (parten conservadoras, hay que ajustarlas en vuelo) ----------
#define MAX_ROLL_DEG     45.0f    // alabeo máximo pedido por el stick
#define MAX_PITCH_DEG    25.0f    // cabeceo máximo pedido por el stick
#define AUTO_PITCH_DEG   0.0f     // cabeceo objetivo en modo autónomo
#define STICK_OVERRIDE   0.25f    // en autónomo, mover el stick más de esto recupera el mando de ese eje
#define STICK_DEADBAND   0.03f

// Roll: lazo externo de ángulo (KANG) y lazo interno de velocidad (KP, KI, KD)
#define ROLL_KANG    4.0f
#define ROLL_RATE_MAX 100.0f
#define ROLL_KP      0.008f
#define ROLL_KI      0.010f
#define ROLL_KD      0.000f
#define ROLL_ILIM    0.25f
// Pitch
#define PITCH_KANG   3.0f
#define PITCH_RATE_MAX 60.0f
#define PITCH_KP     0.010f
#define PITCH_KI     0.008f
#define PITCH_KD     0.000f
#define PITCH_ILIM   0.25f
// Timón: amortiguador de guiñada + mezcla con el alabeo
#define YAW_DAMP     0.010f       // por °/s
#define RUD_MIX      0.30f        // timón por unidad de orden de alerón

// ---------- Failsafe ----------
#define FAILSAFE_TIMEOUT_US  100000   // sin paquetes i-BUS válidos por este tiempo -> failsafe
#define FAILSAFE_RECOVER_US  500000   // la radio debe estar bien este tiempo antes de devolver el mando
#define FS_FLAP          0.35f        // alerones hacia abajo a la vez (flaps), fracción del recorrido
#define FS_BANK_DEG      8.0f         // viraje suave con IMU sana
#define FS_PITCH_DEG     0.0f
#define FS_OPEN_ROLL     0.08f        // valores sin IMU
#define FS_OPEN_PITCH    0.10f        // un poco de elevador arriba para compensar los flaps
#define FS_OPEN_YAW      0.00f

// ---------- Otros ----------
#define PHOTO_INTERVAL_S 5            // en autónomo, una foto cada X segundos
#define TLM_BAUD         460800       // enlace Pico -> ESP32-CAM (telemetría y comandos)
#define TLM_HZ           100          // líneas de telemetría por segundo (divisor de LOOP_HZ)
#define WATCHDOG_MS      300
#define CTRL_STALE_US    50000        // si el núcleo 0 no responde en este tiempo, el núcleo 1 pasa a manual
#define DEBUG_PRINT      1            // telemetría por USB (5 Hz). Sin PC conectado no bloquea.
