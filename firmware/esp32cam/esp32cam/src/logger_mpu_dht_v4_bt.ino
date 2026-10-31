/*
  Logger v4 + Bluetooth serial: MPU6050/6500 (GY-521) + DHT -> microSD (ESP32-CAM)

  NOVEDADES v4 (a partir de la prueba de banco log_009):
    - En log_009 el MPU perdio su configuracion a los ~116 s (volvio a +-2 g / +-250 dps) tras una racha de
      lecturas fallidas, y siguio entregando datos "validos" con la escala equivocada durante 95 s.
      Ahora, cada segundo, se comprueba el registro del giroscopio y se REESCRIBE la configuracion.
      Si habia cambiado: se avisa por pantalla/Bluetooth y el contador "ev" del CSV sube en 1.
    - Nueva columna "ev" en el CSV: numero acumulado de veces que el MPU perdio la configuracion.
      Las filas anteriores al cambio de "ev" (hasta ~1 s antes) pueden tener la escala equivocada.
    - DHT cada 5 s en vez de 2 s: cada lectura del DHT provoca de vez en cuando UNA muestra fallida del MPU.

  PLACA EN EL IDE: "ESP32 Dev Module", PSRAM: Disabled.
  Si el IDE dice "Sketch too big": Tools -> Partition Scheme -> "Huge APP (3MB No OTA/1MB SPIFFS)".

  Conexiones:
    GY-521 VCC -> 3V3   GND -> GND   SDA -> IO4   SCL -> IO13
    DHT    VCC -> 3V3   GND -> GND   DATA -> IO16
    microSD en el slot de la placa (modo 1-bit)

  COMO VER EL ESTADO EN EL CELULAR (Android):
    1) En casa, empareja el celular con "Kestrel-Logger" (Ajustes -> Bluetooth). Solo se hace una vez.
    2) Instala "Serial Bluetooth Terminal" (Google Play), elige "Kestrel-Logger" y conecta.
    3) Al conectar recibes el resumen del arranque (autochequeo del MPU, SD, archivo) y luego una
       linea de estado por segundo: "OK ..." o "ATENCION ... [SD] [MPU] [DHT]".
    iPhone: no soporta Bluetooth Classic serial (SPP). Habria que hacer una version BLE.

  COMANDOS (por Bluetooth o por el monitor serie):
    i = repetir el resumen del arranque
    c = repetir el chequeo del MPU (deja el modulo QUIETO; se pausa el registro ~1 s)
    f = apagar el Bluetooth para volar
    h = ayuda

  IMPORTANTE: apaga el Bluetooth antes de lanzar (comando f). Comparte la banda de 2.4 GHz con tu
  receptor y esta pegado a el en el avion. Ademas se apaga solo a los BT_AUTO_OFF_S segundos del arranque.

  CSV: t_ms, ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps, hum_pct, temp_c, ev
*/

#include <Arduino.h>
#include <stdarg.h>
#include <Wire.h>
#include <FS.h>
#include <SD_MMC.h>
#include <DHT.h>
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error "Bluetooth no esta habilitado en esta placa. Usa ESP32 Dev Module."
#endif

// ---------- Configuracion ----------
#define LOG_HZ          100
#define FLUSH_MS        500
#define PRINT_MS        1000      // linea de estado cada segundo

#define BT_NOMBRE       "Kestrel-Logger"
#define BT_AUTO_OFF_S   600       // el Bluetooth se apaga solo a los 10 minutos

#define DHT_PIN         16
#define DHT_TYPE        DHT11     // DHT11 o DHT22, el que te dio valores razonables
#define DHT_PERIODO_MS  5000

// Escalas validadas en log_002 y log_005 (|a| = 1.01 g en reposo)
#define ACC_FS_BITS       0x18    // +-16 g
#define ACC_LSB_PER_G     2048.0f
#define GYRO_FS_BITS      0x10    // +-1000 dps
#define GYRO_LSB_PER_DPS  32.8f

const int PIN_A = 4;              // SDA segun tu cableado
const int PIN_B = 13;             // SCL segun tu cableado (el codigo prueba ambos ordenes)

#define N_BUF 1024                // muestras en el buffer (~10 s a 100 Hz)

// ---------- Estado ----------
BluetoothSerial SerialBT;
DHT dht(DHT_PIN, DHT_TYPE);
SemaphoreHandle_t mtx;

uint8_t  mpuAddr = 0x68;
int      pinSDA = PIN_A, pinSCL = PIN_B;
File     logFile;
char     nombreArchivo[24] = "";
bool     sdOk = false;

volatile bool btActivo = false;
volatile bool btConectado = false;
volatile bool btReenviarInfo = false;
volatile bool pedirChequeo = false;
volatile bool pedirApagarBT = false;

volatile float humedad = NAN;
volatile float temperatura = NAN;
volatile float ult[6] = {0, 0, 0, 0, 0, 0};
volatile bool  ultValido = false;

struct Muestra { uint32_t t; float v[8]; uint16_t ev; };   // ax ay az gx gy gz hum temp, ev
Muestra buf[N_BUF];
volatile uint32_t head = 0, tail = 0;
volatile uint32_t desbordes = 0;
volatile uint32_t desajustes = 0;   // veces que el MPU perdio la configuracion
int      gyroRef = -1;              // valor de referencia del registro 0x1B
uint32_t ultimoRefresco = 0;

uint32_t proximoLog_us = 0;
volatile uint32_t muestras = 0, fallos = 0, reconfigs = 0;
uint16_t ceroSeguidos = 0, falloSeguidos = 0;
const uint32_t periodo_us = 1000000UL / LOG_HZ;

char   historial[3072];
size_t histLen = 0;

// ---------- Salida: monitor serie + Bluetooth ----------
void emitir(const char *txt, int n, bool guardar) {
  xSemaphoreTake(mtx, portMAX_DELAY);
  Serial.write((const uint8_t *)txt, n);
  if (btActivo && btConectado) SerialBT.write((const uint8_t *)txt, n);
  if (guardar && histLen + n < sizeof(historial) - 1) {
    memcpy(historial + histLen, txt, n);
    histLen += n;
    historial[histLen] = 0;
  }
  xSemaphoreGive(mtx);
}

static int formatear(char *dst, size_t cap, const char *f, va_list ap) {
  int n = vsnprintf(dst, cap, f, ap);
  if (n < 0) return 0;
  if ((size_t)n >= cap) n = (int)cap - 1;
  return n;
}

// infof: se muestra y se guarda en el resumen del arranque
void infof(const char *f, ...) {
  char tmp[320];
  va_list ap;
  va_start(ap, f);
  int n = formatear(tmp, sizeof(tmp), f, ap);
  va_end(ap);
  if (n > 0) emitir(tmp, n, true);
}

// estadof: se muestra pero no se guarda (lineas periodicas)
void estadof(const char *f, ...) {
  char tmp[320];
  va_list ap;
  va_start(ap, f);
  int n = formatear(tmp, sizeof(tmp), f, ap);
  va_end(ap);
  if (n > 0) emitir(tmp, n, false);
}

void btEvento(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
  if (event == ESP_SPP_SRV_OPEN_EVT) { btConectado = true; btReenviarInfo = true; }
  else if (event == ESP_SPP_CLOSE_EVT) { btConectado = false; }
}

void apagarBT(const char *motivo) {
  infof("%s Bluetooth se apaga ahora. Buen vuelo!\n", motivo);
  vTaskDelay(pdMS_TO_TICKS(500));
  btActivo = false;
  btConectado = false;
  SerialBT.end();
  btStop();
}

void procesarComando(char c, bool porBT) {
  if (c == 'i') {
    if (porBT) {
      btReenviarInfo = true;
    } else {
      xSemaphoreTake(mtx, portMAX_DELAY);
      Serial.write((const uint8_t *)historial, histLen);
      xSemaphoreGive(mtx);
    }
  } else if (c == 'c') {
    pedirChequeo = true;
    estadof("Chequeo pedido: deja el modulo quieto...\n");
  } else if (c == 'f') {
    pedirApagarBT = true;
  } else if (c == 'h' || c == '?') {
    estadof("Comandos: i = resumen del arranque | c = repetir chequeo del MPU (modulo quieto) | f = apagar Bluetooth para volar\n");
  }
}

// ---------- MPU ----------
bool mpuEscribir(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(mpuAddr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool mpuLeer(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
  Wire.beginTransmission(mpuAddr);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)mpuAddr, 14) != 14) return false;

  uint8_t b[14];
  for (int i = 0; i < 14; i++) b[i] = Wire.read();

  int16_t rax = (int16_t)((b[0] << 8) | b[1]);
  int16_t ray = (int16_t)((b[2] << 8) | b[3]);
  int16_t raz = (int16_t)((b[4] << 8) | b[5]);
  int16_t rgx = (int16_t)((b[8] << 8) | b[9]);
  int16_t rgy = (int16_t)((b[10] << 8) | b[11]);
  int16_t rgz = (int16_t)((b[12] << 8) | b[13]);

  ax = rax / ACC_LSB_PER_G;     ay = ray / ACC_LSB_PER_G;     az = raz / ACC_LSB_PER_G;
  gx = rgx / GYRO_LSB_PER_DPS;  gy = rgy / GYRO_LSB_PER_DPS;  gz = rgz / GYRO_LSB_PER_DPS;
  return true;
}

bool probarPar(int sda, int scl) {
  Wire.end();
  Wire.begin(sda, scl);
  Wire.setClock(100000);
  Wire.setTimeOut(20);
  delay(5);

  const uint8_t direcciones[2] = {0x68, 0x69};
  for (int i = 0; i < 2; i++) {
    uint8_t a = direcciones[i];
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Wire.beginTransmission(a);
      Wire.write(0x75);
      Wire.endTransmission(false);
      Wire.requestFrom((int)a, 1);
      int who = Wire.available() ? Wire.read() : -1;
      infof("MPU en 0x%02X con SDA=%d SCL=%d (WHO_AM_I=0x%02X)\n", a, sda, scl, who);
      mpuAddr = a; pinSDA = sda; pinSCL = scl;
      return true;
    }
  }
  return false;
}

void esperarMPU() {
  while (true) {
    if (probarPar(PIN_A, PIN_B) || probarPar(PIN_B, PIN_A)) return;
    estadof("MPU no responde. Revisa 3V3, GND, SDA, SCL y soldaduras. Reintentando en 3 s...\n");
    delay(3000);
  }
}

bool mpuConfigurar() {
  bool okw = true;
  okw &= mpuEscribir(0x6B, 0x80);   // reset
  delay(150);
  okw &= mpuEscribir(0x6B, 0x01);   // despertar, reloj del giroscopio X
  delay(20);
  okw &= mpuEscribir(0x6C, 0x00);   // ningun eje en standby
  okw &= mpuEscribir(0x1A, 0x02);   // filtro pasa-bajos ~92 Hz
  okw &= mpuEscribir(0x1B, GYRO_FS_BITS);
  okw &= mpuEscribir(0x1C, ACC_FS_BITS);
  delay(100);
  return okw;
}

// Con el modulo quieto: |a| debe ser ~1 g y ax/az no pueden estar en cero exacto
bool mpuVerificar(float &magOut) {
  float sx = 0, sy = 0, sz = 0;
  int ok = 0, ceros = 0;
  for (int i = 0; i < 30; i++) {
    float ax, ay, az, gx, gy, gz;
    if (mpuLeer(ax, ay, az, gx, gy, gz)) {
      ok++; sx += ax; sy += ay; sz += az;
      if (ax == 0.0f && az == 0.0f) ceros++;
    }
    delay(10);
  }
  if (ok < 25) { magOut = NAN; return false; }
  float mx = sx / ok, my = sy / ok, mz = sz / ok;
  magOut = sqrtf(mx * mx + my * my + mz * mz);
  return (magOut > 0.85f && magOut < 1.15f && ceros < ok / 2);
}

bool iniciarMPU() {
  for (int intento = 1; intento <= 5; intento++) {
    bool w = mpuConfigurar();
    float mag;
    bool v = mpuVerificar(mag);
    infof("Config MPU, intento %d: escrituras %s, |a| = %.2f g -> %s\n",
          intento, w ? "OK" : "con error", mag, v ? "OK" : "NO valida");
    if (w && v) return true;
  }
  return false;
}

int leerReg(uint8_t reg) {
  Wire.beginTransmission(mpuAddr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return -1;
  if (Wire.requestFrom((int)mpuAddr, 1) != 1) return -1;
  return Wire.read();
}

void fijarReferencia() {
  gyroRef = leerReg(0x1B);
}

// Cada segundo: comprueba que el giroscopio conserva su escala y reescribe la configuracion (sin reset)
void mpuRefrescar(uint32_t t) {
  int g = leerReg(0x1B);
  if (g >= 0 && gyroRef >= 0 && g != gyroRef) {
    desajustes = desajustes + 1;
    infof("ATENCION: el MPU perdio la configuracion (t=%lu ms, 0x1B=0x%02X). Reconfigurado.\n", (unsigned long)t, g);
  }
  mpuEscribir(0x6B, 0x01);
  mpuEscribir(0x6C, 0x00);
  mpuEscribir(0x1A, 0x02);
  mpuEscribir(0x1B, GYRO_FS_BITS);
  mpuEscribir(0x1C, ACC_FS_BITS);
}

void recuperarMPU() {
  reconfigs = reconfigs + 1;
  Wire.end();
  Wire.begin(pinSDA, pinSCL);
  Wire.setClock(100000);
  Wire.setTimeOut(20);
  mpuConfigurar();
  fijarReferencia();
}

// ---------- SD (tarea en el nucleo 0) ----------
bool abrirLog() {
  if (!SD_MMC.begin("/sdcard", true)) return false;      // 1-bit

  for (int i = 1; i < 1000; i++) {
    snprintf(nombreArchivo, sizeof(nombreArchivo), "/log_%03d.csv", i);
    if (!SD_MMC.exists(nombreArchivo)) break;
  }
  logFile = SD_MMC.open(nombreArchivo, FILE_WRITE);
  if (!logFile) return false;

  infof("Registrando en %s\n", nombreArchivo);
  logFile.println("t_ms,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,hum_pct,temp_c,ev");
  logFile.flush();
  return true;
}

void tareaSD(void *param) {
  char linea[128];
  uint32_t ultimoFlushMs = millis();
  for (;;) {
    while (tail != head) {
      Muestra m = buf[tail];
      int n = snprintf(linea, sizeof(linea), "%lu,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.1f,%.1f,%u\n",
                       (unsigned long)m.t, m.v[0], m.v[1], m.v[2], m.v[3], m.v[4], m.v[5], m.v[6], m.v[7], (unsigned)m.ev);
      logFile.write((const uint8_t *)linea, n);
      tail = (tail + 1) % N_BUF;
    }
    if (millis() - ultimoFlushMs >= FLUSH_MS) {
      logFile.flush();
      ultimoFlushMs = millis();
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void empujar(uint32_t t, float ax, float ay, float az, float gx, float gy, float gz, float h, float c, uint16_t ev) {
  uint32_t sig = (head + 1) % N_BUF;
  if (sig == tail) { desbordes = desbordes + 1; return; }
  Muestra &m = buf[head];
  m.t = t;
  m.v[0] = ax; m.v[1] = ay; m.v[2] = az;
  m.v[3] = gx; m.v[4] = gy; m.v[5] = gz;
  m.v[6] = h;  m.v[7] = c;
  m.ev = ev;
  __sync_synchronize();
  head = sig;
}

// ---------- DHT (tarea en el nucleo 0) ----------
void tareaDHT(void *param) {
  for (;;) {
    float h = dht.readHumidity();
    float c = dht.readTemperature();
    if (!isnan(h)) humedad = h;
    if (!isnan(c)) temperatura = c;
    vTaskDelay(pdMS_TO_TICKS(DHT_PERIODO_MS));
  }
}

// ---------- Estado + comandos (tarea en el nucleo 0) ----------
void tareaEstado(void *param) {
  uint32_t ultimoEstado = 0;
  for (;;) {
    while (Serial.available()) procesarComando((char)Serial.read(), false);
    if (btActivo) {
      while (SerialBT.available()) procesarComando((char)SerialBT.read(), true);
    }

    if (btReenviarInfo && btActivo && btConectado) {
      btReenviarInfo = false;
      xSemaphoreTake(mtx, portMAX_DELAY);
      SerialBT.print("--- Resumen del arranque ---\n");
      SerialBT.write((const uint8_t *)historial, histLen);
      SerialBT.print("--- Comandos: i=resumen  c=chequeo  f=apagar BT para volar ---\n");
      xSemaphoreGive(mtx);
    }

    if (pedirApagarBT) {
      pedirApagarBT = false;
      if (btActivo) apagarBT("Modo vuelo.");
    }
    if (btActivo && (millis() / 1000) >= (uint32_t)BT_AUTO_OFF_S) apagarBT("Tiempo agotado.");

    uint32_t t = millis();
    if (t - ultimoEstado >= PRINT_MS) {
      ultimoEstado = t;
      float a0 = ult[0], a1 = ult[1], a2 = ult[2], g0 = ult[3], g1 = ult[4], g2 = ult[5];
      float h = humedad, c = temperatura;
      bool valido = ultValido;
      uint32_t cola = (head + N_BUF - tail) % N_BUF;
      bool problema = !sdOk || !valido || isnan(h);
      estadof("%s %lus | a(g) %.2f %.2f %.2f | g(dps) %.0f %.0f %.0f | hum %.1f%% T %.1fC | ok=%lu fallos=%lu reconf=%lu cfg=%lu buf=%lu perdidas=%lu%s%s%s\n",
              problema ? "ATENCION" : "OK", (unsigned long)(t / 1000),
              a0, a1, a2, g0, g1, g2, h, c,
              (unsigned long)muestras, (unsigned long)fallos, (unsigned long)reconfigs, (unsigned long)desajustes,
              (unsigned long)cola, (unsigned long)desbordes,
              sdOk ? "" : " [SD]", valido ? "" : " [MPU]", isnan(h) ? " [DHT]" : "");
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// ---------- Programa ----------
void setup() {
  mtx = xSemaphoreCreateMutex();
  Serial.begin(115200);
  delay(500);

  SerialBT.register_callback(btEvento);
  btActivo = SerialBT.begin(BT_NOMBRE);
  delay(300);

  infof("INICIO\n");               // si se repite solo, la placa se esta reiniciando
  if (btActivo) infof("Bluetooth activo: busca \"%s\" (se apaga con el comando f o a los %d s)\n", BT_NOMBRE, BT_AUTO_OFF_S);
  else          infof("AVISO: el Bluetooth no pudo iniciar\n");

  sdOk = abrirLog();               // SD primero, I2C despues
  if (!sdOk) infof("AVISO: SD no disponible, solo se muestra por pantalla\n");

  dht.begin();
  esperarMPU();
  if (iniciarMPU()) infof("MPU: configuracion verificada\n");
  else              infof("ADVERTENCIA: configuracion del MPU NO verificada. Deja el modulo quieto y pulsa RST.\n");
  fijarReferencia();

  if (sdOk) xTaskCreatePinnedToCore(tareaSD, "sd", 8192, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(tareaDHT, "dht", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(tareaEstado, "estado", 6144, NULL, 1, NULL, 0);

  infof("Listo. Registrando.\n");
  proximoLog_us = micros() + periodo_us;
}

void loop() {
  if (pedirChequeo) {
    pedirChequeo = false;
    infof("Repitiendo chequeo del MPU...\n");
    iniciarMPU();
    fijarReferencia();
    ceroSeguidos = 0;
    falloSeguidos = 0;
    proximoLog_us = micros() + periodo_us;
  }

  uint32_t ahora = micros();
  if ((int32_t)(ahora - proximoLog_us) < 0) return;
  proximoLog_us += periodo_us;
  if ((int32_t)(ahora - proximoLog_us) > (int32_t)(5 * periodo_us)) proximoLog_us = ahora + periodo_us;

  uint32_t t = millis();
  float ax, ay, az, gx, gy, gz;

  if (mpuLeer(ax, ay, az, gx, gy, gz)) {
    muestras = muestras + 1;
    falloSeguidos = 0;
    if (ax == 0.0f && az == 0.0f) ceroSeguidos++; else ceroSeguidos = 0;
    ult[0] = ax; ult[1] = ay; ult[2] = az; ult[3] = gx; ult[4] = gy; ult[5] = gz;
    ultValido = true;
  } else {
    fallos = fallos + 1;
    falloSeguidos++;
    ceroSeguidos = 0;
    ultValido = false;
    ax = ay = az = gx = gy = gz = NAN;
  }

  float h = humedad;
  float c = temperatura;
  if (sdOk) empujar(t, ax, ay, az, gx, gy, gz, h, c, (uint16_t)desajustes);

  if (t - ultimoRefresco >= 1000) {
    ultimoRefresco = t;
    mpuRefrescar(t);
  }

  if (ceroSeguidos >= 30 || falloSeguidos >= 20) {
    infof("MPU sin datos validos: reconfigurando...\n");
    recuperarMPU();
    ceroSeguidos = 0;
    falloSeguidos = 0;
    proximoLog_us = micros() + periodo_us;
  }
}
