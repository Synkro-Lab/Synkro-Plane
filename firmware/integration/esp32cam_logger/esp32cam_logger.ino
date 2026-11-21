/*
  Logger v5 (ESP32-CAM) = tu logger v4 + enlace serie con la Pico + cámara.

  Qué hace:
    - Recibe de la Pico una línea CSV por UART (100 Hz) y la guarda en la microSD: /log_NNN.csv
    - Toma fotos cuando la Pico lo pide (la columna "ph" sube) y las guarda como /foto_LLL_NNN.jpg
      (LLL = número de log, NNN = número de foto). La fila del log donde se pidió la foto lleva el número en la
      columna "foto", así sabes con qué actitud y altitud se tomó.
    - Bluetooth SOLO en tierra (Android, "Serial Bluetooth Terminal"): estado cada segundo y comandos.
      Se apaga solo al ver gas (CH3 > ~1100 us), con el comando f, o a los BT_AUTO_OFF_S segundos.
    - Los mensajes de la Pico que empiezan con '#' se guardan en el CSV como comentarios
      (en Python: pandas.read_csv("log_001.csv", comment="#")).

  PLACA EN EL IDE: "AI Thinker ESP32-CAM" (con PSRAM activado).
  Tools -> Partition Scheme -> "Huge APP (3MB No OTA/1MB SPIFFS)"  (el Bluetooth Classic ocupa mucho).

  Conexiones (ESP32-CAM <-> Pico), 3,3 V, cruzado:
    GPIO13 (RX2)  <-  GP8 (TX de UART1 de la Pico)
    GPIO4  (TX2)  ->  GP9 (RX de UART1 de la Pico)     (GPIO4 es el LED de flash: puede brillar tenue)
    GND común.  5V del riel del BEC.  microSD en el slot de la placa (modo 1 bit).
  Para programar usa el adaptador USB-serie en U0R/U0T y GPIO0 a GND (quítalo para operar).

  COMANDOS (por Bluetooth o por el monitor serie):
    i = repetir el resumen del arranque (incluye lo que reporta la Pico)
    c = recalibrar el giroscopio de la Pico (solo en tierra: modo manual y gas abajo; avión QUIETO ~2 s)
    p = foto de prueba
    f = apagar el Bluetooth para volar
    h = ayuda
*/

#include <Arduino.h>
#include <stdarg.h>
#include <FS.h>
#include <SD_MMC.h>
#include "BluetoothSerial.h"
#include "esp_camera.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error "Bluetooth no esta habilitado en esta placa. Usa la placa AI Thinker ESP32-CAM."
#endif

// ---------- Configuracion ----------
#define UART_RX           13
#define UART_TX           4
#define UART_BAUD         460800
#define N_CAMPOS          33        // campos por linea que manda la Pico
#define LINEA_MAX         220       // largo maximo aceptado de una linea
#define FLUSH_MS          500
#define PRINT_MS          1000

#define BT_NOMBRE         "Kestrel-Logger"
#define BT_AUTO_OFF_S     600       // el Bluetooth se apaga solo a los 10 minutos
#define BT_GAS_APAGA_US   1100      // ...o en cuanto el gas pasa de este valor
#define VENTANA_INFO_MS   15000     // durante el arranque, el resumen de la Pico se guarda en el historial
#define FOTO_MIN_GAP_MS   1500

// Pines de la camara (AI-Thinker)
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

// Posicion de cada campo en la linea de la Pico
enum Campo {
  C_T = 0, C_AX, C_AY, C_AZ, C_GX, C_GY, C_GZ, C_HUM, C_TEMP, C_EV, C_RC, C_MODE, C_ROLL, C_PITCH, C_ALT,
  C_CH1, C_CH2, C_CH3, C_CH4, C_CH5, C_CH6, C_CH7,
  C_OUT_AL, C_OUT_AR, C_OUT_EL, C_OUT_RU, C_OUT_ESC, C_CMD_R, C_CMD_P, C_CMD_Y, C_HS, C_PH, C_DT
};

const char *CABECERA =
  "t_ms,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,hum_pct,temp_c,ev,rc,mode,roll_deg,pitch_deg,alt_m,"
  "ch1,ch2,ch3,ch4,ch5,ch6,ch7,out_al,out_ar,out_el,out_ru,out_esc,cmd_r,cmd_p,cmd_y,hs,ph,dt_us,foto";

// ---------- Estado ----------
struct Linea { uint16_t n; char s[232]; };
struct Msg   { char t[240]; };
struct Ult {
  uint32_t rx_ms = 0, t_pico = 0;
  float a[3] = {NAN, NAN, NAN}, g[3] = {NAN, NAN, NAN}, hum = NAN, temp = NAN, roll = NAN, pitch = NAN, alt = NAN;
  int ev = 0, rc = 0, mode = 0, ch3 = 1000, hs = 0;
};

BluetoothSerial SerialBT;
SemaphoreHandle_t mtx;       // protege Serial/BT/historial
SemaphoreHandle_t sdMtx;     // protege la SD (log y fotos)
QueueHandle_t msgQ;          // mensajes '#' de la Pico hacia la tarea de estado

Linea *ring = nullptr;
uint32_t nRing = 0;
volatile uint32_t head = 0, tail = 0, desbordes = 0;

Ult ult;
volatile uint32_t lineasOk = 0, lineasErr = 0;
bool phInit = false;
uint32_t phPrev = 0;

File logFile;
int  logNum = 0;
bool sdOk = false, camOk = false;
volatile int fotoPendiente = 0;
int  fotoSiguiente = 1;
uint32_t ultimaFotoMs = 0;
volatile uint32_t fotosOk = 0;

volatile bool btActivo = false, btConectado = false, btReenviarInfo = false;
volatile bool pedirApagarBT = false;
const char * volatile motivoBT = "Modo vuelo.";
bool picoInfoRecibida = false;

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

void infof(const char *f, ...) {            // se muestra y se guarda en el resumen
  char tmp[320]; va_list ap; va_start(ap, f);
  int n = formatear(tmp, sizeof(tmp), f, ap); va_end(ap);
  if (n > 0) emitir(tmp, n, true);
}

void estadof(const char *f, ...) {          // se muestra, no se guarda
  char tmp[320]; va_list ap; va_start(ap, f);
  int n = formatear(tmp, sizeof(tmp), f, ap); va_end(ap);
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
    Serial2.print("!c\n");
    estadof("Recalibracion pedida a la Pico: deja el avion quieto, en modo manual y con el gas abajo...\n");
  } else if (c == 'p') {
    if (camOk && fotoPendiente == 0) { fotoPendiente = fotoSiguiente++; estadof("Foto de prueba pedida\n"); }
    else estadof("Foto no disponible (camara %s)\n", camOk ? "ocupada" : "con fallo");
  } else if (c == 'f') {
    motivoBT = "Modo vuelo.";
    pedirApagarBT = true;
  } else if (c == 'h' || c == '?') {
    estadof("Comandos: i = resumen | c = recalibrar gyro de la Pico (en tierra, quieto) | p = foto de prueba | f = apagar Bluetooth para volar\n");
  }
}

// ---------- SD ----------
bool abrirLog() {
  if (!SD_MMC.begin("/sdcard", true)) return false;      // 1 bit: deja libres GPIO12/13/4
  for (int i = 1; i < 1000; i++) {
    char nombre[24];
    snprintf(nombre, sizeof(nombre), "/log_%03d.csv", i);
    if (!SD_MMC.exists(nombre)) { logNum = i; break; }
  }
  char nombre[24];
  snprintf(nombre, sizeof(nombre), "/log_%03d.csv", logNum);
  logFile = SD_MMC.open(nombre, FILE_WRITE);
  if (!logFile) return false;
  infof("Registrando en %s\n", nombre);
  logFile.println(CABECERA);
  logFile.flush();
  return true;
}

void tareaSD(void *param) {
  uint32_t ultimoFlush = millis();
  for (;;) {
    xSemaphoreTake(sdMtx, portMAX_DELAY);
    int k = 0;
    while (tail != head && k < 200) {
      Linea &l = ring[tail];
      logFile.write((const uint8_t *)l.s, l.n);
      tail = (tail + 1) % nRing;
      k++;
    }
    if (millis() - ultimoFlush >= FLUSH_MS) { logFile.flush(); ultimoFlush = millis(); }
    xSemaphoreGive(sdMtx);
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void empujar(const char *txt, int n) {
  if (!sdOk) return;
  uint32_t sig = (head + 1) % nRing;
  if (sig == tail) { desbordes = desbordes + 1; return; }
  Linea &l = ring[head];
  if (n > (int)sizeof(l.s)) n = sizeof(l.s);
  memcpy(l.s, txt, n);
  l.n = n;
  __sync_synchronize();
  head = sig;
}

// ---------- Enlace con la Pico ----------
static int dividir(char *s, char **f, int max) {
  int n = 0;
  f[n++] = s;
  for (char *p = s; *p; p++) {
    if (*p == ',') { *p = 0; if (n < max) f[n++] = p + 1; }
  }
  return n;
}

void procesarLinea(char *ln, int n) {
  if (ln[0] == '#') {                                  // mensaje de la Pico
    char l2[236];
    int m = snprintf(l2, sizeof(l2), "%s\n", ln);
    if (m > 0) {
      if (m >= (int)sizeof(l2)) m = sizeof(l2) - 1;
      empujar(l2, m);                                  // tambien al CSV, como comentario
      Msg msg;
      memcpy(msg.t, l2, m); msg.t[m] = 0;
      xQueueSend(msgQ, &msg, 0);                       // la tarea de estado lo muestra
    }
    return;
  }
  if (n > LINEA_MAX) { lineasErr = lineasErr + 1; return; }

  char tmp[256];
  memcpy(tmp, ln, n); tmp[n] = 0;
  char *f[N_CAMPOS + 1];
  int nf = dividir(tmp, f, N_CAMPOS + 1);
  if (nf != N_CAMPOS) { lineasErr = lineasErr + 1; return; }
  char *fin;
  unsigned long t = strtoul(f[C_T], &fin, 10);
  if (fin == f[C_T] || *fin) { lineasErr = lineasErr + 1; return; }

  ult.t_pico = t;
  ult.a[0] = atof(f[C_AX]); ult.a[1] = atof(f[C_AY]); ult.a[2] = atof(f[C_AZ]);
  ult.g[0] = atof(f[C_GX]); ult.g[1] = atof(f[C_GY]); ult.g[2] = atof(f[C_GZ]);
  ult.hum = atof(f[C_HUM]); ult.temp = atof(f[C_TEMP]);
  ult.ev = atoi(f[C_EV]); ult.rc = atoi(f[C_RC]); ult.mode = atoi(f[C_MODE]);
  ult.roll = atof(f[C_ROLL]); ult.pitch = atof(f[C_PITCH]); ult.alt = atof(f[C_ALT]);
  ult.ch3 = atoi(f[C_CH3]); ult.hs = atoi(f[C_HS]);
  ult.rx_ms = millis();
  lineasOk = lineasOk + 1;

  // Apagado del Bluetooth al ver gas
  if (btActivo && ult.ch3 > BT_GAS_APAGA_US && ult.ch3 < 2100 && !pedirApagarBT) {
    motivoBT = "Gas detectado.";
    pedirApagarBT = true;
  }

  // Peticion de foto: sube la cuenta "ph". La primera linea solo fija la referencia.
  uint32_t ph = strtoul(f[C_PH], nullptr, 10);
  int fotoN = 0;
  if (!phInit) { phPrev = ph; phInit = true; }
  else if (ph != phPrev) {
    phPrev = ph;
    if (camOk && fotoPendiente == 0 && (millis() - ultimaFotoMs) >= FOTO_MIN_GAP_MS) {
      fotoN = fotoSiguiente++;
      fotoPendiente = fotoN;
      ultimaFotoMs = millis();
    }
  }

  char fs[8] = "";
  if (fotoN) snprintf(fs, sizeof(fs), "%d", fotoN);
  char salida[256];
  int m = snprintf(salida, sizeof(salida), "%s,%s\n", ln, fs);
  if (m > 0 && m < (int)sizeof(salida)) empujar(salida, m);
}

void tareaUART(void *param) {
  static char ln[256];
  int n = 0;
  for (;;) {
    while (Serial2.available()) {
      int c = Serial2.read();
      if (c == '\n') {
        if (n > 0) { ln[n] = 0; procesarLinea(ln, n); }
        n = 0;
      } else if (c != '\r') {
        if (n < (int)sizeof(ln) - 1) ln[n++] = (char)c; else n = 0;   // linea demasiado larga: se descarta
      }
    }
    vTaskDelay(1);
  }
}

// ---------- Camara ----------
bool iniciarCamara() {
  camera_config_t cfg = {};
  cfg.ledc_channel = LEDC_CHANNEL_0;
  cfg.ledc_timer = LEDC_TIMER_0;
  cfg.pin_d0 = Y2_GPIO_NUM; cfg.pin_d1 = Y3_GPIO_NUM; cfg.pin_d2 = Y4_GPIO_NUM; cfg.pin_d3 = Y5_GPIO_NUM;
  cfg.pin_d4 = Y6_GPIO_NUM; cfg.pin_d5 = Y7_GPIO_NUM; cfg.pin_d6 = Y8_GPIO_NUM; cfg.pin_d7 = Y9_GPIO_NUM;
  cfg.pin_xclk = XCLK_GPIO_NUM; cfg.pin_pclk = PCLK_GPIO_NUM;
  cfg.pin_vsync = VSYNC_GPIO_NUM; cfg.pin_href = HREF_GPIO_NUM;
  cfg.pin_sccb_sda = SIOD_GPIO_NUM; cfg.pin_sccb_scl = SIOC_GPIO_NUM;
  cfg.pin_pwdn = PWDN_GPIO_NUM; cfg.pin_reset = RESET_GPIO_NUM;
  cfg.xclk_freq_hz = 20000000;
  cfg.pixel_format = PIXFORMAT_JPEG;
  if (psramFound()) {
    cfg.frame_size = FRAMESIZE_UXGA;           // 1600x1200; baja a SVGA para archivos mas chicos
    cfg.jpeg_quality = 12;
    cfg.fb_count = 2;
    cfg.fb_location = CAMERA_FB_IN_PSRAM;
    cfg.grab_mode = CAMERA_GRAB_LATEST;        // siempre el cuadro mas reciente
  } else {
    cfg.frame_size = FRAMESIZE_SVGA;
    cfg.jpeg_quality = 14;
    cfg.fb_count = 1;
    cfg.fb_location = CAMERA_FB_IN_DRAM;
  }
  if (esp_camera_init(&cfg) != ESP_OK) return false;
  for (int i = 0; i < 5; i++) {                // descarta cuadros hasta que la exposicion se estabilice
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb) esp_camera_fb_return(fb);
    delay(100);
  }
  return true;
}

void tomarFoto(int n) {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) { estadof("ATENCION: la camara no entrego imagen (foto %d)\n", n); return; }
  char ruta[32];
  snprintf(ruta, sizeof(ruta), "/foto_%03d_%03d.jpg", logNum, n);
  xSemaphoreTake(sdMtx, portMAX_DELAY);
  File f = SD_MMC.open(ruta, FILE_WRITE);
  bool ok = false;
  if (f) { ok = (f.write(fb->buf, fb->len) == fb->len); f.close(); }
  xSemaphoreGive(sdMtx);
  esp_camera_fb_return(fb);
  if (ok) fotosOk = fotosOk + 1; else estadof("ATENCION: no se pudo guardar %s\n", ruta);
}

// ---------- Estado + comandos + mensajes (tarea en el nucleo 0) ----------
void tareaEstado(void *param) {
  uint32_t ultimoEstado = 0, lineasPrev = 0, ultimoPedidoInfo = 0;
  int intentosInfo = 0;
  for (;;) {
    while (Serial.available()) procesarComando((char)Serial.read(), false);
    if (btActivo) { while (SerialBT.available()) procesarComando((char)SerialBT.read(), true); }

    // Mensajes de la Pico ('#I' = resumen, '#E' = evento, '#N' = aviso)
    Msg msg;
    while (xQueueReceive(msgQ, &msg, 0) == pdTRUE) {
      bool esInfo = (msg.t[1] == 'I');
      if (esInfo) picoInfoRecibida = true;
      emitir(msg.t, strlen(msg.t), esInfo && millis() < VENTANA_INFO_MS);
    }

    // Pide el resumen a la Pico hasta recibirlo (puede estar calibrando al arrancar)
    if (!picoInfoRecibida && intentosInfo < 6 && millis() - ultimoPedidoInfo > 3000) {
      ultimoPedidoInfo = millis(); intentosInfo++;
      Serial2.print("!i\n");
    }

    if (btReenviarInfo && btActivo && btConectado) {
      btReenviarInfo = false;
      xSemaphoreTake(mtx, portMAX_DELAY);
      SerialBT.print("--- Resumen del arranque ---\n");
      SerialBT.write((const uint8_t *)historial, histLen);
      SerialBT.print("--- Comandos: i=resumen c=recalibrar p=foto f=apagar BT para volar ---\n");
      xSemaphoreGive(mtx);
    }

    if (pedirApagarBT) {
      pedirApagarBT = false;
      if (btActivo) apagarBT(motivoBT);
    }
    if (btActivo && (millis() / 1000) >= (uint32_t)BT_AUTO_OFF_S) apagarBT("Tiempo agotado.");

    uint32_t t = millis();
    if (t - ultimoEstado >= PRINT_MS) {
      ultimoEstado = t;
      Ult u = ult;
      uint32_t cola = (head + nRing - tail) % nRing;
      uint32_t lps = lineasOk - lineasPrev; lineasPrev = lineasOk;
      bool uartMuda = (u.rx_ms == 0) || (t - u.rx_ms > 500);
      bool mpuMal = !(u.hs & 1) || !(u.hs & 2);
      bool problema = !sdOk || uartMuda || mpuMal || isnan(u.hum) || !camOk;
      estadof("%s %lus | modo %c roll %.1f pitch %.1f alt %.1f | a(g) %.2f %.2f %.2f | g(dps) %.0f %.0f %.0f | hum %.1f%% T %.1fC | gas %d | lin/s %lu err %lu ev %d rc %d buf %lu perd %lu fotos %lu%s%s%s%s%s%s\n",
              problema ? "ATENCION" : "OK", (unsigned long)(t / 1000),
              "MEAF"[u.mode & 3], u.roll, u.pitch, u.alt,
              u.a[0], u.a[1], u.a[2], u.g[0], u.g[1], u.g[2], u.hum, u.temp, u.ch3,
              (unsigned long)lps, (unsigned long)lineasErr, u.ev, u.rc, (unsigned long)cola, (unsigned long)desbordes, (unsigned long)fotosOk,
              sdOk ? "" : " [SD]", uartMuda ? " [UART]" : "", mpuMal ? " [MPU]" : "", isnan(u.hum) ? " [DHT]" : "",
              camOk ? "" : " [CAM]", (u.hs & 8) ? " [FAILSAFE]" : "");
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// ---------- Programa ----------
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);    // evita reinicios por caidas breves de tension (cuida el riel de 5 V)
  mtx = xSemaphoreCreateMutex();
  sdMtx = xSemaphoreCreateMutex();
  msgQ = xQueueCreate(12, sizeof(Msg));
  Serial.begin(115200);
  delay(500);

  SerialBT.register_callback(btEvento);
  btActivo = SerialBT.begin(BT_NOMBRE);
  delay(300);

  infof("INICIO\n");                  // si se repite solo, la placa se esta reiniciando
  if (btActivo) infof("Bluetooth activo: busca \"%s\" (se apaga con f, al ver gas o a los %d s)\n", BT_NOMBRE, BT_AUTO_OFF_S);
  else          infof("AVISO: el Bluetooth no pudo iniciar\n");
  infof("PSRAM: %s\n", psramFound() ? "si" : "NO (camara en baja resolucion)");

  // Buffer circular de lineas (en PSRAM si hay: ~15 s a 100 Hz)
  if (psramFound()) { nRing = 1500; ring = (Linea *)ps_malloc(sizeof(Linea) * nRing); }
  if (!ring)        { nRing = 150;  ring = (Linea *)malloc(sizeof(Linea) * nRing); }

  sdOk = abrirLog();                  // la SD va antes que el UART2 (comparten pines GPIO4/13 al iniciar)
  if (!sdOk) infof("AVISO: SD no disponible, solo se muestra por pantalla\n");

  Serial2.setRxBufferSize(4096);
  Serial2.begin(UART_BAUD, SERIAL_8N1, UART_RX, UART_TX);

  camOk = iniciarCamara();
  infof("Camara: %s\n", camOk ? "OK" : "FALLO");

  xTaskCreatePinnedToCore(tareaUART, "uart", 6144, NULL, 3, NULL, 0);
  if (sdOk) xTaskCreatePinnedToCore(tareaSD, "sd", 8192, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(tareaEstado, "estado", 8192, NULL, 1, NULL, 0);

  infof("Listo. Esperando datos de la Pico...\n");
}

void loop() {                         // nucleo 1: camara
  int n = fotoPendiente;
  if (n > 0) {
    if (camOk) tomarFoto(n);
    fotoPendiente = 0;
  } else {
    delay(5);
  }
}
