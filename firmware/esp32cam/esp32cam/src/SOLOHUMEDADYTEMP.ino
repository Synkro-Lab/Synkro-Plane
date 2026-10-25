#include <Arduino.h>
#include <FS.h>
#include <SD_MMC.h>
#include <DHT.h>

#define DHT_PIN     13
#define DHT_TYPE    DHT11
#define PERIODO_MS  2000      // el DHT22 no da más de 1 lectura cada ~2 s

DHT dht(DHT_PIN, DHT_TYPE);
File logFile;
bool sdOk = false;
uint32_t proximo = 0;

bool abrirLog() {
  if (!SD_MMC.begin("/sdcard", true)) return false;   // modo 1-bit

  char nombre[24];
  for (int i = 1; i < 1000; i++) {
    snprintf(nombre, sizeof(nombre), "/vuelo_%03d.csv", i);
    if (!SD_MMC.exists(nombre)) break;
  }
  logFile = SD_MMC.open(nombre, FILE_WRITE);
  if (!logFile) return false;

  Serial.printf("Registrando en %s\n", nombre);
  logFile.println("t_ms,hum_pct,temp_c");
  logFile.flush();
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("INICIO");          // si se repite solo, la placa se está reiniciando

  dht.begin();
  sdOk = abrirLog();
  if (!sdOk) Serial.println("AVISO: SD no disponible, solo imprimo por serial");

  proximo = millis() + PERIODO_MS;   // el DHT necesita un momento tras encender
}

void loop() {
  if ((int32_t)(millis() - proximo) < 0) return;
  proximo += PERIODO_MS;

  float h = dht.readHumidity();
  float c = dht.readTemperature();
  uint32_t t = millis();

  if (isnan(h) || isnan(c)) {
    Serial.println("Lectura fallida del DHT22 (revisa DATA, 3V3 y GND)");
    return;
  }

  Serial.printf("%lu ms | Humedad: %.1f %% | Temp: %.1f C\n", t, h, c);

  if (sdOk) {
    logFile.printf("%lu,%.1f,%.1f\n", t, h, c);
    logFile.flush();                 // a esta frecuencia se puede guardar en cada lectura
  }
}