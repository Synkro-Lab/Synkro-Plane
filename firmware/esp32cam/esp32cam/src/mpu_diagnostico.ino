/*
  Diagnostico del MPU6050 (GY-521) en la ESP32-CAM.
  Imprime identidad y registros del chip, y los valores CRUDOS del acelerometro y el giroscopio.
  No usa SD. Monitor serie a 115200.

  Conexiones (las mismas del logger): SDA -> IO13, SCL -> IO4, VCC -> 3V3, GND -> GND
*/

#include <Arduino.h>
#include <Wire.h>

uint8_t addr = 0x68;

uint8_t leerReg(uint8_t reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((int)addr, 1);
  return Wire.available() ? Wire.read() : 0xFF;
}

void escribirReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

bool probar(int sda, int scl) {
  Wire.end();
  Wire.begin(sda, scl);
  Wire.setClock(100000);
  Wire.setTimeOut(20);
  delay(5);
  const uint8_t dirs[2] = {0x68, 0x69};
  for (int i = 0; i < 2; i++) {
    addr = dirs[i];
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("MPU en 0x%02X con SDA=%d SCL=%d\n", addr, sda, scl);
      return true;
    }
  }
  return false;
}

void mostrarRegs(const char *titulo) {
  Serial.printf("%s | WHO_AM_I=0x%02X PWR1=0x%02X PWR2=0x%02X GYRO_CFG=0x%02X ACC_CFG=0x%02X\n",
                titulo, leerReg(0x75), leerReg(0x6B), leerReg(0x6C), leerReg(0x1B), leerReg(0x1C));
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("DIAGNOSTICO MPU");

  while (!(probar(13, 4) || probar(4, 13))) {
    Serial.println("Sin MPU, reintentando...");
    delay(2000);
  }

  mostrarRegs("Antes de configurar");
  escribirReg(0x6B, 0x80);          // reset
  delay(100);
  mostrarRegs("Tras reset         ");

  escribirReg(0x6B, 0x01);          // despertar, reloj del giroscopio X
  escribirReg(0x6C, 0x00);          // ningun eje en standby (acelerometro y giroscopio)
  escribirReg(0x1B, 0x08);          // giroscopio +-500 dps
  escribirReg(0x1C, 0x10);          // acelerometro +-8 g (4096 LSB/g)
  delay(50);
  mostrarRegs("Tras configurar    ");

  Wire.setClock(400000);
  Serial.println("Ahora mueve el modulo por sus 6 caras. Esperado: ~ +-4096 en el eje que apunta arriba/abajo, |a| ~ 1.00 g");
}

void loop() {
  Wire.beginTransmission(addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  if (Wire.requestFrom((int)addr, 14) != 14) {
    Serial.println("Lectura fallida");
    delay(200);
    return;
  }

  uint8_t b[14];
  for (int i = 0; i < 14; i++) b[i] = Wire.read();

  int16_t ax = (int16_t)((b[0] << 8) | b[1]);
  int16_t ay = (int16_t)((b[2] << 8) | b[3]);
  int16_t az = (int16_t)((b[4] << 8) | b[5]);
  int16_t gx = (int16_t)((b[8] << 8) | b[9]);
  int16_t gy = (int16_t)((b[10] << 8) | b[11]);
  int16_t gz = (int16_t)((b[12] << 8) | b[13]);

  float mag = sqrtf((float)ax * ax + (float)ay * ay + (float)az * az) / 4096.0f;

  Serial.printf("raw acc: %6d %6d %6d | |a|=%.2f g | raw gyro: %6d %6d %6d\n",
                ax, ay, az, mag, gx, gy, gz);
  delay(200);
}
