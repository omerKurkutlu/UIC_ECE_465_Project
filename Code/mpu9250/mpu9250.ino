#include <Wire.h>

#define MPU_ADDR 0x68     // MPU6500 (Accel + Gyro)
#define MAG_ADDR 0x0C     // AK8963 Magnetometer

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Wire.begin();

  // Wake up MPU
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  // Enable bypass mode so Arduino can talk to magnetometer directly
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x37);   // INT_PIN_CFG
  Wire.write(0x02);   // BYPASS_EN = 1
  Wire.endTransmission(true);

  // Configure magnetometer
  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x0A);   // CNTL1 register
  Wire.write(0x16);   // 16-bit output, continuous mode 100 Hz
  Wire.endTransmission(true);

  Serial.println("MPU9250 (9-axis) initialized...");
}

void readAccelGyro(int16_t *a, int16_t *g) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  a[0] = (Wire.read()<<8) | Wire.read();
  a[1] = (Wire.read()<<8) | Wire.read();
  a[2] = (Wire.read()<<8) | Wire.read();

  Wire.read(); Wire.read();  // skip temp

  g[0] = (Wire.read()<<8) | Wire.read();
  g[1] = (Wire.read()<<8) | Wire.read();
  g[2] = (Wire.read()<<8) | Wire.read();
}

void readMag(int16_t *m) {
  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x03);
  Wire.endTransmission(false);
  Wire.requestFrom(MAG_ADDR, 7, true);

  m[0] = (Wire.read() | (Wire.read() << 8)); // X
  m[1] = (Wire.read() | (Wire.read() << 8)); // Y
  m[2] = (Wire.read() | (Wire.read() << 8)); // Z
  Wire.read(); // ST2 register (must be read)
}

void loop() {
  int16_t accel[3], gyro[3], mag[3];

  readAccelGyro(accel, gyro);
  readMag(mag);

  Serial.print("A: ");
  Serial.print(accel[0]); Serial.print(" ");
  Serial.print(accel[1]); Serial.print(" ");
  Serial.print(accel[2]); Serial.print(" | ");

  Serial.print("G: ");
  Serial.print(gyro[0]); Serial.print(" ");
  Serial.print(gyro[1]); Serial.print(" ");
  Serial.print(gyro[2]); Serial.print(" | ");

  Serial.print("M: ");
  Serial.print(mag[0]); Serial.print(" ");
  Serial.print(mag[1]); Serial.print(" ");
  Serial.print(mag[2]);

  Serial.println();
  delay(20);
}
