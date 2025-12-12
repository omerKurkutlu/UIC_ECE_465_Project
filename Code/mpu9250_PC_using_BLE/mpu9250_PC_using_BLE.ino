#include <ArduinoBLE.h>
#include <Wire.h>

// ---------- IMU (MPU9250/6500) ----------
#define MPU_ADDR 0x68  // AD0=GND -> 0x68

// scale factors for default config (±2g, ±250 dps)
const float ACCEL_SCALE = 16384.0; // LSB/g
const float GYRO_SCALE  = 131.0;   // LSB/(deg/s)

// orientation (deg)
float roll = 0.0f;
float pitch = 0.0f;
float yaw = 0.0f;

// for gyro integration
unsigned long lastMicros = 0;

// ---------- BLE ----------
const char* BLE_DEVICE_NAME = "Nano33BLE_IMU";

// same UUID as in your Python code
BLEService imuService("12345678-1234-5678-1234-56789abcdef0");
BLECharacteristic rpyChar("12345678-1234-5678-1234-56789abcdef1",
                          BLERead | BLENotify,
                          12); // 3 floats = 12 bytes

// ---------- IMU FUNCTIONS ----------
void imuInit() {
  Wire.begin();

  // Wake up MPU
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);   // PWR_MGMT_1
  Wire.write(0x00);   // clear sleep bit
  Wire.endTransmission(true);

  delay(100);

  // Optionally set accel & gyro full-scale ranges here if you want
  // (default is fine for now)
}

void readAccelGyro(int16_t* ax, int16_t* ay, int16_t* az,
                   int16_t* gx, int16_t* gy, int16_t* gz)
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);           // ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  *ax = (Wire.read() << 8) | Wire.read();
  *ay = (Wire.read() << 8) | Wire.read();
  *az = (Wire.read() << 8) | Wire.read();

  int16_t temp = (Wire.read() << 8) | Wire.read(); // not used

  *gx = (Wire.read() << 8) | Wire.read();
  *gy = (Wire.read() << 8) | Wire.read();
  *gz = (Wire.read() << 8) | Wire.read();
}

// simple complementary filter for R/P
void updateOrientation() {
  static bool firstRun = true;

  int16_t axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw;
  readAccelGyro(&axRaw, &ayRaw, &azRaw, &gxRaw, &gyRaw, &gzRaw);

  // convert to physical units
  float ax = (float)axRaw / ACCEL_SCALE;  // g
  float ay = (float)ayRaw / ACCEL_SCALE;  // g
  float az = (float)azRaw / ACCEL_SCALE;  // g

  float gx = (float)gxRaw / GYRO_SCALE;   // deg/s
  float gy = (float)gyRaw / GYRO_SCALE;   // deg/s
  float gz = (float)gzRaw / GYRO_SCALE;   // deg/s

  // time step
  unsigned long now = micros();
  if (firstRun) {
    lastMicros = now;
    firstRun = false;
    return; // we need dt from next iteration
  }
  float dt = (now - lastMicros) / 1e6f; // seconds
  lastMicros = now;

  // --- accel-based angles (deg) ---
  // assuming: x = forward, y = left, z = up
  float rollAcc  = atan2(ay, az) * 180.0f / PI;
  float pitchAcc = atan2(-ax, sqrt(ay*ay + az*az)) * 180.0f / PI;

  // --- integrate gyro ---
  roll  += gx * dt;
  pitch += gy * dt;
  yaw   += gz * dt;  // yaw will drift over time (no mag correction here)

  // --- complementary filter ---
  const float alpha = 0.98f;
  roll  = alpha * roll  + (1.0f - alpha) * rollAcc;
  pitch = alpha * pitch + (1.0f - alpha) * pitchAcc;
  // yaw stays purely from gyro in this simple example
}

// ---------- BLE SETUP ----------
void bleInit() {
  if (!BLE.begin()) {
    // if BLE init fails, block here
    while (1) {
      // optional: blink LED
    }
  }

  BLE.setLocalName(BLE_DEVICE_NAME);
  BLE.setDeviceName(BLE_DEVICE_NAME);
  BLE.setAdvertisedService(imuService);

  imuService.addCharacteristic(rpyChar);
  BLE.addService(imuService);

  // initial dummy value
  uint8_t buf[12] = {0};
  rpyChar.writeValue(buf, 12);

  BLE.advertise();
}

// ---------- ARDUINO SETUP/LOOP ----------
void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("Starting IMU + BLE...");

  imuInit();
  bleInit();

  lastMicros = micros();
}

void loop() {
  BLEDevice central = BLE.central();

  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());

    while (central.connected()) {
      // update R/P/Y from IMU
      updateOrientation();

      // pack 3 floats into 12-byte buffer (little-endian)
      float r = roll;
      float p = pitch;
      float y = yaw;

      uint8_t data[12];
      memcpy(&data[0],  &r, 4);
      memcpy(&data[4],  &p, 4);
      memcpy(&data[8],  &y, 4);

      // send via BLE notify
      rpyChar.writeValue(data, 12);

      // optional: debug over serial
      Serial.print("R: ");
      Serial.print(r);
      Serial.print("  P: ");
      Serial.print(p);
      Serial.print("  Y: ");
      Serial.println(y);

      delay(20); // ~50 Hz
    }

    Serial.println("Central disconnected");
  }
}
