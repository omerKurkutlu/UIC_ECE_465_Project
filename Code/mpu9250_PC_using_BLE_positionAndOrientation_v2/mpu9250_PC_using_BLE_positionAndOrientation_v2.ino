#include <ArduinoBLE.h>
#include <Wire.h>

// ---------- IMU (MPU9250 / MPU6500) ----------
#define MPU_ADDR 0x68  // AD0=GND -> 0x68

// default full-scale: ±2g, ±250 dps
const float ACCEL_SCALE = 16384.0f; // LSB/g
const float GYRO_SCALE  = 131.0f;   // LSB/(deg/s)
const float G           = 9.80665f; // m/s^2

// orientation (deg)
float roll  = 0.0f;
float pitch = 0.0f;
float yaw   = 0.0f;

// position & velocity in world frame (m, m/s)
float pos[3] = {0.0f, 0.0f, 0.0f};
float vel[3] = {0.0f, 0.0f, 0.0f};

// biases
float accelBias_g[3] = {0.0f, 0.0f, 0.0f};   // in g units
float gyroBias_dps[3] = {0.0f, 0.0f, 0.0f};  // in deg/s

unsigned long lastMicros = 0;
bool firstRun = true;

// ---------- BLE ----------
const char* BLE_DEVICE_NAME = "Nano33BLE_IMU";

BLEService imuService("12345678-1234-5678-1234-56789abcdef0");
BLECharacteristic stateChar("12345678-1234-5678-1234-56789abcdef1",
                            BLERead | BLENotify,
                            24); // 6 floats = 24 bytes

// ---------- IMU FUNCTIONS ----------
void imuInit() {
  Wire.begin();

  // Wake up MPU
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);   // PWR_MGMT_1
  Wire.write(0x00);   // clear sleep bit
  Wire.endTransmission(true);

  delay(100);
}

void readAccelGyroRaw(int16_t* ax, int16_t* ay, int16_t* az,
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

// ---- Calibration: place sensor flat, Z up, and keep it still ----
void calibrateIMU() {
  //Serial.println("Calibrating... keep the sensor flat and still.");
  delay(1000);

  const int N = 2000;
  long sumAx = 0, sumAy = 0, sumAz = 0;
  long sumGx = 0, sumGy = 0, sumGz = 0;

  for (int i = 0; i < N; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    readAccelGyroRaw(&ax, &ay, &az, &gx, &gy, &gz);

    sumAx += ax;
    sumAy += ay;
    sumAz += az;
    sumGx += gx;
    sumGy += gy;
    sumGz += gz;

    delay(2);
  }

  float ax_avg = (float)sumAx / N;
  float ay_avg = (float)sumAy / N;
  float az_avg = (float)sumAz / N;
  float gx_avg = (float)sumGx / N;
  float gy_avg = (float)sumGy / N;
  float gz_avg = (float)sumGz / N;

  // convert accel avg to g
  float ax_g = ax_avg / ACCEL_SCALE;
  float ay_g = ay_avg / ACCEL_SCALE;
  float az_g = az_avg / ACCEL_SCALE;

  // assuming sensor is flat with Z up => expect [0, 0, 1g]
  accelBias_g[0] = ax_g - 0.0f;
  accelBias_g[1] = ay_g - 0.0f;
  accelBias_g[2] = az_g - 1.0f;

  // gyro avg to deg/s, expecting [0,0,0]
  gyroBias_dps[0] = gx_avg / GYRO_SCALE;
  gyroBias_dps[1] = gy_avg / GYRO_SCALE;
  gyroBias_dps[2] = gz_avg / GYRO_SCALE;

  // Serial.println("Calibration done.");
  // Serial.print("Accel bias (g): ");
  // Serial.print(accelBias_g[0], 4); Serial.print(", ");
  // Serial.print(accelBias_g[1], 4); Serial.print(", ");
  // Serial.println(accelBias_g[2], 4);

  // Serial.print("Gyro bias (dps): ");
  // Serial.print(gyroBias_dps[0], 4); Serial.print(", ");
  // Serial.print(gyroBias_dps[1], 4); Serial.print(", ");
  // Serial.println(gyroBias_dps[2], 4);
}

void updateState() {
  int16_t axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw;
  readAccelGyroRaw(&axRaw, &ayRaw, &azRaw, &gxRaw, &gyRaw, &gzRaw);

  // convert to physical units
  float ax_g = (float)axRaw / ACCEL_SCALE;
  float ay_g = (float)ayRaw / ACCEL_SCALE;
  float az_g = (float)azRaw / ACCEL_SCALE;

  float gx_dps = (float)gxRaw / GYRO_SCALE;
  float gy_dps = (float)gyRaw / GYRO_SCALE;
  float gz_dps = (float)gzRaw / GYRO_SCALE;

  // subtract biases
  ax_g -= accelBias_g[0];
  ay_g -= accelBias_g[1];
  az_g -= accelBias_g[2];

  gx_dps -= gyroBias_dps[0];
  gy_dps -= gyroBias_dps[1];
  gz_dps -= gyroBias_dps[2];

  // time step
  unsigned long now = micros();
  if (firstRun) {
    lastMicros = now;
    firstRun = false;
    return; // wait for next iteration for valid dt
  }
  float dt = (now - lastMicros) / 1e6f; // seconds
  lastMicros = now;

  // ---------- Orientation (complementary filter) ----------

  // accel-based angles (deg)
  float rollAcc  = atan2(ay_g, az_g) * 180.0f / PI;
  float pitchAcc = atan2(-ax_g, sqrt(ay_g*ay_g + az_g*az_g)) * 180.0f / PI;

  // integrate gyro
  roll  += gx_dps * dt;
  pitch += gy_dps * dt;
  yaw   += gz_dps * dt;

  // complementary filter
  const float alpha = 0.98f;
  roll  = alpha * roll  + (1.0f - alpha) * rollAcc;
  pitch = alpha * pitch + (1.0f - alpha) * pitchAcc;
  // yaw is gyro-only for now (no mag correction)

  // ---------- Position (world frame) ----------

  // body accel (m/s^2)
  float ax = ax_g * G;
  float ay = ay_g * G;
  float az = az_g * G;

  // rotation matrix (ZYX)
  float r = roll  * PI / 180.0f;
  float p = pitch * PI / 180.0f;
  float y = yaw   * PI / 180.0f;

  float cy = cosf(y), sy = sinf(y);
  float cp = cosf(p), sp = sinf(p);
  float cr = cosf(r), sr = sinf(r);

  float R11 = cy*cp;
  float R12 = cy*sp*sr - sy*cr;
  float R13 = cy*sp*cr + sy*sr;

  float R21 = sy*cp;
  float R22 = sy*sp*sr + cy*cr;
  float R23 = sy*sp*cr - cy*sr;

  float R31 = -sp;
  float R32 = cp*sr;
  float R33 = cp*cr;

  // accel in world frame
  float awx = R11*ax + R12*ay + R13*az;
  float awy = R21*ax + R22*ay + R23*az;
  float awz = R31*ax + R32*ay + R33*az;

  // subtract gravity in world frame (z-up)
  awz -= G;

  // ---- stillness detection (Zero-Velocity Update style) ----
  float accNorm = sqrtf(awx*awx + awy*awy + awz*awz);
  float gyroNorm = sqrtf(gx_dps*gx_dps + gy_dps*gy_dps + gz_dps*gz_dps);

  bool isStill = (accNorm < 0.2f) && (gyroNorm < 1.0f); // tune thresholds

  if (isStill) {
    // if we are almost not moving, force velocity to zero
    vel[0] = 0.0f;
    vel[1] = 0.0f;
    vel[2] = 0.0f;
  } else {
    // integrate acceleration -> velocity
    vel[0] += awx * dt;
    vel[1] += awy * dt;
    vel[2] += awz * dt;
  }

  // integrate velocity -> position
  pos[0] += vel[0] * dt;
  pos[1] += vel[1] * dt;
  pos[2] += vel[2] * dt;
}

// ---------- BLE SETUP ----------
void bleInit() {
  if (!BLE.begin()) {
    while (1) {
      // error state
    }
  }

  BLE.setLocalName(BLE_DEVICE_NAME);
  BLE.setDeviceName(BLE_DEVICE_NAME);
  BLE.setAdvertisedService(imuService);

  imuService.addCharacteristic(stateChar);
  BLE.addService(imuService);

  uint8_t buf[24] = {0};
  stateChar.writeValue(buf, 24);

  BLE.advertise();
}

// ---------- ARDUINO SETUP/LOOP ----------
void setup() {
  Serial.begin(115200);
  while (!Serial);

  //Serial.println("Starting IMU Position+Orientation + BLE (with calibration)...");

  imuInit();
  calibrateIMU();   // <-- important
  bleInit();

  lastMicros = micros();
}

void loop() {
  BLEDevice central = BLE.central();

  if (central) {
    //Serial.print("Connected to central: ");
    //Serial.println(central.address());

    while (central.connected()) {
      updateState();

      // pack: [pos_x, pos_y, pos_z, roll, pitch, yaw]
      float px = pos[0];
      float py = pos[1];
      float pz = pos[2];
      float r  = roll;
      float p  = pitch;
      float y  = yaw;

      uint8_t data[24];
      memcpy(&data[0],  &px, 4);
      memcpy(&data[4],  &py, 4);
      memcpy(&data[8],  &pz, 4);
      memcpy(&data[12], &r,  4);
      memcpy(&data[16], &p,  4);
      memcpy(&data[20], &y,  4);

      stateChar.writeValue(data, 24);

      // optional debug
      // Serial.print("Pos [m]: ");
      // Serial.print(px, 3); Serial.print(", ");
      // Serial.print(py, 3); Serial.print(", ");
      // Serial.print(pz, 3);
      // Serial.print(" | RPY [deg]: ");
      // Serial.print(r, 1); Serial.print(", ");
      // Serial.print(p, 1); Serial.print(", ");
      // Serial.println(y, 1);

      //delay(20); // ~50 Hz
    }

    //Serial.println("Central disconnected");
  }
}
