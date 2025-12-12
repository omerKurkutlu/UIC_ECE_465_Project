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

unsigned long lastMicros = 0;
bool firstRun = true;

// ---------- BLE ----------
const char* BLE_DEVICE_NAME = "Nano33BLE_IMU";

// SAME service UUID base, but we’ll send 6 floats now
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

void updateState() {
  int16_t axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw;
  readAccelGyro(&axRaw, &ayRaw, &azRaw, &gxRaw, &gyRaw, &gzRaw);

  // convert to physical units
  float ax_g = (float)axRaw / ACCEL_SCALE;  // in g
  float ay_g = (float)ayRaw / ACCEL_SCALE;
  float az_g = (float)azRaw / ACCEL_SCALE;

  float gx_dps = (float)gxRaw / GYRO_SCALE; // deg/s
  float gy_dps = (float)gyRaw / GYRO_SCALE;
  float gz_dps = (float)gzRaw / GYRO_SCALE;

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
  // assuming: x = forward, y = left, z = up
  float rollAcc  = atan2(ay_g, az_g) * 180.0f / PI;
  float pitchAcc = atan2(-ax_g, sqrt(ay_g*ay_g + az_g*az_g)) * 180.0f / PI;

  // integrate gyro
  roll  += gx_dps * dt;
  pitch += gy_dps * dt;
  yaw   += gz_dps * dt;   // yaw drift (no mag correction here)

  // complementary filter
  const float alpha = 0.98f;
  roll  = alpha * roll  + (1.0f - alpha) * rollAcc;
  pitch = alpha * pitch + (1.0f - alpha) * pitchAcc;
  // yaw stays gyro-only for now

  // ---------- Position (world frame) ----------

  // convert body accel from g to m/s^2
  float ax = ax_g * G;
  float ay = ay_g * G;
  float az = az_g * G;

  // build rotation matrix R (body -> world), ZYX order
  float r = roll  * PI / 180.0f;
  float p = pitch * PI / 180.0f;
  float y = yaw   * PI / 180.0f;

  float cy = cosf(y), sy = sinf(y);
  float cp = cosf(p), sp = sinf(p);
  float cr = cosf(r), sr = sinf(r);

  // same convention as your Python rpy_to_rotmat (ZYX)
  float R11 = cy*cp;
  float R12 = cy*sp*sr - sy*cr;
  float R13 = cy*sp*cr + sy*sr;

  float R21 = sy*cp;
  float R22 = sy*sp*sr + cy*cr;
  float R23 = sy*sp*cr - cy*sr;

  float R31 = -sp;
  float R32 = cp*sr;
  float R33 = cp*cr;

  // body accel -> world accel
  float awx = R11*ax + R12*ay + R13*az;
  float awy = R21*ax + R22*ay + R23*az;
  float awz = R31*ax + R32*ay + R33*az;

  // subtract gravity in world frame (z-up)
  awz -= G;

  // simple drift suppression when "almost still"
  float accNorm = sqrtf(awx*awx + awy*awy + awz*awz);
  if (accNorm < 0.1f) { // if ~no acceleration, slowly damp velocity
    vel[0] *= 0.98f;
    vel[1] *= 0.98f;
    vel[2] *= 0.98f;
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
      // error state; you could blink LED here
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

  Serial.println("Starting IMU Position+Orientation + BLE...");

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

    Serial.println("Central disconnected");
  }
}
