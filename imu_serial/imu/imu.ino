#include <ArduinoJson.h>
#include <SparkFunMPU9250-DMP.h>

#define SerialPort SerialUSB
// #define SerialPort Serial1

#define TODEG 180.0 / M_PI
#define TORAD M_PI / 180.0

MPU9250_DMP imu;
DynamicJsonDocument doc(1024);

float ACCEL_X_MIN = -16384, ACCEL_X_MAX = 16384;
float ACCEL_Y_MIN = -16384, ACCEL_Y_MAX = 16384;
float ACCEL_Z_MIN = -16384, ACCEL_Z_MAX = 16384;
float ACCEL_X_OFFSET, ACCEL_Y_OFFSET, ACCEL_Z_OFFSET;
float ACCEL_X_SCALE, ACCEL_Y_SCALE, ACCEL_Z_SCALE;
const float GRAVITY = 9.81;

struct Quaternion {
  double w, x, y, z;
};

struct EulerAngles {
  double roll, pitch, yaw;
};

void recalculateAccelCalibration() {
  ACCEL_X_OFFSET = ((ACCEL_X_MIN + ACCEL_X_MAX) / 2.0f);
  ACCEL_Y_OFFSET = ((ACCEL_Y_MIN + ACCEL_Y_MAX) / 2.0f);
  ACCEL_Z_OFFSET = ((ACCEL_Z_MIN + ACCEL_Z_MAX) / 2.0f);
  ACCEL_X_SCALE = (GRAVITY / (ACCEL_X_MAX - ACCEL_X_OFFSET));
  ACCEL_Y_SCALE = (GRAVITY / (ACCEL_Y_MAX - ACCEL_Y_OFFSET));
  ACCEL_Z_SCALE = (GRAVITY / (ACCEL_Z_MAX - ACCEL_Z_OFFSET));
}

EulerAngles ToEulerAngles(Quaternion q) {
  EulerAngles angles;

  double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
  double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
  angles.roll = atan2(sinr_cosp, cosr_cosp);

  double sinp = sqrt(1 + 2 * (q.w * q.y - q.x * q.z));
  double cosp = sqrt(1 - 2 * (q.w * q.y - q.x * q.z));
  angles.pitch = 2 * atan2(sinp, cosp) - M_PI / 2;

  double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
  angles.yaw = atan2(siny_cosp, cosy_cosp);

  return angles;
}

void setup() {
  SerialPort.begin(115200);

  if (imu.begin() != INV_SUCCESS) {
    while (1) {
      delay(5000);
    }
  }

  imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_CAL_GYRO |
               DMP_FEATURE_GYRO_CAL | DMP_FEATURE_SEND_RAW_ACCEL, 10);
  imu.resetFifo();
  delay(1);

  recalculateAccelCalibration();
}

void loop() {
  if (imu.fifoAvailable()) {
    if (imu.dmpUpdateFifo() == INV_SUCCESS) {
      imu.computeEulerAngles();
      printIMUData();
    }
  }
}

void printIMUData(void) {
  float q0 = imu.calcQuat(imu.qw);
  float q1 = imu.calcQuat(imu.qx);
  float q2 = imu.calcQuat(imu.qy);
  float q3 = imu.calcQuat(imu.qz);

  Quaternion quat = {q0, q1, q2, q3};

  float rawAx = imu.ax;
  float rawAy = imu.ay;
  float rawAz = imu.az;

  double AccX = (rawAx - ACCEL_X_OFFSET) * ACCEL_X_SCALE;
  double AccY = (rawAy - ACCEL_Y_OFFSET) * ACCEL_Y_SCALE;
  double AccZ = (rawAz - ACCEL_Z_OFFSET) * ACCEL_Z_SCALE;

  float gyX = imu.calcGyro(imu.gx);
  float gyY = imu.calcGyro(imu.gy);
  float gyZ = imu.calcGyro(imu.gz);

  EulerAngles euler = ToEulerAngles(quat);

  doc["roll"] = imu.roll;
  doc["pitch"] = imu.pitch;
  doc["yaw"] = imu.yaw;

  double alpha = euler.yaw;
  double beta = euler.pitch;
  double gamma = euler.roll;

  double x_ = (cos(alpha) * cos(beta)) * AccX +
              (cos(alpha) * sin(beta) * sin(gamma) - sin(alpha) * cos(gamma)) * AccY +
              (cos(alpha) * sin(beta) * cos(gamma) + sin(alpha) * sin(gamma)) * AccZ;

  double y_ = (sin(alpha) * cos(beta)) * AccX +
              (sin(alpha) * sin(beta) * sin(gamma) + cos(alpha) * cos(gamma)) * AccY +
              (sin(alpha) * sin(beta) * cos(gamma) - cos(alpha) * sin(gamma)) * AccZ;

  double z_ = (-sin(beta)) * AccX +
              (cos(beta) * sin(gamma)) * AccY +
              (cos(beta) * cos(gamma)) * AccZ;

  doc["accX"] = x_;
  doc["accY"] = y_;
  doc["accZ"] = z_;

  String jsonString;
  serializeJson(doc, jsonString);
  SerialPort.println(jsonString);
}