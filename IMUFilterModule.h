// IMUFilterModule.h
#ifndef IMU_FILTER_MODULE_H
#define IMU_FILTER_MODULE_H

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

HardwareSerial BTSerial(PA1, PA0);  // 蓝牙串口
Adafruit_MPU6050 imu;

// === 独立维护朝向 ===
double theta = 0.0;
unsigned long lastUpdate = 0;

// 零偏值
double ax_bias = 0, ay_bias = 0, gz_bias = 0;

// 声明滤波的全局变量（1D）
extern double posX;
extern double velX;

// === 一阶低通滤波相关 ===
double filtered_ax = 0.0;
double filtered_ay = 0.0;
double filtered_gz = 0.0;
const double imu_alpha = 0.2; // EMA系数，可调

double filtered_x = 0.0;
double filtered_y = 0.0;

// === 提供朝向角给其他模块 ===
double getTheta() {
  return theta;
}

void calibrateIMU() {
  Serial.println("开始 IMU 校准中（保持静止）...");
  sensors_event_t a, g, temp;
  const int N = 200;

  for (int i = 0; i < N; i++) {
    imu.getEvent(&a, &g, &temp);
    ax_bias += a.acceleration.x;
    ay_bias += a.acceleration.y;
    gz_bias += g.gyro.z;
    delay(5);
  }

  ax_bias /= N;
  ay_bias /= N;
  gz_bias /= N;

  Serial.println("IMU 校准完成:");
  Serial.print("ax_bias: "); Serial.println(ax_bias, 4);
  Serial.print("ay_bias: "); Serial.println(ay_bias, 4);
  Serial.print("gz_bias: "); Serial.println(gz_bias, 4);
}

void initIMU() {
  // BTSerial.begin(9600);  // 已在MazeBot.ino中初始化
  Serial.println("蓝牙串口已启用");

  if (!imu.begin()) {
    Serial.println("找不到 MPU6050/6500");
    while (1);
  }

  Serial.println("MPU 初始化成功");

  imu.setAccelerometerRange(MPU6050_RANGE_2_G);
  imu.setGyroRange(MPU6050_RANGE_250_DEG);
  imu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  delay(1000);  // 稳定后校准
  calibrateIMU();

  lastUpdate = millis();
}

// === 主循环调用，更新 theta 和滤波 ===
void updateIMUWithFilter() {
  sensors_event_t a, g, temp;
  imu.getEvent(&a, &g, &temp);

  double ax = a.acceleration.x - ax_bias;
  double ay = a.acceleration.y - ay_bias;
  double gz = g.gyro.z - gz_bias;

  filtered_ax = imu_alpha * ax + (1 - imu_alpha) * filtered_ax;
  filtered_ay = imu_alpha * ay + (1 - imu_alpha) * filtered_ay;
  filtered_gz = imu_alpha * gz + (1 - imu_alpha) * filtered_gz;

  // 死区处理，减少静止漂移
  if (fabs(filtered_ax) < 0.5) filtered_ax = 0;
  if (fabs(filtered_ay) < 0.5) filtered_ay = 0;

  unsigned long now = millis();
  double dt = (now - lastUpdate) / 1000.0;
  lastUpdate = now;
  theta += filtered_gz * dt;

  filtered_x += filtered_ax * dt * dt / 2.0;
  filtered_y += filtered_ay * dt * dt / 2.0;
}

// 可选：蓝牙回传当前角速度与朝向
void sendIMUData() {
  sensors_event_t a, g, temp;
  imu.getEvent(&a, &g, &temp);

  double gz = g.gyro.z - gz_bias;

  String data = "IMU:Gz=" + String(gz, 3) + " rad/s";
  String heading = "Theta=" + String(theta, 3) + " rad";

  BTSerial.println(data);
  BTSerial.println(heading);

  Serial.println(data);
  Serial.println(heading);
}

#endif
