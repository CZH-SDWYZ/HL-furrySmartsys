#include <Wire.h>
#include <MPU6050.h>
#include <Kalman.h>
#include <Servo.h>
#include <SoftwareSerial.h>

// 创建对象
MPU6050 mpu;
Kalman kalmanX;
Kalman kalmanY;
SoftwareSerial yeeSerial(10,11);
Servo s1;
Servo s2;
Servo s3;

uint32_t timer;
float currentAngleS1 = 0;
float currentAngleS2 = 180;
float currentAngleS3 = 0;
int incomedate = 0;
bool gyroControlActive = true;

void setup() {
  yeeSerial.begin(9600);
  Wire.begin();
  digitalWrite(10,LOW);
  digitalWrite(11,LOW);

  // 初始化MPU6050
  yeeSerial.print("poweron");
  mpu.initialize();
  if (!mpu.testConnection()) {
    yeeSerial.print("connt");
    while (1);
  }
  yeeSerial.print("ok");

  yeeSerial.print("calibra");
  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);
  mpu.PrintActiveOffsets();

  // 初始化舵机
  s1.attach(2);  // 舵机s1连接到arduion管脚2
  s2.attach(3);  // 舵机s2连接到arduion管脚3
  s3.attach(4);  // 舵机s3连接到arduion管脚4

  s1.write(60);
  s2.write(60);
  s3.write(180);
  delay(500);
  s1.write(currentAngleS1);
  s2.write(currentAngleS2);
  s3.write(currentAngleS3);
  delay(1000);
  yeeSerial.print("yee");

  // 初始化计时器1
  timer = millis();
}
  // 舵机驱动
void smoothMove(Servo &servo, float &currentAngle, float targetAngle) {
  float step = abs(targetAngle - currentAngle) / 10.0;
  step = max(step, 1.5); // 最小步进值为1.5

  if (currentAngle < targetAngle) {
    currentAngle += step;
    delay(5); 
    if (currentAngle > targetAngle) {
      currentAngle = targetAngle;
    }
  } else if (currentAngle > targetAngle) {
    currentAngle -= step;
    delay(5); 
    if (currentAngle < targetAngle) {
      currentAngle = targetAngle;
    }
  }
  servo.write(currentAngle);
}

void checkMPU6050Connection() {
  if (!mpu.testConnection()) {
    yeeSerial.print("poweron");
    mpu.initialize();
    if (!mpu.testConnection()) {
      yeeSerial.print("connt");
      while (1);
    }
    yeeSerial.print("ok");
  }
}
  // 动作系统
void handleyeeSerialCommands() {
    if (yeeSerial.available() > 0) {
    incomedate=yeeSerial.read();
    if (incomedate==01) { // 动作1
      unsigned long startTime = millis();
      while (millis() - startTime < 10000) { // 持续10秒
        for (int pos = 0; pos <= 90; pos += 5) {
          s1.write(pos);
          s2.write(180 - pos);
          delay(30);
        }
        for (int pos = 90; pos >= 0; pos -= 5) {
          s1.write(pos);
          s2.write(180 - pos);
          delay(30);
        }
      }
      s1.write(0);
      s2.write(180);
    } else if (incomedate==02) { // 动作2
      unsigned long startTime = millis();
      while (millis() - startTime < 15000) { // 持续15秒
        s1.write(60);
        s2.write(180);
        delay(500);
        s1.write(0);
        s2.write(120);
        delay(500);
      }
      s1.write(0);
      s2.write(180);
    } else if (incomedate==03) { // 动作3
      unsigned long startTime = millis();
      s1.write(90);
      s2.write(90);
      while (millis() - startTime < 10000) { // 持续10秒
        delay(100); // 以防止阻塞处理其他串口输入
      }
      s1.write(0);
      s2.write(180);
    } else if (incomedate==04) { // 动作4
      unsigned long startTime = millis();
      s1.write(90);
      while (millis() - startTime < 1) { // 持续1秒
        delay(100);
      }
      s1.write(0);
      s2.write(180);
    } else if (incomedate==05) { // 动作5
      unsigned long startTime = millis();
      s2.write(90);
      while (millis() - startTime < 1) { // 持续1秒
        delay(100);
      }
      s1.write(0);
      s2.write(180);
    } else if (incomedate==06) {// 动作6
      unsigned long startTime = millis();
      s1.write(90);
      s2.write(90);
      while (millis() - startTime < 1) { // 持续1秒
        delay(100);
      }
      s1.write(0);
      s2.write(180);
    } 
  }
}

void loop() {
  handleyeeSerialCommands();
  checkMPU6050Connection();
  // 陀螺仪数据读取
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float axg = ax / 16384.0;
  float ayg = ay / 16384.0;
  float azg = az / 16384.0;

  // 陀螺仪数据换算
  float gxDeg = gx / 131.0;
  float gyDeg = gy / 131.0;
  float gzDeg = gz / 131.0;

  // 加速度计角度计算
  float rollAcc = atan2(ayg, azg) * 180 / M_PI;
  float pitchAcc = atan2(-axg, sqrt(ayg * ayg + azg * azg)) * 180 / M_PI;

  float dt = (float)(millis() - timer) / 1000.0;
  timer = millis();

  float roll = kalmanX.getAngle(rollAcc, gxDeg, dt);
  float pitch = kalmanY.getAngle(pitchAcc, gyDeg, dt);

  // 输出到舵机
  float targetAngleS1 = currentAngleS1;
  float targetAngleS2 = currentAngleS2;

  // 动耳算法
  if (pitch > 10) {
    targetAngleS1 = map(pitch, 10, 40, 0, 90);
    targetAngleS1 = constrain(targetAngleS1, 0, 90);
    targetAngleS2 = 180;
  } else if (pitch < -10) {
    targetAngleS2 = 180 - map(abs(pitch), 10, 40, 0, 90);
    targetAngleS2 = constrain(targetAngleS2, 90, 180); 
    targetAngleS1 = 0;
  } else if (roll > 10) {
    targetAngleS1 = map(roll, 10, 40, 0, 90);
    targetAngleS1 = constrain(targetAngleS1, 0, 90);
    targetAngleS2 = 180 - targetAngleS1;
  } else if (roll < -10) {
    targetAngleS1 = map(abs(roll), 10, 40, 0, 90);
    targetAngleS1 = constrain(targetAngleS1, 0, 90);
    targetAngleS2 = 180 - targetAngleS1;
  } else {
    targetAngleS1 = 0;
    targetAngleS2 = 180;
  smoothMove(s1, currentAngleS1, targetAngleS1);
  smoothMove(s2, currentAngleS2, targetAngleS2);

 }
}

