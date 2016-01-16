// ================================================================
// ===                加速度ジャイロの宣言関連                  ===
// ================================================================
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be p  assed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// ================================================================
// ===                    サーボの宣言関連                      ===
// ================================================================
#include <Servo.h>
Servo Yaw_Servo;    //加速度ジャイロと組み合わせて発射機構の左右を決める
Servo Pitch_Servo;  //加速度ジャイロと組み合わせて発射機構の上下を決める

#define YAW_OFFSET 70
#define PITCH_OFFSET 85
#define CENTER_CIRCLE 3
#define INNER_CIRCLE 6
#define OUTER_CIRCLE 9

char input[4];  // 文字列格納用
int i = 0;      // 文字数のカウンタ
int val = 0;    // 受信した数値
int Pitch_angle = 0;    // サーボ1の角度
int Yaw_angle = 0;    // サーボ2の角度

// ================================================================
// ===                  カラーセンサの宣言関連                  ===
// ================================================================
/* 色判定用に */
#include <Math.h>
byte i2cWriteBuffer[10];
byte i2cReadBuffer[10];
String arrow_color;
#define SensorAddressWrite 0x29 //
#define SensorAddressRead 0x29 // 
#define EnableAddress 0xa0 // register address + command bits
#define ATimeAddress 0xa1 // register address + command bits
#define WTimeAddress 0xa3 // register address + command bits
#define ConfigAddress 0xad // register address + command bits
#define ControlAddress 0xaf // register address + command bits
#define IDAddress 0xb2 // register address + command bits
#define ColorAddress 0xb4 // register address + command bits

String current_color = "none";
String prev_color = "none";
int pull_power = 0;

#define Color_Sensor_LED 4   //モータドライバ側の7pin

// ================================================================
// ===          シリアルに出すデバッグ作用のデファイン          ===
// ================================================================
//#define SERIAL_DEBUG

// ================================================================
// ===                      的の宣言関連                        ===
// ================================================================
//グローバル変数
int val22 = 0;     // 読み取った値を保持する変数
int val23 = 0;
int val24 = 0;
int val25 = 0;
int val26 = 0;
int val27 = 0;
int val28 = 0;
int val29 = 0;
int val30 = 0;
int val31 = 0;
int val32 = 0;
int val33 = 0;
int val34 = 0;
int flag = 1;

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void setup() {
  Serial.begin(9600);
  /* ジャイロセンサ */
  Gyro_I2C_SET();

  /* カラーセンサ */
  init_TCS34725();
  get_TCS34725ID();

  /* サーボモータ */
  Init_Servo();
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  /* ジャイロセンサの値取得 */
  //  Gyro_I2C_GET();
  /* カラーセンサの値取得 */
  //  get_Colors();
  /* 弓をの状態を取得 */
  //  Arrow_Status();
}

// ================================================================
// ===                      MY FUNCTION                         ===
// ================================================================
// ================================================================
// ===                   加速度ジャイロ関係                     ===
// ================================================================
void Gyro_I2C_SET() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize serial communication
  // (115200 chosen because it is required for Teapot Demo output, but it's
  // really up to you depending on your project)
  //  Serial.begin(115200);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
  // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
  // the baud timing being too misaligned with processor ticks. You must use
  // 38400 or slower in these cases, or use some kind of external separate
  // crystal solution for the UART timer.

  // initialize device
#ifdef SERIAL_DEBUG
  Serial.println(F("Initializing I2C devices..."));
#endif
  mpu.initialize();

  // verify connection
#ifdef SERIAL_DEBUG
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
#endif

  // load and configure the DMP
#ifdef SERIAL_DEBUG
  Serial.println(F("Initializing DMP..."));
#endif
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  /* 毎回動作させる前に専用のスケッチでオフセットを求める必要がある */
  mpu.setXGyroOffset(65);
  mpu.setYGyroOffset(4);
  mpu.setZGyroOffset(2);
  mpu.setZAccelOffset(1643); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
#ifdef SERIAL_DEBUG
    Serial.println(F("Enabling DMP..."));
#endif
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
#ifdef SERIAL_DEBUG
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
#endif
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
#ifdef SERIAL_DEBUG
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
#endif
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
#ifdef SERIAL_DEBUG
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
#endif
  }
}

void Gyro_I2C_GET() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
    // other program behavior stuff here
    // .
    // .
    // .
    // if you are really paranoid you can frequently test in between other
    // stuff to see if mpuInterrupt is true, and if so, "break;" from the
    // while() loop to immediately process the MPU data
    // .
    // .
    // .
  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
#ifdef SERIAL_DEBUG
    Serial.println(F("FIFO overflow!"));
#endif
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
#ifdef SERIAL_DEBUG
    Serial.print("ypr\t");
    Serial.print(ypr[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(ypr[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(ypr[2] * 180 / M_PI);
    Serial.print("\t");
#endif

    Yaw_angle = ypr[0] * 180 / M_PI;     //センサの値を取得
    Pitch_angle = ypr[2] * 180 / M_PI;   //センサの値を取得

    Yaw_angle = int(Yaw_angle);       //小数点切り捨て
    Pitch_angle = int(Pitch_angle);   //小数点切り捨て

    Yaw_angle /= 4;
    Pitch_angle /= 4;

    Yaw_angle += YAW_OFFSET;
    Pitch_angle += PITCH_OFFSET;

    Yaw_Servo.write(Yaw_angle);     // サーボの角度を設定
    Pitch_Servo.write(Pitch_angle); // サーボの角度を設定

#ifdef SERIAL_DEBUG
    Serial.print("Pitch_angle:");
    Serial.print(Pitch_angle);
    Serial.print(" ");
    Serial.print("Yaw_angle:");
    Serial.println(Yaw_angle);
#endif
  }
}

// ================================================================
// ===                     色センサー関連                       ===
// ================================================================
void Writei2cRegisters(byte numberbytes, byte command)
{
  byte i = 0;

  Wire.beginTransmission(SensorAddressWrite);   // Send address with Write bit set
  Wire.write(command);                          // Send command, normally the register address
  for (i = 0; i < numberbytes; i++)                 // Send data
    Wire.write(i2cWriteBuffer[i]);
  Wire.endTransmission();

  delayMicroseconds(100);      // allow some time for bus to settle
}

/*
  Send register address to this function and it returns byte value
  for the magnetometer register's contents
*/
byte Readi2cRegisters(int numberbytes, byte command)
{
  byte i = 0;

  Wire.beginTransmission(SensorAddressWrite);   // Write address of read to sensor
  Wire.write(command);
  Wire.endTransmission();

  delayMicroseconds(100);      // allow some time for bus to settle

  Wire.requestFrom(SensorAddressRead, numberbytes);  // read data
  for (i = 0; i < numberbytes; i++)
    i2cReadBuffer[i] = Wire.read();
  Wire.endTransmission();

  delayMicroseconds(100);      // allow some time for bus to settle
}

void init_TCS34725(void) {
  i2cWriteBuffer[0] = 0x10;
  Writei2cRegisters(1, ATimeAddress);   // RGBC timing is 256 - contents x 2.4mS =
  i2cWriteBuffer[0] = 0x00;
  Writei2cRegisters(1, ConfigAddress);  // Can be used to change the wait time
  i2cWriteBuffer[0] = 0x00;
  Writei2cRegisters(1, ControlAddress); // RGBC gain control
  i2cWriteBuffer[0] = 0x03;
  Writei2cRegisters(1, EnableAddress);   // enable ADs and oscillator for sensor
}

void get_TCS34725ID(void) {
  Readi2cRegisters(1, IDAddress);
  if (i2cReadBuffer[0] = 0x44) {
    //    Serial.println("TCS34725 is present");
  } else {
    //    Serial.println("TCS34725 not responding");
  }
}

void get_Colors(void) {
  unsigned int clear_color = 0;
  unsigned int red_color = 0;
  unsigned int green_color = 0;
  unsigned int blue_color = 0;

  Readi2cRegisters(8, ColorAddress);
  clear_color = (unsigned int)(i2cReadBuffer[1] << 8) + (unsigned int)i2cReadBuffer[0];
  red_color = (unsigned int)(i2cReadBuffer[3] << 8) + (unsigned int)i2cReadBuffer[2];
  green_color = (unsigned int)(i2cReadBuffer[5] << 8) + (unsigned int)i2cReadBuffer[4];
  blue_color = (unsigned int)(i2cReadBuffer[7] << 8) + (unsigned int)i2cReadBuffer[6];

  // Basic RGB color differentiation can be accomplished by comparing the values and the largest reading will be
  // the prominent color

  if ((red_color > blue_color) && (red_color > green_color)) {
    arrow_color = "red";
  }  else if ((green_color > blue_color) && (green_color > red_color)) {
    arrow_color = "green";
  } else if ((blue_color > red_color) && (blue_color > green_color)) {
    arrow_color = "blue";
  } else {
    arrow_color = "none";
  }
}
// ================================================================
// ===                     弓矢の状態関連                       ===
// ================================================================
void Arrow_Status(void) {
  int Yaw_diff = 0;
  int Pitch_diff = 0;

  prev_color = current_color;
  current_color = arrow_color;

  Yaw_diff = abs(YAW_OFFSET - Yaw_angle);
  Pitch_diff = abs(PITCH_OFFSET - Pitch_angle);

  /*発射の検知*/
  if (current_color == "none" && !(prev_color == "none")) {
    if ( sqrt(Yaw_diff ^ 2 + Pitch_diff ^ 2) <= CENTER_CIRCLE) {
      /* 真ん中の判定 */
      Serial.print(1);
    } else if ( (sqrt(Yaw_diff ^ 2 + Pitch_diff ^ 2) >= CENTER_CIRCLE) && (sqrt(Yaw_diff ^ 2 + Pitch_diff ^ 2) <= INNER_CIRCLE) ) {
      /* 内円のどこに当たっているか */
      if (YAW_OFFSET > Yaw_angle ) {
        if (PITCH_OFFSET > Pitch_angle) {
          Serial.print(6);
        } else {
          Serial.print(4);
        }
      } else {
        if (PITCH_OFFSET > Pitch_angle) {
          Serial.print(7);
        } else {
          Serial.print(5);
        }
      }
    } else if ( (sqrt(Yaw_diff ^ 2 + Pitch_diff ^ 2) >= INNER_CIRCLE) && (sqrt(Yaw_diff ^ 2 + Pitch_diff ^ 2) <= OUTER_CIRCLE) ) {
      /* 外円のどこに当たっているか */
      if (YAW_OFFSET > Yaw_angle ) {
        if (PITCH_OFFSET > Pitch_angle) {
          Serial.print(8);
        } else {
          Serial.print(2);
        }
      } else {
        if (PITCH_OFFSET > Pitch_angle) {
          Serial.print(9);
        } else {
          Serial.print(3);
        }
      }
    } else {
      /* ハズレ */
      Serial.print(0);
    }
  }

#ifdef SERIAL_DEBUG
  Serial.print("color : ");
  Serial.print(current_color);
#endif
}

// ================================================================
// ===                   サーボモーター関連                     ===
// ================================================================
void Init_Servo() {
  /* 初期のピン設定 */
  Yaw_Servo.attach(13);    //回転
  Pitch_Servo.attach(12);  //上下

  /* 初期のサーボの角度指定 */
  /* 的の真ん中に向けられるように */
  Yaw_Servo.write(YAW_OFFSET); // サーボの初期角度を設定
  Pitch_Servo.write(PITCH_OFFSET); // サーボの初期角度を設定
}
