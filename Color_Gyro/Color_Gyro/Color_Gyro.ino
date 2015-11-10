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
// specific I2C addresses may be passed as a parameter here
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

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };

#define Color_Addr 0x29

#define Gyro_Addr 0x68

/* サーボの結合 */
#include <Servo.h>
Servo servo1;        //MEGA:D22,UNO:D9 Servoオブジェクトを作成
Servo servo2;        //MEGA:D22,UNO:D10 Servoオブジェクトを作成
//グローバル関数の宣言
char input[4];  // 文字列格納用
int i = 0;      // 文字数のカウンタ
int val = 0;    // 受信した数値
int deg1 = 0;    // サーボの角度
int deg2 = 0;    // サーボの角度

/* サーボのデバッグ用宣言 */
//#define DEBUG_SERVO

/* 色判定用に */
#include <Math.h>
byte i2cWriteBuffer[10];
byte i2cReadBuffer[10];
#define SensorAddressWrite 0x29 //
#define SensorAddressRead 0x29 // 
#define EnableAddress 0xa0 // register address + command bits
#define ATimeAddress 0xa1 // register address + command bits
#define WTimeAddress 0xa3 // register address + command bits
#define ConfigAddress 0xad // register address + command bits
#define ControlAddress 0xaf // register address + command bits
#define IDAddress 0xb2 // register address + command bits
#define ColorAddress 0xb4 // register address + command bits



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
  pinMode(3, OUTPUT);   // 出力に設定
  digitalWrite(3, LOW);   // LEDをオン
  Gyro_I2C_SET();
  init_TCS34725();
  get_TCS34725ID();     // get the device ID, this is just a test to see if we're connected
#ifdef DEBUG_SERVO
  servo1.attach(9);  //D9ピンをサーボの信号線として設定
  servo2.attach(10);  //D9ピンをサーボの信号線として設定
  servo1.write(90); // サーボの角度を設定
  servo2.write(90); // サーボの角度を設定
#endif
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  // Gyro_I2C_GET();
  get_Colors();
}

// ================================================================
// ===                      MY FUNCTION                         ===
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
  Serial.begin(115200);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
  // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
  // the baud timing being too misaligned with processor ticks. You must use
  // 38400 or slower in these cases, or use some kind of external separate
  // crystal solution for the UART timer.

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  /* 毎回動作させる前に専用のスケッチでオフセットを求める必要がある */
  mpu.setXGyroOffset(77);
  mpu.setYGyroOffset(-103);
  mpu.setZGyroOffset(39);
  mpu.setZAccelOffset(1969); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
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
    Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

#ifdef OUTPUT_READABLE_YAWPITCHROLL
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    Serial.print("ypr\t");
    Serial.print(ypr[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(ypr[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.println(ypr[2] * 180 / M_PI);
#endif
#ifdef DEBUG_SERVO
    deg1 = ypr[0] * 180 / M_PI;   //センサの値を取得
    deg2 = ypr[2] * 180 / M_PI;   //センサの値を取得
    deg1 = int(deg1);   //小数点切り捨て
    deg1 += 90; //サーボ1の初期位置を180度にする
    deg2 = int(deg2);   //小数点切り捨て
    deg2 += 90; //サーボ2の初期位置を180度にする
    servo1.write(deg1); // サーボの角度を設定
    servo2.write(deg2); // サーボの角度を設定
    Serial.print("deg1\t");
    Serial.print(deg1);
    Serial.print("deg2\t");
    Serial.println(deg2);
#endif
  }
}

/*
Send register address and the byte value you want to write the magnetometer and
loads the destination register with the value you send
*/
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

void init_TCS34725(void)
{
  i2cWriteBuffer[0] = 0x10;
  Writei2cRegisters(1, ATimeAddress);   // RGBC timing is 256 - contents x 2.4mS =
  i2cWriteBuffer[0] = 0x00;
  Writei2cRegisters(1, ConfigAddress);  // Can be used to change the wait time
  i2cWriteBuffer[0] = 0x00;
  Writei2cRegisters(1, ControlAddress); // RGBC gain control
  i2cWriteBuffer[0] = 0x03;
  Writei2cRegisters(1, EnableAddress);   // enable ADs and oscillator for sensor
}

void get_TCS34725ID(void)
{
  Readi2cRegisters(1, IDAddress);
  if (i2cReadBuffer[0] = 0x44)
    Serial.println("TCS34725 is present");
  else
    Serial.println("TCS34725 not responding");
}

/*
Reads the register values for clear, red, green, and blue.
*/
void get_Colors(void)
{
  unsigned int clear_color = 0;
  unsigned int red_color = 0;
  unsigned int green_color = 0;
  unsigned int blue_color = 0;

  Readi2cRegisters(8, ColorAddress);
  clear_color = (unsigned int)(i2cReadBuffer[1] << 8) + (unsigned int)i2cReadBuffer[0];
  red_color = (unsigned int)(i2cReadBuffer[3] << 8) + (unsigned int)i2cReadBuffer[2];
  green_color = (unsigned int)(i2cReadBuffer[5] << 8) + (unsigned int)i2cReadBuffer[4];
  blue_color = (unsigned int)(i2cReadBuffer[7] << 8) + (unsigned int)i2cReadBuffer[6];

  // send register values to the serial monitor

  Serial.print("clear color=");
  Serial.print(clear_color, DEC);
  Serial.print(" red color=");
  Serial.print(red_color, DEC);
  Serial.print(" green color=");
  Serial.print(green_color, DEC);
  Serial.print(" blue color=");
  Serial.print(blue_color, DEC);


  // Basic RGB color differentiation can be accomplished by comparing the values and the largest reading will be
  // the prominent color

  if ((red_color > blue_color) && (red_color > green_color))
    Serial.println("detecting red");
  else if ((green_color > blue_color) && (green_color > red_color))
    Serial.println("detecting green");
  else if ((blue_color > red_color) && (blue_color > green_color))
    Serial.println("detecting blue");
  else
    Serial.println("color not detectable");

}
