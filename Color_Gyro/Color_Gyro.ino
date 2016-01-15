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
//#include "VarSpeedServo.h"    //速度調節用のサーボライブラリ
//VarSpeedServo Yaw_Servo;    //加速度ジャイロと組み合わせて発射機構の左右を決める
//VarSpeedServo Pitch_Servo;  //加速度ジャイロと組み合わせて発射機構の上下を決める

Servo Yaw_Servo;    //加速度ジャイロと組み合わせて発射機構の左右を決める
Servo Pitch_Servo;  //加速度ジャイロと組み合わせて発射機構の上下を決める

//グローバル関数の宣言
char input[4];  // 文字列格納用
int i = 0;      // 文字数のカウンタ
int val = 0;    // 受信した数値
int deg1 = 0;    // サーボ1の角度
int deg2 = 0;    // サーボ2の角度

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
// ===              ステッピングモータの宣言関連                ===
// ================================================================
#include <SPI.h>       //SPI通信

// ピン定義
//3pin_DriverSide GND
//4pin_DriverSide 5v
#define PIN_SPI_MOSI 11   //7pin_DriverSide UNO:11,MEGA:51
#define PIN_SPI_MISO 12   //5pin_DriverSide UNO:12,MEGA:50
#define PIN_SPI_SCK 13    //6pin_DriverSide UNO:13,MEGA:52
#define PIN_SPI_SS 10     //8pin_DriverSide UNO:10,MEGA:53

boolean launch_status = false;

// ================================================================
// ===          シリアルに出すデバッグ作用のデファイン          ===
// ================================================================
#define SERIAL_DEBUG

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

  /* ステッピングモータ(使わなくなったので消してもいいかも) */
  //  Init_Stepping();
  //  L6470_gohome();

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

    deg1 = ypr[0] * 180 / M_PI;   //センサの値を取得
    deg2 = ypr[2] * 180 / M_PI;   //センサの値を取得

    deg1 = int(deg1);   //小数点切り捨て
    deg2 = int(deg2);   //小数点切り捨て
    deg1 /= 5;
    deg2 /= 5;
    deg1 += 90;
    deg2 += 155;

    Yaw_Servo.write(deg2); // サーボの角度を設定
    Pitch_Servo.write(deg1); // サーボの角度を設定

#ifdef SERIAL_DEBUG
    Serial.print("deg1:");
    Serial.print(deg1);
    Serial.print("\t");
    Serial.print("deg2:");
    Serial.println(deg2);
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

  //カラーセンサのLEDを消す
  //  pinMode(Color_Sensor_LED, OUTPUT);     // 出力に設定
  //  digitalWrite(Color_Sensor_LED, LOW);   // LEDをオフ
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
  prev_color = current_color;
  current_color = arrow_color;

  /*発射の検知*/
  if (current_color == "none" && prev_color == "red") {
  } else if (current_color == "none" && prev_color == "green") {
    Serial.print(1);
  } else if (current_color == "none" && prev_color == "blue") {
  } else if (current_color == "none" && prev_color == "none") {
    pull_power = 0;
  } else {
  }

#ifdef SERIAL_DEBUG
  Serial.print("\t");
  Serial.print("color : ");
  Serial.print(current_color);
#endif
}

void sendIntData(int value) {
  Serial.write(value);
}
// ================================================================
// ===                ステッピングモーター関連                  ===
// ================================================================
void Init_Stepping() {
  // ピン設定
  pinMode(PIN_SPI_MOSI, OUTPUT);  //SPI通信用ピン
  pinMode(PIN_SPI_MISO, INPUT);   //SPI通信用ピン
  pinMode(PIN_SPI_SCK, OUTPUT);   //SPI通信用ピン
  pinMode(PIN_SPI_SS, OUTPUT);    //SPI通信用ピン

  digitalWrite(PIN_SPI_SS, HIGH); //SPI通信用ピンの一部をHighにする

  //SPI通信開始
  SPI.begin();               //SPI通信開始
  SPI.setDataMode(SPI_MODE3);//SCKの立ち上がりでテータを送受信＆アイドル時はpinをHIGHに設定
  SPI.setBitOrder(MSBFIRST); //MSBから送信
  //SPI通信の前のコマンドの引数を消去
  L6470_send(0x00);//nop
  L6470_send(0x00);
  L6470_send(0x00);
  L6470_send(0x00);
  //SPI通信デバイスリセットコマンド
  L6470_send(0xc0);//ResetRevice
  L6470_setup();//ステッピングモータセットアップ(L6470を設定)
  delay(10);
}

/* ステッピングモータセットアップ */
void L6470_setup() {
  //最大回転スピード
  L6470_send(0x07);//レジスタアドレス(0x07)
  L6470_send(0x40);//値(10bit),デフォルト0x41
  L6470_send(0x00);
  //モータ停止中の電圧設定
  L6470_send(0x09);//レジスタアドレス(0x09)
  L6470_send(0x00);//値(8bit),デフォルト0x29
  //モータ定速回転時の電圧設定
  L6470_send(0x0a);//レジスタアドレス(0x0a)
  L6470_send(0xff);//値(8bit),デフォルト0x29
  //加速中の電圧設定
  L6470_send(0x0b);//レジスタアドレス(0x0b)
  L6470_send(0xa0);//値(8bit),デフォルト0x29
  //減速中の電圧設定
  L6470_send(0x0c);//レジスタアドレス(0x0c)
  L6470_send(0x60);//値(8bit),デフォルト0x29
  //フ ル ス テ ッ プ,ハ ー フ ス テ ッ プ,1/4, 1/8,…,1/128 ステップの設定
  L6470_send(0x16);//レジスタアドレス(0x16)
  L6470_send(0x00);//値(8bit)
  //失速電流しきい値設定？
  L6470_send(0x14);// レジスタアドレス
  L6470_send(0x40);//値(7bit) デフォルト0x40
}

/* SPI通信でドライバーと通信 */
void L6470_send(unsigned char add_or_val) {
  digitalWrite(PIN_SPI_SS, LOW);
  SPI.transfer(add_or_val); // アドレスもしくはデータ送信。
  digitalWrite(PIN_SPI_SS, HIGH);
}

void Rotate_Stepping() {
  L6470_move(0, 30);
}

void Reverse_Stepping() {
  L6470_move(1, 30);
}

void L6470_transfer(int add, int bytes, long val) {
  int data[3];
  L6470_send(add);
  for (int i = 0; i <= bytes - 1; i++) {
    data[i] = val & 0xff;
    val = val >> 8;
  }
  if (bytes == 3) {
    L6470_send(data[2]);
  }
  if (bytes >= 2) {
    L6470_send(data[1]);
  }
  if (bytes >= 1) {
    L6470_send(data[0]);
  }
}

void L6470_move(int dia, long n_step) {
  if (dia == 1)
    L6470_transfer(0x41, 3, n_step);
  else
    L6470_transfer(0x40, 3, n_step);
}

void L6470_run(int dia, long spd) {
  if (dia == 1)
    L6470_transfer(0x51, 3, spd);
  else
    L6470_transfer(0x50, 3, spd);
}

void L6470_gohome() {
  L6470_transfer(0x70, 0, 0);
}

void L6470_resetdevice() {
  L6470_send_u(0x00);//nop命令
  L6470_send_u(0x00);
  L6470_send_u(0x00);
  L6470_send_u(0x00);
  L6470_send_u(0xc0);
}

void L6470_send_u(unsigned char add_or_val) { //busyを確認せず送信するため用
  digitalWrite(PIN_SPI_SS, LOW); // ~SSイネーブル。
  SPI.transfer(add_or_val); // アドレスもしくはデータ送信。
  digitalWrite(PIN_SPI_SS, HIGH); // ~SSディスエーブル。
}

// L6470_softstop();　//回転停止、保持トルクあり
void L6470_softstop() {
  L6470_transfer(0xb0, 0, 0);
}
// L6470_hardstop();　//回転急停止、保持トルクあり
void L6470_hardstop() {
  L6470_transfer(0xb8, 0, 0);
}
// L6470_softhiz(); //回転停止、保持トルクなし
void L6470_softhiz() {
  L6470_transfer(0xa0, 0, 0);
}
// L6470_hardhiz(); //回転急停止、保持トルクなし
void L6470_hardhiz() {
  L6470_transfer(0xa8, 0, 0);
}

/*L6470 コントロール　コマンド
  引数-----------------------
  dia   1:正転 0:逆転,
  spd  (20bit)(0.015*spd[step/s])
  pos  (22bit)
  n_step (22bit)
  act   1:絶対座標をマーク  0:絶対座標リセット
  mssec ミリ秒
  val 各レジスタに書き込む値
  ---------------------------
  L6470_run(dia,spd); //指定方向に連続回転
  L6470_stepclock(dia); //指定方向にstepピンのクロックで回転
  L6470_move(dia,n_step); //指定方向に指定数ステップする
  L6470_goto(pos);　//指定座標に最短でいける回転方向で移動
  L6470_gotodia(dia,pos);　//回転方向を指定して指定座標に移動
  L6470_gountil(act,dia,spd);　//指定した回転方向に指定した速度で回転し、スイッチのONで急停止と座標処理
  L6470_relesesw(act,dia);　//スイッチがOFFに戻るまで最低速度で回転し、停止と座標処理
  L6470_gohome();　//座標原点に移動
  L6470_gomark();　//マーク座標に移動
  L6470_resetpos();　//絶対座標リセット
  L6470_resetdevice(); //L6470リセット
  L6470_softstop();　//回転停止、保持トルクあり
  L6470_hardstop();　//回転急停止、保持トルクあり
  L6470_softhiz(); //回転停止、保持トルクなし
  L6470_hardhiz(); //回転急停止、保持トルクなし
  L6470_getstatus(); //statusレジスタの値を返す （L6470_getparam_status(); と同じ）
*/
// ================================================================
// ===                   サーボモーター関連                     ===
// ================================================================
void Init_Servo() {
  /* 初期のピン設定 */
  Yaw_Servo.attach(12);    //回転
  Pitch_Servo.attach(13); //上下

  /* 初期のサーボの角度指定 */
  /* 的の真ん中に向けられるように */
  Yaw_Servo.write(85); // サーボの初期角度を設定
  Pitch_Servo.write(160); // サーボの初期角度を設定
}
