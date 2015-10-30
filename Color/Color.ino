#include <Wire.h>

#define Color_Addr 0x29

void setup() {
  pinMode(2, OUTPUT);   // 出力に設定
  digitalWrite(2, HIGH);   // LEDをオン
  Wire.begin();
  Serial.begin(38400);
  I2C_SET();
}

//100ms毎にカラーセンサーのRGB値を取得してシリアルで送信
void loop() {
  I2C_GET();
  delay(100);
}

void I2C_GET(void) {
  int cnt_data = 32;
  byte data[cnt_data];
  int cnt_out = 0;
  byte out_data[4];

  //32Byte分の全レジスタ情報取得
  cnt_out = Wire.requestFrom(Color_Addr, cnt_data);
  if (cnt_out >= cnt_data)
  {
    for (int i = 0; i < cnt_data; i++) {
      data[i] = Wire.read();
    }
  }

  //RGB値を抜き取り
  out_data[0] = data[21];
  out_data[1] = data[23];
  out_data[2] = data[25];
  out_data[3] = data[27];

  Serial.print(out_data[0]);    Serial.print(",");
  Serial.print(out_data[1]);    Serial.print(",");
  Serial.print(out_data[2]);    Serial.print(",");
  Serial.println(out_data[3]);
}

/* 色センサに書き込み（必要なこと？） */
void I2C_SET(void) {
  Wire.beginTransmission(Color_Addr);
  Wire.write(0x00);     //ENABLE レジスタ指定
  Wire.write(0x03);     //PON = 1,  AEN = 1　にセット
  Wire.endTransmission();
}
