#include <Servo.h> 
Servo servo1;        //MEGA:D22,UNO:D9 Servoオブジェクトを作成
Servo servo2;        //MEGA:D22,UNO:D10 Servoオブジェクトを作成

//グローバル関数の宣言
char input[4];  // 文字列格納用
int i = 0;      // 文字数のカウンタ
int val = 0;    // 受信した数値
int deg1 = 0;    // サーボの角度
int deg2 = 0;    // サーボの角度
 
void setup()
{ 
  servo1.attach(9);  //D9ピンをサーボの信号線として設定
  servo2.attach(10);  //D9ピンをサーボの信号線として設定
  Serial.begin(9600);  //シリアル通信開始

} 

//シリアル通信関数
int interruct(){
  input[i] = Serial.read(); //シリアルデータの読み込み 
   // 文字数が2以上 or 末尾文字がある場合の処理
   if (i > 1 || input[i] == '.') {
      input[i] = '\0';      // 末尾に終端文字の挿入
      val = atoi(input);    // 文字列を数値に変換
      Serial.write(input); // 文字列を送信
      Serial.write("\n");
      i = 0;      // カウンタの初期化
    }
    else { 
      i++; 
    }
  return val; 
}

//メインループ
void loop(){ 
  deg1 = interruct();   //センサの値を取得
  deg2 = interruct();   //センサの値を取得
  deg1 = int(deg1);   //小数点切り捨て
  deg1+=180;  //サーボ1の初期位置を180度にする
  deg2 = int(deg1);   //小数点切り捨て
  deg2+=180;  //サーボ2の初期位置を180度にする
  
  servo1.write(deg1); // サーボの角度を設定
  servo2.write(deg2); // サーボの角度を設定
}

