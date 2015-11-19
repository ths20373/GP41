#include <SPI.h>       //SPI通信

// ピン定義
#define PIN_SPI_MOSI 11   //7 
#define PIN_SPI_MISO 12   //5
#define PIN_SPI_SCK 13    //6
#define PIN_SPI_SS 10     //8

// グローバル変数
char Toggle1 = 0; //ステッピングモータ正転用トグル
char Toggle2 = 0; //ステッピングモータ逆転用トグル

// 初期化関数(初めに呼び出される)
void setup() {
  Serial.begin(9600);  //シリアル通信開始
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

// ループ関数(何も関数が動いてない時はここでアイドリング)
void loop() {
  if (Serial.available() > 0) warikomi();  //PCからシリアルデータが送られてきたら割り込み関数へ
}

// 割り込み関数
void warikomi() {
  // 変数定義
  char c;        //warikomi用シリアルデータ格納変数

  c = Serial.read();  //シリアルデータ読み込み

  //正転
  if (c == 'z') {
    Toggle1 ^= 1;
    if (Toggle1 == 1) {
      Toggle2 = 0;
      L6470_send(0x51);//Run(DIR,SPD),0x51:正転,0x50:逆転　
      L6470_send(0x00);//SPD値(20bit)
      L6470_send(0x10);
      L6470_send(0x00);
      Serial.print("アーム　正転\n");
    } else {
      L6470_send(0xB0);//SoftStop
      Serial.print("停止\n");
    }
  }

  //逆転
  if (c == 'c') {
    Toggle2 ^= 1;
    if (Toggle2 == 1) {
      Toggle1 = 0;
      L6470_send(0x50);//Run(DIR,SPD),0x51:正転,0x50:逆転　
      L6470_send(0x00);//SPD値(20bit)
      L6470_send(0x10);
      L6470_send(0x00);
      Serial.print("アーム　逆転\n");
    } else {
      L6470_send(0xB0);//SoftStop
      Serial.print("停止\n");
    }
  }

}

//---------------------------------------------------------------------------//
/* SPI通信でドライバーと通信 */
void L6470_send(unsigned char add_or_val) {
  digitalWrite(PIN_SPI_SS, LOW);
  SPI.transfer(add_or_val); // アドレスもしくはデータ送信。
  digitalWrite(PIN_SPI_SS, HIGH);
}

/* ステッピングモータセットアップ */
void L6470_setup() {
  //最大回転スピード
  L6470_send(0x07);//レジスタアドレス
  L6470_send(0x00);//値(10bit),デフォルト0x41
  L6470_send(0x30);
  //モータ停止中の電圧設定
  L6470_send(0x09);//レジスタアドレス
  L6470_send(0x20);//値(8bit),デフォルト0x29
  //モータ定速回転時の電圧設定
  L6470_send(0x0a);//レジスタアドレス
  L6470_send(0x40);//値(8bit),デフォルト0x29
  //加速中の電圧設定
  L6470_send(0x0b);//レジスタアドレス
  L6470_send(0x29);//値(8bit),デフォルト0x29
  //減速中の電圧設定
  L6470_send(0x0c);//レジスタアドレス
  L6470_send(0x29);//値(8bit),デフォルト0x29
  //フ ル ス テ ッ プ,ハ ー フ ス テ ッ プ,1/4, 1/8,…,1/128 ステップの設定
  L6470_send(0x16);//レジスタアドレス
  L6470_send(0x00);//値(8bit)
}
//---------------------------------------------------------------------------//
