//モーターの配線色
//OUT1A 青
//OUT2A 赤
//OUT2B 緑
//OUT1B 黒

#include <SPI.h>       //SPI通信

// ピン定義
//3pin_DriverSide GND
//4pin_DriverSide 5v
#define PIN_SPI_MOSI 11   //7pin_DriverSide UNO:11,MEGA:51
#define PIN_SPI_MISO 12   //5pin_DriverSide UNO:12,MEGA:50
#define PIN_SPI_SCK 13    //6pin_DriverSide UNO:13,MEGA:52
#define PIN_SPI_SS 10     //8pin_DriverSide UNO:10,MEGA:53

// グローバル変数
char Toggle1=0;//ステッピングモータ正転用トグル
char Toggle2=0;//ステッピングモータ逆転用トグル

// 初期化関数(初めに呼び出される)
void setup() {
  Serial.begin(115200);  //シリアル通信開始
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
    Toggle1^=1;
    if(Toggle1==1){
      Toggle2=0;
      L6470_send(0x51);//Run(DIR,SPD),0x51:正転,0x50:逆転　
      L6470_send(0x41);//SPD値(20bit)
      L6470_send(0x00);
      L6470_send(0x20);
      Serial.print("forward\n");
      
      delay(3000);
      
      L6470_send(0x51);//Run(DIR,SPD),0x51:正転,0x50:逆転　
      L6470_send(0xd1);//SPD値(20bit)
      L6470_send(0x05);
      L6470_send(0x05);
      Serial.print("forward30\n");
      
      delay(3000);
      
      L6470_send(0x51);//Run(DIR,SPD),0x51:正転,0x50:逆転　
      L6470_send(0xe1);//SPD値(20bit)
      L6470_send(0x10);
      L6470_send(0x10);
      Serial.print("forward60\n");
    }else{
      L6470_send(0xB0);//SoftStop
      Serial.print("STOP\n");
    }
  }
  
  //逆転
  if (c == 'c') {
    Toggle2^=1;
    if(Toggle2==1){
      Toggle1=0;
      L6470_send(0x50);//Run(DIR,SPD),0x51:正転,0x50:逆転　
      L6470_send(0x80);//SPD値(20bit)
      L6470_send(0x20);
      L6470_send(0x40);
      Serial.print("arm reverce\n");
    }else{
      L6470_send(0xB0);//SoftStop
      Serial.print("Stop\n");
    }
  }

  
  
}

//---------------------------------------------------------------------------//
/* SPI通信でドライバーと通信 */
void L6470_send(unsigned char add_or_val){
  digitalWrite(PIN_SPI_SS, LOW); 
  SPI.transfer(add_or_val); // アドレスもしくはデータ送信。
  digitalWrite(PIN_SPI_SS, HIGH); 
}

/* ステッピングモータセットアップ */
void L6470_setup(){
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

//---------------------------------------------------------------------------//


