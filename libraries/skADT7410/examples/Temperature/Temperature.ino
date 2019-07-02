// 温度センサー(ADT7410)のテスト
#include <Wire.h>
#include "skADT7410.h"


#define SENSOR_ADRS     0x48       // デバイスのI2Cアドレス

skADT7410  Temp(SENSOR_ADRS) ;     // 温度センサーライブラリの生成を行う

void setup()
{
     int ans ;

     // シリアルモニターの設定
     Serial.begin(9600) ;
     // Ｉ２Ｃの初期化
     Wire.begin() ;                     // マスターとする
     // 温度センサーの初期化を行う(16bitの解像度 動作モードはシャットダウン)
     ans = Temp.Begin() ;
     if (ans == 0) Serial.println("Initialization normal") ;
     else {
          Serial.print("Initialization abnormal ans=") ;
          Serial.println(ans) ;
     }
     // 動作モードを"連続測定モード"にする
     Temp.ActionMode(ADT_MODE_CONTINUE) ;

     delay(3000) ;  // 3秒後に開始
}
void loop()
{
     int   ans  ;
     float temp ;

     // 動作モードを"ワンショット測定モード"にする
     //Temp.ActionMode(ADT_MODE_ONESHOT) ;
     //delay(240) ;
     // 温度データ(摂氏温度)をセンサーから読み出す
     ans = Temp.Read(&temp) ;
     if (ans == 0) {
          Serial.print(temp,4) ;
          Serial.write(0xDF) ; // ℃
          Serial.println("C") ;
     } else Serial.println("NG") ;

     delay(1000) ;  // １秒後に繰り返す
}
