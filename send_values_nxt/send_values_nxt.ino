/* 海洋ロボット　チームA
 * 使用デバイス　Arduino MEGA 2560
 */

/* 基本仕様
 * ・シリアル通信で各センサの値を送信
 * ・ディクショナリ型の書式に統一
 * ・実行、停止等はシリアル通信で制御可能
 * ・使用センサは以下
 *    フォトリフレクタ：4 
 *    温度センサIC：5
 *    9軸方位センサ：1 (roll,pitch,yaw)
 *    
 * ・デバックモードにするとシリアル通信で制御処理内容の文字が出力される
 * ・待機時間は変更可能（上限、下限の設定あり）
 *    
 * 実装予定
 * ・電流計の実装
 * ・フォトリフレクタの実装
 * ・状態や変数ををLED出力する
 * ・割り込み時間間隔による速度表記
 * ・圧力センサの初期値設定
 */
 
 /* 接続ポート
 * A0:2C
 * A1:3C3S
 * A2:3C2S
 * A3:3C1S
 * A7:圧力センサ
 * A11サーミスタ
 * A12サーミスタ
 * A13サーミスタ
 * A14サーミスタ
 * 
 * D0(RX):シリアル通信
 * D1(TX):シリアル通信
 * D2(init0):フォトリフレクタ割り込み0
 * D3(init1):フォトリフレクタ割り込み1
 * D6:マグネットスイッチ
 * D7:汎用ポート
 * D8(pwm):プロポ接続時使用
 * D9(pwm):プロポ接続時使用
 * D10(pwm):プロポ接続時使用
 * D11(pwm):プロポ接続時使用
 * D12(pwm):サーボroll
 * D13(pwm):サーボpitch
 * D18(init5):フォトリフレクタ割り込み3
 * D19(init4):フォトリフレクタ割り込み2
 * D20(SDA):9軸センサ
 * D21(SCL):9軸センサ
 * D50(pcint3)流量計0（割り込み）
 * D51(pcint)流量計1（割り込み）
 * D52(pcint)流量計2（割り込み）
 * D53(pcint)流量計3（割り込み）
 * 
 */
 
#include <Wire.h>
#include <skADT7410.h>
#include <KellerLD.h>
#include <MS5837.h>
#include <avr/io.h>
#include <avr/interrupt.h>  //割り込みを使用するため

#include "BNO055.h"

//#define SW_INT_PCICR  ((1<<PCIE0)|(1<<PCIE1)|(1<<PCIE2))
#define SW_INT_PCICR  (1<<PCIE0)
//#define SW_INT_PCMSK0 ((1<<PCINT7) |(1<<PCINT6) |(1<<PCINT5) |(1<<PCINT4) |(1<<PCINT3) |(1<<PCINT2) |(1<<PCINT1) |(1<<PCINT0))
#define SW_INT_PCMSK0 ((1<<PCINT3) |(1<<PCINT2) |(1<<PCINT1) |(1<<PCINT0))
//#define SW_INT_PCMSK1 ((1<<PCINT15)|(1<<PCINT14)|(1<<PCINT13)|(1<<PCINT12)|(1<<PCINT11)|(1<<PCINT10)|(1<<PCINT9) |(1<<PCINT8))
//#define SW_INT_PCMSK2 ((1<<PCINT23)|(1<<PCINT22)|(1<<PCINT21)|(1<<PCINT20)|(1<<PCINT19)|(1<<PCINT18)|(1<<PCINT17)|(1<<PCINT16))



#define ROT_NUM 4 //Rotary encoder of numbers ロータリーエンコーダの個数
#define PWM_NUM 4
#define CUR_NUM 3
#define THR_NUM 4
#define FLW_NUM 4
#define C2R1   1200
#define C2R2   1000
#define C3S3R1 5100
#define C3S3R2 3000
#define C3S2R1 1200
#define C3S2R2 1200

BOARDC_BNO055 sensor1(BNO055_I2C_DEFADDR, 400000); //I2Cアドレス0x28, スピード200KHz
MS5837 sensor2;

skADT7410 Temp(0x4A);

//制御動作用変数   ※目的に合わせて値を変更
bool debug = 0;   //1だとデバッグモード if(debug)Serial.print
int run_flg = 1;  //データ送信を　0:stop(停止)  1:run(実行)  2:run d9(方位センサの値だけ送信)
unsigned int delay_time = 100;  //無限ループ待機時間の初期値(ms)
 
bool yaw_zero = 1;  //起動時の方向yawを0にする機能　0:無効、1:有効
bool depth_zero = 0;//起動時の方向depthを0にする機能　0:無効、1:有効
int pwm_flg = 0;    //0:停止   1:実行
int rot_flg = 1;    //0:停止   1:実行
int lipo_flg = 1;   //0:停止   1:実行
String state = "normal";

//測定値代入変数   
unsigned int rot[ROT_NUM] = {};
int rpm[ROT_NUM] = {};  //debugモードのみ
int rcv[PWM_NUM] = {};
int duty[PWM_NUM] = {};
int flw[FLW_NUM] = {};
double cur[CUR_NUM] = {};
double thr[THR_NUM] = {};
float lipo[4] = {};
double ez,ey,ex;  //方位センサの値を受け取る
double compass;
bool mgs = 1;
int pres;
float depth;
float tempic;
float tempwt;

//計算処理用変数   ※値の変更は要確認
float voltaged;
float rtLsit[100];
// 抵抗分圧用の10kΩ抵抗
int R1 = 10000;
int R2 = 200;  //圧力センサ用の抵抗
double ez_zero = 0.0; //方位センサの初期値
int pres_zero = 0.0; //圧力センサの初期値
int btR1[] = {C2R1, C3S3R1, C3S2R1};
int btR2[] = {C2R2, C3S3R2, C3S2R2};
float cur_calc[] ={2.5,6.0,2.5};
int rot_cnt[ROT_NUM] = {};

//仮領域用変数  ※値の変更は要確認
String label = "";  //受信文字配列 
char buf[33]; //受信文字の仮容量
char s[16];   //文字列変換処理用変数
unsigned long lasttime[ROT_NUM] = {};
int oldpb = 0xFF;
int newpb = 0xFF;

void setup() {
  
  Serial.begin(115200);
  if (debug)Serial.println("Set up now.");

  Wire.begin();
  //Temp.Begin();
  //Temp.ActionMode(ADT_MODE_CONTINUE);
  delay(240);

  // 0-100℃までの抵抗値/温度リストを生成
  for (int i = 0; i < 100; i++) {
    // 温度から抵抗値を算出する
    rtLsit[i] = NTCThermistor_res(3435, 10000, 25, i);
  }

  
  if (debug)Serial.print(" .");

  for(int i=0; i < FLW_NUM; i++){
    pinMode(rcv[i],INPUT);
  }

  pinMode(13,OUTPUT);
  pinMode(6,INPUT);

   //入力PWM信号の設定
  for(int i=0; i < PWM_NUM; i++){ //入力PWM信号の設定
    rcv[i] = 8 + i;
    if(rcv[i] <= 13 ){
      pinMode(rcv[i],INPUT);
    }
    else{
       if (debug)Serial.print("\nINPUT_PWM_PORT_ERROR");
       while (1);
    }
  }
  if (debug)Serial.println("PWM [OK]");


  //割り込みピンの設定
  attachInterrupt(0, in_rot0, FALLING);//D2
  attachInterrupt(1, in_rot1, FALLING);//D3
  attachInterrupt(4, in_rot2, FALLING);//D19
  attachInterrupt(5, in_rot3, FALLING);//D18
  if (debug)Serial.println("INT [OK]");

  //sensor1.setAxisRemap_topview_bottomright();//1pinが表側左下
  unsigned char ret = sensor1.initialize(true);
  
  //sensor1.setAxisRemap_topview_topleft();
  if (debug)Serial.println("BNO055 [OK] ");
  if(ret != 0)state = "d9_error";
  
  delay(1000);  //コンパスの動作が安定するのを待つ
 
  if (debug)Serial.println("9DSET [OK]");
  
  //方位センサ、圧力センサの初期設定
  sensor2.init();
  sensor2.setFluidDensity(1029); // kg/m^3 (997 freshwater, 1029 for seawater)
  
  //sensor1.setMagOffsetZ(5.0);
  acquireValue();
  
  //pres_zero = pres;
  ez_zero = ez;
  if (debug)Serial.println("ZEROSET [OK]");
  

  PCICR  |= SW_INT_PCICR;
  PCMSK0 |= SW_INT_PCMSK0;
  //PCMSK1 |= SW_INT_PCMSK1;
  //PCMSK2 |= SW_INT_PCMSK2;
  sei();

  if (debug)Serial.print("\nStart operation!\n");
  if (debug)Serial.println("");


}//setup end

void loop() {

  int index = 0;
  bool hasData = false;

  //入力された文字列の取得を試みる
  while (Serial.available() > 0) {
    hasData = true;
    buf[index] = Serial.read();
    index++;
    //バッファ以上の場合は中断
    if (index >= 32) {
      break;
    }
  }

  if (hasData == true) {
    hasData = false;

    label = buf;
    label.trim();
    if (debug)Serial.print(">>" + label);
    if (label.equals("run")) {
      run_flg = 1;
      if (debug)Serial.println("  ->Start transmission");
      digitalWrite(13,HIGH);
    }
    else if (label.equals("stop")) {
      run_flg = 0;
      if (debug)Serial.println("  ->Stop transmission");
      digitalWrite(13,LOW);
    }
    else if (label.equals("reboot")) {
      if (debug)Serial.print("  ->reboot \n");
      delay(10);
      software_reset();
    }
    else if (label.equals("debug")) {
      debug = 1;
      if (debug)Serial.println("  ->debug mode");    
    }
    else if (label.equals("debug off")) {
      Serial.println("");    
      debug = 0;
    }
    else if (label.equals("yaw_zero off")) {
      yaw_zero = 0;
      if (debug)Serial.println("  ->yaw_zero off");    
    }
    else if (label.equals("yaw_zero on")) {
      yaw_zero = 1;
      if (debug)Serial.println("  ->yaw_zero on");    
    }
    else if (label.equals("pwm off")) {
      pwm_flg = 0;
      if (debug)Serial.println("  ->pwm off");    
    }
    else if (label.equals("pwm on")) {
      pwm_flg = 1;
      if (debug)Serial.println("  ->pwm on");    
    }
    else if (label.indexOf("time ") == 0) {
      run_flg = 0;
      if (debug)Serial.println("  ->Stop transmission");
      String sary = label.substring(5);
      int slen = sary.length();
      if (debug)Serial.print("len:");
      if (debug)Serial.println(slen);
      if (slen > 4 || slen < 2) {
        if (debug)Serial.println("  ->over delay time");
      }
      else {
        int s[4] = {0, 0, 0, 0};
        byte var;
        for (int i = 0; i < slen ; i++) {
          var = sary[slen - i - 1];
          s[i] = var - 0x30;
        }
        delay_time = s[0] + (s[1] * 10) + (s[2] * 100) + (s[3] * 1000);
        if (debug)Serial.print("  ->delay time ");
        if (debug)Serial.println(delay_time);
      }
    }
    else if (label.equals("reset rot")) {
      for (int i = 0; i < ROT_NUM; i++ ){
        rot[i] = 0;
      }
    }
    else if (label.equals("reset yaw")) {
      ez_zero = 0.0;
      sensor1.getEulerFromQ(ez,ex,ey);
      ez_zero = ez;
    } 
    else if (label.equals("remove error")) {
      if(sensor1.initialize(true) == 0){
        state = "normal";
        if (debug)Serial.println("  ->state is normal \n");
      }
      delay(10);
    }
    else if (label.equals("help")) {
      if (debug)Serial.println();
      Serial.print("{");
      Serial.print("'run':'シリアル通信を開始する。', ");
      Serial.print("  'stop':'シリアル通信を停止する。', ");
      Serial.print("  'reboot':'Arduinoを再起動する。', ");
      Serial.print("  'reset xxx':'curまたはrotの値をリセットする。', ");
      Serial.print("  'debug':'デバッグモードに移行offをつけると通常モードに戻る。', ");
      Serial.print("  'time XXXX':'無限ループ時の待機時間をXXXXミリ秒にする。', ");
      Serial.print("  'yaw_zero off':'yawの初期リセット値を無効化', ");
      Serial.print("  'remove error':'状態を確認し問題なかったらstateをnormalにする', ");
      //Serial.print("'':'', ");
      Serial.println("}");
     
    }
    else {      //設定されていない無効な命令受信時
      if (debug)Serial.println("  ->unavailable");
    }
    memset( buf , '\0' , 33 );  //バッファをリセット
  }

  if(run_flg == 1) { //測定開始　データ出力 
    
    acquireValue();//データの取得

    //送信データの作成
    String data = "";
    if (debug)data += label;
    data += "{";
    //if (debug)
    {
      data += "'time':";
      //data += String(millis()/1000);
      data += dtostrf((double)millis()/1000, 6, 2, s);
      data += ", ";
    }
    insertCUR(&data);
    if(rot_flg == 1 || debug)insertROT(&data);
    if(lipo_flg == 1 || debug)insertLIPO(&data);
    insertD9(&data);
    data += "'compass':";
    data += dtostrf(compass, 6, 2, s);
    data += ", ";
    if(pwm_flg == 1)insertDUTY(&data);
    insertDEPTH(&data);
    insertTHR(&data);
    insertFLW(&data);

    data += "'mgs':";
    data += String(mgs);
    data += ", ";

  
 
    data += "'state':'";
    data += state;
    data += "'";
    data += "}";

    Serial.println(data);
  }
  delay(80);
  
}//loop end


void software_reset() {
  asm volatile ("  jmp 0");  
} 


void in_rot0() {
 // detachInterrupt(0);
// noInterrupts();
  rot_cnt[0]++;
  if(rot_cnt[0] >= 2){
    rot[0]++;
    rot_cnt[0] = 0;
//    interrupts();
//     attachInterrupt(0, in_rot0, FALLING);//D2
    if(debug){
      unsigned long nowtime = millis();
      rpm[0] = 60000 / (nowtime - lasttime[0]);  
      lasttime[0] = millis();
    }
  }
}
void in_rot1() {
  rot_cnt[1]++;
  if(rot_cnt[1] >= 2){
    rot[1]++;
    rot_cnt[1] = 0;
    if(debug){
      unsigned long nowtime = millis();
      rpm[1] = 60000 / (nowtime - lasttime[1]);  
      lasttime[1] = millis();
    }
  }
}
void in_rot2() {
  rot[2]++;
   if(debug){
    unsigned long nowtime = millis();
    rpm[2] = 60000 / (nowtime - lasttime[2]);  
    lasttime[2] = millis();
  }
}
void in_rot3() {
  rot[3]++;
   if(debug){
    unsigned long nowtime = millis();
    rpm[3] = 60000 / (nowtime - lasttime[3]);  
    lasttime[3] = millis();
  }
}

void insertROT(String *data){
  for (int i = 0; i < ROT_NUM ; i++) {  //ロータリーエンコーダのデータ挿入
      *data += "'rot";
      *data += String(i);
      *data += "':";
      *data += String(rot[i]);
      *data += ", ";
      if (debug){
        *data += "'rpm:";
        *data += String(i);
        *data += "':";
        *data += String(rpm[i]);
        *data += ", ";
      }
    }
}

void insertFLW(String *data){
  for (int i = 0; i < FLW_NUM ; i++) {  //ロータリーエンコーダのデータ挿入
      *data += "'flw";
      *data += String(i);
      *data += "':";
      *data += String(flw[i]);
      *data += ", ";
    }
}

void insertCUR(String *data){
  for (int i = 0; i < CUR_NUM ; i++) {  //電流値のデータ挿入
      *data += "'cur";
      *data += String(i);
      *data += "':";
       *data += dtostrf(cur[i], 6, 2, s);
      *data += ", ";
    }
}


void insertLIPO(String *data){
      *data += "'lipoC2':";
      *data += dtostrf(lipo[0], 6, 2, s);
      *data += ", ";
      *data += "'lipoC3S3':";
      *data += dtostrf(lipo[1], 6, 2, s);
      *data += ", ";
      if(debug)
      {
        *data += "'lipoC3S2':";
        *data += dtostrf(lipo[2], 6, 2, s);
        *data += ", ";
         *data += "'lipoC3S1':";
        *data += dtostrf(lipo[3], 6, 2, s);
        *data += ", ";
      }
}

void insertD9(String *data){
 
  *data += "'roll':";
  *data += dtostrf(ex, 6, 2, s);
  *data += ", ";
  *data += "'pitch':";
  *data += dtostrf(ey, 6, 2, s);
  *data += ", ";
  
  if (debug){
    *data += "'yaw_zero':";
    *data += dtostrf(ez_zero, 6, 2, s);
    *data += ", ";
  }
  *data += "'yaw':";
  *data += dtostrf(ez, 6, 2, s);
  *data += ", ";

}

void insertDUTY(String *data){
  for (int num = 0; num < PWM_NUM ; num++) {  //流速計のデータ挿入
      *data += "'duty";
      *data += String(num);
      *data += "':";
      *data += String(duty[num]);
      *data += ", ";
    }
}


void insertDEPTH(String *data){     //圧力センサのデータ挿入（水深）
  if(debug)
  {
    *data += "'pres':";
    *data += String(pres);
    *data += ", ";
  }
  if(debug)
  {
    *data += "'pres_zero':";
    *data += String(pres);
    *data += ", ";
  }
  if(debug)
  {
    *data += "'volt':";
    *data += dtostrf(voltaged, 4, 4, s);
    *data += ", ";
  }
  *data += "'depth':";
  *data += dtostrf(depth, 4, 3, s);
//  *data += sensor2.depth(); 
  *data += ", ";

    *data += "'watertemp':";
    *data += dtostrf(tempwt, 6, 2, s);
    *data += ", ";

  
}

void insertTHR(String *data){
    for (int n = 0; n < THR_NUM ; n++) {  //サーミスタのデータ挿入
      *data += "'thr";
      *data += String(n);
      *data += "':";
      *data += dtostrf(thr[n], 6, 2, s);
      *data += ", ";
    }
}
void acquireValue(){

  sensor1.getEulerFromQ(ez,ey,ex);//9dコンパスから値を取得
  compass = ez;
  if(yaw_zero == 1){
    //compass = ez;
    ez = ez -ez_zero;
    if(ez > 180){
      ez = ez - 360;
    }
    else if( ez < -180){
      ez = 360 + ez ;
    }
  }
  sensor2.read();
  depth = sensor2.depth(); 
  tempwt =sensor2.temperature();
/*
  pres = analogRead(7);
  if(depth_zero == 1)pres = pres - pres_zero;
  voltaged = ( pres / 1024.0) * 5.0;   //圧力センサから値を取得
  depth = ((voltaged - (0.004*R2))/((0.016*R2)/50));
  //depth *= 10.1974; //単位をセンチメートルに変換
 */
  

 
  for (int n = 0; n < 3 ; n++) {// バッテリーのアナログ入力の値を電圧(V)に変換
    float voltage = (analogRead(n) / 1024.0) * 5.0;
    lipo[n] = (voltage / btR2[n]) * (float)(btR1[n] + btR2[n]);
  }
  lipo[3] =  (analogRead(3) / 1024.0) * 5.0;

  for (int n = 0; n < 3 ; n++) {//電流センサのアナログ入力の値を電圧(V)に変換
    float voltage = (analogRead(n+8) / 1024.0) * 5.0;
    cur[n] = voltage * cur_calc[n];
  }


  for (int n = 0; n < THR_NUM ; n++) {//サーミスタのアナログ入力の値を電圧(V)に変換
    float voltage = (analogRead(n+11) / 1024.0) * 5.0;
    float resistance = voltage / (5.0 - voltage) * R1;
    for (int i = 0; i < 100; i++) {
      if (resistance == rtLsit[i]) {
          thr[n] = i;
          break;
      }
      if (resistance > rtLsit[i]) {
          thr[n] = i - 1;
          break;
      }
    }
  }
  //Temp.Read(&tempic) ;
  
  if(pwm_flg == 1){
    for (int n = 0; n < PWM_NUM ; n++) {  //PWMの測定
      duty[n] = pulseIn(rcv[n],HIGH);
    }
  }
  mgs = digitalRead(6);
}
float NTCThermistor_res(int B, int R0 , float T1, float T2) {
  const float e = 2.7182818284; // ネイピア数
  float exponent = B * ((1.0 / (T2 + 273.15)) - (1.0 / (T1 + 273.15)));
  return (R0 * pow(e, exponent));
}
ISR(PCINT0_vect){
  newpb = PINB;
  sei();
  //PCINT1 53
  if((newpb&0x01) > (oldpb & 0x01) ){
    flw[0]++;
  }
  
  //PCINT1 52
  if((newpb&0x02) > (oldpb & 0x02)){
    flw[1]++;
  }
  
  //PCINT2 51
  if((newpb&0x04) > (oldpb & 0x04)){
    flw[2]++;
  }
  
  //PCINT3 50
  if((newpb&0x08) > (oldpb & 0x08)){
    flw[3]++;
  }
  
  oldpb = newpb;
}
