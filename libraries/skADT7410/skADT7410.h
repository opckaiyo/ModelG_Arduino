/*******************************************************************************
*  skADT7410.h - 温度センサADT7410用関数ライブラリのインクルードファイル       *
*                                                                              *
* ============================================================================ *
* ============================================================================ *
*   VERSION  DATE        BY             CHANGE/COMMENT                         *
* ---------------------------------------------------------------------------- *
*   1.00     2015-06-05  きむ茶工房     Create                                 *
* ============================================================================ *
*   Arduino IDE 1.7.3    (Ardino Duemilanove 328/UNO)(Arduino Zero Pro)        *
*******************************************************************************/
#ifndef SKADT7410_h
#define SKADT7410_h

#include "arduino.h"

// デバイスのレジスタアドレス
#define TEMP_DATA_ADRS    0x00         // 読み出すデータの先頭レジスタアドレス
#define STAT_REG_ADRS     0x02         // ステータスレジスタアドレス
#define CONF_REG_ADRS     0x03         // コンフィギュレーションの設定レジスタアドレス

// コンフィギュレーションの設定
// (INT/CT機能は利用しないのでデフォルトのままとする)
#define CONF_REG_DATA     0b11100000   // 分解能16Bit シャットダウンモード

// モードの設定
#define ADT_MODE_SHUTDOWN 0x01100000   // シャットダウンモード
#define ADT_MODE_ONESHOT  0x00100000   // 単発測定モード
#define ADT_MODE_CONTINUE 0x00000000   // 連続測定モード
#define ADT_MODE_1SPS     0x01000000   // １秒間隔測定モード


/*******************************************************************************
*	クラスの定義                                                              *
*******************************************************************************/
class skADT7410
{
  private:
    uint8_t Sensor_adrs ;

  public:
            skADT7410(uint8_t address) ;
    uint8_t Begin(void) ;
    uint8_t ActionMode(uint8_t mode) ;
    uint8_t Receive(char reg_adrs,unsigned char *data,char kosu) ;
    uint8_t Send(char reg_adrs,unsigned char *data,char kosu) ;
    uint8_t Read(float *temp) ;
} ;

#endif
