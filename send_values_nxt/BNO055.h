/* Copyright (c) 2016 Yajirushi(Cursor)
 *
 * Released under the MIT license
 * http://opensource.org/licenses/mit-license.php
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
 
/* 更新履歴
 * 2017/05/30
 *     四元数よりオイラー角を求める関数getEulerFromQを追加
 * 2017/06/25
 *     関数getEulerFromQの符号付き不具合を修正
 */

#ifndef BNO055_H
#define BNO055_H

#include <Arduino.h>
#include <Wire.h>

//UART通信に使用するSerialの名称
#define BNO055_UART_NAME	Serial
//I2C通信に使用するWireの名称
#define BNO055_I2C_NAME	Wire
//UART通信に使用するバッファの最大サイズ
#define BNO055_UART_BUF_MAXLEN		24		//[byte]
//UART通信オーバーラン対策用待ち時間
#define BNO055_UART_OVRCMWAIT_US	80		//[us]
//I2Cデフォルトスレーブアドレス
#define BNO055_I2C_DEFADDR			0x28





#define BNO055_RESULT_OK			0x00
#define BNO055_RESULT_FAIL			0xFF

#define BNO055_PAGE_ID				0x07

#define BNO055P0_CHIP_ID			0x00
#define BNO055P0_ACC_ID				0x01
#define BNO055P0_MAG_ID				0x02
#define BNO055P0_GYR_ID				0x03
#define BNO055P0_SW_REV_ID_LSB		0x04
#define BNO055P0_SW_REV_ID_MSB		0x05
#define BNO055P0_BL_REV_ID			0x06
#define BNO055P0_ACC_DATA_X_LSB		0x08
#define BNO055P0_ACC_DATA_X_MSB		0x09
#define BNO055P0_ACC_DATA_Y_LSB		0x0A
#define BNO055P0_ACC_DATA_Y_MSB		0x0B
#define BNO055P0_ACC_DATA_Z_LSB		0x0C
#define BNO055P0_ACC_DATA_Z_MSB		0x0D
#define BNO055P0_MAG_DATA_X_LSB		0x0E
#define BNO055P0_MAG_DATA_X_MSB		0x0F
#define BNO055P0_MAG_DATA_Y_LSB		0x10
#define BNO055P0_MAG_DATA_Y_MSB		0x11
#define BNO055P0_MAG_DATA_Z_LSB		0x12
#define BNO055P0_MAG_DATA_Z_MSB		0x13
#define BNO055P0_GYR_DATA_X_LSB		0x14
#define BNO055P0_GYR_DATA_X_MSB		0x15
#define BNO055P0_GYR_DATA_Y_LSB		0x16
#define BNO055P0_GYR_DATA_Y_MSB		0x17
#define BNO055P0_GYR_DATA_Z_LSB		0x18
#define BNO055P0_GYR_DATA_Z_MSB		0x19
#define BNO055P0_EUL_HEADING_LSB	0x1A
#define BNO055P0_EUL_HEADING_MSB	0x1B
#define BNO055P0_EUL_ROLL_LSB		0x1C
#define BNO055P0_EUL_ROLL_MSB		0x1D
#define BNO055P0_EUL_PITCH_LSB		0x1E
#define BNO055P0_EUL_PITCH_MSB		0x1F
#define BNO055P0_QUA_DATA_W_LSB		0x20
#define BNO055P0_QUA_DATA_W_MSB		0x21
#define BNO055P0_QUA_DATA_X_LSB		0x22
#define BNO055P0_QUA_DATA_X_MSB		0x23
#define BNO055P0_QUA_DATA_Y_LSB		0x24
#define BNO055P0_QUA_DATA_Y_MSB		0x25
#define BNO055P0_QUA_DATA_Z_LSB		0x26
#define BNO055P0_QUA_DATA_Z_MSB		0x27
#define BNO055P0_LIA_DATA_X_LSB		0x28
#define BNO055P0_LIA_DATA_X_MBS		0x29
#define BNO055P0_LIA_DATA_Y_LSB		0x2A
#define BNO055P0_LIA_DATA_Y_MBS		0x2B
#define BNO055P0_LIA_DATA_Z_LSB		0x2C
#define BNO055P0_LIA_DATA_Z_MBS		0x2D
#define BNO055P0_GRV_DATA_X_LSB		0x2E
#define BNO055P0_GRV_DATA_X_MSB		0x2F
#define BNO055P0_GRV_DATA_Y_LSB		0x30
#define BNO055P0_GRV_DATA_Y_MSB		0x31
#define BNO055P0_GRV_DATA_Z_LSB		0x32
#define BNO055P0_GRV_DATA_Z_MSB		0x33
#define BNO055P0_TEMP				0x34
#define BNO055P0_CALIB_STAT			0x35
#define BNO055P0_ST_RESULT			0x36
#define BNO055P0_INT_STA			0x37
#define BNO055P0_SYS_CLK_STATUS		0x38
#define BNO055P0_SYS_STATUS			0x39
#define BNO055P0_SYS_ERR			0x3A
#define BNO055P0_UNIT_SEL			0x3B
#define BNO055P0_OPR_MODE			0x3D
#define BNO055P0_PWR_MODE			0x3E
#define BNO055P0_SYS_TRIGGER		0x3F
#define BNO055P0_TEMP_SOURCE		0x40
#define BNO055P0_AXIS_MAP_CONFIG	0x41
#define BNO055P0_AXIS_MAP_SIGN		0x42
#define BNO055P0_ACC_OFFSET_X_LSB	0x55
#define BNO055P0_ACC_OFFSET_X_MSB	0x56
#define BNO055P0_ACC_OFFSET_Y_LSB	0x57
#define BNO055P0_ACC_OFFSET_Y_MSB	0x58
#define BNO055P0_ACC_OFFSET_Z_LSB	0x59
#define BNO055P0_ACC_OFFSET_Z_MSB	0x5A
#define BNO055P0_MAG_OFFSET_X_LSB	0x5B
#define BNO055P0_MAG_OFFSET_X_MSB	0x5C
#define BNO055P0_MAG_OFFSET_Y_LSB	0x5D
#define BNO055P0_MAG_OFFSET_Y_MSB	0x5E
#define BNO055P0_MAG_OFFSET_Z_LSB	0x5F
#define BNO055P0_MAG_OFFSET_Z_MSB	0x60
#define BNO055P0_GYR_OFFSET_X_LSB	0x61
#define BNO055P0_GYR_OFFSET_X_MSB	0x62
#define BNO055P0_GYR_OFFSET_Y_LSB	0x63
#define BNO055P0_GYR_OFFSET_Y_MSB	0x64
#define BNO055P0_GYR_OFFSET_Z_LSB	0x65
#define BNO055P0_GYR_OFFSET_Z_MSB	0x66
#define BNO055P0_ACC_RADIUS_LSB		0x67
#define BNO055P0_ACC_RADIUS_MSB		0x68
#define BNO055P0_MAG_RADIUS_LSB		0x69
#define BNO055P0_MAG_RADIUS_MSB		0x6A

#define BNO055P1_ACC_CONFIG			0x08
#define BNO055P1_MAG_CONFIG			0x09
#define BNO055P1_GYR_CONFIG_0		0x0A
#define BNO055P1_GYR_CONFIG_1		0x0B
#define BNO055P1_ACC_SLEEP_CONFIG	0x0C
#define BNO055P1_GYR_SLEEP_CONFIG	0x0D
#define BNO055P1_INT_MSK			0x0F
#define BNO055P1_INT_EN				0x10
#define BNO055P1_ACC_AM_THRES		0x11
#define BNO055P1_ACC_INT_SETTINGS	0x12
#define BNO055P1_ACC_HG_DURATION	0x13
#define BNO055P1_ACC_HG_THRES		0x14
#define BNO055P1_ACC_NM_THRES		0x15
#define BNO055P1_ACC_NM_SET			0x16
#define BNO055P1_GYR_INT_SETING		0x17
#define BNO055P1_GYR_HR_X_SET		0x18
#define BNO055P1_GYR_DUR_X			0x19
#define BNO055P1_GYR_HR_Y_SET		0x1A
#define BNO055P1_GYR_DUR_Y			0x1B
#define BNO055P1_GYR_HR_Z_SET		0x1C
#define BNO055P1_GYR_DUR_Z			0x1D
#define BNO055P1_GYR_AM_THRES		0x1E
#define BNO055P1_GYR_AM_SET			0x1F


class BNO055_CTRL{
public:
	BNO055_CTRL();
	virtual ~BNO055_CTRL();
protected:
	bool page1;
	unsigned char *ary;
	unsigned char lastError;
	unsigned char lastLength;
public:
	unsigned char getNowPage();
	unsigned char getLastError();
	unsigned char getLastLength();
	virtual void init();
	virtual unsigned char rr(bool isPage1, unsigned char regAddr);
	virtual unsigned char rrc(bool isPage1, unsigned char startRegAddr, unsigned char *receiveBytes, unsigned char length);
	virtual unsigned char wr(bool isPage1, unsigned char regAddr, unsigned char wBytes);
	virtual unsigned char wrc(bool isPage1, unsigned char startRegAddr, unsigned char *Bytes, unsigned char length);
};

class BNO055_UART_CTRL : public BNO055_CTRL{
public:
	BNO055_UART_CTRL();
	virtual ~BNO055_UART_CTRL();
	virtual void init();
	virtual unsigned char rr(bool isPage1, unsigned char regAddr);
	virtual unsigned char rrc(bool isPage1, unsigned char startRegAddr, unsigned char *receiveBytes, unsigned char length);
	virtual unsigned char wr(bool isPage1, unsigned char regAddr, unsigned char wBytes);
	virtual unsigned char wrc(bool isPage1, unsigned char startRegAddr, unsigned char *Bytes, unsigned char length);
};

class BNO055_I2C_CTRL : public BNO055_CTRL{
public:
	BNO055_I2C_CTRL(unsigned char addr, unsigned int freq);
	virtual ~BNO055_I2C_CTRL();
private:
	unsigned char i2c_addr;
	unsigned int i2c_freq;
public:
	virtual void init();
	virtual unsigned char rr(bool isPage1, unsigned char regAddr);
	virtual unsigned char rrc(bool isPage1, unsigned char startRegAddr, unsigned char *receiveBytes, unsigned char length);
	virtual unsigned char wr(bool isPage1, unsigned char regAddr, unsigned char wBytes);
	virtual unsigned char wrc(bool isPage1, unsigned char startRegAddr, unsigned char *Bytes, unsigned char length);
};

class BOARDC_BNO055{
public:
	BOARDC_BNO055();
	BOARDC_BNO055(unsigned char addr, unsigned int freq);
	~BOARDC_BNO055();

private:
	BNO055_CTRL *ctrl;
	float scaleACC;
	float scaleMAG;	//fixed
	float scaleGYRO;
	float scaleTEMP;
	float scaleEuler;
	float scaleLIA; //=scaleACC
	float scaleGV; //=scaleACC
	double scaleQuaternion; //fixed
	unsigned char axisRemap;
	unsigned char axisSign;
	bool clkExt;

public:
	unsigned char initialize(bool resetIface=true);
	unsigned char getIfaceLastError();
	unsigned char getIfaceLastLength();

	unsigned char customRead(bool isPage1, unsigned char regAddr);
	unsigned char customReadC(bool isPage1, unsigned char startRegAddr, unsigned char *receiveBytes, unsigned char length);
	unsigned char customWrite(bool isPage1, unsigned char regAddr, unsigned char wBytes);
	unsigned char customWriteC(bool isPage1, unsigned char startRegAddr, unsigned char *Bytes, unsigned char length);

	unsigned char getPage();
	void setPage(unsigned char pageNo);

	unsigned char getChipID();
	unsigned char getAccChipID();
	unsigned char getMagChipID();
	unsigned char getGyroChipID();

	unsigned short getRevision();
	unsigned char getBootRevision();

	float getAccScale();
	float getMagScale();
	float getGyroScale();
	float getTempScale();
	float getEulerScale();
	float getLinearScale();
	float getGVScale();
	double getQuaternionScale();

	void getAccDataAll(short &accX, short &accY, short &accZ);
	short getAccDataX();
	short getAccDataY();
	short getAccDataZ();

	void getMagDataAll(short &magX, short &magY, short &magZ);
	short getMagDataX();
	short getMagDataY();
	short getMagDataZ();

	void getGyroDataAll(short &gyroX, short &gyroY, short &gyroZ);
	short getGyroDataX();
	short getGyroDataY();
	short getGyroDataZ();

	void getEulerDataAll(short &E_heading, short &E_roll, short &E_pitch);
	short getEulerDataHeading();
	short getEulerDataYaw();
	short getEulerDataRoll();
	short getEulerDataPitch();

	void get9Axis(short *box);
	void get9AxisAndEUL(short *box);

	void getQuaternion(short &q1, short &q2, short &q3, short &q4);
	void getEulerFromQ(double &E_heading, double &E_roll, double &E_pitch);

	void getLinearAccDataAll(short &L_accX, short &L_accY, short &L_accZ);
	short getLinearAccDataX();
	short getLinearAccDataY();
	short getLinearAccDataZ();

	void getGVectorDataAll(short &gvX, short &gvY, short &gvZ);
	short getGVectorDataX();
	short getGVectorDataY();
	short getGVectorDataZ();

	unsigned char getTemperature();

	void getCalibStatusAll(unsigned char &sys, unsigned char &acc, unsigned char &mag, unsigned char &gyro);
	unsigned char getCalibStatusSys();
	unsigned char getCalibStatusAcc();
	unsigned char getCalibStatusMag();
	unsigned char getCalibStatusGyro();

	unsigned char getSelfTestResultAll();
	bool getSelfTestResultMCU();
	bool getSelfTestResultAcc();
	bool getSelfTestResultMag();
	bool getSelfTestResultGyro();

	unsigned char triggeredIntALL();
	bool triggeredACC_NM();
	bool triggeredACC_AM();
	bool triggeredACC_HIGH_G();
	bool triggeredGYR_HIGH_RATE();
	bool triggeredGYRO_AM();

	bool isSystemClockFixed();

	unsigned char getSystemStatus();
	unsigned char getSystemError();

	unsigned char getUNIT_SEL();
	unsigned char setUNIT_SEL(unsigned char selectValue);
	unsigned char setUNIT_AccUnit(bool isMeterPerSec2=true);
	unsigned char setUNIT_GyroUnit(bool isDps=true);
	unsigned char setUNIT_EulerUnit(bool isDegrees=true);
	unsigned char setUNIT_Temperature(bool isCelsius=true);
	unsigned char setUNIT_OrientationMode(bool ori_Android=true);

	unsigned char getOperationMode();
	unsigned char setOperationMode(unsigned char modeValue);
	unsigned char setOperation_CONFIG();
	unsigned char setOperation_ACCONRY();
	unsigned char setOperation_MAGONRY();
	unsigned char setOperation_GYROONRY();
	unsigned char setOperation_ACCMAG();
	unsigned char setOperation_ACCGYRO();
	unsigned char setOperation_MAGGYRO();
	unsigned char setOperation_AMG();
	unsigned char setOperation_Fusion_IMU();
	unsigned char setOperation_Fusion_COMPASS();
	unsigned char setOperation_Fusion_M4G();
	unsigned char setOperation_Fusion_NDOF_FMC_OFF();
	unsigned char setOperation_Fusion_NDOF();

	unsigned char getPowerMode();
	unsigned char setPowerMode(unsigned char modeValue);
	unsigned char setPowerMode_Normal();
	unsigned char setPowerMode_LowPower();
	unsigned char setPowerMode_Suspend();

	unsigned char setSysTrigger(unsigned char regVal);
	unsigned char setSys_ExternalCrystal(bool isExternal=true);
	unsigned char resetInterrupt();
	unsigned char soft_reset();
	unsigned char execSelfTest();

	unsigned char getTempSource();
	unsigned char setTempSource(bool Accelerometer=true);

	unsigned char getAxisMapConfig();
	unsigned char setAxisMapConfig(unsigned char val);
	unsigned char getAxisMapSign();
	unsigned char setAxisMapSign(unsigned char val);
	unsigned char setAxisRemap_topview_topleft();
	unsigned char setAxisRemap_topview_topright();
	unsigned char setAxisRemap_topview_bottomleft();
	unsigned char setAxisRemap_topview_bottomright();
	unsigned char setAxisRemap_bottomview_topleft();
	unsigned char setAxisRemap_bottomview_topright();
	unsigned char setAxisRemap_bottomview_bottomleft();
	unsigned char setAxisRemap_bottomview_bottomright();
	unsigned char getAxisRemap_type();

	void getAccOffsetAll(float &offsetX, float &offsetY, float &offsetZ);
	float getAccOffsetX();
	float getAccOffsetY();
	float getAccOffsetZ();
	unsigned char setAccOffsetAll(float offsetX, float offsetY, float offsetZ);
	unsigned char setAccOffsetX(float offset);
	unsigned char setAccOffsetY(float offset);
	unsigned char setAccOffsetZ(float offset);

	void getMagOffsetAll(float &offsetX, float &offsetY, float &offsetZ);
	float getMagOffsetX();
	float getMagOffsetY();
	float getMagOffsetZ();
	unsigned char setMagOffsetAll(float offsetX, float offsetY, float offsetZ);
	unsigned char setMagOffsetX(float offset);
	unsigned char setMagOffsetY(float offset);
	unsigned char setMagOffsetZ(float offset);

	void getGyroOffsetAll(float &offsetX, float &offsetY, float &offsetZ);
	float getGyroOffsetX();
	float getGyroOffsetY();
	float getGyroOffsetZ();
	unsigned char setGyroOffsetAll(float offsetX, float offsetY, float offsetZ);
	unsigned char setGyroOffsetX(float offset);
	unsigned char setGyroOffsetY(float offset);
	unsigned char setGyroOffsetZ(float offset);

	unsigned short getAccRadius();
	unsigned char setAccRadius(short LSB);

	unsigned short getMagRadius();
	unsigned char setMagRadius(short LSB);

	unsigned char getAccConfig();
	unsigned char setAccConfig(unsigned char regVal);
	unsigned char setAccConfig(unsigned char gRange, unsigned char bandWidth, unsigned char powMode);
	unsigned char setAccRange(unsigned char G);

	unsigned char getMagConfig();
	unsigned char setMagConfig(unsigned char regVal);
	unsigned char setMagConfig(unsigned char rate, unsigned char oprMode, unsigned char powMode);

	unsigned char getGyroConfig_0();
	unsigned char setGyroConfig_0(unsigned char regVal);
	unsigned char setGyroConfig_0(unsigned char range, unsigned char bandWidth);
	unsigned char getGyroConfig_1();
	unsigned char setGyroConfig_1(unsigned char powMode);
	unsigned char setGyroRange(unsigned short dps);

	unsigned char getAccSleepConfig();
	unsigned char setAccSleepConfig(unsigned char regVal);
	unsigned char setAccSleepConfig(unsigned char duration, unsigned char mode);

	unsigned char getGyroSleepConfig();
	unsigned char setGyroSleepConfig(unsigned char regVal);
	unsigned char setGyroSleepConfig(unsigned char autoSleepDuration, unsigned char duration);

	unsigned char getInterruptMask();
	unsigned char setInterruptMask(unsigned char mask);

	unsigned char getInterruptEnable();
	unsigned char setInterruptEnable(unsigned char mask);

	float getAccAnyMotionThreashold(bool ismg=true);
	unsigned char setAccAnyMotionThreashold(bool ismg, float threashold);

	unsigned char getAccInterruptSettings();
	unsigned char setAccInterruptSettings(unsigned char settings);

	unsigned short getAccHighGduration();
	unsigned char setAccHighGduration(short ms);

	float getAccHighGThreashold(bool ismg=true);
	unsigned char setAccHighGThreashold(bool ismg, float threashold);

	float getAccNMThreashold(bool ismg=true);
	unsigned char setAccNMThreashold(bool ismg, float threashold);

	unsigned char getAccNMsetting();
	unsigned char setAccNMsetting(unsigned char setting);

	unsigned char getGyroInterruptSettings();
	unsigned char setGyroInterruptSettings(unsigned char settings);

	unsigned char getGyroHighRateXsetting();
	void getGyroHighRateXsetting_dps(float &hyst, float &thres);
	unsigned char setGyroHighRateXsetting(unsigned char setting);
	unsigned char setGyroHighRateXsetting_dps(float hystVal, float thresVal);
	float getGyroHighRateXduration();
	unsigned char setGyroHighRateXduration(float duration);

	unsigned char getGyroHighRateYsetting();
	void getGyroHighRateYsetting_dps(float &hyst, float &thres);
	unsigned char setGyroHighRateYsetting(unsigned char setting);
	unsigned char setGyroHighRateYsetting_dps(float hystVal, float thresVal);
	float getGyroHighRateYduration();
	unsigned char setGyroHighRateYduration(float duration);

	unsigned char getGyroHighRateZsetting();
	void getGyroHighRateZsetting_dps(float &hyst, float &thres);
	unsigned char setGyroHighRateZsetting(unsigned char setting);
	unsigned char setGyroHighRateZsetting_dps(float hystVal, float thresVal);
	float getGyroHighRateZduration();
	unsigned char setGyroHighRateZduration(float duration);

	float getGyroAnyMotionThreashold();
	unsigned char setGyroAnyMotionThreashold(float threashold);

	unsigned char getAccAnyMotionSetting();
	unsigned char setAccAnyMotionSetting(unsigned char setting);
};

#endif
