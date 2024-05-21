#ifndef __BME280_H__
#define __BME280_H__

#include <Wire.h>


//アドレス指定
#define BME280_ADDR 0x76
#define CONFIG 0xF5
#define CTRL_MEAS 0xF4
#define CTRL_HUM 0xF2

//--------------------------基本関数(外部より引用される)--------------------------
void BME280_Setup(void);//Setup関数に記述する
void BME280_SetConhig(void);//測定コンフィグの設定
void BME280_Senceing(void);//測定
float BME280_GetTmp(void);//データ取得(Tmp:温度)
float BME280_GetHum(void);//データ取得(Hum:湿度)
float BME280_GetPre(void);//データ取得(Pre:気圧)

//--------------------------補正関数(外部より引用されない)--------------------------
int32_t BME280_compensate_T_int32(int32_t);//温度補正関数
uint32_t bme280_compensate_H_int32(int32_t);//湿度補正関数
uint32_t BME280_compensate_P_int32(int32_t);//気圧補正関数

#endif
