/* SkyProject電装loger mainOBCソースコード
 * 作成者：藤本翔太 2024.3.31 Ver 0.1.0
 * HPA24 mainOBCひな型
 * 方針:いったん早めに終わらせる
 * 保存周期のみに絞ってタイマーを動かす戦略にする
 * 昨年の思想を引き継いでコーディングする
 * 可読性/メンテナンス性を高めるためタイマーを多用する方法はやめる
 */

// パッケージインクルード群
#include "common.h"
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <utility/imumaths.h>
#include <SoftwareSerial.h>//ソフトウェアシリアルライブラリ
#include <TinyGPS++.h>//gnssライブラリ
#include <Wire.h>
#include "Timer.h"
#include "BME280.h"

// DataHandring
String data_memo[8];
// data_memo address mapping
#define TIME_DATA 0
#define RSOBC_DATA 1
#define GPS_DATA 2
#define BNO055_DATA 3
#define BME280_DATA 4
#define ALTITUDE_DATA 5
#define AIROBC_DATA 6
#define PILOT_INPUT_DATA 7

// pin uumber mapping
// uart mapping part
#define software_serial1_rx 10
#define software_serial1_tx 9
#define software_serial2_rx 68
#define software_serial2_tx 69

// CS mapping part
#define PILOT_INPUT_PIN 11
#define ALTITUDE_PIN 39

//class mame mapping
#define GPS_UART Serial
#define LOGER_UART Serial1
#define RSOBC_UART Serial2
#define AIROBC_UART Serial3

// クラスインスタンス生成群
// SoftwareSirialクラス インストラクタ(SoftwareSerialクラスのインスタンス生成)
SoftwareSerial pilot_input_uart(software_serial1_rx, software_serial1_tx);
SoftwareSerial altitude_uart(software_serial2_rx, software_serial2_tx);

// shibatimerクラス　インストラクタ(shibatimerクラスのインスタンス生成)
Timer uart_tx;
Timer subMC;

// TinygnssPulsクラス インストラクタ(TinygnssPulsクラスのインスタンス生成)
TinyGPSPlus gps;

// bno
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// debugモード
bool GPS_Debug = false;

// subマネジメント
int sub_management = 0;
int management = 0;

void setup() {
  // put your setup code here, to run once:

  // HW/SW uart setup
  GPS_UART.begin(9600); // gps serial
  LOGER_UART.begin(115200); // Output(SD&ble) serial
  RSOBC_UART.begin(115200); // Twe-1(回転計/RSOBC) serial
  AIROBC_UART.begin(115200); // Twe-2(ピトー管/AirOBC) serial
  pilot_input_uart.begin(115200); // 操舵入力
  altitude_uart.begin(115200); // 高度入力
  // HW/SW uart.readString()methodセットアップ用のタイムアウト時間設定
  GPS_UART.setTimeout(10); // gps serial
  LOGER_UART.setTimeout(10); // Output(SD&ble) serial
  RSOBC_UART.setTimeout(10); // Twe-1(回転計/RSOBC) serial
  AIROBC_UART.setTimeout(10); // Twe-2(ピトー管/AirOBC) serial
  pilot_input_uart.setTimeout(10); // 操舵入力
  altitude_uart.setTimeout(10); // 高度入力

  // GPSデータ取得周期を5Hzに変更
  //GPS_UART.write("$PMTK220,100*2C");

  //IIC setup
  Wire.begin();//I2Cを初期化
  BME280_Setup();//BME関連処理
  // There was a problem detecting the BNO055 ... check your connections 
  if(!bno.begin()){
    LOGER_UART.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  bno.setExtCrystalUse(true);

  // pinmode setup
  pinMode(PILOT_INPUT_PIN, OUTPUT);
  pinMode(ALTITUDE_PIN, OUTPUT);
  digitalWrite(PILOT_INPUT_PIN, HIGH);
  digitalWrite(ALTITUDE_PIN, HIGH);

  // Timer setup
  uart_tx.starter(C_1SEC*0.1);//10Hz
  subMC.starter(C_1SEC*0.1);
  timer_Init();
  timer_EI();
  delay(200);

  // uart initialdata send
}

void loop() {
  // put your main code here, to run repeatedly:
  // timer管理・実行
  timer_management();

  // uart rx管理
  uart_rx_management();

  // 能動実行管理
  active_management();

  // subマイコン管理
  sub_MC_management();
  //LOGER_UART.print(".");
}

// timer管理(今のところ1個だが形式を合わせるために作成)
void timer_management(){
  // ロガーにデータ送信
  if(uart_tx.checker() == true){
    uart_tx_task();
    uart_tx.repeater();
    }
}

void uart_tx_task(){
  //フォーマットの形成並びに実行
  String format = "app:,";
  static int num = 0;
  data_memo[TIME_DATA] = String(num);
  String data_handler[8] = {"num,",",RS,",",GPS,",",BNO,",",BME,",",ALT,",",AIR,","PIL"};
  for(int i=0 ; i< 8 ; i++){
    format.concat(data_handler[i]);
    format.concat(data_memo[i]);
  }
  LOGER_UART.println(format);
  num++; // 1step加算
}

// uart受信割り込みマネジメント
void uart_rx_management(){
  // GPSデータアップデート
  if(GPS_UART.available()>0) gps_data_update();
  // RSOBCデータアップデート
  if(RSOBC_UART.available()>0) rsobc_data_update();
  // AirOBCデータアップデート
  if(AIROBC_UART.available()>0) airobc_data_update();
}

// gpsデータアップデート
void gps_data_update(){
  while(GPS_UART.available()>0){
    char d = GPS_UART.read();
    gps.encode(d);
  }
  if(gps.location.isUpdated()){
    data_memo[GPS_DATA] = gps_data_input_string();
    // gps-debug
    if(GPS_Debug){
      LOGER_UART.println("------------------------------------------------------------------");
      LOGER_UART.println(data_memo[GPS_DATA]);
      LOGER_UART.println("------------------------------------------------------------------");
    } 
  }
}

// gps情報を手前のフォーマットに変換
String gps_data_input_string(){
  String GPS = "";
  GPS.concat(gps.date.year()); // Year (2000+) (u16)
  GPS.concat("/");
  GPS.concat(gps.date.month()); // Month (1-12) (u8)
  GPS.concat("/");
  GPS.concat(gps.date.day()); // Day (1-31) (u8)
  GPS.concat(",");
  GPS.concat(gps.time.hour()); // Hour (0-23) (u8)
  GPS.concat(":");
  GPS.concat(gps.time.minute()); // Minute (0-59) (u8)
  GPS.concat(":");
  GPS.concat(gps.time.second()); // Second (0-59) (u8)
  GPS.concat(",");
  GPS.concat(gps.location.rawLat().negative ? "-" : "+");
  GPS.concat(gps.location.rawLat().deg); // Raw latitude in whole degrees
  GPS.concat(".");
  GPS.concat(gps.location.rawLat().billionths);// ... and billionths (u16/u32)
  GPS.concat(",");
  GPS.concat(gps.location.rawLng().negative ? "-" : "+");
  GPS.concat(gps.location.rawLng().deg); // Raw longitude in whole degrees
  GPS.concat(".");
  GPS.concat(gps.location.rawLng().billionths);// ... and billionths (u16/u32)
  GPS.concat(",");
  GPS.concat(gps.speed.mps()); // Speed in meters per second (double)
  GPS.concat(",");
  GPS.concat(gps.course.value()); // Raw course in 100ths of a degree (i32)
  GPS.concat(",");
  GPS.concat(gps.course.deg()); // Course in degrees (double)
  GPS.concat(",");
  GPS.concat(gps.altitude.meters()); // Altitude in meters (double)
  GPS.concat(",");
  GPS.concat(gps.satellites.value()); // Number of satellites in use (u32)

  return GPS;
}

void rsobc_data_update(){
  data_memo[RSOBC_DATA] = RSOBC_UART.readString();
}

void airobc_data_update(){
  data_memo[AIROBC_DATA] = AIROBC_UART.readString();
}

// パルス生成
void pulse_generation(int pin_num){
  digitalWrite(pin_num, LOW);
  delay(10);
  digitalWrite(pin_num, HIGH);
}

void rsobc_activation(){
  RSOBC_UART.print("!");
}

void airobc_activation(){
  AIROBC_UART.print("!");
}

void bno055_data_update(){
  imu::Quaternion quat = bno.getQuat();
  data_memo[BNO055_DATA] = Quat2Euler(quat.w(), quat.x(), quat.y(), quat.z());
}

void bne280_data_update(){
  BME280_SetConhig();
  BME280_Senceing();
  data_memo[BME280_DATA] = String(BME280_GetTmp()) + "," + String(BME280_GetHum()) + "," + String(BME280_GetPre());
}

/*-----------------------------------------------------------------------------------------------------------*/
//Quaternion to roll-pitch-yaw Euler変換
/***********************************
 引数：double Quaternion(4-parameter)
 戻値：Euler angle(String)
 処理：クォータニオンからオイラー角に変換する

 //参考URL(データ取得):https://qiita.com/Ninagawa_Izumi/items/4f673a9b7e5162fdda7d
 //参考URL(回転表現変換):http://l52secondary.blog.fc2.com/blog-entry-50.html
 ***********************************/
 String Quat2Euler(double w, double x, double y, double z){
  //クォータニオン→オイラー角変換
  double ysqr = y * y;
  // roll (x-axis rotation)
  double t0 = +2.0 * (w * x + y * z);
  double t1 = +1.0 - 2.0 * (x * x + ysqr);
  double roll = atan2(t0, t1);
  // pitch (y-axis rotation)
  double t2 = +2.0 * (w * y - z * x);
  t2 = t2 > 1.0 ? 1.0 : t2;
  t2 = t2 < -1.0 ? -1.0 : t2;
  double pitch = asin(t2);
  // yaw (z-axis rotation)
  double t3 = +2.0 * (w * z + x * y);
  double t4 = +1.0 - 2.0 * (ysqr + z * z);  
  double yaw = atan2(t3, t4);

  return String(roll) + "," + String(pitch) + "," + String(yaw);
 }

 void active_management(){
  switch(management){
    case 0:
      rsobc_activation();
      management++;
      break;
    case 1:
      airobc_activation();
      management++;
      break;
    case 2:
      bno055_data_update();
      management++;
      break;
    case 3:
      bne280_data_update();
      management++;
      break;
    default:
      management = 0;
      break;
  }
}

void sub_MC_management(){ 
  if(subMC.checker()){
    subMC.repeater();
    //LOGER_UART.print(sub_management);
    switch(sub_management){
      case 0:
        pilot_input_data_MC_activaiton();
        sub_management++;
        break;
      case 2:
        pilot_input_data_update();
        sub_management++;
        break;
      case 1:
        altitude_data_MC_activaiton();
        sub_management++;
        break;
      case 3:
        altitude_data_update();
        sub_management++;
        break;
      default:
        sub_management = 0;
        break;
    }
  }
}

void pilot_input_data_MC_activaiton(){
  pilot_input_uart.listen();
  pulse_generation(PILOT_INPUT_PIN);
}

void pilot_input_data_update(){
  if(pilot_input_uart.available() > 0){
    data_memo[PILOT_INPUT_DATA] = pilot_input_uart.readString();
  }
}

void altitude_data_MC_activaiton(){
  altitude_uart.listen();
  pulse_generation(ALTITUDE_PIN);
}

void altitude_data_update(){
  if(altitude_uart.available() > 0){
    data_memo[ALTITUDE_DATA] = altitude_uart.readString();
  }
}