/*-----------------------------------------------------------
 *Timer.h
 *
 * プログラム内容:
 * 　-　タイマクラス用ライブラリ(ヘッダファイル)
 * 　-　V1.0 - 20210413
 *   - 柴田さんからいただいたコードのリファクタリング
 *   
 *----------------------------------------------------------*/
#ifndef __TIMER_H__
#define __TIMER_H__
#include "common.h"
#include "MsTimer2.h"

/*マクロ宣言.*/
//タイマー割込み周期　(1[ms]に設定)
//宣言の時に定数倍する
#define TIMER2_INTERVAL  ((uint32_t)2)

//低周期処理用のタイマーのマクロ(外部定義便宜用)
#define MSEC2C(MSEC) (MSEC/TIMER2_INTERVAL)      //ミリ秒
#define SEC2C(SEC)  (SEC*(1000/TIMER2_INTERVAL))//秒

#define C_10MSEC       (MSEC2C(10))
#define C_100MSEC      (MSEC2C(100))
#define C_1SEC         (SEC2C(1))
#define C_1MIN         (SEC2C(60))
#define C_1HOUR        (SEC2C(3600))

//システムクロック
//構造体の定義
typedef struct{
  volatile uint8_t cnt;   //タイマ割込み発生回数カウンタ
  uint8_t last_cnt;       //カウンタ参照時のカウント値
  uint32_t tick;          //低周期処理用タイマカウンタの増分
} systemTimer;

//外部変数宣言
extern systemTimer clocker;

//プロトタイプ宣言
void timerInterrupt(void); //時間割込み呼び出し関数
void timer_Init(void);     //時間割込みの初期化
void timer_Clr_Val(void);  //システムの変数をクリア
void timer_EI(void);       //時間割込みの有効化
void timer_DI(void);       //時間割込みの無効化
uint32_t timer_get();      //タイマの増分値取得関数

//Timer crass(オプションタイマー)
class Timer{
  public:
    starter(uint32_t);     //Timer開始(コンストラクタ:周期を定義)
      void restarter(void);//Timer再始動
      void restarter(uint32_t);//Timer再始動(時刻再設定)
      void stoper(void);   //Timer停止
      bool checker(void);  //規定時間が経過したかチェックする
      void repeater(void); //変数pastを再設定
  private:  
      uint32_t start;      //Timer開始時のカウント値
      uint32_t goal;       //低周期処理用の設定周期
      uint32_t past;       //経過時間記録用
      uint32_t val;
};

#endif