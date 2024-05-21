#include "Timer.h"

//システムクロック
//構造体の実体化
systemTimer clocker;

//時間割込み呼び出し関数
void timerInterrupt(void){
  clocker.cnt++;
}

//時間割込みの初期化
void timer_Init(void){
  //タイマ割込み関数の初期化
  timer_Clr_Val();
  //タイマ割込みインターバル及び関数の設定
  MsTimer2::set(TIMER2_INTERVAL, timerInterrupt);
}

//システムの変数をクリア
void timer_Clr_Val(void){
  //構造体の初期化
  memset(&clocker, 0 ,sizeof(clocker));
}

//時間割込みの有効化
void timer_EI(void){
  MsTimer2::start();
}

//時間割込みの無効化
void timer_DI(void){
  MsTimer2::stop();
}

//タイマの増分値取得関数
uint32_t timer_get(){
  uint8_t old;

  //前回取得したカウント値をoldに格納する
  old = clocker.last_cnt;
  //現在のカウントをlast_cntに格納
  clocker.last_cnt = clocker.cnt;
  //上記の差分をとり、tickに格納(Uint32_t型にキャストすること)
  //uint8_t同士の引き算だが、符号あり変数となるため0xFF(=0x0000FF)でAND
  clocker.tick += (uint32_t)(clocker.last_cnt-old) & 0xFF;

  return (clocker.tick);
}



//Timer crass
//Timer開始(コンストラクタ:周期を定義)
Timer::starter(uint32_t a){
  this->val = a;
  this->start = timer_get();
  this->goal  = val;
  this->past  = 0;
  //動いていない状態=ゼロ動作
  //start値がゼロにならないように,ゼロであれば1を足す処理
  if(this->start == 0){
    this->start++;
  }
}

//Timer再始動
void Timer::restarter(void){
 //this->start = timer_get();
  this->start = timer_get();
  this->goal  = val;
  this->past  = 0;
  //動いていない状態=ゼロ動作
  //start値がゼロにならないように,ゼロであれば1を足す処理
  if(this->start == 0){
    this->start++;
  }
}

//Timer再始動
void Timer::restarter(uint32_t a){
  this->val = a;
  this->start = timer_get();
  this->goal  = val;
  this->past  = 0;
  //動いていない状態=ゼロ動作
  //start値がゼロにならないように,ゼロであれば1を足す処理
  if(this->start == 0){
    this->start++;
  }
}

//Timer停止
void Timer::stoper(void){
  this->start = 0;
  this->goal  = 0;
  this->past  = 0;
}

//規定時間が経過しかかチェックする
bool Timer::checker(void){
  uint32_t tick;
  //タイマが動作中かチェックする
  if(this->start==0) return 0;//動作していない場合ゼロを返す
  tick = timer_get();
  //経過時間をチェックしてpastに保存
  this->past = tick - this->start;
  return (this->past>=this->goal);
}

//一定周期処理タイマを再実行する関数
void Timer::repeater(void){
  this->start += this->goal;
}