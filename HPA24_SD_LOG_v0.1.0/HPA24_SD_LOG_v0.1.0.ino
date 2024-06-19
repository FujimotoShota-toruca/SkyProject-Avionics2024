/*********************************************************************************************
2023年度SkyProject　SD-Card loging code を流用
作成日:2023年7月10日
2023年06月02日
流用:2024/06/04
作成者:藤本翔太
2023/06/02-SDカードデータ書き込み
2023/07/10-チェックサム付与(藤本翔太to渡邉龍一郎)
***********************************************************************************************/
//ライブラリのインクルード
#include <SPI.h>
#include <SD.h>

//SDカードモジュールのチップセレクト
int SD_CS = 10;

//SDカードの状態を格納する変数
File logfile;

//SDカード内部に保存するファイルのパス及びファイル名
String logfilepath = "/sample.txt";

//チェックサムの可否
bool check = false;
String SD_write;

void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);
  Serial.setTimeout(15);

  //ファイルが開けない状態であるときの例外処理
  if(!SD.begin(SD_CS)){
    while(1);
  }  
  delay(100);
}

void loop() {
  // put your main code here, to run repeatedly:

  //ログデータ保存処理
  if(Serial.available()>0){
    String s;
    s = Serial.readString();
    
    if(check){ //チェックサムが有効である場合
      int commaCount = 0;
      int nonNumericCount = 0;
      //コンマの個数と非数の個数のカウント
      for(int i=0 ; i<s.length() ; i++){
        if(s.charAt(i) == ',') commaCount++;
        if(!('0' <= s.charAt(i) &&  s.charAt(i) <= '9')) nonNumericCount++;
      }
      //データフォーマット形成(チェックサム付与)
      SD_write = String(commaCount);
      SD_write.concat(",");
      SD_write.concat(String(nonNumericCount));
      SD_write.concat(",");
      SD_write.concat((s));
    }

    //ログファイル保存
    logfile = SD.open(logfilepath, FILE_WRITE);
    if( logfile ){
      if( check) logfile.print(SD_write);
      if(!check) logfile.print(s);
      logfile.close();
    }

  }
}
