/*=====================================================
 
 PID制御論を実装したライントレーサー開発における
 メインマイコン(ATmega328P)用制御プログラム
 Date: 2015/07/13 ~ 受験終わりのプロトタイプ発注基板PLF用
 Designer: YUKI OGASAWAWRA
 
 対象マイコン：ATmega328P / Ext.16MHz
 ブートローダ書き込み済
 書き込み装置：Arduino as ISP
 
 <NEWEST LOG>
 吉村先生のおかげで途中でマイコンリセットがかかる問題が解決！！まじで神！感謝御礼です！！
 ツインモーターギアボックスにしたのでギア比が344:1から204:1になっているので速度を全て0.6倍したい！
 <MEMO>
 LCD(AQM0802A)はI2Cなので接続端子は自動的に
 SDA(アナログ4ピン)とSCL(アナログ5ピン)
 
 4.96[v]の時
 Ku=1.14, Pu=0.63[s]
 Kp = Ku*0.6 = 1.14*0.6 = 0.684
 Td = 0.125*Pu = 0.125*0.63 = 0.07875
 Kd = Kp*Td = 0.053865
 
 =====================================================*/

#include <Wire.h>  //I2C
#include <ST7032.h>  //AQM0802A用ヘッダファイル

//操作ボタンの割り当て
#define ENTER 7
#define SELECT 6
#define BACK 5
//スピーカーの割り当て
#define BEAT 80  // 音の長さを指定
#define PINNO 9  // 圧電スピーカを接続したピン番号

//PID処理周期(4msec)
#define DELTA_T 0.004

//モーターの定常値のパスル信号
#define Steady_Motor_Power 80

/*========================TA7291Pについて===========================
 １：GND（Arduino／GND端子）と共有
 　２：モータの端子へ接続
 　３：非接続
 　４：PWM端子（Arduino／アナログ出力端子）へ接続
 　５：信号用端子（Arduino／デジタル出力端子）へ接続
 　６：信号用端子（Arduino／デジタル出力端子）へ接続
 　７：5V電源（Arduino／５V端子）と共有
 　８：外部電源のプラス端子へ接続（乾電池など）
 　９：非接続
 １０：モータの端子へ接続
 
 ５番ピンが「LOW」、６番ピンが「LOW」の場合は静止
 ５番ピンが「HIGH」、６番ピンが「LOW」の場合は正転
 ５番ピンが「LOW」、６番ピンが「HIGH」の場合は逆転
 ==================================================================*/

//ライントレーサーの進行方向に対しての左右
//左モーター用デジタルﾋﾟﾝ割り当て
//ここミスって11,10,9にしたかったけどこんなんになった…
#define MotorL_pwm 11
#define MotorL_signal_5 10
#define MotorL_signal_6 12
//右モーター用デジタルﾋﾟﾝ割り当て2,3,4
#define MotorR_pwm 3
#define MotorR_signal_5 2
#define MotorR_signal_6 4

//DCモーターは回転方向で得意不得意があり、おなじパルスを入力していてもズレが発生するので、
//そのズレを抑える為のオフセット値です。
#define Motor_offset 0

//アナログピン割り当て(A4,A5はLCDに接続)
const int Adjust = A0;  //フォトリフレクタ調整用の可変抵抗
const int photo_pid = A2;  //PIDモード用のフォトリフレクタ値
const int photo_1 = A3;  //ON-OFF用のフォトリフレクタ値１と２
const int photo_2 = A1;

//LCDの宣言
ST7032 lcd;


//メソッドのプロトタイプ宣言
void LCD_ini();
void Mode_pid_selected();
void Mode_on_off_selected();
void pid_run_selected();
void pid_adjust_selected();
void on_off_run_selected();
void on_off_adjust_selected();
void pid_run();
void pid_adjust_gain_selected();
void pid_adjust_photo_selected();
void pid_gain();
void pid_photo();
void on_off_run();
void on_off_adjust();
void Motor_test();
void MotorR( int pwm );
void MotorL( int pwm );
void Motor_stop();


//センサー値格納用グローバル変数
int sensorValue = 0;
float val_P = 68, val_I = 0.01, val_D = 5.3;



//セットアップ
void setup()
{

  //操作ボタンの入力設定
  pinMode(ENTER, INPUT);
  pinMode(SELECT, INPUT);
  pinMode(BACK, INPUT);

  //モータードライバーの信号ピン出力設定
  pinMode(MotorL_signal_5, OUTPUT);
  pinMode(MotorL_signal_6, OUTPUT);
  pinMode(MotorR_signal_5, OUTPUT);
  pinMode(MotorR_signal_6, OUTPUT);

  //LCDの初期化
  lcd.begin(8, 2);
  lcd.setContrast(10);

  //ハードウェアシリアルの開始
  Serial.begin(19200);

}


//*********************メインループ！！************************
void loop()
{

  LCD_ini();
  delay(250);
  while(1)Mode_pid_selected();


}
//************************************************************



//↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓以下自作関数↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓

//PIDモードかON/OFFモードかでのPID選択画面
void Mode_pid_selected()
{
  lcd.clear();

  while(1)
  {
    lcd.setCursor(0, 0);
    lcd.print("+PID");
    lcd.setCursor(0, 1);
    lcd.print(" ON/OFF");
    delay(125);

    if( digitalRead( ENTER) == HIGH ) 
    {
      tone(PINNO,1500,BEAT);  
      delay(200);
      pid_run_selected();

    }
    else if( digitalRead( SELECT ) == HIGH )
    { 
      tone(PINNO,1500,BEAT);  
      delay(200);
      Mode_on_off_selected();      
    }
    else if( digitalRead( BACK ) == HIGH )
    {
      tone(PINNO,1500,BEAT);
      delay(100);
      tone(PINNO,1500,BEAT);
      delay(100);
      tone(PINNO,1500,BEAT);
      delay(100);
    }   
  }
}


void Mode_on_off_selected()
{
  lcd.clear();

  while(1)
  {
    lcd.setCursor(0, 0);
    lcd.print(" PID");
    lcd.setCursor(0, 1);
    lcd.print("+ON/OFF");
    delay(125);

    if( digitalRead( ENTER) == HIGH ) 
    {
      tone(PINNO,1500,BEAT);  
      delay(200);
      on_off_run_selected();

    }
    else if( digitalRead( SELECT ) == HIGH )
    { 
      tone(PINNO,1500,BEAT);  
      delay(200);
      Mode_pid_selected();

    }
    else if( digitalRead( BACK ) == HIGH )
    {
      tone(PINNO,1500,BEAT);
      delay(100);
      tone(PINNO,1500,BEAT);
      delay(100);
      tone(PINNO,1500,BEAT);
      delay(100);
    }   
  }
}



void pid_run_selected()
{
  lcd.clear();

  while(1)
  {
    lcd.setCursor(0, 0);
    lcd.print("+RUN");
    lcd.setCursor(0, 1);
    lcd.print(" ADJUST");
    delay(125);

    if( digitalRead( ENTER ) == HIGH ) 
    {
      tone(PINNO,1500,BEAT);  
      delay(500);
      pid_run();

    }
    else if( digitalRead( SELECT ) == HIGH )
    { 
      tone(PINNO,1500,BEAT);  
      delay(200);
      pid_adjust_selected();

    }
    else if( digitalRead( BACK ) == HIGH )
    {
      tone(PINNO,1500,BEAT);
      delay(200);
      Mode_pid_selected();

    }   
  }
}



//+++++++++++++++++++++++++++PID制御プログラム！！！！！！+++++++++++++++++++++++++++++++
void pid_run()
{

  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print("GET");
  lcd.setCursor(2, 1);
  lcd.print("READY");

  delay(500);
  tone(PINNO,1500,BEAT);
  delay(1000);
  tone(PINNO,1500,BEAT);
  delay(1000);
  //隠しコード！！==
  if( digitalRead( SELECT ) == HIGH )
  {
    tone(PINNO,1500,BEAT);
    delay(100);
    tone(PINNO,1500,BEAT);
    delay(100);
    tone(PINNO,1500,BEAT);
    delay(100);
    Motor_test();
  }   
  //================
  tone(PINNO,2000,500);
  delay(800);
  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("RUNNING");
  lcd.setCursor(0, 1);
  lcd.print("diff");



  //***********************PID本制御*************************


  //偏差用の配列変数
  float diff[2] = {0,0};
  //積分計算に用いる時間の変数
  float integral;
  //各操作量の演算結果
  float p, i, d;
  //操作量の合計
  float sum;
  int count = 0;

  lcd.clear();

  while(1)
  {

    diff[0] = diff[1];
    //偏差を取得
    diff[1] = ( analogRead( photo_pid ) - analogRead( Adjust ) ) / 12;
    integral += ( diff[1] + diff[0] ) / 2.0 * DELTA_T;


    //LCDに偏差を表示
    lcd.setCursor(4, 1);
    lcd.print(diff[1]);

    count++;
    if(count > 12) {
      Serial.write(int(diff[1])+127);
      count = 0;
    }

    p = (val_P/100) * diff[1];
    i = (val_I/100) * integral;      
    d = (val_D/100) * ( diff[1] - diff[0] ) / DELTA_T;

    if( val_D/100 > 0.9 )sum = p;
    else sum = p + d;

    //最大値、最小値を制限
    if ( 255 < sum + Steady_Motor_Power ) sum = 255 - Steady_Motor_Power;
    else if ( sum + Steady_Motor_Power  < 0 ) sum = -Steady_Motor_Power;
    else ;

    /*
    if( diff[1] < 8 && diff[1] > -8 ) {
     MotorR(Steady_Motor_Power);
     MotorL(Steady_Motor_Power);
     }
     turn(sum);
     */


    MotorL(Steady_Motor_Power-sum );
    MotorR(Steady_Motor_Power+sum );

    //4msecで回す。
    delay(4);



    if( digitalRead( BACK ) == HIGH )
    { 
      tone(PINNO,1500,BEAT);  
      Motor_stop();
      delay(200);
      pid_run_selected();
    }

  }

  //********************************************************


}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//旋回関数→結局使わんかった…
void turn(float sum) {
  int turn = int(sum);

  if( turn > 0 ) {
    MotorR( turn );
    MotorL( 0 );
  }
  else if( turn < 0 ) { 
    /*
    digitalWrite( MotorR_signal_5, LOW );
     digitalWrite( MotorR_signal_6, LOW);
     */
    MotorR( 0 ); 
    MotorL( turn );
  }
  else ;  
}



//モーター調整用隠し関数じゃ！！！！！
void Motor_test()
{
  int count = 0;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Power:");

  while(1)
  {

    int val=analogRead( Adjust ); 
    val = map(val, 0, 1023, 0, 255);

    //これモータードライバー的には逆回転だけど、ライントレーサー的には前進！！
    digitalWrite( MotorL_signal_5, LOW );
    digitalWrite( MotorL_signal_6, HIGH);
    digitalWrite( MotorR_signal_5, LOW );
    digitalWrite( MotorR_signal_6, HIGH);
    //可変抵抗の値に応じて回転
    analogWrite( MotorL_pwm, val+Motor_offset ); 
    analogWrite( MotorR_pwm, val ); 


    count++;
    if( count == 2000 )
    {
      //桁あふれを防ぐ処理
      if( val  >= 100 ) lcd.setCursor(3, 1);
      else if( val  >= 10 && val < 100 ) {
        lcd.setCursor(3, 1);
        lcd.print("0");
        lcd.setCursor(4, 1);
      }
      else if( val < 10 ) {
        lcd.setCursor(3, 1);
        lcd.print("00");
        lcd.setCursor(5, 1);
      }
      lcd.print( val );  //値表示
      count = 0;
    }


    if( digitalRead( BACK ) == HIGH )
    {
      digitalWrite( MotorL_signal_5, LOW );
      digitalWrite( MotorL_signal_6, LOW );
      digitalWrite( MotorR_signal_5, LOW );
      digitalWrite( MotorR_signal_6, LOW );
      tone(PINNO,1500,BEAT);
      delay(200);
      pid_run_selected();
    }  

  }    


}



void pid_adjust_selected()
{

  lcd.clear();
  while(1)
  {

    lcd.setCursor(0, 0);
    lcd.print(" RUN");
    lcd.setCursor(0, 1);
    lcd.print("+ADJUST");
    delay(125);

    if( digitalRead( ENTER) == HIGH ) 
    {
      tone(PINNO,1500,BEAT);  
      delay(200); 
      pid_adjust_gain_selected();

    }
    else if( digitalRead( SELECT ) == HIGH )
    { 
      tone(PINNO,1500,BEAT);  
      delay(200);
      pid_run_selected();

    }
    else if( digitalRead( BACK ) == HIGH )
    {
      tone(PINNO,1500,BEAT);
      delay(200);
      Mode_pid_selected();

    }  
  }

}


void pid_adjust_gain_selected()
{

  lcd.clear();
  while(1)
  {

    lcd.setCursor(0, 0);
    lcd.print("+PIDGAIN");
    lcd.setCursor(0, 1);
    lcd.print(" PHOTO-D");
    delay(125);

    if( digitalRead( ENTER) == HIGH ) 
    {
      tone(PINNO,1500,BEAT);  
      delay(200); 
      pid_gain();

    }
    else if( digitalRead( SELECT ) == HIGH )
    { 
      tone(PINNO,1500,BEAT);  
      delay(200);
      pid_adjust_photo_selected();

    }
    else if( digitalRead( BACK ) == HIGH )
    {
      tone(PINNO,1500,BEAT);
      delay(200);
      pid_adjust_selected();

    }  
  }

}


void pid_adjust_photo_selected()
{

  lcd.clear();
  while(1)
  {

    lcd.setCursor(0, 0);
    lcd.print(" PIDGAIN");
    lcd.setCursor(0, 1);
    lcd.print("+PHOTO-D");
    delay(125);

    if( digitalRead( ENTER) == HIGH ) 
    {
      tone(PINNO,1500,BEAT);  
      delay(200); 
      pid_photo();

    }
    else if( digitalRead( SELECT ) == HIGH )
    { 
      tone(PINNO,1500,BEAT);  
      delay(200);
      pid_adjust_gain_selected();

    }
    else if( digitalRead( BACK ) == HIGH )
    {
      tone(PINNO,1500,BEAT);
      delay(200);
      pid_adjust_selected();

    }  
  }

}


/*
PID制御におけるゲインを決定するメソッド
 ATtiny85が、取得した可変抵抗の値をソフトウェアシリアル通信を用いて送信し、ATmega328P（コレ）が
 ハードウェアシリアルで受信し、LCDに表示する。
 */
void pid_gain()
{

  int count = 0;

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("p");
  lcd.setCursor(1, 0);
  lcd.print( val_P/100 );
  lcd.setCursor(4, 0);
  lcd.print("i");
  lcd.setCursor(5, 0);
  lcd.print( val_I/100 );
  lcd.setCursor(0, 1);
  lcd.print("d");
  lcd.setCursor(1, 1);
  lcd.print( val_D/100 );

  while(1)
  {

    //ここにシリアル通信のプログラムを挿入
    if( digitalRead( SELECT ) == HIGH  )
    {
      count++;
      if(count == 4)count=1;
      lcd.clear();
      delay(125);
    }

    if(count == 1)
    {
      Serial.write(1);
      delay(10);
      if( Serial.available() )val_P = Serial.read();
      lcd.setCursor(0, 0);
      lcd.print("P:");
      lcd.setCursor(2, 0);
      lcd.print( val_P/100 );
      lcd.setCursor(0, 1);
      lcd.print("i:");
      lcd.setCursor(2, 1);
      lcd.print( val_I/100 );
    }

    else if(count == 2)
    {
      Serial.write(2);
      delay(10);
      if( Serial.available() )val_I = Serial.read();
      lcd.setCursor(0, 0);
      lcd.print("p:");
      lcd.setCursor(2, 0);
      lcd.print( val_P/100 );
      lcd.setCursor(0, 1);
      lcd.print("I:");
      lcd.setCursor(2, 1);
      lcd.print( val_I/100 );      
    }

    if(count == 3)
    {
      Serial.write(3);
      delay(10);
      if( Serial.available() )val_D = Serial.read();
      lcd.setCursor(0, 0);
      lcd.print("i:");
      lcd.setCursor(2, 0);
      lcd.print( val_I/100 );
      lcd.setCursor(0, 1);
      lcd.print("D:");
      lcd.setCursor(2, 1);
      lcd.print( val_D/100 );
    }

    delay(10);

    if( digitalRead( ENTER ) == HIGH )
    {
      tone(PINNO,1500,BEAT);
      delay(200);
      pid_run_selected();
    }  

    if( digitalRead( BACK ) == HIGH )
    {
      tone(PINNO,1500,BEAT);
      delay(200);
      pid_adjust_gain_selected();
    }  
  }

}


void pid_photo()
{

  while(1)
  {
    /*
    //センサー値を５回取得して平均する。
     int photo_pid_array[4];
     int photo_pid_ave = 0;
     int i;
     for( i=0;i<5;i++ )
     {
     photo_pid_array[i] = analogRead( photo_pid );
     photo_pid_ave += photo_pid_array[i];
     }
     photo_pid_ave = photo_pid_ave / 5;
     */

    //表示プロセス
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("SENSOR_");
    lcd.setCursor(0, 1);
    lcd.print("VAL:");
    //桁あふれを防ぐ処理
    if( analogRead( photo_pid ) >= 1000 ) lcd.setCursor(4, 1);
    else {
      lcd.setCursor(4, 1);
      lcd.print("0");
      lcd.setCursor(5, 1);
    }

    lcd.print( analogRead( photo_pid ) );
    delay(125);

    //ENTERで値の設定画面に移行
    if( digitalRead( ENTER ) == HIGH )
    {
      tone(PINNO,1500,BEAT);
      delay(200);
      lcd.clear();
      while(1)
      {
        lcd.setCursor(0, 0);
        lcd.print("SETVALUE");
        lcd.setCursor(0, 1);
        lcd.print("VAL:");
        //桁あふれを防ぐ処理
        if( analogRead( Adjust ) >= 1000 ) lcd.setCursor(4, 1);
        else if( analogRead( Adjust ) >= 100 && analogRead( Adjust ) < 1000 ) {
          lcd.setCursor(4, 1);
          lcd.print("0");
          lcd.setCursor(5, 1);
        }
        else {
          lcd.setCursor(4, 1);
          lcd.print("00");
          lcd.setCursor(6, 1);
        }
        lcd.print( analogRead( Adjust ) );
        delay(125);

        if( digitalRead( ENTER ) == HIGH )
        {
          tone(PINNO,1500,BEAT);
          delay(200);
          pid_run_selected();
        }
        else if( digitalRead( BACK ) == HIGH )
        {
          tone(PINNO,1500,BEAT);
          delay(200);
          pid_adjust_photo_selected();
        }
      }

    }

    else if( digitalRead( BACK ) == HIGH )
    {
      tone(PINNO,1500,BEAT);
      delay(200);
      pid_adjust_photo_selected();
    }

  } 

}


void on_off_run_selected()
{

  lcd.clear();

  while(1)
  {

    lcd.setCursor(0, 0);
    lcd.print("+RUN");
    lcd.setCursor(0, 1);
    lcd.print(" ADJUST");
    delay(125);

    if( digitalRead( ENTER) == HIGH ) 
    {
      tone(PINNO,1500,BEAT);  
      delay(500);
      on_off_run();

    }
    else if( digitalRead( SELECT ) == HIGH )
    { 
      tone(PINNO,1500,BEAT);  
      delay(200);
      on_off_adjust_selected();

    }
    else if( digitalRead( BACK ) == HIGH )
    {
      tone(PINNO,1500,BEAT);
      delay(200);
      Mode_on_off_selected();

    }   
  }

}


void on_off_adjust_selected()
{

  lcd.clear();

  while(1)
  {

    lcd.setCursor(0, 0);
    lcd.print(" RUN");
    lcd.setCursor(0, 1);
    lcd.print("+ADJUST");
    delay(125);

    if( digitalRead( ENTER) == HIGH ) 
    {
      tone(PINNO,1500,BEAT);  
      delay(200); 
      on_off_adjust();

    }
    else if( digitalRead( SELECT ) == HIGH )
    { 
      tone(PINNO,1500,BEAT);  
      delay(200);
      on_off_run_selected();

    }
    else if( digitalRead( BACK ) == HIGH )
    {
      tone(PINNO,1500,BEAT);
      delay(200);
      Mode_on_off_selected();

    }   
  }

}



//======================ON-OFF制御プログラム！！！=============================
void on_off_run()
{

  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print("GET");
  lcd.setCursor(2, 1);
  lcd.print("READY");

  delay(500);
  tone(PINNO,1500,BEAT);
  delay(1000);
  tone(PINNO,1500,BEAT);
  delay(1000);
  tone(PINNO,2000,500);
  delay(100);

  int obj = analogRead( Adjust );

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("RUNNING!");
  lcd.setCursor(0, 1);
  lcd.print("========");


  //走行開始！！！！！
  while(1)
  {

    /*
    digitalWrite( MotorL_signal_5, LOW );
     digitalWrite( MotorL_signal_6, HIGH);
     digitalWrite( MotorR_signal_5, LOW );
     digitalWrite( MotorR_signal_6, HIGH);
     //可変抵抗の値に応じて回転
     analogWrite( MotorL_pwm, val ); 
     analogWrite( MotorR_pwm, val ); 
     */

    if( analogRead( photo_2 ) < obj )  
    {
      digitalWrite( MotorR_signal_5, LOW );
      digitalWrite( MotorR_signal_6, LOW );
      digitalWrite( MotorL_signal_5, LOW );
      digitalWrite( MotorL_signal_6, HIGH);
      analogWrite( MotorR_pwm, 0 );
      analogWrite( MotorL_pwm, 130 );
    }
    else if( analogRead( photo_1 ) < obj )
    {
      digitalWrite( MotorR_signal_5, LOW );
      digitalWrite( MotorR_signal_6, HIGH);
      digitalWrite( MotorL_signal_5, LOW );
      digitalWrite( MotorL_signal_6, LOW );
      analogWrite( MotorR_pwm, 130 );
      analogWrite( MotorL_pwm, 0 );
    }

    else if( digitalRead( BACK ) == HIGH )
    {
      digitalWrite( MotorL_signal_5, LOW );
      digitalWrite( MotorL_signal_6, LOW );
      digitalWrite( MotorR_signal_5, LOW );
      digitalWrite( MotorR_signal_6, LOW );
      tone(PINNO,1500,BEAT);
      delay(200);
      on_off_run_selected();
    }

    else {
      digitalWrite( MotorR_signal_5, LOW );
      digitalWrite( MotorR_signal_6, HIGH );
      digitalWrite( MotorL_signal_5, LOW );
      digitalWrite( MotorL_signal_6, HIGH);
      analogWrite( MotorR_pwm, 90 );
      analogWrite( MotorL_pwm, 90 + Motor_offset );
    }  
  }

}
//==============================================================================



/*オンオフ制御用のフォトリフレクタ制御画面
 左右のフォトリフレクタの値が表示される。ENTERを押すと実際の設定値の画面になり
 フォトリフレクタ調整用の可変抵抗の値が表示される。
 もう一度ENTERを押すとその値が保存され、オンオフ制御のラン選択画面に移行する。*/
void on_off_adjust()
{

  while(1)
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("SETVALUE");
    lcd.setCursor(0, 1);
    lcd.print("R");
    lcd.setCursor(1, 1);
    if( analogRead( photo_2 ) < 1000 )lcd.print( analogRead( photo_2 ) );
    else lcd.print( "OVR" );
    lcd.setCursor(4, 1);
    lcd.print("L");
    lcd.setCursor(5, 1);
    if( analogRead( photo_1 ) < 1000 )lcd.print( analogRead( photo_1 ) );
    else lcd.print( "OVR" );
    delay(125);

    //ENTERで値の設定画面に移行
    if( digitalRead( ENTER ) == HIGH )
    {
      tone(PINNO,1500,BEAT);
      delay(200);
      lcd.clear();
      while(1)
      {
        lcd.setCursor(0, 0);
        lcd.print("SETVALUE");
        lcd.setCursor(0, 1);
        lcd.print("VAL:");
        //桁あふれを防ぐ処理
        if( analogRead( Adjust ) >= 1000 ) lcd.setCursor(4, 1);
        else if( analogRead( Adjust ) >= 100 && analogRead( Adjust ) < 1000 ) {
          lcd.setCursor(4, 1);
          lcd.print("0");
          lcd.setCursor(5, 1);
        }
        else {
          lcd.setCursor(4, 1);
          lcd.print("00");
          lcd.setCursor(6, 1);
        }
        lcd.print( analogRead( Adjust ) );
        delay(125);

        if( digitalRead( ENTER ) == HIGH )
        {
          tone(PINNO,1500,BEAT);
          delay(200);
          on_off_run_selected();
        }
        else if( digitalRead( BACK ) == HIGH )
        {
          tone(PINNO,1500,BEAT);
          delay(200);
          on_off_adjust_selected();
        }
      }

    }
    else if( digitalRead( BACK ) == HIGH )
    {
      tone(PINNO,1500,BEAT);
      delay(200);
      on_off_adjust_selected();
    } 
  } 

}


//右モーター駆動用関数
//入力パスルを引数としてモーターを駆動させる。
void MotorR( int pwm )
{

  digitalWrite( MotorR_signal_5, LOW );
  digitalWrite( MotorR_signal_6, HIGH);
  /*
  analogWrite( MotorR_pwm , 150 );
   delay(250);
   */
  analogWrite( MotorR_pwm , pwm );

}


//左モーター駆動用関数
//入力パスルを引数としてモーターを駆動させる。
void MotorL( int pwm )
{

  digitalWrite( MotorL_signal_5, LOW );
  digitalWrite( MotorL_signal_6, HIGH);
  /*
  analogWrite( MotorL_pwm , 150 );
   delay(250);
   */
  analogWrite( MotorL_pwm , pwm + Motor_offset );

}


//両方のモーターを停止させる。
void Motor_stop()
{

  digitalWrite( MotorL_signal_5, LOW );
  digitalWrite( MotorL_signal_6, LOW);
  digitalWrite( MotorR_signal_5, LOW );
  digitalWrite( MotorR_signal_6, LOW);

}


//LCDの起動時アニメーション
void LCD_ini()
{
  int i;
  for(i=0;i<7;i++)
  {
    lcd.setCursor(i, 0);
    lcd.print("==");
    lcd.setCursor(i, 1);
    lcd.print("==");
    delay(75);
  }
  delay(500);

  lcd.setCursor(0, 0);
  lcd.print("PID-LINE");
  lcd.setCursor(0, 1);
  lcd.print("=TRACER=");

  tone(PINNO,1500,BEAT);  // ド523
  delay(125);
  tone(PINNO,1500,BEAT);  // ド523
  delay(500);
}


//==================================================================================================
//==================================================================================================
//==================================================================================================













