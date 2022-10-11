

// 모터드라이 보드(모터4개), 릴레이 1개(모터펌프), 서보모터 1개, 광감지센서 2개, 온습도센서 1개, IR리모콘수신부 1개,
// LCD ( 16x2), 3색 LED, 토양습도센서 1개,


#include <SoftwareSerial.h>
#include <IRremote.h>           // IR 리모컨
#include <Wire.h>                //i2c
#include <LiquidCrystal_I2C.h>   //LCD
#include "DHT.h"                 // 온습도센서( DHT11 Sensor)
#include <VarSpeedServo.h>        //서보모터 

int PanCtrlMotorDriverF( int sp ); // DC Motor forward
int PanCtrlMotorDriverB( int sp ); // DC Motor Backward
int PanCtrlMotorDriverL( int sp ); // DC Motor Left
int PanCtrlMotorDriverR( int sp ); // DC Motor Right
int PanCtrlMotorDriverS(); // DC Motor Stop
int read_soil_humidity(); // 토양습도 센서에서 값을 읽어오는 함수
int read_Humidity_Temperature(); // 온습도를 센서에서 읽어옴. (온도 및 습도값)
void DisplayLCD(const char *buff1, const char *buff2);
int Max_Light_dir(); //조도센서의 값을 읽어서 가장 밝은 방향을 찾음. (45각도로 360도 읽음.)

#define M1 11        // 모터 1의 속도 PWM
#define M1IN1 41    //  모터1     (모터드라이브를 사용할 경우 analogwrite())
#define M1IN2 43    //  모터1     pan1 ON/OFF
#define M2IN1 45    //  모터2     모터드라이브를 사용할 경우)analogwrite()
#define M2IN2 47   //  모터2      pan2 ON/OFF
#define M2 12      //모터2의 속도 

#define Relay_Motor 2      //모터펌프 전원용 릴레이 

#define Trig_1 3   // HC-SR04 초음파센서 1 ( forward)
#define Echo_1 4

#define Trig_2 5   // HC-SR04 초음파센서 2 ( Back)
#define Echo_2 6

#define S_Motor_pin 7  //180도 회전하는 서보모터(조도값을 감지하기 위한모터)

#define DHTPIN1 8     // 온습도 센서 연결 pin
#define DHTTYPE DHT11   // DHT 11 온습도센서의 모델을 선언한 것(핀번호 아님)

#define IR_recon 9 // 리모컨수광부 핀번호

#define LED_B1 48    // 3색 LED Red   High : ON, 
#define LED_G1 50    // 3색 LED G
#define LED_R1 52    // 3색 LED B

#define Light_sensor1 A0   // 광센서 , 햇빛을 찾기위한 센서
#define Light_sensor2 A1   //
#define SH_sensor A5   // 토양습도 센서


VarSpeedServo myservo;  // create servo object to control a servo 
                        // a maximum of eight servo objects can be created 
IRrecv irrecv(IR_recon); // IR 센서 리시버의 9번핀 사용 선언
//IRrecv irrecv(9); // IR 센서 리시버의 9번핀 사용 선언
decode_results results; //IR리모콘으로 부터 받은 신호값을 변수 선언

LiquidCrystal_I2C lcd(0x27,16,2);  // LCD 선언
DHT dht1(DHTPIN1, DHTTYPE);  // 온습도 센서를 위한 함수

//double duration;      // 초음파센서로 돌아오는 시간및 거리측정
//double distance_F1; // 앞쪽  초음파센서 (센서 1)
//double distance_F2; // 뒷쪽  초음파센서 (센서 2)
int distance_F1; // 앞쪽  초음파센서 (센서 1)
int distance_F2; // 뒷쪽  초음파센서 (센서 2)

float h1 = 0;  // 센서1의 습도 값
float t1 = 0;  // 온도
float f1 = 0;  //...
int soil_h2 = 0;  // 토양습도 센서 값 ( 0~1023을  0~100 % 으로 표시함.)  (A3)

int max_angle_value = 0;  // 가장 밝은 방향의 조도 값.
int max_angle_value1 = 0;  // 가장 밝은 방향의 조도 값.
int max_angle_value2 = 0;  // 가장 밝은 방향의 조도 값.
int light_step1 = 0; // 현재 조도값 (이동 후의 조도의 세기를 비교하기 위함.)
int light_step2 = 0; //전 조도 최고값
int light_step3 = 0; //전전 조도 최고값

int Sun_Light = 92; // 햇빛에 도착했을때의 기준값

unsigned long Now_Millis = 0; // 내부 클럭을 이용한 millis() 함수사용.
unsigned long Start_Millis_Temp = 0;  // 

unsigned long Motor_Stop_time = 0; // 모터구동시간을 제어하기 위한 시간체크 (이용한 millis() 함수사용).
unsigned long Motor_Start_time = 0;  // 

int Turn_count_Left = 0; // 왼쪽으로 회전한 숫자 ( 회전한 숫자로 각도를 계산하여, 360도 회전시 정지함.)
int Turn_count_Right = 0; // 오른쪽으로 회전한 숫자               

int speed_F = 250; // 정주행시 속도
int speed_B = 250; // 후진 주행시 속도
int speed_T = 250; // 방향전환시 속도
int Motor_drive_timeM = 400; // 리모콘 모드에서의 모터동작 시간 //25cm 이동
//int Motor_drive_timeT = 330;  // 회전시 모터가 동작하는 시간 //45도 회전
int Motor_drive_timeT = 250;  // 회전시 모터가 동작하는 시간 //30도 회전

int Password_OK = 10; //  리모콘 모드 : 10,   자동모드 : 0
int dis_Errorcode = 0; // 에러가 발생(-1), 정상(0)
void setup() {

  pinMode(Relay_Motor, OUTPUT); // 모터펌프 제어용 릴레이
  digitalWrite(Relay_Motor, LOW);

  // LCD 초기화
  lcd.init();
  lcd.backlight();
  DisplayLCD("Hello ", "Smart Flowerpot"); 

  dht1.begin();  // 온습도센서 초기화

// 모터 드라이버 핀셋업
  pinMode(M1, OUTPUT);
  pinMode(M1IN1, OUTPUT);
  pinMode(M1IN2, OUTPUT);
  pinMode(M2IN1, OUTPUT);
  pinMode(M2IN2, OUTPUT);
  pinMode(M2, OUTPUT);

// 초음파센서 핀 셋업
  pinMode(Trig_1, OUTPUT);
  pinMode(Echo_1, INPUT);
  pinMode(Trig_2, OUTPUT);
  pinMode(Echo_2, INPUT);

//조도센서 및 토양습도 센서 핀 셋업
  pinMode(Light_sensor1, INPUT);
  pinMode(Light_sensor2, INPUT);
  pinMode(SH_sensor, INPUT);
 
  // Open serial communications and wait for port to open
  Serial.begin(9600);
    while (!Serial) { ;} // wait for serial port to connect. 

    myservo.attach(S_Motor_pin);  // attaches the servo 
                  
    irrecv.enableIRIn(); //리모콘 수신 사용.  
    delay(100);

  myservo.write(0,50,false); // set the intial position of the servo, as fast as possible, wait until done
//  myservo.write(0,50,true); 
  delay(500);

  pinMode(LED_R1, OUTPUT);  // LED 구동을 위한 Pin설정
  pinMode(LED_G1, OUTPUT);
  pinMode(LED_B1, OUTPUT);
  DisplayLED(13); //Blue 초기 리모컨 모드
}


void loop() { // run over and over

int Light_angle = 0; // 조도센서의 값에의한 밝은곳의 각도 변수  (현재 방향기준에서 반시계방향)

//  1)온습도및 토양습도센서들의 값 읽기
     Now_Millis = millis();
     
     if(( ( Now_Millis - Start_Millis_Temp )> 3000) || (Start_Millis_Temp == 0)) // 3초마다 센서값을 읽는다.
        {
 
           read_soil_humidity(); // 토양습도 센서에서 값을 읽어오는 함수
           read_Humidity_Temperature(); // 온습도를 센서에서 읽어옴
           
           if( Password_OK == 10 && dis_Errorcode == 0 ){DisplayLCDValue(t1, h1, soil_h2, "Menu" );}
           else if ( Password_OK == 0 && dis_Errorcode == 0){ DisplayLCDValue(t1, h1, soil_h2, "Auto" );}
           else if ( dis_Errorcode == -1){ DisplayLCDValue(t1, h1, soil_h2, "Err" );}
           else{;}
           
//           DisplayLCDValue(t1, h1, soil_h2);  // LCD에 습도, 온도, 토양습도 센서값 표시

           if( (soil_h2 <= 20) && (Password_OK == 0 ))// 토양습도가 20% 이하이고 자동모드일 경우 모터펌프동작.
             {
                  digitalWrite(Relay_Motor, HIGH); // 0.5초 물주기
                  delay(500);
                  digitalWrite(Relay_Motor, LOW);
             }
          Start_Millis_Temp = Now_Millis; // 현재시간을 시작시간으로 한다.

      } // end if( ( Now_Millis - Start_Millis_Temp )> 3000) // 3초마다 센서값을 읽는다
    
//2) 리모콘 신호를 입력받고 리모콘신호에 따른 명령처리

  if( irrecv.decode(&results) == true ) //리모콘으로 받은신호가 있으면
    {
       dis_Errorcode = 0; //에러코드를 정상으로 복귀함.
       Serial.print("리모콘 코드 = "); // 모니터로 출력
       Serial.println(results.value, HEX); // 모니터로 출력
       
       if ( Password_OK == 10 ) // 리모콘 모드 일경우.
          {
           DisplayLCDValue(t1, h1, soil_h2, "Menu" );
           DisplayLED(13); //blue 리모컨 모드

           light_step1 = 0; // Auto 모드에서 저장된 괘도의 조도값은 초기화
           light_step2 = 0; //
           light_step3 = 0; //
           
           switch (results.value) 
            {
             case 0xFF18E7 : //2번 정방향 ^
                               Serial.println(" 정주행 ");

                               HCSR04distance(1); // 초음파센서의 장애물 거리
                               if( distance_F1 <= 30 )
                                   {   
                                      PanCtrlMotorDriverS();
                                      DisplayLED(11); // int rgb 13은 Red 임. 
                                      dis_Errorcode = -1; //에러 표시
                                      DisplayLCDValue(t1, h1, soil_h2, "Err" );
                                      Serial.print("장애물 거리 = "); // 모니터로 출력
                                      Serial.println(distance_F1); // 모니터로 출력
                                      
                                   }

                               else {PanCtrlMotorDriverFd( speed_F, Motor_drive_timeM );}

                               break;
            case 0xFF4AB5 :  //8번 후진 V
                              Serial.println(" 후진");
                               HCSR04distance(2); // 초음파센서의 장애물 거리
                               if( distance_F2 <= 30 )
                                   {   
                                      PanCtrlMotorDriverS();
                                      DisplayLED(11); // int rgb 은 Red 임.
                                      dis_Errorcode = -1; //에러 표시
                                      DisplayLCDValue(t1, h1, soil_h2, "Err" );
                                      Serial.print("장애물 거리 = "); // 모니터로 출력
                                      Serial.println(distance_F2); // 모니터로 출력
                                      
                                   }
                              else {PanCtrlMotorDriverBd( speed_F, Motor_drive_timeM );}

                               break;
            case 0xFF10EF :  // 4번 좌회전 <-
                             Serial.println(" 좌회전");

                             PanCtrlMotorDriverLd( speed_F, Motor_drive_timeT );

                               break;
            case 0xFF5AA5 :  // 6번 우회전   -> 
                            Serial.println(" 우회전");

                            PanCtrlMotorDriverRd( speed_F, Motor_drive_timeT );

                               break;
            case 0xFF38C7 :// 5번 정지 OK
                           Serial.println(" 정지");
                           PanCtrlMotorDriverS();
                               break;
            case 0xFFFFFFFF : // FFFFFFFF 일 경우
                              Serial.println("FFFFFFFF 통과 "); // 모니터로 출력 
                               break;
            case 0xFF906F : // '9' 일 경우, 자동모드로 전환함.
                              
                            Password_OK = 0;
                            DisplayLCDValue(t1, h1, soil_h2, "Auto" );
                            DisplayLED(12); // GREEN
                            PanCtrlMotorDriverS();

                            Serial.println("자동모드로 전환 "); // 모니터로 출력  
                            break;   
            case 0xFFB04F : // '#' 일 경우, 화분에 물주기( 모터펌프 구동).
                                                        
                            Serial.println("물주기 "); // 모니터로 출력 
                           
                            digitalWrite(Relay_Motor, HIGH); // 0.5초 물주기
                            delay(500);
                            digitalWrite(Relay_Motor, LOW);
                            break;     
            case 0xFFE21D : // '3' 일 경우, 조도센서 서보모터 테스트.
                                                        
                            Serial.println("서보모터 테스트 "); // 모니터로 출력 
                           // Max_Light_dir();
                           // Reverse_Max_Light_dir();
                            Max_Light_dir();
                            break;                      
                                                
            default : 
                      Serial.println(" 정지");
                      PanCtrlMotorDriverS(); 
                               break;
            }// end_switch (results.value) 
          }//end_if ( Password_OK == 10 )
 
       else{
          if( results.value == 0xFFA25D)  // '1' 이면 리모콘모드(수동) 전환함.
             {

               Serial.print("리모콘 모드 "); // 모니터로 출력
               Password_OK = 10;  // 1번 입력시 리모콘 모드로 전환
               DisplayLED(13); // blue
               PanCtrlMotorDriverS();
             }
          else {   
                     Serial.print(" pass ");
                }
           
        } //end_else
        
       irrecv.resume(); //리모콘 다음신호 수신
    }// end_if( irrecv.decode(&results) == true ) //리모콘으로 받은 신호가 있으면

  
// 3)조도에 의한 밝은곳 찾는 자동모드 동작 
// 조도의 밝은 방향과 조도의 강도를 기억해서, 읽어들인 조도가 이전보다 작을 경우 정지 함.  
//    Serial.print(" Password_OK ");  Serial.println(Password_OK);
   if ( Password_OK == 0 ) // 자동 모드 일경우.
   {
      DisplayLED(12); //RED 리모컨 모드
      Light_angle = Max_Light_dir(); //조도를 8개방향에서 읽은후 가장밝은 방향을 숫자로 받은.
                                     // 0 ~ 360를 반시계방향으로 45도씩 나눈 순번
      Serial.print("Light_angle == "); Serial.println(Light_angle);// 최고조도 순번
      
        switch (Light_angle) //  8가지 경우에 대해서 1개씩 정의함
              {
                 case 0:  // 0도 방향
                            Serial.println("0 도 ");
                            PanCtrlMotorDriverRd( speed_F, Motor_drive_timeT );
                            delay(200);
                            PanCtrlMotorDriverRd( speed_F, Motor_drive_timeT );
                            delay(200);
                            PanCtrlMotorDriverRd( speed_F, Motor_drive_timeT );
                            delay(200);
                            
                            HCSR04distance(1); // 초음파센서의 장애물 거리
                            PanCtrlMotorDriverFBd_if(1);// 1번방향(정주행 방향)
                                                  
                           break;
                 case 1: //30도 방향
                           Serial.println("30 도 ");
                           PanCtrlMotorDriverRd( speed_F, Motor_drive_timeT );
                           delay(200);
                           PanCtrlMotorDriverRd( speed_F, Motor_drive_timeT );
                           delay(200);
                           
                           HCSR04distance(1); // 초음파센서의 장애물 거리
                           PanCtrlMotorDriverFBd_if(1); // 1번방향(정주행 방향)
                           
                           break;
                 case 2: //60동 방향
                           Serial.println("60 도 "); 
                           PanCtrlMotorDriverRd( speed_F, Motor_drive_timeT );
                           delay(200);
                           
                           HCSR04distance(1); // 초음파센서의 장애물 거리
                           PanCtrlMotorDriverFBd_if(1); // 1번방향(정주행 방향)
                           
                           break;
                 case 3: //90동 방향
                           Serial.println("90 도 "); 
                                                                          
                           HCSR04distance(1); // 초음파센서의 장애물 거리
                           PanCtrlMotorDriverFBd_if(1); // 1번방향(정주행 방향)
                           
                           break;
                 case 4: //120도 방향
                           Serial.println("120 도 "); // 
                           PanCtrlMotorDriverLd( speed_F, Motor_drive_timeT );
                           delay(200);
                                                      
                           HCSR04distance(1); // 초음파센서의 장애물 거리
                           PanCtrlMotorDriverFBd_if(1); // 1번방향(직진주행)
                           
                           break;
                 case 5: //150도 방향
                           Serial.println("150 도 "); // 
                           PanCtrlMotorDriverLd( speed_F, Motor_drive_timeT );
                           delay(200);
                           PanCtrlMotorDriverLd( speed_F, Motor_drive_timeT );
                           delay(200);
                                                                                 
                           HCSR04distance(1); // 초음파센서의 장애물 거리
                           PanCtrlMotorDriverFBd_if(1); // 1번방향(직진주행)
                           
                           break;
                 case 6: //180도 방향
                           Serial.println("180 도 ");
                           PanCtrlMotorDriverLd( speed_F, Motor_drive_timeT );
                           delay(200);
                           PanCtrlMotorDriverLd( speed_F, Motor_drive_timeT );
                           delay(200);
                           PanCtrlMotorDriverLd( speed_F, Motor_drive_timeT );
                           delay(200);
                           
                           HCSR04distance(1); // 1번 초음파센서의 장애물 거리
                           PanCtrlMotorDriverFBd_if(1); // 1번방향(직진주행 방향)
                           break;

                 case 7: //210도 방향
                           Serial.println("210 도 ");
                           PanCtrlMotorDriverRd( speed_F, Motor_drive_timeT );
                           delay(200);
                           PanCtrlMotorDriverRd( speed_F, Motor_drive_timeT );
                           delay(200);                           
                           
                           HCSR04distance(2); // 2번 초음파센서의 장애물 거리
                           PanCtrlMotorDriverFBd_if(2); // 2번방향(후진주행 방향)
                                                     
                           break;
                 case 8: //240도 방향
                           Serial.println("240 도 "); //2회 회전
                           PanCtrlMotorDriverRd( speed_F, Motor_drive_timeT );
                           delay(200);                                                   
                           
                           HCSR04distance(2); // 2번 초음파센서의 장애물 거리
                           PanCtrlMotorDriverFBd_if(2); // 2번방향(후진주행 방향)
                                                     
                           break; 
  
                 case 9: //270도 방향
                           Serial.println("270 도 ");
                                                     
                           HCSR04distance(2); // 초음파센서의 장애물 거리
                           PanCtrlMotorDriverFBd_if(2); // 2번방향(후진 방향)
                           break;
                 case 10: //300도 방향
                           Serial.println("300 도 ");
                           PanCtrlMotorDriverLd( speed_F, Motor_drive_timeT );
                           delay(200);
                                                      
                           HCSR04distance(2); // 초음파센서의 장애물 거리
                           PanCtrlMotorDriverFBd_if(2); // 2번방향(후진 방향)
                           break;

                 case 11: //330도 방향
                           Serial.println("315 도 ");
                           PanCtrlMotorDriverLd( speed_F, Motor_drive_timeT );
                           delay(200);
                           PanCtrlMotorDriverLd( speed_F, Motor_drive_timeT );
                           delay(200);
                           
                           HCSR04distance(2); // 초음파센서의 장애물 거리
                           PanCtrlMotorDriverFBd_if(2); // 1번방향(정주행 방향)
                           break;

                 default : 
                            Serial.println(" wrong number");
                            break;

              }//end_switch (Light_angle) //  8가지 경우에 대해서 1개씩 정의함
    
       if ( max_angle_value > light_step1)  // 읽어온 최고 조도와 지난번 조도를 비교
          {
            light_step2 = light_step1;
            light_step1 = max_angle_value;
          }
      
       
       else if (( max_angle_value <= light_step1) && (max_angle_value <= light_step2 ))
          {
            if ( max_angle_value >= Sun_Light)// 목적지 도착 함.
                  {
                       DisplayLED(20); // LED ALL, 목적지 도착 함.
                                
                       Password_OK == 10;
                       Serial.println("목적지 도착 ");
                       DisplayLCDValue(t1, h1, soil_h2, "Done" );
                       PanCtrlMotorDriverS();
                      
                  }
             else {  // 목적지를 못찾음  
                       DisplayLED(14); // 껌짐  
                       Password_OK = 10;   //리모컨 모드로 전환                
                       Serial.println("목적지를 못찾음 "); 
                       DisplayLCDValue(t1, h1, soil_h2, "None" );
                       PanCtrlMotorDriverS();           
                  }
            }//end_else if (( max_angle_value <= light_step1) && (max_angle_value <= light_step2 ))
          else {;}
   
   }// end_if ( Password_OK == 0 )
    delay( 100);
 } // end Loop()

 
 // 정 주행 Forward
int PanCtrlMotorDriverF( int sp )
 {
    
   digitalWrite(M1IN1, HIGH);  // 정방향
   digitalWrite(M1IN2, LOW);
   digitalWrite(M2IN1, HIGH);  // 정방향
   digitalWrite(M2IN2, LOW);
   
   analogWrite(M1, sp); 
   analogWrite(M2, sp); 
  }

//  후진 Backward
int PanCtrlMotorDriverB( int sp )
 {
   digitalWrite(M1IN1, LOW);  // Back
   digitalWrite(M1IN2, HIGH);
   digitalWrite(M2IN1, LOW);  // Back
   digitalWrite(M2IN2, HIGH);

   analogWrite(M1, sp);  
   analogWrite(M2, sp); 
  }

// Left
int PanCtrlMotorDriverR(int sp )
 {
   digitalWrite(M1IN1, LOW);  // Back
   digitalWrite(M1IN2, HIGH);
   digitalWrite(M2IN1, HIGH);  // 정방향
   digitalWrite(M2IN2, LOW);
   
   analogWrite(M1, sp); 
   analogWrite(M2, sp);
  }
// Right
int PanCtrlMotorDriverL(int sp )
 {
   digitalWrite(M1IN1, HIGH);  // Forward
   digitalWrite(M1IN2, LOW);
   digitalWrite(M2IN1, LOW);  // Backward
   digitalWrite(M2IN2, HIGH);
   
   analogWrite(M1, sp); 
   analogWrite(M2, sp);
  }
// Stop
int PanCtrlMotorDriverS()
 {
//   analogWrite(M1, LOW); 
//   analogWrite(M2, LOW);

   digitalWrite(M1IN1, LOW);  // Forward
   digitalWrite(M1IN2, LOW);
   digitalWrite(M2IN1, LOW);  // Backward
   digitalWrite(M2IN2, LOW);
  }


 // 정 주행 Forward
int PanCtrlMotorDriverFd( int sp, int time_delay )
 {
    
   PanCtrlMotorDriverF( sp );
   delay( time_delay);
   PanCtrlMotorDriverS();
  }

//  후진 Backward
int PanCtrlMotorDriverBd( int sp, int time_delay )
 {
   PanCtrlMotorDriverB( sp ); 
   delay( time_delay);
   PanCtrlMotorDriverS();
  }

// Left
int PanCtrlMotorDriverRd(int sp, int time_delay )
 {
   PanCtrlMotorDriverR( sp );
   delay( time_delay);
   PanCtrlMotorDriverS();
  }
// Right
int PanCtrlMotorDriverLd(int sp, int time_delay )
 {
   PanCtrlMotorDriverL( sp );
   delay( time_delay);
   PanCtrlMotorDriverS();
  }

void PanCtrlMotorDriverFBd_if(int f_b)
 {
   if ( f_b == 1) 
     {
       if( distance_F1 <= 30 ) // 진행방향의 장애물이 20Cm 이내에 있을경우
         {   
            dis_Errorcode = -1; //에러 표시
            Password_OK = 10; //  리모콘 모드 : 10,   자동모드 : 0
            DisplayLCDValue(t1, h1, soil_h2, "Err" );
            PanCtrlMotorDriverS();
            DisplayLED(11); // int rgb 11은 Red 임.  
                                          
         }
       else if(distance_F1 > 110)
            {
              PanCtrlMotorDriverFd( speed_F, Motor_drive_timeM *3 ); // 정주행
            }
       else if(distance_F1 > 80 && distance_F1 <= 110)
            {
              PanCtrlMotorDriverFd( speed_F, Motor_drive_timeM *2 ); // 정주행
            }
       else{ PanCtrlMotorDriverFd( speed_F, Motor_drive_timeM);} // 정주행
     }
    else if ( f_b == 2) // 2번 방향(후진방향)
     {
      if( distance_F2 <= 30 ) // 후진방향의 장애물이 20Cm 이내에 있을경우
        {   
            dis_Errorcode = -1; //에러 표시
            Password_OK = 10; //  리모콘 모드 : 10,   자동모드 : 0
            DisplayLCDValue(t1, h1, soil_h2, "Err" );
            PanCtrlMotorDriverS();
            DisplayLED(11); // int rgb 11은 Red 임.  
 //           delay(5000);                                             
        }
     else if(distance_F2 > 110)
            {
              PanCtrlMotorDriverBd( speed_F, Motor_drive_timeM *3 ); // 정주행
            }
     else if(distance_F2 > 80 && distance_F2 <= 110)
            {
              PanCtrlMotorDriverBd( speed_F, Motor_drive_timeM *2 ); // 정주행
            }
     else {PanCtrlMotorDriverBd( speed_F, Motor_drive_timeM );} // 후진
    }
 }

void DisplayLED(int rgb)
   {
     
  /* // LED 조명이 조도센세에 영향을 줌에 따라서 임시 정지 함.
        if ( rgb == 11)
          {digitalWrite(LED_R1, HIGH); digitalWrite(LED_G1, LOW); digitalWrite(LED_B1, LOW);}
        else if ( rgb == 12)
          {digitalWrite(LED_R1, LOW); digitalWrite(LED_G1, HIGH); digitalWrite(LED_B1, LOW);} 
        else if ( rgb == 13)
          {digitalWrite(LED_R1, LOW); digitalWrite(LED_G1, LOW); digitalWrite(LED_B1, HIGH);}  
        else if ( rgb == 20) // ALL
          {digitalWrite(LED_R1, HIGH); digitalWrite(LED_G1, HIGH); digitalWrite(LED_B1, HIGH);} 
        else if ( rgb == 10) // OFF
          {digitalWrite(LED_R1, LOW); digitalWrite(LED_G1, LOW); digitalWrite(LED_B1, LOW);} 
   */    
   }  

    

// 초음파 센서로 거리측정하는 함수
int HCSR04distance(int num)
    {
 double duration = 0;
 
      if( num == 1)
        {
          digitalWrite(Trig_1, LOW);
          delayMicroseconds(3);   
            
          digitalWrite(Trig_1, HIGH);
          delayMicroseconds(10);
          digitalWrite(Trig_1, LOW);
             // 초음파 신호 수신
         duration = pulseIn(Echo_1,HIGH);
              // 거리계산 
         distance_F1 = int(duration*340/10000/2);  
         Serial.print("distance_F1 = ");       Serial.println(distance_F1);
        }
     else if( num == 2)
        {
          digitalWrite(Trig_2, LOW);
          delayMicroseconds(3);   
            
          digitalWrite(Trig_2, HIGH);
          delayMicroseconds(10);
          digitalWrite(Trig_2, LOW);
             // 초음파 신호 수신
         duration = pulseIn(Echo_2,HIGH);
              // 거리계산 
         distance_F2 = int (duration*340/10000/2); 
         Serial.print("distance_F2 = ");       Serial.println(distance_F2);
        }
       else{;} 
    }

 // 조도센서 값을 읽어오는 함수
int read_Light_Value(int num) 
 {
  int sensorValue = 0;
  int light_Sensor1 = 0;
  int light_Sensor2 = 0;
   
   if( num == 1)
   {
//    sensorValue = analogRead(Light_sensor1);
    light_Sensor1 = map(analogRead(Light_sensor1), 0, 1023, 100, 0); //0~1023을  0~100%로 변환
    return light_Sensor1;
    
//    Serial.print("light_Sensor1 =");       Serial.println(light_Sensor1);
      return light_Sensor1;
   }
  else if( num == 2)
   {
//    sensorValue = analogRead(Light_sensor2);
    light_Sensor2 = map(analogRead(Light_sensor2), 0, 1023, 100, 0); //0~1023을  0~100%로 변환
//    Serial.print("light_Sensor2 =");       Serial.println(light_Sensor2);
    return light_Sensor2;
   }
  
  return 1;
 }
 
// 조도센서의 값으로 밝은 결정함.  30도 각도로 서보모터 방향전환하면 찾음.
int Foward_Max_Light_dir()
{
    int dir[12]={0,0,0,0,0,0,0,0,0,0,0,0};
    int max_angle = 0;        // 가장밝은 방향을 찾기위한 변수.
    int temp_angle_value = 0;  // 가장 밝은 방향의 조도 값.
        
    for( int i = 0; i < 6; i++)
     {
        dir[i] = read_Light_Value(1);
        dir[i+6] = read_Light_Value(2);
//        myservo.write(i*30,50,false);
        myservo.write(i*30,50,true); 

         delay(800);    
     }
     
//    myservo.write(0,50,false); // 서보모터 원래상대로 돌려놓음.
//    myservo.write(0,50,true); // 서보모터 원래상대로 돌려놓음.
    for( int j = 0; j <12; j++)
    {
      // 현재의 조도값이 지난번의 초고 큰 조도값 보다 클 경우, 현재값을 선택함.
      if (dir[j] > temp_angle_value )
        {
           max_angle = j;
           temp_angle_value = dir[j];
        }
      // 현재의 조도값과 지난번의 최고 큰 조도값 이 동일할 경우
      // 1) 전후 3개의 조도값을 합계해서 큰것을 선택
      // 2) 3개 합계값이 같을경우, 지나번 최고값의 이전각도 조도값과 현재각도의 다음각도의 조도값을 비교
      else if (dir[j] == temp_angle_value ) //
        {
            int j_After = j+1;  // 현재의 각도 순서 보다 45도 큰 각도의 순서
            int max_angle_Before = max_angle -1;  // 앞에 저장된 가장큰 조도의 각도 순서 보다 앞전 각도순서
            if ((j_After) > 11) { j_After = 0; }  // 현재순서에 다음순서가 8이면 0으로 대입
            if ((max_angle_Before) < 0) { max_angle_Before = 11; } // 지난번최고값 순서 보다 앞전수서가 0보다 적으면..
                
            int Sum_max_angle = dir[max_angle_Before]+dir[max_angle]+dir[max_angle+1];
             // j번째(현재순번) 값의 전후 3개의 값을 합산함.
            int Sum_j = dir[j-1]+dir[j]+dir[j_After];  // 전후 3개의 조도값을 합해서 비교
            

           if( Sum_j > Sum_max_angle)
             {
                max_angle = j;
                temp_angle_value = dir[j];
             }
           else if( Sum_j < Sum_max_angle)
             {
                max_angle = max_angle;
                temp_angle_value = temp_angle_value;
             }
           
           else if (Sum_j == Sum_max_angle)
            {
               
               if( dir[j_After] > dir[ max_angle_Before])
                   {
                      max_angle = j;
                      temp_angle_value = dir[j];
                   }
               else if (dir[j_After] == dir[max_angle_Before])
                  {
                      int j_before = j-1;
                      int max_angle_after = max_angle +1;
                      if ((j_before) < 0) { j_before = 11; }
                      if (max_angle_after > 11) { max_angle_after = 0; }

                      if( dir[j_before] > dir[ max_angle_after])
                      {
                         max_angle = j;
                         temp_angle_value = dir[j];
                      }
                  }
            }// end_else if (Sum_j == Sum_max_angle)
         }// end_else if (dir[j] == temp_angle_value )
    }// end_for( int j = 0; j <8; j++)
    
   Serial.print("0도 =");       Serial.println(dir[0]);
   Serial.print("30도 =");       Serial.println(dir[1]);
   Serial.print("60도 =");       Serial.println(dir[2]);
   Serial.print("90도 =");       Serial.println(dir[3]);
   Serial.print("120도 =");       Serial.println(dir[4]);
   Serial.print("150도 =");       Serial.println(dir[5]);
   Serial.print("180도 =");       Serial.println(dir[6]);
   Serial.print("210도 =");       Serial.println(dir[7]);
   Serial.print("240도 =");       Serial.println(dir[8]);
   Serial.print("270도 =");       Serial.println(dir[9]);
   Serial.print("300도 =");       Serial.println(dir[10]);
   Serial.print("330도 =");       Serial.println(dir[11]);
   
   
   Serial.print("최고값 각도 = ");     Serial.print(max_angle *30); 
   Serial.print("최고값 조도 = ");     Serial.println(temp_angle_value);
     max_angle_value1 = temp_angle_value; //지역변수의 값을 전역변수의 max_angle_valu1e 대체함.
     
     return max_angle;    // RC Car는 ( max_angle * 30 ) 각도에서 가장 밝은 빛이 감지됨.
  }

// 조도센서의 값으로 밝은 결정함.  30도 각도로 리버스 서보모터 방향전환하면 찾음.
int Reverse_Max_Light_dir()
{
    int dir[12]={0,0,0,0,0,0,0,0,0,0,0,0};
    int max_angle = 0;        // 가장밝은 방향을 찾기위한 변수.
//    int max_angle_value = 0;  // 가장 밝은 방향의 조도 값.
    int temp_angle_value = 0;  // 가장 밝은 방향의 조도 값.
        
    for( int i = 5; i >= 0; i--)
     {
        dir[i] = read_Light_Value(1);
        dir[i+6] = read_Light_Value(2);
//        myservo.write(i*30,50,false);
        myservo.write(i*30,50,true); 

         delay(800);    
     }
     
//    myservo.write(0,50,false); // 서보모터 원래상대로 돌려놓음.
    myservo.write(0,50,true); // 서보모터 원래상대로 돌려놓음.
    for( int j = 0; j <12; j++)
    {
      // 현재의 조도값이 지난번의 초고 큰 조도값 보다 클 경우, 현재값을 선택함.
      if (dir[j] > temp_angle_value )
        {
           max_angle = j;
           temp_angle_value = dir[j];
        }
      // 현재의 조도값과 지난번의 최고 큰 조도값 이 동일할 경우
      // 1) 전후 3개의 조도값을 합계해서 큰것을 선택
      // 2) 3개 합계값이 같을경우, 지나번 최고값의 이전각도 조도값과 현재각도의 다음각도의 조도값을 비교
      else if (dir[j] == temp_angle_value ) //
        {
            int j_After = j+1;  // 현재의 각도 순서 보다 45도 큰 각도의 순서
            int max_angle_Before = max_angle -1;  // 앞에 저장된 가장큰 조도의 각도 순서 보다 앞전 각도순서
            if ((j_After) > 11) { j_After = 0; }  // 현재순서에 다음순서가 8이면 0으로 대입
            if ((max_angle_Before) < 0) { max_angle_Before = 11; } // 지난번최고값 순서 보다 앞전수서가 0보다 적으면..
                
            int Sum_max_angle = dir[max_angle_Before]+dir[max_angle]+dir[max_angle+1];
             // j번째(현재순번) 값의 전후 3개의 값을 합산함.
            int Sum_j = dir[j-1]+dir[j]+dir[j_After];  // 전후 3개의 조도값을 합해서 비교
            

           if( Sum_j > Sum_max_angle)
             {
                max_angle = j;
                temp_angle_value = dir[j];
             }
           else if( Sum_j < Sum_max_angle)
             {
                max_angle = max_angle;
                temp_angle_value = temp_angle_value;
             }
           
           else if (Sum_j == Sum_max_angle)
            {
               
               if( dir[j_After] > dir[ max_angle_Before])
                   {
                      max_angle = j;
                      temp_angle_value = dir[j];
                   }
               else if (dir[j_After] == dir[max_angle_Before])
                  {
                      int j_before = j-1;
                      int max_angle_after = max_angle +1;
                      if ((j_before) < 0) { j_before = 11; }
                      if (max_angle_after > 11) { max_angle_after = 0; }

                      if( dir[j_before] > dir[ max_angle_after])
                      {
                         max_angle = j;
                         temp_angle_value = dir[j];
                      }
                  }
            }// end_else if (Sum_j == Sum_max_angle)
         }// end_else if (dir[j] == temp_angle_value )
    }// end_for( int j = 0; j <8; j++)
    
   Serial.print("0도 =");       Serial.println(dir[0]);
   Serial.print("30도 =");       Serial.println(dir[1]);
   Serial.print("60도 =");       Serial.println(dir[2]);
   Serial.print("90도 =");       Serial.println(dir[3]);
   Serial.print("120도 =");       Serial.println(dir[4]);
   Serial.print("150도 =");       Serial.println(dir[5]);
   Serial.print("180도 =");       Serial.println(dir[6]);
   Serial.print("210도 =");       Serial.println(dir[7]);
   Serial.print("240도 =");       Serial.println(dir[8]);
   Serial.print("270도 =");       Serial.println(dir[9]);
   Serial.print("300도 =");       Serial.println(dir[10]);
   Serial.print("330도 =");       Serial.println(dir[11]);
   
   
   Serial.print("최고값 각도 = ");     Serial.print(max_angle *30); 
   Serial.print("최고값 조도 = ");     Serial.println(temp_angle_value);
     max_angle_value2 = temp_angle_value; //지역변수의 값을 전역변수의 max_angle_value2 대체함.
     
     return max_angle;    // RC Car는 ( max_angle * 30 ) 각도에서 가장 밝은 빛이 감지됨.
  }

// 서보모터를 반시계방향으로 180도 회전하면서 조도를 읽고, 시계방향으로 회전하면서 조도를 읽은후 각도계산
int Max_Light_dir() 
  {
       max_angle_value = (max_angle_value1 + max_angle_value2)/2;
       int avr = int (Foward_Max_Light_dir() + Reverse_Max_Light_dir())/2;
       Serial.print("평균 밝기 각도 = ");     Serial.print(avr *30); 
       return avr;
  }

  // 토양습도 센서에서 값을 읽어오는 함수
int read_soil_humidity() 
 {
  int rain_sensorValue = 0;
  rain_sensorValue = analogRead(SH_sensor);  // 0 ~ 1023
//  Serial.print(" 토양 센서값 : "); Serial.println(rain_sensorValue);
  soil_h2 = map(rain_sensorValue, 0, 1023, 100, 0); //0~1023을  0~100%로 변환 
 
  return 1;
 }

// 온습도 센서에서 값을 읽어오는 함수
int read_Humidity_Temperature() 
     {
       h1 = dht1.readHumidity();

    // Read temperature as Celsius (the default)
       t1 = dht1.readTemperature();

    // Read temperature as Fahrenheit (isFahrenheit = true)
       f1 = dht1.readTemperature(true);


  // Check if any reads failed and exit early (to try again).
      if (isnan(h1) || isnan(t1) || isnan(f1)) {

      return -1;
      }
      //      Serial.print("Temperature: ");
//      Serial.print(" 온습도 센서 ");
//     Serial.print(t1);
//      Serial.print(F("°C    ")); 

//      Serial.print(F("Humidity: "));
//      Serial.println(h1);
      
      return 1;
     }
  
  // LCD에 Display 하는 함수 ( 16x2 )
void DisplayLCD(const char *buff1, const char *buff2)
   {
     lcd.init();
//     lcd.backlight();
     lcd.setCursor(0, 0); 
     lcd.print(buff1);
     lcd.setCursor(0, 1); 
     lcd.print(buff2);
   }
// LCD에 Display 하는 함수 ( 16x2 )


// 온도 습도를 LCD에 표시
void DisplayLCDValue(int Num1, int Num2, int Num3, const char *buff )  //
   {
       lcd.init();
       lcd.setCursor(0, 0); 
       lcd.print("H:   ");
       lcd.print(Num2);        // 스마트팜 습도
       lcd.print("%");
       
       lcd.setCursor(10, 0); 
       lcd.print("T: ");
       lcd.print(Num1);       // 스마트팜 온도
       lcd.print("C");

       lcd.setCursor(0, 1); 
       lcd.print("S_H: "); 
       lcd.print(Num3);       // 토양 습도 
       lcd.print("%");

       lcd.setCursor(11, 1);  
//       lcd.print("M:"); 
       lcd.print(buff);       // 리모컨 모드 (R), 자동모드 표시(A), Error (E)
//       lcd.print("%");      
   } 
