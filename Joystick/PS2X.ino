#include <PS2X_lib.h>
#include <LiquidCrystal.h>
#include <Servo.h>
#define PS2_DAT        11//51  //14 miso   
#define PS2_CMD        12//50  //15 mosi
#define PS2_SEL        10//53  //16 ss
#define PS2_CLK        13//52  //17 sck
#define pressures   false
#define rumble      false

Servo myservo;
const int rs = A5, rw=3, en = A4, d4 = 4, d5 = A2, d6 = A3, d7 = A1;
int Stick = 0,LX,LY,RX,RY;
byte type = 0;
byte vibrate = 0;
int  DataKirim =00;
PS2X ps2x; 
LiquidCrystal lcd(rs,rw, en, d4, d5, d6, d7);
void setup(){
 //repair:
  Serial.begin(9600);
  myservo.attach(6);
  lcd.begin(20, 4);
  repair:
  delay(300);  //tunggu pairing wirless
   
  //setup pins and settings: GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error
 
  Stick = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
  
  if(Stick == 0){
    Serial.print("Found Controller");
    lcd.setCursor(0, 0);
    lcd.print("Stick Siap");}
    else {
    Serial.print("Not Found Controller");
    lcd.setCursor(0, 0);
    lcd.print("Stick belum Siap");
   goto  repair;}
}

void loop() { 
   //Serial.println(int(DataKirim));
   lcd.setCursor(12, 1);
   lcd.print(int(DataKirim));
   if(Stick == 1) //repairing terus kalau gagal pair
    return; 
    
  else { //DualShock Controller
    ps2x.read_gamepad(true, vibrate);
      lcd.setCursor(0, 1);
       LY=(ps2x.Analog(PSS_LY));
       LX=(ps2x.Analog(PSS_LX));
       RY=(ps2x.Analog(PSS_RY));
       RX=(ps2x.Analog(PSS_RX));
      if(ps2x.Button(PSB_START))      {Serial.print(int(DataKirim));  lcd.print("start");   DataKirim=101;   myservo.write(180);    delay(15);} else
      if(ps2x.Button(PSB_SELECT))     {Serial.print(int(DataKirim));  lcd.print("Select");  DataKirim=102;   } else    
      if(ps2x.Button(PSB_PAD_UP))     {Serial.print(int(DataKirim));  lcd.print("UP");      DataKirim=103;   } else
      if(ps2x.Button(PSB_PAD_RIGHT))  {Serial.print(int(DataKirim));  lcd.print("Right");   DataKirim=104;   } else
      if(ps2x.Button(PSB_PAD_LEFT))   {Serial.print(int(DataKirim));  lcd.print("left");    DataKirim=105;   } else
      if(ps2x.Button(PSB_PAD_DOWN))   {Serial.print(int(DataKirim));  lcd.print("Down");    DataKirim=106;   } else   
      if(ps2x.Button(PSB_L1))         {Serial.print(int(DataKirim));  lcd.print("L1");      DataKirim=107;   } else 
      if(ps2x.Button(PSB_L2))         {Serial.print(int(DataKirim));  lcd.print("l2");      DataKirim=108;   } else 
      if(ps2x.Button(PSB_R1))         {Serial.print(int(DataKirim));  lcd.print("R1");      DataKirim=109;   } else 
      if(ps2x.Button(PSB_R2))         {Serial.print(int(DataKirim));  lcd.print("R2");      DataKirim=110;  } else 
      if(ps2x.Button(PSB_L3))         {Serial.print(int(DataKirim));  lcd.print("L3");      DataKirim=111;  } else
      if(ps2x.Button(PSB_R3))         {Serial.print(int(DataKirim));  lcd.print("R3");      DataKirim=112;  } else
      if(ps2x.Button(PSB_L2))         {Serial.print(int(DataKirim));  lcd.print("L2");      DataKirim=113;  } else
      if(ps2x.Button(PSB_R2))         {Serial.print(int(DataKirim));  lcd.print("R2");      DataKirim=114;  } else
      if(ps2x.Button(PSB_TRIANGLE))   {Serial.print(int(DataKirim));  lcd.print("Segitiga");DataKirim=115;  } else       
      if(ps2x.Button(PSB_CIRCLE))     {Serial.print(int(DataKirim));  lcd.print("O");       DataKirim=116;  } else
      if(ps2x.Button(PSB_CROSS))      {Serial.print(int(DataKirim));  lcd.print("X");       DataKirim=117;  } else
      if(ps2x.Button(PSB_SQUARE))     {Serial.print(int(DataKirim));  lcd.print("[]");      DataKirim=118;  } else 
      if(LY==0)                       {Serial.print(int(DataKirim));  lcd.print("L naik");  DataKirim=119;  } else  
      if(LY==255)                     {Serial.print(int(DataKirim));  lcd.print("L Turun"); DataKirim=120;  } else 
      if(LX==255)                     {Serial.print(int(DataKirim));  lcd.print("L kanan");  DataKirim=121;  } else  
      if(LX==0)                       {Serial.print(int(DataKirim));  lcd.print("L kiri");   DataKirim=122;  } else 
      if(RY==0)                       {Serial.print(int(DataKirim));  lcd.print("R naik");   DataKirim=123;  } else  
      if(RY==255)                     {Serial.print(int(DataKirim));  lcd.print("R Turun");  DataKirim=124;  } else 
      if(RX==255)                     {Serial.print(int(DataKirim));  lcd.print("R kanan");  DataKirim=125;  } else  
      if(RX==0)                       {Serial.print(int(DataKirim));  lcd.print("R kiri");   DataKirim=126;  } else  
         {Serial.print(int(DataKirim));lcd.print("          "); DataKirim=0;  myservo.write(85);}
       
  }
  delay(10);   
}
