
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include "stdlib.h"
#include "lcd_txt.h"
#include "math.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim12;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
//aktif LOW
#define LIMIT1 HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_6)
#define LIMIT2 HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_3)
#define LIMIT3 HAL_GPIO_ReadPin(limit8_GPIO_Port,limit8_Pin) //PE12
#define LIMIT4 HAL_GPIO_ReadPin(limit9_GPIO_Port,limit9_Pin) 
#define LIMIT5 HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_6)
#define LIMIT6 HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_4)
#define LIMIT7 HAL_GPIO_ReadPin(limit5_GPIO_Port,limit5_Pin)
#define LIMIT8 HAL_GPIO_ReadPin(limit6_GPIO_Port,limit6_Pin)
#define LIMIT9 HAL_GPIO_ReadPin(limit7_GPIO_Port,limit7_Pin)
#define LIMIT10 HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_7)   
#define LIMIT11 HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_5)
#define LIMIT12 HAL_GPIO_ReadPin(limit10_GPIO_Port,limit10_Pin) //Limit Maju bawah
#define LIMIT13 HAL_GPIO_ReadPin(limit11_GPIO_Port,limit11_Pin) //Limit Turun
#define LIMIT14 HAL_GPIO_ReadPin(limit12_GPIO_Port,limit12_Pin) //LIMIT naik turun griper
//////////Rumus OTW	
int jarakX,jarakY,JarakXY;
float move_XYline,rate_XYline,last_error_XYline;
double asudut,XP,YP,sudut_tampil,sudut_dipakai;
	

//////////fungsional && MENU
int Lauto=0;
int step=0,z,Halaman,serong=0;
int Xmenu=1,Ymenu=4;
int izinjalan=0;
int pepet,PepetX,SetXambil,SetYambil;
int StepPenumatik,steplempar,SyaratLempar,SyaratPickup;
int tungguPenumatik,AmbilSelsai,Lemparselesai,ambilke2,saghai;
/////////////////////PID NEW
int move_line, rate_line, last_enkoder,
		move_line2, rate_line2, last_enkoder2;
///////////////PID KOMpas
int aksi_kiri_kompas, aksi_kanan_kompas,aksi_belakang_kompas, I_kompas,
    rate_kompas,      error_kompas,      last_error_kompas,     move_kompas, kecepatan_kompas; 
int kp_kompas=1, kd_kompas=1500, ki_kompas=0;

/////////////////////enkoder
char text[99];
int32_t Enc_sampling,Enc_sampling2,Enc_sampling3,Enc_sampling5,enkoder,enkoder2,enkoder3,enkoder5;
int32_t Enc_Count,Enc_Count2,Enc_Count3,Enc_Count5;
int32_t enkoder_PID,enkoder_PID2,enkoder_PID3,enkoder_PID5,last,last2,last3,last5;
int32_t enkoder_,enkoder_2,enkoder_3,enkoder_5;

////////////////////////GYRO
int wait_imu=0,step_imu=0,nilai_kalibrasi=0,hasildata=0;
int sudutimu=0;
int data_kompas;
int sethteha=0;
char myDataTx[5];
int imuRAZOR;
int d_kom,PosHeading=0;

/////////////odomerty
float xx1,xx2,xx3,xx4,xx5,xx6,xx7,xx8,vxx;
float akar_odometri,hasil_akar_odometri,cos_encoder_420,degcos_encoder_420,hasil_cos_encoder_420;
float tiperr,tiper;
float hasil_cos_encoder,hasil_sin_encoder;
float degsin_encoder,degcos_encoder;
float tipe,tipe1,tipe2,tipe3,tipe4;
float vx,vy,roda1,roda2,roda3,vx1,vy1;
float degcos_odometri;
float hasil_cos_odometri;
float	degsin_odometri,hasil_sin_odometri,xx11,xx12,XD,xx14,xx15,YD,x_global,y_global;

int   sin_encoder,cos_encoder;
int   v1_encoder,v2_encoder,v3_encoder;
int   sudut_theta,sudut_theta1;
int   data_kiri_kompas,data_kanan_kompas,data_belakang_kompas;
int   data_kompas;
int   cos_odometri;
int   sin_odometri;    
int   LinierX,LinierY;

float X_Rot,Y_Rot,T_Rot,T_Rad,Pos_Y,Pos_X,Heading;
float KordinatX,KordinatY;
float Kunci_X=100;


////////////////////Joystick
int atas,kanan,kiri,bawah,segitiga,bulat,kotak,silang;
int L1,R1,L2,R2,select,start;
int Lnaik,Lturun,Lkanan,Lkiri,Rnaik,Rturun,Rkanan,Rkiri;
int Stick=0;
char stick[3];
int tungguHeading,Akselerasi;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM5_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM12_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_UART4_Init(void);
                                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
int16_t Get_Enc1(){
	Enc_sampling=TIM1->CNT - 65535;			 	
	TIM1->CNT=65535;	
	Enc_Count+=(Enc_sampling);
	Enc_sampling=0;	
	return Enc_Count;
}
int16_t Get_Enc3(){ //// pemasangan chanel A & B terbalik 
	Enc_sampling3=TIM3->CNT - 65535;			 	
	TIM3->CNT=65535;	
	Enc_Count3-=(Enc_sampling3);
	Enc_sampling3=0;	
	return Enc_Count3;
}
int16_t Get_Enc5(){
	Enc_sampling5=TIM5->CNT - 65535;			 	
	TIM5->CNT=65535;	
	Enc_Count5-=(Enc_sampling5);
	Enc_sampling5=0;	
	return Enc_Count5;
}
void Encoder_Init(){
	HAL_TIM_Encoder_Start_IT(&htim1,TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start_IT(&htim1,TIM_CHANNEL_2);
	Enc_sampling=0;
	TIM1->CNT=65535;
		
	HAL_TIM_Encoder_Start_IT(&htim3,TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start_IT(&htim3,TIM_CHANNEL_2);
	Enc_sampling3=0;
	TIM3->CNT=65535;
	
	HAL_TIM_Encoder_Start_IT(&htim5,TIM_CHANNEL_ALL);
	Enc_sampling2=0;
	TIM5->CNT=65535;
	
	
}
void Reset_Enc(){
	Enc_sampling=0;
	Enc_Count=0;
	TIM1->CNT=65535;  

	Enc_sampling3=0;
	Enc_Count3=0;
	TIM3->CNT=65535;  

	Enc_sampling5=0;
	Enc_Count5=0;
	TIM5->CNT=65535;  
	
}
void BacaEncoder(){	
	enkoder =  (float) Get_Enc1();
  enkoder2=  (float) Get_Enc5();
  enkoder3=  (float) Get_Enc3();
 

	enkoder_+=enkoder-last;     //ENKODER ODOMETRY
	enkoder_2+=enkoder2-last2;
	enkoder_3+=enkoder3-last3;
	
	last=enkoder;
	last2=enkoder2;
	last3=enkoder3;
	
	Reset_Enc();
	
	enkoder_PID+=enkoder_; //ENKODER KHUSHS PID
	enkoder_PID2+=enkoder_2;
	enkoder_PID3+=enkoder_3;
	
}
void Driver(int no ,int pwm){
	
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);  
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);  
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);  
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);  
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);  
  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);  
  HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
	
	 
if (no==6){	
			if(pwm>0){ 
							htim2.Instance->CCR1 =pwm;
							htim2.Instance->CCR2 =0;} 
      if(pwm<0){
							htim2.Instance->CCR1 =0;
							htim2.Instance->CCR2 =-pwm;}
      if(pwm==0) {htim2.Instance->CCR1 =0 ;htim2.Instance->CCR2 =0;}}
if (no==2){	
		 if(pwm<0){
							htim2.Instance->CCR3 =-pwm;
							htim2.Instance->CCR4 =0;} 
     if(pwm>0){
							htim2.Instance->CCR3 =0;
							htim2.Instance->CCR4 =pwm;}
     if(pwm==0) {htim2.Instance->CCR4 =0 ;htim2.Instance->CCR3 =0;}}
	
if (no==3){	
		 if(pwm>0){
							htim8.Instance->CCR1 =pwm;
							htim8.Instance->CCR2 =0;} 
     if(pwm<0){
							htim8.Instance->CCR1 =0;
							htim8.Instance->CCR2 =-pwm;}
     if(pwm==0){
htim8.Instance->CCR1 =0;htim8.Instance->CCR2 =0;}}
	 
if (no==4){	
	if(pwm>0){ //depan
      htim8.Instance->CCR4 =pwm;
	    htim8.Instance->CCR3 =0;} 
      if(pwm<0){
      htim8.Instance->CCR4 =0;
      htim8.Instance->CCR3 =-pwm;}
    if(pwm==0) {htim8.Instance->CCR3 =0;htim8.Instance->CCR4 =0;}}
	 
if (no==5){	
	if(pwm>0){ //depan
      htim9.Instance->CCR1 =pwm;
	    htim9.Instance->CCR2 =0;} 
      if(pwm<0){
      htim9.Instance->CCR1 =0;
      htim9.Instance->CCR2 =-pwm;}
    if(pwm==0) {htim9.Instance->CCR1 =0 ;htim9.Instance->CCR2 =0;}}
		
if (no==1){	
if(pwm>0){ //depan
      htim12.Instance->CCR1 =pwm;
	    htim12.Instance->CCR2 =0;} 
      if(pwm<0){
      htim12.Instance->CCR1 =0;
      htim12.Instance->CCR2 =-pwm;}
    if(pwm==0) {htim12.Instance->CCR1 =0;htim12.Instance->CCR2 =0;}}\
	 
	
	

}
void Gerak_4omni(int sudut, int Kecepatan, int Theta){

float Akar,Respon1,Respon2,Degsin,Degcos,HasilCos,HasilSin,PWMsin,PWMcos;
float Gerak_motor1,Gerak_motor2,Gerak_motor3,Gerak_motor4,PWMsinT,PWMcosT;    
int sudutCos,SudutSin;
float Jarak_roda_ke_tengah=30,PWMsinI,PWMcosI,RESPON1,RESPON2,RESPON3,RESPON4;
void pid_kompas();

sudutCos=sudut;           //input nilai Sudut
Degcos=(sudutCos/57.315); //rubah Deg ke Rad
HasilCos=cos(Degcos) ;

SudutSin=sudut;           //input nilai Sudut
Degsin=(SudutSin/57.315); //rubah Deg ke Rad
HasilSin=sin(Degsin) ;
Akar=sqrt(3);             //akar  3

//Rumus perbandingan kecepatan dengan nilai sudut heading
Respon1=((-1/4*HasilCos)+((1/Akar)*HasilCos));
Respon2=((-1/4*HasilSin)+((1/Akar)*HasilSin));

PWMsin=(((Respon1*Kecepatan)+((Jarak_roda_ke_tengah/2)*Theta))*1.8);
PWMcos=(((Respon2*Kecepatan)-((Jarak_roda_ke_tengah/2)*Theta))*1.8);

PWMsinI=(((Respon1*Kecepatan)-((Jarak_roda_ke_tengah/2)*Theta))*1.8);
PWMcosI=(((Respon2*Kecepatan)+((Jarak_roda_ke_tengah/2)*Theta))*1.8);

PWMsinT=((Respon1*Kecepatan)*1.8);
PWMcosT=((Respon2*Kecepatan)*1.8);
//Penerapan perbandingan di Motor (hasil berupa PWM 0 ~ 1000)

Gerak_motor2=PWMsin;//I
Gerak_motor3=PWMcos;//I
Gerak_motor4=-PWMsin;
Gerak_motor1=-PWMcos;//t

RESPON1=(Gerak_motor1-(aksi_kiri_kompas));
RESPON2=(Gerak_motor2-(-aksi_kanan_kompas));
RESPON3=(Gerak_motor3-(aksi_kiri_kompas));
RESPON4=(Gerak_motor4-(-aksi_kanan_kompas));

Driver(1,RESPON1);
Driver(2,RESPON2);
Driver(3,RESPON3);
Driver(4,RESPON4);

    //sprintf(text,"%4d",(int)(RESPON1 ));lcd_puts(2,0,(int8_t* )text);
    //sprintf(text,"%4d",(int)(RESPON2 ));lcd_puts(2,5,(int8_t* )text);
    //sprintf(text,"%4d",(int)(RESPON3 ));lcd_puts(3,5,(int8_t* )text);
    //sprintf(text,"%4d",(int)(RESPON4 ));lcd_puts(3,0,(int8_t* )text);
//    
}



void pid_kompas (){
//void tuning_kompas();
 void IMUSAKTI();
 int sudutsusah,SensorSudut;
  
    SensorSudut=hasildata-PosHeading;
    if(SensorSudut<=180)
		{
		rate_kompas = SensorSudut - last_error_kompas;
    last_error_kompas = SensorSudut ; 			//3.3								//0.6
	  move_kompas = (float) ((SensorSudut * 3.4) + (rate_kompas * 0.7));

		if(move_kompas>50){move_kompas=65;} //80
    if(move_kompas<-50){move_kompas=-65;}	
	 
	  aksi_kiri_kompas     =  move_kompas*5.1;//3.4
    aksi_kanan_kompas    =  move_kompas*5.1; 
	  }
		
		if(SensorSudut>180)
	  {
		sudutsusah=SensorSudut-360;
		rate_kompas = sudutsusah - last_error_kompas;
    last_error_kompas = sudutsusah ; 
	  move_kompas = (float) ((sudutsusah * 3.4) + (rate_kompas * 0.7));
    
    if(move_kompas>50){move_kompas=65;}
    if(move_kompas<-50){move_kompas=-65;}
		 
		aksi_kiri_kompas     =  move_kompas*5.1;
    aksi_kanan_kompas    =  move_kompas*5.1; 
    }
		
//		Driver(1,-aksi_kiri_kompas);
//    Driver(2,aksi_kanan_kompas);
//    Driver(3,-aksi_kiri_kompas);
//		Driver(4,aksi_kanan_kompas);
}
void Rotasi_Lambat (){
//void tuning_kompas();
 void IMUSAKTI();
 int sudutsusah,SensorSudut;
  
    SensorSudut=hasildata-PosHeading;
    
	 if(SensorSudut<=180)
		{
		rate_kompas = SensorSudut - last_error_kompas;
    last_error_kompas = SensorSudut ; 			//3.3								//0.6
	  move_kompas = (float) ((SensorSudut * 3.4) + (rate_kompas * 0.7));

		if(move_kompas>50){move_kompas=65;} //80
    if(move_kompas<-50){move_kompas=-65;}	
	 
	  aksi_kiri_kompas     =  move_kompas*2.1;//3.4
    aksi_kanan_kompas    =  move_kompas*2.1; 
	  }
		
		if(SensorSudut>180)
	  {
		sudutsusah=SensorSudut-360;
		rate_kompas = sudutsusah - last_error_kompas;
    last_error_kompas = sudutsusah ; 
	  move_kompas = (float) ((sudutsusah * 3.4) + (rate_kompas * 0.7));
    
    if(move_kompas>50){move_kompas=65;}
    if(move_kompas<-50){move_kompas=-65;}
		 
		aksi_kiri_kompas     =  move_kompas*2.1;
    aksi_kanan_kompas    =  move_kompas*2.1; 
    }
		
		Driver(1,-aksi_kiri_kompas);
    Driver(2,aksi_kanan_kompas);
    Driver(3,-aksi_kiri_kompas);
		Driver(4,aksi_kanan_kompas);
}
void tuning_Theta(){
//void kalibrasi_IMU();

		data_kompas=hasildata;
if (data_kompas<350 && data_kompas>180){sethteha=3;} else
if (data_kompas==350){sethteha=3;} else
if (data_kompas==351){sethteha=2;} else
if (data_kompas==352){sethteha=2;} else
if (data_kompas==353){sethteha=2;} else
if (data_kompas==354){sethteha=2;} else
if (data_kompas==355){sethteha=2;} else
if (data_kompas==356){sethteha=1;} else
if (data_kompas==357){sethteha=1;} else
if (data_kompas==358){sethteha=1;} else
if (data_kompas==359){sethteha=1;} else
if (data_kompas==0){sethteha=0;} else
if (data_kompas==1){sethteha=1;} else
if (data_kompas==2){sethteha=1;} else
if (data_kompas==3){sethteha=1;} else
if (data_kompas==4){sethteha=1;} else
if (data_kompas==5){sethteha=1;} else
if (data_kompas==6){sethteha=1;} else
if (data_kompas==7){sethteha=2;} else
if (data_kompas==8){sethteha=2;} else
if (data_kompas==9){sethteha=2;} else
if (data_kompas==10){sethteha=3;} else
if (data_kompas>10 && data_kompas<=180){sethteha=3;}

}
void IMUSAKTI_int(){
  //HAL_UART_Init(&huart2);
	
	HAL_UART_Receive_DMA(&huart2,(uint8_t*)myDataTx,5);  ///WAJIB PAKAI DMA DI RECIVE nya 
	imuRAZOR = atof(myDataTx);		 

	if(imuRAZOR>720)
	{imuRAZOR=0;}	
}
void kalibrasi_IMU(){
IMUSAKTI_int();
int erorHed;
void Serial_stick();
if(step_imu==0)  {wait_imu++;}
if(wait_imu>30) {step_imu=1;} //tunggu 20 detik
if(step_imu==1 || silang==1)  
{
nilai_kalibrasi=imuRAZOR;
wait_imu=0;
step_imu=2;
enkoder_PID=0;
enkoder_PID2=0;
enkoder_PID3=0;
PosHeading=0;
}
if(step_imu==2)  {hasildata=imuRAZOR-nilai_kalibrasi;}
if(hasildata<0)  {hasildata=360+hasildata;}
}
void motion_balance (int data_input, int Tspeed){ 
int Ttheta;
Ttheta=sethteha;
kalibrasi_IMU();
tuning_Theta();
//sprintf(text,"%3d",(Ttheta));lcd_puts(1,7,(int8_t*)text);
sprintf(text,"%4d",(d_kom ));lcd_puts(2,9,(int8_t* )text);
sprintf(text,"%4d",(Ttheta ));lcd_puts(3,9,(int8_t* )text);

sudutimu = hasildata;	
//if(sudutimu<0){sudutimu=0;}
if(hasildata==0)                    								{Gerak_4omni(data_input,Tspeed,0);}
    else if((hasildata>0)&&(hasildata<1))           {Gerak_4omni(data_input,Tspeed,0);}
    else if((hasildata>359)&&(hasildata<360))       {Gerak_4omni(data_input,Tspeed,0);}
    
		else if((hasildata>180)&&(hasildata<=358)){                                                                                                                                       
        
							if(data_input>=0 && data_input<=90)
								{
									d_kom=(data_input+(360-hasildata));
									Gerak_4omni(d_kom,Tspeed,Ttheta);
									} 
				
							else 								if(data_input>90 && data_input<=180)
								
								{
									d_kom=(data_input + (360-hasildata));
									Gerak_4omni(d_kom,Tspeed,-Ttheta);
								}	 
			
							else if(data_input>180 && data_input<=270)
								{
									d_kom=(data_input+(360-hasildata));
									Gerak_4omni(d_kom,Tspeed,-Ttheta);
								} 
							  
							else if(data_input>270 && data_input<=360)
								
								{
									d_kom=(data_input+(360-hasildata));
									Gerak_4omni(d_kom,Tspeed,Ttheta);
								}
         				
   }
    else if ((hasildata>=2) && (hasildata<=180)){  
		    
			if(data_input>=0 && data_input<=90)
		    {
						 d_kom=(data_input-hasildata);		
					   Gerak_4omni(d_kom,Tspeed,-Ttheta);
        }
				
				else 	if (data_input>90 && data_input<=180)
			  {
						d_kom=(data_input-hasildata);
						Gerak_4omni(d_kom,Tspeed,Ttheta);
				}
				
				else if(data_input>180 && data_input<=270)
		    {
						 d_kom=(data_input-hasildata);		
					   Gerak_4omni(d_kom,Tspeed,Ttheta);
        }
				
				else if (data_input>270 && data_input<=360)
			  {
						d_kom=(data_input-(hasildata));
						Gerak_4omni(d_kom,Tspeed,-Ttheta);
				}
		}
}

void odometri(){  	
	//------------------------------------------
	  cos_encoder=(60);//input cos
    degcos_encoder=(cos_encoder/57.325);
    hasil_cos_encoder=cos(degcos_encoder);

    sin_encoder=60;//input sin
    degsin_encoder=(sin_encoder/57.325);
    hasil_sin_encoder=sin(degsin_encoder);
    //----------------akar odometry------------
    akar_odometri=3;
    hasil_akar_odometri=sqrt(akar_odometri);

	///instalasi enkoder  
		v1_encoder=enkoder;          //data encoder v1  //kiri
    v2_encoder=enkoder2;         //data encoder v2  //kanan
    v3_encoder=enkoder3;         //data encoder v3  //depan

//		roda1=((enkoder2/45.5)*6.9);
//    roda2=((enkoder3/45.5)*6.9);
//    roda3=((enkoder/45.5)*6.9);  
//    sudut_theta=((roda1+roda2+roda3)/3);
//		
//		sudut_theta1+=sudut_theta;
//   
	  Reset_Enc();
		//----------------vx--------------------------
    tipe=(v3_encoder*hasil_cos_encoder);
    tiper=(v1_encoder*hasil_cos_encoder);
    tiperr=(tipe-tiper); //187.2 = 52cm //.6.24==30

    tipe1=(tiperr-(v2_encoder*hasil_cos_encoder_420));
    vx=(tipe1/100);              //data per cm(74.9)
    vx1+=vx;
		//----------------vy--------------------------
    tipe3=(v1_encoder*hasil_sin_encoder);
    tipe2=(v2_encoder*hasil_sin_encoder);
    
		tipe4=((tipe2-tipe3)/(100));
    vy=(tipe4);
    vy1+=vy;
		
    cos_odometri=hasildata;//sudut cos
    degcos_odometri=(cos_odometri/57.325);
    hasil_cos_odometri=cos(degcos_odometri);
    
    sin_odometri=hasildata;//sudut cos
    degsin_odometri=(sin_odometri/57.325);
    hasil_sin_odometri=sin(degsin_odometri);
    
    xx11=(vx*hasil_cos_odometri); 
    xx12=(vy*hasil_sin_odometri);
    XD+= (xx11-xx12); 
    
    xx14=(vx*hasil_sin_odometri);
    xx15=(vy*hasil_cos_odometri);
    YD+=(xx14+xx15);  

		 x_global=(XD*10.97560);	//koordinat real time X
     y_global=(YD*10.97560);	//koordinat real time Y
LinierX = (int) x_global; 
LinierY = (int) y_global;
//-------------------------------------------------------
}
void ordoPENS(){

	//LOCAL 
	X_Rot = enkoder_PID3 - (enkoder_PID*0.5f) - (enkoder_PID2*0.5f);
	Y_Rot = (enkoder_PID*0.866f) - (enkoder_PID2*0.866f);
	

//=====================World Frame=============================	
	if (hasildata>180){Heading = (double)(hasildata-360) / 100;}
	
  if (hasildata<180){Heading = (double)(hasildata) / 100;}
	
	if(Heading==0)	T_Rad=0;
	
	else 						T_Rad=(double)(Heading/57.325);	 
	
	Pos_X=((double) X_Rot * cos(T_Rad) + (double)Y_Rot * sin(T_Rad))*-1;
	Pos_Y=(double) Y_Rot * cos(T_Rad) - (double)X_Rot * sin(T_Rad);
	
	KordinatX=	(Pos_X*-0.151377);
	KordinatY=	(Pos_Y*0.151377);
	
}
void risetORDO(){
KordinatX=0;
KordinatY=0;
Pos_X=0;
Pos_Y=0;
enkoder_PID=0;	
enkoder_PID2=0;
enkoder_PID3=0;
}
void PID_lockX(int kecepatan){

pid_kompas();
float  kpx=0.8,kdy=0.2,aksi_kiri_Xline,aksi_kanan_Xline;
float  rate_Xline,error_Xline,last_error_Xline;
float  Xrespon1,Xrespon2,Xrespon3,Xrespon4;	
	 
if (kecepatan<=300){kpx=0.8,kdy=0.2;}
else if (kecepatan>300){kpx=1.1,kdy=0.3;}
	
 	  rate_Xline = KordinatX - last_error_Xline;
    last_error_Xline = KordinatX ;
    move_line = (int) ((KordinatX * kpx) + (rate_line * kdy))  ;

    aksi_kiri_Xline   =   kecepatan + move_line;   //+5
    aksi_kanan_Xline  =   -kecepatan + move_line;     //+5

    if (aksi_kiri_Xline>700)  {aksi_kiri_Xline=700;}
    if (aksi_kanan_Xline>700) {aksi_kanan_Xline=700;}

    if (aksi_kiri_Xline<-700)  {aksi_kiri_Xline=-700;}
    if (aksi_kanan_Xline<-700) {aksi_kanan_Xline=-700;}

		
Xrespon1=(-aksi_kiri_Xline-(aksi_kiri_kompas/2));
Xrespon2=(aksi_kanan_Xline-(-aksi_kanan_kompas/2));
Xrespon3=(aksi_kiri_Xline-(aksi_kiri_kompas/2));
Xrespon4=(-aksi_kanan_Xline-(-aksi_kanan_kompas/2));
		
    Driver(1,Xrespon1);
    Driver(2,Xrespon2);
    Driver(3,Xrespon3);
		Driver(4,Xrespon4);
		
//		sprintf(text,"%4d",(int)(-aksi_kiri_Xline ));lcd_puts(2,0,(int8_t* )text);
//    sprintf(text,"%4d",(int)(aksi_kanan_Xline ));lcd_puts(2,5,(int8_t* )text);
//    sprintf(text,"%4d",(int)(-aksi_kiri_Xline ));lcd_puts(3,5,(int8_t* )text);
//    sprintf(text,"%4d",(int)(aksi_kanan_Xline ));lcd_puts(3,0,(int8_t* )text);
//    
}

void PID_lockY(int kecepatan){
pid_kompas();

float  kpx=0.8,kdy=0.2,aksi_kiri_Yline,aksi_kanan_Yline;
float  rate_Yline,error_Yline,last_error_Yline;
float  Yrespon1,Yrespon2,Yrespon3,Yrespon4;	
	 
if (kecepatan<=300){kpx=0.8,kdy=0.2;}
else if (kecepatan>300){kpx=1.1,kdy=0.3;}
	
		rate_Yline = KordinatY - last_error_Yline;
    last_error_Yline = KordinatY ;
    move_line = (int) ((KordinatY * kpx) + (rate_line * kdy))  ;

    aksi_kiri_Yline   =   kecepatan + move_line;   //+5
    aksi_kanan_Yline  =   -kecepatan + move_line;     //+5

    if (aksi_kiri_Yline>700)  {aksi_kiri_Yline=700;}
    if (aksi_kanan_Yline>700) {aksi_kanan_Yline=700;}

    if (aksi_kiri_Yline<-700)  {aksi_kiri_Yline=-700;}
    if (aksi_kanan_Yline<-700) {aksi_kanan_Yline=-700;}

		

//		Driver(1,-aksi_kanan_Yline);
//    Driver(2,-aksi_kanan_Yline);
//    Driver(3,aksi_kiri_Yline);
//		Driver(4,aksi_kiri_Yline);
		
Yrespon1=(-aksi_kiri_Yline-(-aksi_kiri_kompas/2));
Yrespon2=(aksi_kanan_Yline-(aksi_kanan_kompas/2));
Yrespon3=(aksi_kiri_Yline-(-aksi_kiri_kompas/2));
Yrespon4=(-aksi_kanan_Yline-(aksi_kanan_kompas/2));
		
    Driver(1,-Yrespon3);
    Driver(2,-Yrespon2);
    Driver(3,-Yrespon1);
		Driver(4,-Yrespon4);
		
//		sprintf(text,"%4d",(int)(Yrespon1 ));lcd_puts(2,0,(int8_t* )text);
//    sprintf(text,"%4d",(int)(Yrespon2 ));lcd_puts(2,5,(int8_t* )text);
//    sprintf(text,"%4d",(int)(Yrespon3 ));lcd_puts(3,5,(int8_t* )text);
//    sprintf(text,"%4d",(int)(Yrespon4 ));lcd_puts(3,0,(int8_t* )text);
//    
}

void OTW(int X_target,int Y_target,int max_speed,float akselerasi){
pid_kompas();
ordoPENS();


  XP=((double)X_target);
  YP=((double)Y_target);
	
  jarakX=(KordinatX-X_target);
  jarakY=(KordinatY-Y_target);	

	asudut=((180/3.1415)*atan2((jarakX),(jarakY)));
 	
	sudut_dipakai=(asudut-45);
  if (asudut>0){sudut_tampil=sudut_dipakai;}
  if (asudut<0){sudut_tampil=360+sudut_dipakai;}
	
  if(X_target==0){JarakXY=-jarakY;}
  if(Y_target==0){JarakXY=-jarakX;}
	else
  {JarakXY=sqrt((jarakX*jarakX) + (jarakY*jarakY));}
	//  {JarakXY=(jarakX*0.8) + (jarakY*0.8);}

	rate_XYline = JarakXY - last_error_XYline;
  last_error_XYline = JarakXY ;
  move_XYline = (float)((JarakXY * akselerasi) + (rate_line * 1.2))  ;

  if(move_XYline>max_speed){move_XYline=max_speed;}
  if(move_XYline>=10 && move_XYline<90 ){move_XYline=100;} //kalau eror di hapus aja
	//if(move_XYline<5 && move_XYline>1 ){move_XYline=0;}    //kalau eror di hapus aja
	
	Gerak_4omni(sudut_dipakai,-move_XYline,0);

  sprintf(text,"%3d",(int)(sudut_dipakai));lcd_puts(2,0,(int8_t* )text);
  //sprintf(text,"%3d",(int)(JarakXY ));lcd_puts(3,0,(int8_t* )text);
		
}
void Serial_stick(){
	HAL_UART_Receive_DMA(&huart1,(uint8_t*)stick,3);  ///WAJIB PAKAI DMA DI RECIVE nya 
	Stick = atof(stick);		 
	//sprintf(text,"%3d",(Stick));lcd_puts(3,9,(int8_t* )text); 
}
void baca_stick(){
  Serial_stick();
  if (Stick==0){lcd_puts(0,13,(int8_t*)"       ");}  else
	if (Stick==101){ start = 1; lcd_puts(0,14,(int8_t*)"start");}  else  {start = 0;}
  if (Stick==102){ select = 1; lcd_puts(0,14,(int8_t*)"select");}  else  {select = 0;}
	if (Stick==103){ atas = 1; lcd_puts(0,14,(int8_t*)"atas");}  else  {atas = 0;}	  
	if (Stick==104){ kanan = 1;lcd_puts(0,14,(int8_t*)"kanan");} else  {kanan = 0;}	  
	if (Stick==105){ kiri = 1; lcd_puts(0,14,(int8_t*)"kiri");}  else  {kiri = 0;}	  
	if (Stick==106){ bawah = 1;lcd_puts(0,14,(int8_t*)"bawah");} else  {bawah = 0;}
  if (Stick==107){ L1 = 1;lcd_puts(0,14,(int8_t*)"L1");} else  {L1 = 0;}
  if (Stick==109){ R1 = 1;lcd_puts(0,14,(int8_t*)"R1");} else  {R1 = 0;}
	if (Stick==108){ L2 = 1;lcd_puts(0,14,(int8_t*)"L2");} else  {L2 = 0;}
	if (Stick==110){ R2 = 1;lcd_puts(0,14,(int8_t*)"R2");} else  {R2 = 0;}
  if (Stick==115){ segitiga = 1;lcd_puts(0,14,(int8_t*)"stiga");} else  {segitiga = 0;}
  if (Stick==116){ bulat = 1;lcd_puts(0,14,(int8_t*)"bulat");} else  {bulat = 0;}
  if (Stick==117){ silang = 1;lcd_puts(0,13,(int8_t*)"silang");} else  {silang = 0;}
  if (Stick==118){ kotak = 1;lcd_puts(0,14,(int8_t*)"kotak");} else  {kotak = 0;}
	if (Stick==119){ Lnaik = 1;lcd_puts(0,14,(int8_t*)"Lnaik");} else  {Lnaik = 0;}
	if (Stick==120){ Lturun = 1;lcd_puts(0,14,(int8_t*)"Lturun");} else  {Lturun = 0;}
	if (Stick==121){ Lkanan = 1;lcd_puts(0,14,(int8_t*)"Lkanan");} else  {Lkanan = 0;}
	if (Stick==122){ Lkiri = 1;lcd_puts(0,14,(int8_t*)"Lkiri");} else  {Lkiri = 0;}
	if (Stick==123){ Rnaik = 1;lcd_puts(0,14,(int8_t*)"Rnaik");} else  {Rnaik = 0;}
	if (Stick==124){ Rturun = 1;lcd_puts(0,14,(int8_t*)"Rturun");} else  {Rturun = 0;}
	if (Stick==125){ Rkanan = 1;lcd_puts(0,14,(int8_t*)"Rkanan");} else  {Rkanan = 0;}
  if (Stick==126){ Rkiri = 1;lcd_puts(0,14,(int8_t*)"Rkiri");} else  {Rkiri = 0;}
}

void Gerak_stick(int speed,int theta){
int batas=150;
if(Lnaik==1){Akselerasi++; 
						if(Akselerasi>=batas){Gerak_4omni(135,speed,theta);}
					  if(Akselerasi<batas){Gerak_4omni(135,(speed/2),theta);}}
else
if(Lkiri==1){Akselerasi++;
						if (Akselerasi>=batas){Gerak_4omni(45,speed,theta);}
						if (Akselerasi<batas){Gerak_4omni(45,(speed/2),theta);}}
else
if(Lkanan==1){Akselerasi++;
					if (Akselerasi>=batas){Gerak_4omni(225,speed,theta);}
					if (Akselerasi<batas){Gerak_4omni(225,(speed/2),theta);}}
else
if(Lturun==1){Akselerasi++; 
					if (Akselerasi>=batas){Gerak_4omni(315,speed,theta);}
					if (Akselerasi<=batas){{Gerak_4omni(315,(speed/2),theta);}}					}
else
if(Rkanan==1){tungguHeading++;Gerak_4omni(0,0,0);
		if (tungguHeading>1 && tungguHeading<3){PosHeading=PosHeading+5;}}
else
if(Rkiri==1){tungguHeading++;Gerak_4omni(0,0,0);
		if (tungguHeading>1 && tungguHeading<3){PosHeading=PosHeading-5;}}
else
if (kanan==1){{tungguHeading++;Gerak_4omni(0,0,0);
		if (tungguHeading>1 && tungguHeading<3){PosHeading=PosHeading+90;}}}
else
if(kiri==1){tungguHeading++;Gerak_4omni(0,0,0);
		if (tungguHeading>1 && tungguHeading<3){PosHeading=PosHeading-90;}}

else
{Gerak_4omni(135,0,0);tungguHeading=0;Akselerasi=0;}
}
void ciumtembok(){
		
		if (LIMIT3==1 && LIMIT4==1){
				Gerak_4omni(320,180,0);}
		if (LIMIT3==1 && LIMIT4==0){
				Gerak_4omni(290,300,0);}
		if (LIMIT3==0 && LIMIT4==1){
				Gerak_4omni(340,300,0);}
		if (LIMIT3==0 && LIMIT4==0){
				Driver(1,0);
			  Driver(2,0);
			  Driver(3,0);
			  Driver(4,0);
		}
}
void Penumatik(int penumatik,int status){
if (penumatik==1){HAL_GPIO_WritePin(Selenoid_2_GPIO_Port,Selenoid_2_Pin,status);} 							//naik turun gripper
else
if (penumatik==2){HAL_GPIO_WritePin(Selenoid_3_GPIO_Port,Selenoid_3_Pin,status);} 							//gripper
else
if (penumatik==3){HAL_GPIO_WritePin(Selenoid_5_GPIO_Port,Selenoid_5_Pin,status);} 						  //naik setengah			
else
if (penumatik==4){HAL_GPIO_WritePin(Selenoid_4_GPIO_Port,Selenoid_4_Pin,status);} 						  //ekspansi Atas
else
if (penumatik==5){HAL_GPIO_WritePin(Selenoid_6_GPIO_Port,Selenoid_6_Pin,status);} 						  //ekspansi bawah
else
if (penumatik==6){HAL_GPIO_WritePin(Selenoid_8_GPIO_Port,Selenoid_8_Pin,status);} 						  //Naik full
else
if (penumatik==7){HAL_GPIO_WritePin(Selenoid_7_GPIO_Port,Selenoid_7_Pin,status);} 			    	  //sodok
}
void LenganOper(){
if((R1==1||segitiga==1) && LIMIT5==1 ){Driver(5,1000); }//naik
else
if(R2==1 && LIMIT6==1 ){Driver(5,-170);}
else
{Driver(5,0);}
}
void AutoLempar(){
//==============================Auto Lempar Penuatic

if(LIMIT12==1 && steplempar<1 && Lemparselesai==0 ){steplempar=1;Penumatik(2,1);Lauto=1; }
if(steplempar==1){tungguPenumatik++;
									Penumatik(5,0);																			//Mundur atas
									Penumatik(6,0); 																		//turun Full
								} 																		//Turun Setengah
if(steplempar==1 && tungguPenumatik >300){steplempar=2;
																					Penumatik(5,1);						//Maju atas
																					tungguPenumatik=0;}
if(LIMIT12==0 && (L1==1||Lauto==1) ){steplempar=3;}
if(steplempar==3){tungguPenumatik++;}
if(steplempar==3 && tungguPenumatik >100){Penumatik(2,0);}						//Buka Grip
if(steplempar==3 && tungguPenumatik >400){Penumatik(7,1);}						//Tembak
if(steplempar==3 && tungguPenumatik >800){Penumatik(7,0);					  	//Balek
																					 Penumatik(5,0);						//Mundur atas
																					 tungguPenumatik=0;
																					 steplempar=0;
																					 Lemparselesai=1;
																					 Lauto=0;
																					}


}
void AutoPickup(){
//============================Auto Grib Penumatik
		if(segitiga==1 && StepPenumatik<3){StepPenumatik=1;}
	
		if(StepPenumatik==1){AmbilSelsai=0;
												 Penumatik(3,1); 														//Turun Setenngah
												 Penumatik(6,0); 														//Turun Full
												 tungguPenumatik++;
		if(StepPenumatik==1 && tungguPenumatik>1450){StepPenumatik=2;tungguPenumatik=0;}}
		
		if(StepPenumatik==2){Penumatik(4,1); 														//Maju atas
												 tungguPenumatik++;
		if(LIMIT13==0 ){StepPenumatik=3;tungguPenumatik=0;}}

		if(StepPenumatik==3){ Penumatik(1,1);      											//Grip Turun
												  if(bulat==1){Penumatik(2,1);}
												else
													if(bulat==0){Penumatik(2,0);}													
												}										
		
	  if(segitiga==1 && StepPenumatik>2){StepPenumatik=4;}
												 
		if(StepPenumatik==4){Penumatik(2,1);
												 Penumatik(1,0);									 
												 tungguPenumatik++;
		if(tungguPenumatik>1650){StepPenumatik=5;tungguPenumatik=0;}}
		if(StepPenumatik==5){Penumatik(4,0); 														//Mundur Atas
													tungguPenumatik++;
		if(tungguPenumatik>600){Penumatik(3,0);tungguPenumatik=0;StepPenumatik=0;
														AmbilSelsai=1;}} 		//naik Setengah
		
}
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM5_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_TIM9_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_TIM12_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */
  lcd_init();
  Encoder_Init();
  Get_Enc1();
	Get_Enc3();
	Get_Enc5();
  lcd_clear();
  lcd_clear();
	Penumatik(6,1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {		
		  baca_stick();
		  pid_kompas();
		  kalibrasi_IMU();  //Kalibbrasi Sensor sudut (jika X di tekan nilai Sensor sudut=0)
		  IMUSAKTI_int();   //Komunikasi dengan Sensor Sudut
			BacaEncoder();  	//membaca nilai enkoder 
	   	ordoPENS();
			pid_kompas ();
		
	   if(select==1 && step==0){z=1;lcd_clear();}
		 if(z==0 || silang==1){
			 sprintf(text,"%3d",(int)(PepetX));lcd_puts(3,7,(int8_t* )text);
			  sprintf(text,"%3d",(int)(step));lcd_puts(3,0,(int8_t* )text);
			  sprintf(text,"%6d",(enkoder_PID ));lcd_puts(1,0,(int8_t* )text);
	      sprintf(text,"%6d",(enkoder_PID2 ));lcd_puts(1,6,(int8_t* )text);
		    sprintf(text,"%6d",(enkoder_PID3 ));lcd_puts(1,12,(int8_t* )text);
        sprintf(text,"%3d",(int)(hasildata));lcd_puts(0,8,(int8_t* )text);
		    sprintf(text,"%5d",(int)(KordinatX));lcd_puts(2,12,(int8_t* )text);
        sprintf(text,"%5d",(int)(KordinatY));lcd_puts(3,12,(int8_t* )text);
		    }
		 
		 if(z==1){
				lcd_puts(1,5,(int8_t*)"Cek Sensor");
				if(LIMIT1==1){lcd_puts(2,0,(int8_t*)"1");}	
				if(LIMIT1==0){lcd_puts(2,0,(int8_t*)"0");}  
				
				if(LIMIT2==1){lcd_puts(2,2,(int8_t*)"1");} 	
				if(LIMIT2==0){lcd_puts(2,2,(int8_t*)"0");} 	
				
				if(LIMIT3==1){lcd_puts(2,4,(int8_t*)"1");} 	
				if(LIMIT3==0){lcd_puts(2,4,(int8_t*)"0");} 	
				
				if(LIMIT4==1){lcd_puts(2,6,(int8_t*)"1");} 	
				if(LIMIT4==0){lcd_puts(2,6,(int8_t*)"0");} 	
				
				if(LIMIT5==1){lcd_puts(2,8,(int8_t*)"1");}
				if(LIMIT5==0){lcd_puts(2,8,(int8_t*)"0");}
				
				if(LIMIT6==1){lcd_puts(2,10,(int8_t*)"1");}
				if(LIMIT6==0){lcd_puts(2,10,(int8_t*)"0");}
				
				if(LIMIT7==1){lcd_puts(2,12,(int8_t*)"1");}
				if(LIMIT7==0){lcd_puts(2,12,(int8_t*)"0");}
				
				if(LIMIT8==1){lcd_puts(2,14,(int8_t*)"1");}
				if(LIMIT8==0){lcd_puts(2,14,(int8_t*)"0");}
				
				if(LIMIT9==1){lcd_puts(2,16,(int8_t*)"1");}
				if(LIMIT9==0){lcd_puts(2,16,(int8_t*)"0");}
				
				if(LIMIT10==1){lcd_puts(2,18,(int8_t*)"1");}
				if(LIMIT10==0){lcd_puts(2,18,(int8_t*)"0");}
				
				if(LIMIT11==1){lcd_puts(3,0,(int8_t*)"1");}
				if(LIMIT11==0){lcd_puts(3,0,(int8_t*)"0");}
				
        if(LIMIT12==1){lcd_puts(3,2,(int8_t*)"1");}
				if(LIMIT12==0){lcd_puts(3,2,(int8_t*)"0");}
				
        if(LIMIT13==1){lcd_puts(3,4,(int8_t*)"1");}
				if(LIMIT13==0){lcd_puts(3,4,(int8_t*)"0");}
				//if(LIMIT14==1){lcd_puts(3,6,(int8_t*)"1");}
			  //if(LIMIT14==0){lcd_puts(3,6,(int8_t*)"0");}		
				}
		
//===========================//Mode Manual//==============================		
    if(step==0){Gerak_stick(500,0);LenganOper();
		
    if(step==0 && segitiga==1 && SyaratPickup==0 && LIMIT7==1){SyaratPickup=1;AmbilSelsai=0;}
			if(SyaratPickup==1){AutoPickup();}
				 if(SyaratPickup==1 && AmbilSelsai==1){SyaratPickup=0;}
		
		if(step==0 && L1==1 && SyaratLempar==0){SyaratLempar=1; Lemparselesai=0;}
													if(SyaratLempar==1){AutoLempar();}
														if(SyaratLempar==1 && Lemparselesai==1){SyaratLempar=0;}}
		
 		if(silang==1){lcd_clear();OTW(0,0,0,0);step=0;ambilke2=0;z=0;AmbilSelsai=0;}

//=========================//Motion//=============================
if(LIMIT10==0)
{lcd_puts(0,0,(int8_t*)"BIRU ");
		if(kotak==1 && step<1){step=2;}
		
		if(step==2){OTW(-1500,-1150,600,1.3);} 												//SERONG
		if(step==2 && KordinatY<=-1150){step=3;}
		
		if(step==3){OTW(-1300,-2000,500,1.3);} 												//MAJU
		if(step==3 &&  KordinatY<=-2000){step=4;}
		
    if(step==4){OTW(-380,-2300,600,1.3);} 												//SERONG
		if(step==4 &&  KordinatY<=-2300){step=5;}
		
		if(step==5){OTW(KordinatX,-3400,500,1);} 											//MAJU
		if(step==5 &&  KordinatY<=-3100){step=6;}

		if(step==6){OTW(-1580,-4000,600,1.3);} 												//SERONG
		if(step==6 && KordinatY<=-3800){step=7;} //4000
		
		if(step==7){OTW(-1590,-5100,450,1.5);} 												//maju
		if(step==7 && ((KordinatX<=-1590 && KordinatX>=-1600) || KordinatY<-5100) ){step=8;} 

//==========================//SERONG ke jembatan//===========================		
		if(step==8){OTW(-1020,-5640,300,1.3);} 												//SERONG ke jembatan
		
		if(step==8 && kotak==1){step=9;}//																								1018
  
//==========================//Lurus di jembatan=============================
		//													400 
		 if(step==9){OTW(-1020,-8400,300,1.5);}

	 
//========= ===============//Antisipasi Miring di jembatan===================
	  if((step==9)&& (KordinatY<=-6000)){OTW(-1200,-8800,400,1.3);} 			

//========================//Lolos dari Jembatan=============================							
		if((step==9 || step==10 ||  step==11)&&(KordinatY<=-8200)){step=12;}

//========================//Di tempat oper
		//600
    if(step==12){OTW(-4644,-8250,800,1.5);}//-4300
    if(step==12 && KordinatX>=-4100 && KordinatX<-3900){OTW(-4644,-8250,50,1.3);}	 	//lurus ke tempat oper
		if((step==12 && KordinatX<=-4100)|| (step<=1 && bulat==1  && LIMIT13==1)) {step=13;}
      
		if(step==13){		
									if(LIMIT3==1 && LIMIT4==1){ciumtembok();}
										if(LIMIT1==0 && LIMIT3==0 && LIMIT4==0){Gerak_4omni(44,180,3);}
											if(LIMIT1==1 && LIMIT3==0 && LIMIT4==0){step=14;risetORDO();Gerak_stick(0,0);}}
		
		if(step==14 && LIMIT1==1){OTW(-150,20,120,1.3);LenganOper();}
		if(step==14 && LIMIT1==0){Gerak_stick(0,0);step=15;}
		if(step==15)	{
								      if((LIMIT3==0 && LIMIT4==0)){Gerak_stick(180,5);LenganOper();}
												else
													ciumtembok();}
//=========================//Auto ambil Garage		
		if(step==15 && segitiga==1 && SyaratPickup==0){SyaratPickup=1;AmbilSelsai=0;}
												if(SyaratPickup==1){AutoPickup();}
													if(SyaratPickup==1 && AmbilSelsai==1){SyaratPickup=0;}
		
//=========================//Jalan KE tempat Lempar
		if (step==15 && kotak==1)						{step=16;}	
		if (step==16)												{OTW(800,-680,500,1.3);}																						//Serong 
		if (step==16 && KordinatX>650)			{PosHeading=90; if (hasildata>89){PosHeading=180;}}  //680																										//Rotasi dan standbay
		if (step==16 && (hasildata<=183 && hasildata>=177) && kotak==1){PepetX=KordinatX;step=17;}																//Izin Maju ke Tembok
		if (step==17)	  {PosHeading=180; OTW((PepetX-100),-1800,500,1.8);}																															//Mulai maju
		if (step==17 && KordinatY<-1700 && ambilke2==0) {Gerak_4omni(90,300,0);}																//Mepet tembok
		if (step==17 && KordinatY<-1700 && ambilke2==1) {Gerak_4omni(90,300,0);}																//Mepet tembok Saghai ke 2 & 3
		if (step==17 && LIMIT9==0){PepetX=KordinatX;step=18;}
		if (step==18 && LIMIT9==0) {OTW((PepetX+300),-4880,400,1.3);} //Maju Sambil Mepet tembok
    if (step==18 && LIMIT9==1) {Gerak_4omni(45,180,0);}		
		if (step==18 && (LIMIT7==0)) {Gerak_4omni(134,100,0);	}//Penumatik(5,1);		}
		if (step==18 && (KordinatY<-5000 || LIMIT8==0)) {step=181;}					//Sampai Tujuan Langsung rotasi
		if (step==181)  {Gerak_4omni(185,200,0); serong=1;}
		if (step==181 && serong==1 && (KordinatX<PepetX-300)){PosHeading=163;}
    if (step==181 && hasildata<164 && hasildata>161){step=19;}														//Siap Tembak
//=========================//Auto di sudut lempar 	
		if (step==19){
										  	Gerak_stick(180,0);
									  		if(L1==1 && SyaratLempar==0){SyaratLempar=1; Lemparselesai=0;}
													if(SyaratLempar==1){AutoLempar();}
														if(SyaratLempar==1 && Lemparselesai==1){SyaratLempar=0;}
												
												if(kotak==1){Penumatik(2,0);Penumatik(5,0);PosHeading=180;step=191;}} 
		if(step==191){Gerak_stick(180,0);PosHeading=180; 
		                        if (hasildata<182 && hasildata>178){step=20;}}

//========================//Ambil Garage ke2 & 3
		if (step==20)	{OTW((PepetX-500),-680,400,1.3);}																																	//Kembali Ke tempat standbay
		if (step==20 	&& KordinatY<=-650 && KordinatY>=-660){step=21;}
		if (step==21)	{PosHeading=90; Gerak_4omni(0,200,0);}																										//Rotasi Untuk mengambil Saghai
		if (step==21 	&& hasildata<92 && hasildata>88){step=22;}						
		if (step==22)	{OTW(1200,-600,200,1.3);}//Mepet ke tembok
    if (step==22 &&  LIMIT9==0) {step=24; PepetX=KordinatX;}
		if (step==22  && KordinatX>1180 && KordinatX<=1250){step=23;}
	  if (step==23) {OTW(1550,-680,200,1.2);}																																  //Hantam Tembok
		if (step==23  && LIMIT9==0) {step=24; PepetX=KordinatX;}
		if (step==24) {																																												  //Mengambil Saghai ke 2&3
												Gerak_stick(180,0);
												ambilke2=1;
												if (segitiga==1){SyaratPickup=1;AmbilSelsai=0;}
												if (SyaratPickup==1){AutoPickup();}
												if (SyaratPickup==1 && AmbilSelsai==1){SyaratPickup=0;}
												if (kotak==1){step=25;}}

		if (step==25){OTW((PepetX-500),-400,300,1.2);}																																	//Ke tempat standbay
		if (step==25 && KordinatY<-390 && KordinatY>-410){step=26;}																			
		if (step==26) {PosHeading=180;OTW((PepetX-600),-680,300,1.2);}																										//Robot Ready di tempat stanbay
		if (step==26 && hasildata<=183 && hasildata>=178 && kotak==1){step=16;}
}
										
if(LIMIT10==1)
{lcd_puts(0,0,(int8_t*)"MERAH"); 
if(kotak==1 && step<1 && AmbilSelsai==0){step=2;}
//X kanan Kiri
//Y maju  mudnur
/* 24.6
		if(step==2){OTW(1150,-950,800,1.1);} 												//SERONG
		if(step==2 && KordinatY<=-900){step=3;}
		
		if(step==3){OTW(1250,-2000,800,1.1);} 												//MAJU
		if(step==3 &&  KordinatY<=-1800){step=4;}
		
    if(step==4){OTW(290,-2800,800,1.3);} 												//SERONG
		if(step==4 &&  KordinatY<=-2600){step=5;}
		
		if(step==5){OTW(280,-3300,800,1.1);} 											//MAJU
	  if(step==5 &&  KordinatY<=-3100 && LIMIT11==1){step=6;}//2900 Mode Normal
	  if(step==5 &&  KordinatY<=-3100 && LIMIT11==0){step=6;}//2300 Mode Upnormal

		if(step==6){OTW(353,-3650,800,1.1);} 												//SERONG
		if(step==6 && KordinatX>320 ){step=7;} //3800
		
		if(step==7){OTW(1480,-3930,600,1.2);} 												//maju
		if(step==7 && KordinatY<1480){step=8;} 
//								820	
	 if(step==8){OTW(1480,-5200,700,1.3);} 		                  //Maju
*/
	  if(step==2){OTW(1200,-1250,600,1.3);} 												//SERONG
		if(step==2 && KordinatY<=-1150){step=3;}
		
		if(step==3){OTW(1200,-2100,600,1.3);} 												//MAJU
		if(step==3 &&  KordinatY<=-2000){step=4;}
		
    if(step==4){OTW(400,-2400,600,1.3);} 							 					//SERONG
		if(step==4 &&  KordinatY<=-2300){step=5;}
		
		if(step==5){OTW(410,-3300,600,1.3);} 										    	//MAJU
		if(step==5 &&  KordinatY<=-3020 && LIMIT11==1){step=6;}//2900 Mode Normal
		if(step==5 &&  KordinatY<=-3020 && LIMIT11==0){step=6;}//2300 Mode Upnormal

		if(step==6){OTW(1580,-3880,600,1.3);} 												//SERONG
		if(step==6 && KordinatY<=-3650){step=7;} //3800
		//								1200
		if(step==7){OTW(1290,-5400,500,1);} 												  //maju
		if(step==7 && KordinatX>1270){step=8;} 
 
	  if(step==8){OTW(970,-5400,500,1);} 														//Kanan
		if(step==8 && KordinatX<970){step=9;} //masih rawan di akhir
		
//==========================//Lurus di jembatan=============================
		//													400 //8100//8400
		if(step==9){OTW(820,-7800,300,1.5);}
    if(step==9 && KordinatY<=-5800 ){OTW(800,-7800,600,1.3);} //350		
//========= ===============//Antisipasi Miring di jembatan===================
  	if((step==9 && KordinatY<=-7800)){OTW(800,-7800,800,1.3);} 			//700

//========================//Lolos dari Jembatan=============================							
		if(step==9  && KordinatY<=-7700){step=12;} //8100
  
    if(step==12 && KordinatX<4000 && LIMIT1==1 ){OTW(5500,-8000,900,1.3);}												 //lurus ke tempat oper 500
		if(step==12 && KordinatX>4000 && LIMIT1==1 ){OTW(5500,-8000,300,1.3);}
		if(step==12 && LIMIT1==0 ){OTW(5500,-8000,50,1.3);}

		if(step==12 && LIMIT9 ==1 && KordinatX>5000 ){OTW(5750,-8000,50,1.3);}
    if(step==12 && LIMIT9 ==0 ){step=13;}
		if(step==13){	
									if(LIMIT9 ==0)
										{	
											if((LIMIT3==1 && LIMIT4==1)){ciumtembok();}
											if((LIMIT3==0 && LIMIT4==0)){step=15;risetORDO();}
										}
								  else
									Gerak_4omni(45,100,0);
								}
		
		if(step==15)	{
									if(LIMIT9 == 0){
								      if((LIMIT3==0 && LIMIT4==0)){
													//LenganOper(); 
													if(LIMIT6==1 ){Driver(5,-100);}
													if(LIMIT6==0 ){Driver(5,0);}
												}
												else
											ciumtembok();}
									else
										Gerak_4omni(45,100,0);
									}
//=========================//Auto ambil Garage		
		if(step==15 && segitiga==1 && SyaratPickup==0){SyaratPickup=1;AmbilSelsai=0;}
												if(SyaratPickup==1){AutoPickup();}
													if(SyaratPickup==1 && AmbilSelsai==1){SyaratPickup=0;}
   
  //if (step==0 && LIMIT9==1 && bulat==1 && LIMIT3==0 && LIMIT4==0){OTW(200,0,100,1.3);}
//=========================//Jalan KE tempat Lempar
		if (step==15 && kotak==1 && StepPenumatik != 3)						{step=16;}
    if (step==0 &&  kotak==1 && LIMIT9==0 && LIMIT3==0 && LIMIT4==0 && StepPenumatik != 3 && AmbilSelsai==1)						{step=16;}		
		if (step==16)												{OTW(-2300,-550,400,1.3);}																						//Serong 
		if (step==16 && KordinatX<-2295 && KordinatY<545)	{step=17;}																												//Rotasi dan standbay
		if (step==17){Rotasi_Lambat();PosHeading=195;}
		if (step==17 && kotak==1){step=18;}																//Izin Maju ke Tembok
		if (step==18 && KordinatY>-4100){OTW(-1370,-5000,700,1.2);}
		if (step==18 && KordinatY<-4200){Penumatik(5,1);}
		if (step==18 && KordinatY<-4100){OTW(-1370,-5000,100,1.2);}
		if (step==18 && (KordinatY<-4900 || LIMIT7==0) ){OTW(-1370,-5000,10,1.2);}
		if (step==18 && KordinatX<-1340 && KordinatY<-4950){step=19;}
		if (step==19){
										  	Gerak_stick(180,0);
									  		if(L1==1 && SyaratLempar==0 && LIMIT12==0){SyaratLempar=1; Lemparselesai=0;}
													if(SyaratLempar==1){AutoLempar();}
														if(SyaratLempar==1 && Lemparselesai==1){SyaratLempar=0;}
												
												if(kotak==1){step=20;Penumatik(2,0);Penumatik(5,0);PosHeading=180;}}

//========================//Ambil Garage ke2 
		if (step==20)	{OTW((PepetX+600),-680,400,1.3);}
	
//  if (step==20)	{OTW(-700,-680,400,1.3);}																																	//Kembali Ke tempat standbay
		if (step==20 	&& (KordinatX>(PepetX+600)) && KordinatY>=-670){step=21;}
		if (step==21)	{PosHeading=270;OTW(-700,-680,400,1.3);}																										//Rotasi Untuk mengambil Saghai
		if (step==21 	&& hasildata<273 && hasildata>268 && KordinatX<=-690 && KordinatX>=-710 ){step=22;}						
		if (step==22)	{OTW(-1200,-600,200,1.3);}																																  //Mepet ke tembok
		if (step==22  && ((KordinatX<-1100 && KordinatX>-1250)||LIMIT9==0)){step=23;}
	  if (step==23) {OTW(-1350,-680,180,1.2);}																																  //Hantam Tembok
		if (step==23  && (LIMIT9==0 || KordinatX<-1250)) {step=24;SetXambil=KordinatX;SetYambil=KordinatY;}
		if (step==24) {																																												  //Mengambil Saghai ke 2&3
												Gerak_stick(180,0);
												ambilke2=1;
												if (segitiga==1){SyaratPickup=1;AmbilSelsai=0;}
												if (SyaratPickup==1){AutoPickup();}
												if (SyaratPickup==1 && AmbilSelsai==1){SyaratPickup=0;}
												if (kotak==1){step=25;}}
		
		if (step==25) {OTW((SetXambil+400),(SetYambil+300),400,1.3);} //{OTW((PepetX+600),-680,300,1.2);}																																	//Ke tempat standbay
		if (step==25 && move_XYline<10){step=26;}
    //if (step==25 && KordinatX>-1000 && KordinatX<-900){step=26;}																			
		if (step==26) {PosHeading=180;OTW(-800,-680,350,1.2);}//700																										//Robot Ready di tempat stanbay
		if (step==26 && hasildata<=183 && hasildata>=178 && kotak==1){step=27;PosHeading=180;}
		if (step==27){OTW(-1022,-1700,450,1.3);}
		if (step==27 && KordinatY<-1600){step=17;}
}
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void) //pwm
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 80;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1024;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim2);

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM5 init function */
static void MX_TIM5_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 65535;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM8 init function */
static void MX_TIM8_Init(void) //pwm
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 80;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 1024;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim8);

}

/* TIM9 init function */
static void MX_TIM9_Init(void) //pwm
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 80;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 1024;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim9);

}

/* TIM12 init function */
static void MX_TIM12_Init(void) //pwm
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 80;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 1024;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim12);

}

/* UART4 init function */
static void MX_UART4_Init(void)
{

  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_8;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, D7_Pin|Selenoid_1_Pin|Selenoid_2_Pin|Selenoid_3_Pin 
                          |D6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Selenoid_6_Pin|Selenoid_5_Pin|D4_Pin|D5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, Selenoid_4_Pin|Selenoid_8_Pin|Selenoid_7_Pin|RS_Pin 
                          |Selenoid_9_Pin|RW_Pin|E_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : limit7_Pin limit9_Pin limit8_Pin limit6_Pin */
  GPIO_InitStruct.Pin = limit7_Pin|limit9_Pin|limit8_Pin|limit6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : D7_Pin D6_Pin */
  GPIO_InitStruct.Pin = D7_Pin|D6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Selenoid_1_Pin Selenoid_2_Pin Selenoid_3_Pin */
  GPIO_InitStruct.Pin = Selenoid_1_Pin|Selenoid_2_Pin|Selenoid_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : Selenoid_6_Pin Selenoid_5_Pin */
  GPIO_InitStruct.Pin = Selenoid_6_Pin|Selenoid_5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Selenoid_4_Pin Selenoid_8_Pin Selenoid_7_Pin Selenoid_9_Pin */
  GPIO_InitStruct.Pin = Selenoid_4_Pin|Selenoid_8_Pin|Selenoid_7_Pin|Selenoid_9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : limit10_Pin limit11_Pin */
  GPIO_InitStruct.Pin = limit10_Pin|limit11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : limit12_Pin */
  GPIO_InitStruct.Pin = limit12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(limit12_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RS_Pin RW_Pin E_Pin */
  GPIO_InitStruct.Pin = RS_Pin|RW_Pin|E_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : limit2_Pin limit3_Pin limit1_Pin limit4_Pin */
  GPIO_InitStruct.Pin = limit2_Pin|limit3_Pin|limit1_Pin|limit4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : Stop_Atas_2_Pin Stop_Atas_1_Pin */
  GPIO_InitStruct.Pin = Stop_Atas_2_Pin|Stop_Atas_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : D4_Pin D5_Pin */
  GPIO_InitStruct.Pin = D4_Pin|D5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : limit5_Pin */
  GPIO_InitStruct.Pin = limit5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(limit5_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
