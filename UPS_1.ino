 //  SABLOUNG Company
 //  UPS Program
 //  StartDate   : 6 /6/2017
 //  Last Update : 10/6/2017 

#include <avr/io.h>
#include <avr/interrupt.h>

volatile enum UPS_Mode {Charging, Inverting, SystemError} UPS_State; 
 
#define SinDiv (200)// Sub divisions of sisusoidal wave.
static int microMHz = 16; // Micro clock frequency
static int freq     = 50; // Sinusoidal frequency
static long int period;   // Period of PWM in clock cycles. (1+Top) ICR1=Top
static long int hperiod;   // Half Period of PWM in clock cycles.

volatile unsigned int LUH[SinDiv];
volatile unsigned int LUL[SinDiv];



const byte digOverloadLedOutPin  =  2; // digital OverloadLed Out pin 28 
const byte digChargerLedOutPin   =  3; // digital ChargerLed Out pin 27 
const byte digInverterLEDOutPin  =  4; // digital Inverter loop Out pin 19 
const byte digSwitchRelayOutPin  =  5;  // Command switch Mains/Battery Relay pin 13 
const byte digBuzzerOutPin       =  6; // digital BuzzerOut pin 12 
const byte digACMainSensePin     =  8; // digital AC Main Sense pin 14
const byte digOutPWM_APin        =  9; // digital OutPWM_A pin 15
const byte digOutPWM_BPin        =  10; // digital OutPWM_B pin 16
const byte digSwitchI_COutPin    =  11;  // Command switch Inverter/chargert pin 17 
const byte digTempINPin          =  12; // digital  pin 18
const byte digFanOutPin          =  13; // digital  pin 19 

const byte anaAC_FBSensePin      = A0; // Analog input pin 23 PC0
const byte anaMainsSensePin      = A1; // Analog input pin 24 PC1
const byte anaBatteryVSensePin   = A2; // Analog input pin 25 PC2
const byte anaBatteryISensePin   = A3; // Analog input pin 26 PC3

// variables
volatile unsigned int StartCycle20ms=0;
volatile unsigned int StartCycle01ms=0;

////// Inverter Variables //////////////////////////////////
float VBatLow  = 10.7;
float VBatVeryLow  = 10.5;

uint16_t   FBV = 0;        
float Vo = 0.0;
float Vo_1 = 0.0;
float VBat  = 0.01;
float VBat_1= 0.01;
float ei = 0.0;
float ei_1 = 0.0;
  // References
static float Vref=0.9; // corresponds to 220V rms
static float aoRef = 0.50;
  // Limiter
static float aoH = 0.98;
static float aoL = 0.30;
  // PID Controller
static float K  =0.6;
static float Ti =0.06;     //
static float Twindup=0.06; // rule of thumb  sqrt(Ti*Td)
static float Td = 0.002;   //  Ts*N/Td  in [0.2,0.6]  OR Td=Ti/4
static float N=8;          //8 - 20
static float Pterm = 0.0;
static float Iterm = 0.0;
static float Dterm = 0.0;
static float da = 0.0;
static float daL  = 0.0;
static float Ts = 0.02;


///// Charger Variables /////////////////////////////////
volatile unsigned int ch_Power;
volatile unsigned int DCycle;
float BatV  = 0.01;
float VbatToVreg  = 10;
float  VBatMax =12.8;
float  VBatMin =12.6;

unsigned int DCycleMin=4;
unsigned int DCycleMax=40;
int CtrlCounter;
unsigned int int_temp;
double tt;

float temp; 
uint16_t VBt;
int MainsOK;
int StateChanged=0;



void setup()
{
   //Serial.begin(9600);
   double temp; 
   period = microMHz*1e6/freq/SinDiv;// Period of PWM in clock cycles    TimerTop=period-1
   hperiod = period /2;
   
   // Generating the look up table of A leg.
   for(int i = 0; i < SinDiv; i++)
   { 
     //temp = aoRef*LUSin[i]+hperiod;
     temp = 0.5*aoRef*(period-2)*sin(i*2*M_PI/SinDiv)+0.5*period;
     LUH[i] = (unsigned int)(temp+0.5);       // Round to integer. 
   }

   // Register initilisation, see datasheet for more detail.
   TCCR1A = 0b10100010;
   TCCR1B = 0b00011001;
   TIMSK1 = 0b00000001;
   ICR1   = period-1;     // Period for 16MHz crystal 
 
  // sei();             // Enable global interrupts.
   
   //DDRB = 0b00000110; // Set PB1 and PB2 as outputs.
   pinMode(digOverloadLedOutPin,OUTPUT);
   pinMode(digChargerLedOutPin,OUTPUT);
   pinMode(digInverterLEDOutPin,OUTPUT);
   pinMode(digSwitchRelayOutPin,OUTPUT);
   pinMode(digBuzzerOutPin     ,OUTPUT);
   pinMode(digACMainSensePin   ,INPUT);

   pinMode(digOutPWM_APin      ,OUTPUT);
   pinMode(digOutPWM_BPin      ,OUTPUT);
   pinMode(digSwitchI_COutPin  ,OUTPUT);
   pinMode(digTempINPin        ,INPUT);
   pinMode(digFanOutPin        ,OUTPUT);

  
   // Hysteris initilisation    // Charging mode
   CtrlCounter=0;
   int_temp=DCycleMax;
   ch_Power = 1;
   DCycle=round(DCycleMax*period/100);

  // Enter the right mode
  MainsOK=digitalRead(digACMainSensePin);
  if (MainsOK==1) {
     UPS_State=Charging;    
  } 
  else{
    UPS_State=Charging;
  }

  sei();             // Enable global interrupts.
 } 
// --------------------- End setup -----------------


void loop()
{ 
  // State machine switching updater
  if (StartCycle01ms==0) {
    MainsOK=digitalRead(digACMainSensePin);
    VBt= analogRead(anaBatteryVSensePin);
    BatV =  VBt * (5.0 / 1023.0) * VbatToVreg;
    FBV= analogRead (anaAC_FBSensePin);
    Vo = FBV * (5.0 / 1023.0);
    //delay(1); 
 
    StateChanged=0;
    switch (UPS_State) {
      case Inverting:
         if (MainsOK==1) {
            UPS_State=Charging;
            StateChanged=1; 
            digitalWrite(digSwitchRelayOutPin,HIGH);
            digitalWrite(digSwitchI_COutPin,LOW);
            CtrlCounter=0;
            int_temp=DCycleMax;
            ch_Power = 1;
            DCycle=round(DCycleMax*period/100);
         } 
         else{
           if (VBt<VBatVeryLow) {
             StateChanged=1; 
             UPS_State=SystemError;
             tone(digBuzzerOutPin,300);
           }
           else{
             if (VBt<VBatLow){ 
             tone(digBuzzerOutPin,100);  
             }
           } 
         }
         break;
      case Charging:
        if (MainsOK==0) {
          UPS_State=Inverting; 
          StateChanged=1;   
          digitalWrite(digSwitchRelayOutPin,LOW);
          digitalWrite(digSwitchI_COutPin,HIGH);
        } 
        break;
    } 
  }       

  // Synchro with first half sine wave of inverter 
  if (StartCycle20ms==1)
  {  
    StartCycle20ms=0;
    /// Charging, Inverting, SystemErro
    switch (UPS_State) {
    case Inverting:
       FBV= analogRead (anaAC_FBSensePin);
       Vo = FBV * (5.0 / 1023.0);
       ei = Vref-Vo;
       //PID Controller
       Pterm=K*ei;
       Dterm=(Td/(Td+N*Ts))*Dterm-(K*Td*N/(Td+N*Ts))*(Vo-Vo_1);
       Vo_1 =Vo;
       da=Pterm+Iterm+Dterm;
       //Limiter of control error
       daL=da;
       if (da>=0){
         if (da>(aoH-aoRef)){
           daL=aoH-aoRef;
         }
       } else {
         if (abs(da)>(aoRef-aoL)){
           daL=-(aoRef-aoL);
         }
       }
       Iterm=Iterm+(K*Ts/Ti)*ei+(Ts/Twindup)*(daL-da);
       for(int i = 0; i < SinDiv; i++) { 
         temp = 0.5*(aoRef+daL)*(period-2)*sin(i*2*M_PI/SinDiv)+0.5*period;
         LUL[i] = (unsigned int)(temp);       // Round to integer. 
       }
       for(int i = 0; i < SinDiv; i++) { 
         LUH[i] = LUL[i];       
       }
       break;
    case Charging:
       VBt= analogRead(anaBatteryVSensePin);
       BatV =  VBt * (5.0 / 1023.0) * VbatToVreg;
       // Soft start-up
       if (CtrlCounter>=0) {  
         tt=(0.00001+DCycleMin+CtrlCounter*(int_temp -DCycleMin)/1000);
         DCycleMax=(unsigned int)(tt+0.5);
         CtrlCounter=CtrlCounter+1;
         if (CtrlCounter>=1000) { // 5 sec interval 
         CtrlCounter=-1;
         DCycleMax=int_temp;
         }
       }
       // Hysterisis control of charging
       if ( (ch_Power == 1) && (BatV > VBatMax) ) {
         ch_Power=0;
       }
       else if ( (ch_Power == 0) && (BatV < VBatMin) ) {
         ch_Power=1;
       }
       if  (ch_Power == 1) {
        tt=0.00001+DCycleMax*period/100;
        DCycle=(unsigned int)(tt+0.5);
       } 
       else { // (ch_Power == 0) 
         tt=0.00001+DCycleMin*period/100;
         DCycle=(unsigned int)(tt+0.5);
       }
       break;
    case SystemError:
       tone(digFanOutPin,400); 
       break;
    }// End of switch
  }   
}
// --------------------- End loop -----------------

ISR(TIMER1_OVF_vect){
   static unsigned int num;
   static char trig;

   switch (UPS_State) {
   case Inverting:
     OCR1A = LUH[num];
     OCR1B = LUH[SinDiv-1-num];
     break;
   case Charging:
     OCR1A = DCycle;
     OCR1B = DCycle;
     break;
   case SystemError:  
     OCR1A = 0;
     OCR1B = 0;
     break;
   }
   StartCycle01ms=num % 10;
   if(++num >= SinDiv)
   {  
      num = 0;       // Reset num.
      StartCycle20ms=1;
      trig = trig^0b00000001;
      if (UPS_State==Inverting){
        digitalWrite(digInverterLEDOutPin,trig);
        digitalWrite(digChargerLedOutPin,0);
      }
      if (UPS_State==Charging){
        digitalWrite(digChargerLedOutPin,trig);
        digitalWrite(digInverterLEDOutPin,0);
      }
   }
}
//--- end of Timer1 interrupt






// End of program End of program End of program



    
 //if (Yk <10.5) // Stop Inverting an Alarm that
  //{
  //  cli();
  //  OCR1A = 0;
  //  OCR1B = 0;
  //  TCCR1A = 0b00000000;
  //  TCCR1B = 0b00000001;
  //  digitalWrite(13,LOW);
  //} 
  //else if (Yk<10.7)
  //{
    // Buzzer
 //}

/* Examples
 pin 12 Buzzer  AINO
 pin 28 SCL Micro is working


 if VBat <10.7 then buzzer
 if VBat <10.5 Stop Inverting an Alarm that
 if VBat >14.6 Stop Charging an Alarm that !!!!
 

  uint16_t value = analogRead(ANALOG_INPUT);
*/
 

//  ---------- Atmega328p  Pin description   -------------
//1   PC6 Reset
//2   PD0 D0 - Rxd
//3   PD1 D1 - Txd
//4   PD2 D2 -  
//5   PD3 D3 - 
//6   PD4 D4 - 
//7   VCC
//8   GND
//9   PB6  Crystal
//10  PB7  Crystal
//11  PD5 D5 - 
//12  PD6 D6 - OUT Buzzer  
//13  PD7 D7 - OUT Mains/Battery Switch relay  
//14  PB0 D8 - IN  Digital AC Main sense 

//15  PB1 D9 - OUT  PB1  pwm A  
//16  PB2 D10- OUT  PB2  pwm B  
//17  PB3 D11- OUT Command switch Inverter/charger
//18  PB4 D12- OUT Fan Out
//19  PB5 D13- OUT Inverter LED On
//20  AVCC
//21  AREF
//22  GND
//23  PC0 A0 - IN  AC Voltage Sense 220
//24  PC1 A1 - IN  Temreture sense
//25  PC2 A2 - IN  Battery Voltage Sense 
//26  PC3 A3 - IN  Battery Current Sense 
//27  PC4 A4 - OUT ChargerON Led OR LCD SDA
//28  PC5 A5 - OUT Overload Led  OR LCD SCL

