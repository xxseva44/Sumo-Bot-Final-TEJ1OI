/*Configs
FAST_UART | Switch from long, easily read strings to single numbers to reduce load on the Hardware Interface Module(Arduino UNO) as it uses an emulated UART interface along side its hardware interface used by USB.
DEBUG | Enable logging of subsystems. Needed when using other debug options.
DEBUG_UART | Enable logging of UART subsystem.
DEBUG_MOTOR | Enable logging of motor subsystem.
CONFIG_BitBangSerial | Enable emulated serial interface
*/

/*
NOTES:
- The way debugging is done on the uno is outdated and never recived the newer setup as seen on the esp32 module
- Commands meant for the UNO can be sent via the ESP which will pass them through to the UNO
  - SUBNOTE: Commands can still be sent directly to the UNO via usb serial Interface
- Command structure:
  [MODULE]:[SUBSYSTEM]:[CMD1]:[CMD2]:[CMD3]:
  Examples: [NON FAST_UART]  HIM_INTF:Motor_Driver:FWD:255:255: \ [FAST_UART]  2:6:1:255:255:s
            [NON FAST_UART]  HIM_INTF:LED:0:TRUE: \ [FAST_UART]  2:7:1:1
  - NOTE: Commands vary depending on the FAST_UART config(only for commmands that get send to the UNO, aka HIM_INTF). As of the latest build, it was enabled to refer to the fast uart strings. 
*/

#define DEBUG
#define DEBUG_UART
#define DEBUG_MOTOR
#define CONFIG_BitBangSerial

//UART Configs
#define MAX_UART_PARAMS 8

#ifdef CONFIG_BitBangSerial
#include <SoftwareSerial.h>
#define SWserial_tx 5
#define SWserial_rx 4
SoftwareSerial SWserial (SWserial_rx, SWserial_tx);
#endif

//Hardware Defines
#define PWM_EN_A_L298N 11
#define PWM_EN_B_L298N 10
#define IN1_L298N 13
#define IN2_L298N 12
#define IN3_L298N 9
#define IN4_L298N 8

#define MaxSonar 3
#define HC_SR04_TRIG_1 2
#define HC_SR04_ECHO_1 3
#define HC_SR04_TRIG_2 A0
#define HC_SR04_ECHO_2 A1
#define HC_SR04_TRIG_3 A2
#define HC_SR04_ECHO_3 A3

#define MaxLED 2
#define LED_BCK 3
#define LED_FWD 6
int LED_HW[MaxLED] = {LED_BCK, LED_FWD};

#ifdef FAST_UART
int CMD_Buffer[10];
//## FLAGS ##
const String Priority  = "-1";
//## DataTypes/Command Interfaces ##
const String Status = "0";
const String Data = "1";
const String HIM_INTF = "2";
const String MCU_CONTROL = "3";
//## Subsystems ##
const String UART = "4";
const String Sonar_Data = "5";
const String Motor_Driver = "6";
const String LED = "7";
// Motor_Driver
const String FWD = "1";
const String BCK = "2";
const String LFT = "3";
const String RGT = "4";
const String HLT = "5";
const String LFT_HRD = "6";
const String RGT_HRD = "7";
#endif


#ifndef FAST_UART
const String Priority = "P";
//## DataTypes/Command Interfaces ##
const String Status = "Status";
//#define Data 1
const String HIM_INTF = "HIM_INTF";
const String MCU_CONTROL = "MCU_CONTROL";
//## Subsystems ##
const String UART = "UART";
const String Sonar_Data  = "Sonar_Data";
const String Motor_Driver = "Motor_Driver";
const String LED = "LED";
// Motor_Driver
const String FWD = "FWD";
const String BCK = "BCK";
const String LFT = "LFT";
const String RGT = "RGT";
const String HLT = "HLT";
const String LFT_HRD = "LFT_HRD";
const String RGT_HRD = "RGT_HRD";
#endif

struct Status {
  int State_HIM = 0;
  int State_UART = 0;
  int State_Sonar_Array = 0;
  int State_Motor_Driver = 0;
};

struct Motor_L298N{
  byte Direction;
  int Speed;
};

struct Line_Detection{
  bool Rear_Line;
  bool Front_Line;
};


struct UART{
  int Index_val_DataType;
  int Index_Subsystem;
  int Index_Field_1;
  int Index_Field_2;
  int Index_Field_3;
  String UART_SEND;
  char Separator = ':';

  bool DATA_READY;

  String UART_DATA_INPUT[MAX_UART_PARAMS];

  //Uart data interface. 
  void UART_DATA(bool mode, String Data){
    switch(mode){
      case 0:
      Data = Data.substring(0, Data.length() -1);
      UART_DATA_INPUT[0] = Data;
      Index_val_DataType = Data.indexOf(Separator);
      UART_DATA_INPUT[1] = Data.substring(0, Index_val_DataType);
      Index_Subsystem = Data.indexOf(Separator, Index_val_DataType+1);
      UART_DATA_INPUT[2] = Data.substring(Index_val_DataType+1, Index_Subsystem);
      Index_Field_1 = Data.indexOf(Separator, Index_Subsystem+1);
      UART_DATA_INPUT[3] = Data.substring(Index_Subsystem+1, Index_Field_1);
      Index_Field_2 = Data.indexOf(Separator, Index_Field_1+1);
      UART_DATA_INPUT[4] = Data.substring(Index_Field_1+1, Index_Field_2);
      Index_Field_3 = Data.indexOf(Separator, Index_Field_2+1);
      UART_DATA_INPUT[5] = Data.substring(Index_Field_2+1, Index_Field_3);
      #ifdef DEBUG_UART
      Serial.print("Incoming data | ");
      Serial.println(Data);
      Serial.print("Data Type: ");
      Serial.println(UART_DATA_INPUT[0]);
      Serial.print("Subsystem: ");
      Serial.println(UART_DATA_INPUT[1]);
      Serial.print("Field 1: ");
      Serial.println(UART_DATA_INPUT[2]);
      Serial.print("Field 2: ");
      Serial.println(UART_DATA_INPUT[3]);
      Serial.print("Field 3: ");
      Serial.println(UART_DATA_INPUT[4]);
      #endif
      DATA_READY = true;
      break;
      case 1:
      #ifdef DEBUG_UART
      Serial.print("Transmitting data | ");
      Serial.println(Data);
      #endif
      SWserial.print(Data += "\n");
      break;
    }
    Data = "";
  }
};


struct HardwareInterfaceModule{
  struct Status State;
  struct Sonar_System Sonar;
  struct Motor_L298N L298N;
  struct Line_Detection Line_Detect;
  struct UART UART_INTF;
};
HardwareInterfaceModule HIM;

void setup() {
  HIM.State.State_HIM = 1;
  Serial.begin(9600);
  #ifdef CONFIG_BitBangSerial
  pinMode(SWserial_rx, INPUT);
  pinMode(SWserial_tx, OUTPUT);
  SWserial.begin(9600);
  #endif
  #ifdef DEBUG
  DebugUtil(0, "Init", "Hardware Interface Module Active!!", 0, 'I');
  #endif
  for(int x = 0; x < MaxSonar; x++){
    pinMode(HIM.Sonar.HC_SR04_sonar_array_trig[x], OUTPUT);
    pinMode(HIM.Sonar.HC_SR04_sonar_array_echo[x], INPUT);
  }

  pinMode(IN1_L298N, OUTPUT);
  pinMode(IN2_L298N, OUTPUT);
  pinMode(IN3_L298N, OUTPUT);
  pinMode(IN4_L298N, OUTPUT);
  pinMode(PWM_EN_A_L298N, OUTPUT);
  pinMode(PWM_EN_B_L298N, OUTPUT);
  pinMode(LED[0], OUTPUT);
  pinMode(LED[1], OUTPUT);
  HIM.State.State_HIM = 1;
}

void loop() {
  Serial_Interface();
  if(HIM.UART_INTF.DATA_READY == true){
    HIM_UART_PARSER();
  }
}


//Takes decomposed uart data and reads it, sending the commands to the appropriate functions. 
void HIM_UART_PARSER(){
    if(HIM.UART_INTF.DATA_READY == true){
      if(HIM.UART_INTF.UART_DATA_INPUT[1] == Status){
        StatusUpdater();
      }
      else if(HIM.UART_INTF.UART_DATA_INPUT[1] == HIM_INTF){
        if(HIM.UART_INTF.UART_DATA_INPUT[2] == Motor_Driver){
          Motor_Driver_INTF(HIM.UART_INTF.UART_DATA_INPUT[3], HIM.UART_INTF.UART_DATA_INPUT[4].toInt(), HIM.UART_INTF.UART_DATA_INPUT[5].toInt());
        }
        else if(HIM.UART_INTF.UART_DATA_INPUT[2] == Sonar_Data){
          //Sonar_Array_Interface();
        }
        else if(HIM.UART_INTF.UART_DATA_INPUT[2] == LED){
          LED_INTF(HIM.UART_INTF.UART_DATA_INPUT[3].toInt(), HIM.UART_INTF.UART_DATA_INPUT[4].toInt());
        }
      }
      else if(HIM.UART_INTF.UART_DATA_INPUT[2] == MCU_CONTROL){
        HIM.UART_INTF.UART_DATA(1, HIM.UART_INTF.UART_DATA_INPUT[1]);
      }
    }
    else{
      #ifdef DEBUG
      DebugUtil(0, "UART_PARSER", "DATA NOT READY!", 0, 'E');
      #endif
    }
  HIM.UART_INTF.DATA_READY = false;
}

void Motor_Driver_INTF(String Direction, int Speed_L, int Speed_R){
  HIM.State.State_Motor_Driver = 1;
  if((Speed_L > 255) || (Speed_L < 0) || (Speed_R > 255) || (Speed_R < 0)){
    #ifdef DEBUG_MOTOR
    DebugUtil(0, "L29xxx Motor Control", "Invalid Speed!", 0, 'E');
    #endif
    return;
  }
  else{
    analogWrite(PWM_EN_A_L298N, Speed_R);
    analogWrite(PWM_EN_B_L298N, Speed_L);
  }
  if(Direction == FWD){
    digitalWrite(IN1_L298N, 1);
    digitalWrite(IN2_L298N, 0);
    digitalWrite(IN3_L298N, 1);
    digitalWrite(IN4_L298N, 0);
  }
  else if(Direction == LFT){
    digitalWrite(IN1_L298N, 0);
    digitalWrite(IN2_L298N, 1);
    digitalWrite(IN3_L298N, 0);
    digitalWrite(IN4_L298N, 0);
  }
  else if(Direction == RGT){
    digitalWrite(IN1_L298N, 0);
    digitalWrite(IN2_L298N, 0);
    digitalWrite(IN3_L298N, 0);
    digitalWrite(IN4_L298N, 1);
  }
  else if(Direction == LFT_HRD){
    digitalWrite(IN1_L298N, 0);
    digitalWrite(IN2_L298N, 1);
    digitalWrite(IN3_L298N, 1);
    digitalWrite(IN4_L298N, 0);
  }
  else if(Direction == RGT_HRD){
    digitalWrite(IN1_L298N, 1);
    digitalWrite(IN2_L298N, 0);
    digitalWrite(IN3_L298N, 0);
    digitalWrite(IN4_L298N, 1);
  }
  else if(Direction == BCK){
    digitalWrite(IN1_L298N, 0);
    digitalWrite(IN2_L298N, 1);
    digitalWrite(IN3_L298N, 0);
    digitalWrite(IN4_L298N, 1);
  }
  else if(Direction == HLT){
    digitalWrite(IN1_L298N, 0);
    digitalWrite(IN2_L298N, 0);
    digitalWrite(IN3_L298N, 0);
    digitalWrite(IN4_L298N, 0);
  }
  #ifdef DEBUG_MOTOR
  Serial.print("Direction: ");
  Serial.println(Direction);
  Serial.print("Speed L: ");
  Serial.println(Speed_L);
  Serial.print("Speed R: ");
  Serial.println(Speed_R);
  #endif
}

//Serial(hardware) + SW Serial
void Serial_Interface(){
  static String receivedMessage;
  static String command;
  static String function;
  static int val_func;
  char received_data;
  char* F = "Serial Interface";
  if(Serial.available() || SWserial.available()){
    if(Serial.available()){received_data = Serial.read();}
    else if(SWserial.available()){received_data = SWserial.read();}
    if(received_data == '\n') {
      HIM.UART_INTF.UART_DATA(0, receivedMessage);
      receivedMessage = "";
    }
    else{receivedMessage += received_data;}
  }
}

void StatusUpdater(){
  String UART_SEND;
  UART_SEND = Status + ':' + String(HIM.State.State_HIM) + ':' + String(HIM.State.State_UART) + ':' + String(HIM.State.State_Sonar_Array) + ':' + String(HIM.State.State_Motor_Driver) + ':';
  HIM.UART_INTF.UART_DATA(1, UART_SEND);
}

void LED_INTF(int LED, int Brightness){
  Serial.print("LED ADJUSTMENT: ");
  Serial.println(LED_HW[LED]);
  Serial.print("Brightness ");
  Serial.println(Brightness);
  analogWrite(LED_HW[LED], Brightness);
}

#ifdef DEBUG
/* 
Debug Functionality.

Helps make messages clearer. 

*/
void DebugUtil(bool mode, char* Function, char* msg, int val, char level){
  #ifdef CONFIG_BitBangSerial //When using hardware serial, don't do anything with this function as Uno r3 has one serial line that is shared with the usb interface
  Serial.print(Function);
  Serial.print(" | "); //divider
  switch(level){
    case 'I' :
    Serial.print("INFO:");
    break;
    case 'W' :
    Serial.print("WARN:");
    break;
    case 'E' :
    Serial.print("ERROR:");
    break;
    case 'F' :
    while(level == 'f'){
      Serial.print("FATAL: ");
      Serial.println(msg);
      Serial.println("CRITIAL FAULT, HALTING!!!");
      Serial.println("PLEASE HARD RESET THE DEVICE TO RECOVER!!!");
    }
    break;
  }
  Serial.print(" "); //spacer
  if(mode == 0){Serial.println(msg);}
  else if(mode == 1){
    Serial.print(msg); 
    Serial.print(" ");
    Serial.println(val);
  }
  #endif
} 
#endif