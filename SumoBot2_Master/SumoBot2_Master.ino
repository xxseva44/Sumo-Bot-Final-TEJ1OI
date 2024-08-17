/* Configs 
BYPASS_BROWNOUT | Used to bypass brownout detection on ESP32. Useful for when voltage may not be stable.
DEBUG | Enable logging of subsystems. Needed when using other debug options.
LIVE_DEBUG | Show log messages on the Display. [WIP]
DEBUG_MOTOR | Enable logging of motor subsystem.
DEBUG_SONAR_ARRAY_SYSTEM | Enable logging of sonar subsystem.
DEBUG_UART | Enable logging of UART subsystem.
SUPPORT_OTA | Enable Over The Air updates via wifi to the ESP32. [WIP]
DEBUG_MCU_CONTROL | Enable logging of Main Control Unit Interface.
DEBUG_TIMER | Enable logging of timer subsystem.
BLUETOOTH_INTF | Enable usage of the bluetooth interface. 
REQUIRE_BT_CONNECTION | Require BT Connection to fully activate. 
FAST_UART | Switch from long, easily read strings to single numbers to reduce load on the Hardware Interface Module(Arduino UNO) as it uses an emulated UART interface along side its hardware interface used by USB.
*/

/*
NOTES:
- Commands meant for the UNO can be sent via the ESP which will pass them through to the UNO
- Commands can be sent either through USB serial or bluetooth
- Command structure:
  [MODULE]:[SUBSYSTEM]:[CMD1]:[CMD2]:[CMD3]:
  Examples: [NON FAST_UART]  HIM_INTF:Motor_Driver:FWD:255:255: \ [FAST_UART]  2:6:1:255:255:s
            [NON FAST_UART]  HIM_INTF:LED:0:TRUE: \ [FAST_UART]  2:7:1:1

            MCU_CONTROL:HUTCH:
            MCU_CONTROL:Apply_prop:LineDetect_FWD_Enable:TRUE:
            MCU_CONTROL:Drive_Enable:TRUE;

  - NOTE: Commands vary depending on the FAST_UART config(only for commmands that get send to the UNO, aka HIM_INTF). As of the latest build, it was enabled to refer to the fast uart strings. 
*/

#define BYPASS_BROWNOUT
//#define DEBUG
//#define LIVE_DEBUG
//#define DEBUG_MOTOR
//#define DEBUG_SONAR_ARRAY_SYSTEM
//#define DEBUG_UART
//#define SUPPORT_OTA //Not enough Space
//#define DEBUG_MCU_CONTROL
//#define DEBUG_TIMER
#define BLUETOOTH_INTF
#define REQUIRE_BT_CONNECTION
#define FAST_UART
//#define DEBUG_INFRARED

/* Libraries */
#ifdef BYPASS_BROWNOUT
#include <soc/soc.h>
#include <soc/rtc_cntl_reg.h>
#endif

#include <SPI.h>
#include <TFT_eSPI.h>
#include <BluetoothSerial.h>
#include <HardwareSerial.h>

#ifdef SUPPORT_OTA
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#endif

/* Misc. Display items */
#include "WHY_2.h"
#include "NotoSansBold15.h"
#define AA_FONT_SMALL NotoSansBold15

#define MAX_UART_PARAMS 8

#define MaxSonar 3

//Props (Make proper property class later)
int Sonar_SyncRate = 0;
int Sonar_Distance_CLEAR = 40;
int Sonar_Distance_WARN = 35;
int Sonar_Distance_ATTACK = 10;
int Sonar_Distance_EVADE = 5;

int Drive_Speed_BASE = 60;
int Drive_Speed_MED = 70;
int Drive_Speed_MAX = 255;

//Do some math later to figure this out.
int Drive_Speed_Rotate_TIMER_90 = 1000;
int Drive_Speed_Rotate_TIMER_180 = 2000;


/* HW defines */
#define LED_DETECT_FWD 12
#define IR_DISTANCE_SENSOR 34

//Sonar
#define MaxSonar 3
#define HC_SR04_TRIG_1 22
#define HC_SR04_ECHO_1 23
#define HC_SR04_TRIG_2 27
#define HC_SR04_ECHO_2 26
#define HC_SR04_TRIG_3 12
#define HC_SR04_ECHO_3 14

int pwmChannel = 0;
int freq = 1000;
int resolution = 8;

/* Object Creation */
TFT_eSPI tft = TFT_eSPI();
HardwareSerial SerialPort(2); //Use uart interface 2 on the esp32
BluetoothSerial SerialBT;

#ifdef FAST_UART
int CMD_Buffer[10];
//## FLAGS ##
const String Priority  = "-1";
//## DataTypes/Command Interfaces ##
const String Status = "0";
const String Data = "1";
const String HIM_INTF = "2";
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
//## Subsystems ##
const String UART = "UART";
const String Sonar_Data  = "Sonar_Data";
const String Motor_Driver = "Motor_Driver";
// Motor_Driver
const String FWD = "FWD";
const String BCK = "BCK";
const String LFT = "LFT";
const String RGT = "RGT";
const String HLT = "HLT";
const String LFT_HRD = "LFT_HRD";
const String RGT_HRD = "RGT_HRD";
#endif

const String MCU_CONTROL = "MCU_CONTROL";
const String Apply_PROP = "Apply_PROP";

//Props
const String PROP_Drive = "Drive_Enable";
const String PROP_LineDetect_FWD_Enable = "LineDetect_FWD_Enable";
const String PROP_LineDetect_REAR_Enable = "LineDetect_REAR_Enable";

#ifdef SUPPORT_OTA
const String PROP_StartSystemUpdate = "StartSystemUpdate";
#endif

const String PROP_Restart = "Restart";

#ifdef SUPPORT_OTA
const char* ssid = "..........";
const char* password = "..........";
#endif

#ifdef DEBUG
struct DEBUG_INTF{
  #ifdef LIVE_DEBUG
  bool SHOW_DEBUG;
  #endif

/* 
Debug utility to simplify logging and debugging
Mode 0 = Text Only
Mode 1 = Text + Number
*/
  void DebugUtil(bool mode, String Function, String msg, int val, char level){
    String DEBUG_MESSAGE;
    String DEBUG_TYPE;
    switch(level){
      case 'I' :
      DEBUG_TYPE = "INFO";
      break;
      case 'W' :
      DEBUG_TYPE = "WARN";
      break;
      case 'E' :
      DEBUG_TYPE = "ERROR";
      break;
      case 'F' :
      DEBUG_TYPE = "FATAL";
    }
    DEBUG_MESSAGE = Function + ' ' + '|' + ' ' + DEBUG_TYPE + ':' + ' ';
    if(mode == 0){
      DEBUG_MESSAGE = DEBUG_MESSAGE + msg;
      }
    else if(mode == 1){
      DEBUG_MESSAGE = DEBUG_MESSAGE + msg + ':'  + ' ' + val;
    }
    Serial.println(DEBUG_MESSAGE);

    #ifdef BLUETOOTH_INTF
    if(SerialBT.hasClient()){
    SerialBT.println(DEBUG_MESSAGE);
    }
    #endif

    #ifdef LIVE_DEBUG
    if(SHOW_DEBUG == true){
      tft.setTextColor(TFT_YELLOW, TFT_BLACK);
      tft.println(DEBUG_MESSAGE);
      }
    #endif
  }
};
#endif

//work on this later
struct Properties{
  void create(){}
  void set_bool(){}
  void set_int(){}
  void set_String(){}
};

//Sonar variables and arrays
struct Sonar_System{
  //Use 2D array later
  byte HC_SR04_sonar_array_trig[MaxSonar] = {HC_SR04_TRIG_1, HC_SR04_TRIG_2, HC_SR04_TRIG_3};
  byte HC_SR04_sonar_array_echo[MaxSonar] = {HC_SR04_ECHO_1, HC_SR04_ECHO_2, HC_SR04_ECHO_3};
  int SONAR_DISTANCE[MaxSonar];
  long SONAR_DURATION[MaxSonar];
  int OBJ_detect_Front;
  int OBJ_detect_Rear_Left;
  int OBJ_detect_Rear_Right;
  String SonarData;
  bool SONAR_READY;
};

//Statuses
//NOTE TO SELF: Why didn't I use bools?
struct Status_Data {
  int State_MCU = 0;
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


struct HardwareInterfaceModule{
  Status_Data State;
  Sonar_System Sonar;
  Motor_L298N L298N;
  Line_Detection Line_Detect;
};
HardwareInterfaceModule HIM;

struct UART{
  #ifdef DEBUG
  struct DEBUG_INTF Debug;
  #endif
  int Index_val_DataType;
  int Index_Subsystem;
  int Index_Field_1;
  int Index_Field_2;
  int Index_Field_3;
  String UART_SEND;
  char Separator = ':';

  bool DATA_READY;

  String UART_DATA_INPUT[MAX_UART_PARAMS];
  int UART_DATA_INPUT_INDEX[MAX_UART_PARAMS];
  //Uart data interface. 
  void UART_DATA(bool mode, String Data){
    String F = "UART_DATA";
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
      Debug.DebugUtil(0, F, String("incoming Data"), 0, 'I');
      Debug.DebugUtil(0, F, String(Data), 0, 'I');
      #endif
      DATA_READY = true;
      break;
      case 1:
      #ifdef DEBUG_UART
      Debug.DebugUtil(0, F, String("Transmitting Data"), 0, 'I');
      Debug.DebugUtil(0, F, String(Data), 0, 'I');
      #endif
      SerialPort.print(Data += "\n");
      break;
    }
    Data = "";
  }
};

//Device property class. [WIP]
struct SystemProperties{
  bool Drive_Enable;
  bool LineDetect_FWD_Enable;
  bool LineDetect_REAR_Enable;
  bool StartSystemUpdate;
  bool Restart;

  //REWORK LATER
  void Apply_PROP(String PROP, String CTL_STR){
    bool CTL;
    if((CTL_STR == "1") || (CTL_STR == "TRUE")){
      CTL = true;
    }
    else if((CTL_STR == "0") || (CTL_STR == "FALSE")){
      CTL = false;
    }

    if(PROP == PROP_Drive){
      Drive_Enable = CTL;
    }
    if(PROP == PROP_Drive){
      Drive_Enable = CTL;
    }
    else if(PROP == PROP_LineDetect_FWD_Enable){
      Drive_Enable = CTL;
    }
    else if(PROP == PROP_LineDetect_REAR_Enable){
      Drive_Enable = CTL;
    }
    #ifdef SUPPORT_OTA
    else if(PROP == PROP_StartSystemUpdate){
      StartSystemUpdate = CTL;
    }
    #endif
    else if(PROP == PROP_Restart){
      Restart = CTL;
    }
  }
};

struct MainControlUnit{
  struct UART UART_INTF;
  struct Status_Data State;
  struct Sonar_System Sonar;
  struct SystemProperties Properties;
  #ifdef DEBUG
  struct DEBUG_INTF Debug;
  #endif
};

MainControlUnit MCU;

void setup() {
  #ifdef DEBUG
  String F = "INIT";
  #endif
  #ifdef BYPASS_BROWNOUT
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  #endif

  #ifdef OTA_SUPPORT
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  #endif

  //HW init
  //Make sensors behave as expected. ESP and UNO analog read are different, so change ESP bitwidth to match UNO. 
  analogReadResolution(10);
  analogSetWidth(10);
  pinMode(IR_DISTANCE_SENSOR, INPUT);
  pinMode(LED_DETECT_FWD, INPUT);
  for(int x = 0; x < MaxSonar; x++){
    pinMode(MCU.Sonar.HC_SR04_sonar_array_trig[x], OUTPUT);
    pinMode(MCU.Sonar.HC_SR04_sonar_array_echo[x], INPUT);
  }

  //LCD
  tft.init();
  tft.setRotation(3);
  tft.setSwapBytes(true);
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(0, 0);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.loadFont(AA_FONT_SMALL);
  tft.println("GLIZZYBOT M.C.U");


  #ifdef BLUETOOTH_INTF
  SerialBT.begin("GlizzyBot BT");
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.println("Bluetooth interface up!");
  tft.print("Look for: ");
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.println("GlizzyBot BT");
  tft.setTextColor(TFT_WHITE, TFT_BLACK);

  #ifdef REQUIRE_BT_CONNECTION
  tft.print("Waiting for connection");
  while(SerialBT.hasClient() == false){
    if(Timer(800)){
      tft.print(".");
    }
  }
  #endif //REQUIRE_BT_CONNECTION
  if(SerialBT.hasClient()){
    tft.println(".");
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.println("Connected!");
  }
  #endif //BLUETOOTH_INTF

  #ifdef SUPPORT_OTA
  //wifi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  #endif

  //Serial
  Serial.begin(115200);
  SerialPort.begin(9600, SERIAL_8N1, 16, 17); //Uart connection to Arduino UNO AKA H.I.M

  //Pull statuses from H.I.M
  #ifdef DEBUG
  MCU.Debug.DebugUtil(0, F, String("Main Control Unit UP!"), 0, 'I');
  #endif
  delay(500);

  //Enable LEDS on boot. As of the latest build, I commented this out. However I have uncommented it. 
  MCU.UART_INTF.UART_DATA(1, (HIM_INTF + ':' + LED + ':' + String(0) + ':' + String(255) + ':'));
  MCU.UART_INTF.UART_DATA(1, (HIM_INTF + ':' + LED + ':' + String(1) + ':' + String(255) + ':'));
}

void loop() {
  Serial_Interface();
  if(MCU.UART_INTF.DATA_READY == true){
    MCU_UART_PARSER();
  }
  if(MCU.Properties.Drive_Enable == true){
  Drive_Logic();
  }
  #ifdef OTA_SUPPORT
  if(MCU.Properties.StartSystemUpdate == true){
    ArduinoOTA.handle();
  }
  #endif

  #ifdef LIVE_DEBUG
  SensorData();
  #endif
}

void Display_INTF(){
  //Use to handle display rather than having display stuff everywhere
  //[WIP]
}

//Sensor works but readings are way off, fix later.
float IR_Sense(){
  #ifdef DEBUG_INFRARED
  String F = "IR_SENSE";
  #endif
  static int intSensorResult;
  static float IR_Result;
  intSensorResult = analogRead(IR_DISTANCE_SENSOR);
  IR_Result = (6787.0 / (intSensorResult - 3.0)) - 4.0;
  delay(200);
  #ifdef DEBUG_INFRARED
  MCU.Debug.DebugUtil(0, F, String("Distance"), 0, 'I');
  MCU.Debug.DebugUtil(0, F, String(IR_Result), 0, 'I');
  #endif
  return IR_Result;
}

int LINE_DETECT(){
  if(analogRead(LED_DETECT_FWD) < 500){
    return true;
  }
  else{
    return false;
  }
}

//Status display mode
void MCU_Status(){
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(0, 0);

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.print("Hardware Interface Module:");
  if(MCU.State.State_MCU <= 0){
    tft.setTextColor(TFT_RED, TFT_BLACK);
  }
  else {
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  }
  tft.println(MCU.State.State_MCU);
  tft.println("");

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.print("UART Link State:");
  if(MCU.State.State_UART <= 0){
    tft.setTextColor(TFT_RED, TFT_BLACK);
  }
  else {
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  }
  tft.println(MCU.State.State_UART);
  tft.println(""); //NEW LINE

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.print("Sonar Array:");
  if(MCU.State.State_Sonar_Array <= 0){
    tft.setTextColor(TFT_RED, TFT_BLACK);
  }
  else {
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  }
  tft.println(MCU.State.State_Sonar_Array);
  tft.println(""); //NEW LINE

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.print("L298N Status:");
  /*if(MCU.State.State_L298N <= 0){
    tft.setTextColor(TFT_RED, TFT_BLACK);
  }
  else {
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  }
  tft.println(MCU.State.State_L298N);
  tft.println(""); //NEW LINE
  */
}

String command;
String function;
int val_func;

//Uart + Bluetooth interfaces and Serial via USB
void Serial_Interface(){
  static String receivedMessage;
  static String command;
  static String function;
  static int val_func;
  char received_data;
  char* F = "Serial Interface";
  if(Serial.available() || SerialBT.available() || SerialPort.available()){
    if(Serial.available()){received_data = Serial.read();}
    else if(SerialBT.available()){received_data = SerialBT.read();}
    else if(SerialPort.available()>0){received_data = SerialPort.read();}
    if(received_data == '\n') {
      MCU.UART_INTF.UART_DATA(0, receivedMessage);
      receivedMessage = "";
    }
    else{receivedMessage += received_data;}
  }
}

void Sonar_Array_Sync(){
  String F = "Sonar Array";
  MCU.Sonar.SONAR_READY = false;
  //I can't do a for loop as it delays uart for too long. 

  //Sensor 0
  if(Timer(Sonar_SyncRate)==true){
    digitalWrite(MCU.Sonar.HC_SR04_sonar_array_trig[0], LOW);
    digitalWrite(MCU.Sonar.HC_SR04_sonar_array_trig[1], LOW);
    digitalWrite(MCU.Sonar.HC_SR04_sonar_array_trig[2], LOW);
    delayMicroseconds(2);
    digitalWrite(MCU.Sonar.HC_SR04_sonar_array_trig[0], HIGH);
    digitalWrite(MCU.Sonar.HC_SR04_sonar_array_trig[1], HIGH);
    digitalWrite(MCU.Sonar.HC_SR04_sonar_array_trig[2], HIGH);
    delayMicroseconds(10);

    digitalWrite(MCU.Sonar.HC_SR04_sonar_array_trig[0], LOW);
    MCU.Sonar.SONAR_DURATION[0] = pulseIn(MCU.Sonar.HC_SR04_sonar_array_echo[0], HIGH);
    MCU.Sonar.SONAR_DISTANCE[0] = MCU.Sonar.SONAR_DURATION[0] * 0.034 / 2;
    #ifdef DEBUG_SONAR_ARRAY_SYSTEM
    if(MCU.Sonar.SONAR_DISTANCE[0] > 200){
      MCU.Debug.DebugUtil(0, F, String("SONAR 0 OUT OF RANGE!!!"), 0, 'I');
    }
    #endif
    //End of Sensor 0
    //Sensor 1
    digitalWrite(MCU.Sonar.HC_SR04_sonar_array_trig[1], LOW);
    MCU.Sonar.SONAR_DURATION[1] = pulseIn(MCU.Sonar.HC_SR04_sonar_array_echo[1], HIGH);
    MCU.Sonar.SONAR_DISTANCE[1] = MCU.Sonar.SONAR_DURATION[1] * 0.034 / 2;
    #ifdef DEBUG_SONAR_ARRAY_SYSTEM
    if(MCU.Sonar.SONAR_DISTANCE[1] > 200){
      MCU.Debug.DebugUtil(0, F, String("SONAR 1 OUT OF RANGE!!!"), 0, 'I');
    }
    #endif
    //End of Sensor 1
    //Sensor 2
    digitalWrite(MCU.Sonar.HC_SR04_sonar_array_trig[2], LOW);
    MCU.Sonar.SONAR_DURATION[2] = pulseIn(MCU.Sonar.HC_SR04_sonar_array_echo[2], HIGH);
    MCU.Sonar.SONAR_DISTANCE[2] = MCU.Sonar.SONAR_DURATION[2] * 0.034 / 2;
    #ifdef DEBUG_SONAR_ARRAY_SYSTEM
    if(MCU.Sonar.SONAR_DISTANCE[2] > 200){
      MCU.Debug.DebugUtil(0, F, String("SONAR 2 OUT OF RANGE!!!"), 0, 'I');
    }
    #endif
    //End of Sensor 2
    MCU.Sonar.SONAR_READY = true;
    #ifdef DEBUG_SONAR_ARRAY_SYSTEM
    MCU.Debug.DebugUtil(1, F, String("Distance FWD"), MCU.Sonar.SONAR_DISTANCE[0], 'I');
    MCU.Debug.DebugUtil(1, F, String("Distance Rear_R"), MCU.Sonar.SONAR_DISTANCE[1], 'I');
    MCU.Debug.DebugUtil(1, F, String("Distance Rear_L"), MCU.Sonar.SONAR_DISTANCE[2], 'I');
    #endif
  }
}

//Takes decomposed uart data and reads it, sending the commands to the appropriate functions. 
void MCU_UART_PARSER(){
  #ifdef DEBUG_UART
  String F = "UART_PARSER";
  #endif
    if(MCU.UART_INTF.DATA_READY == true){
      if(MCU.UART_INTF.UART_DATA_INPUT[1] == Status){
        StatusUpdater();
      }
      else if(MCU.UART_INTF.UART_DATA_INPUT[1] == HIM_INTF){
        MCU.UART_INTF.UART_DATA(1, MCU.UART_INTF.UART_DATA_INPUT[0]);
      }
      else if(MCU.UART_INTF.UART_DATA_INPUT[1] == MCU_CONTROL){
        if(MCU.UART_INTF.UART_DATA_INPUT[2] == Apply_PROP){
          MCU.Properties.Apply_PROP(MCU.UART_INTF.UART_DATA_INPUT[3], MCU.UART_INTF.UART_DATA_INPUT[4]);
        }
        MCU_Control(MCU.UART_INTF.UART_DATA_INPUT[2]);
      }
    }
    else{
      #ifdef DEBUG_UART
      MCU.Debug.DebugUtil(0, F, String("DATA NOT READY!"), 0, 'E');
      #endif
    }
  MCU.UART_INTF.DATA_READY = false;
}

void StatusUpdater(){
  /*
  MCU.State.State_MCU = StateMCU;
  MCU.State.State_L298N = StateMotor
  MCU.State.State_Sonar_Array = StateSonar;
  */
}

void MCU_Control(String CMD){
  if(CMD == "Status"){
    MCU_Status();
  }
  else if(CMD == "Info"){
    Info();
  }
  else if(CMD == "SensorData"){
    SensorData();
  }
  else if(CMD == "Hutch"){
    tft.fillScreen(TFT_BLACK);
    tft.pushImage(0, 0,320,240, WHY_2);
  }
  // TEMP: Use property class to toggle this properties. 
  #ifdef LIVE_DEBUG
  else if(CMD == "SHOW_DEBUG"){
    if(MCU.Debug.SHOW_DEBUG != true){MCU.Debug.SHOW_DEBUG = true;}
    else{MCU.Debug.SHOW_DEBUG = false;}
  }
  #endif
}

//Sensor Data display mode
void SensorData(){
    #ifndef LIVE_DEBUG
    tft.fillScreen(TFT_BLACK);
    #endif
    tft.setCursor(0, 100);
    tft.setTextColor(TFT_WHITE, TFT_BLACK, true);
    tft.println("Sonar 1: ");
    tft.println("Sonar 2: ");
    tft.println("Sonar 3: ");
    tft.println("Infrared sensor 1: ");
    tft.println("Line Detect     1: ");
    #ifndef LIVE_DEBUG
    while(true){
    Sonar_Array_Sync();
    if(MCU.Sonar.SONAR_READY == true){
    tft.setCursor(65, 0);
    tft.print(MCU.Sonar.SONAR_DISTANCE[0]);
    tft.setCursor(65, 17);
    tft.print(MCU.Sonar.SONAR_DISTANCE[1]);
    tft.setCursor(65, 34);
    tft.print(MCU.Sonar.SONAR_DISTANCE[2]);
    tft.setCursor(65, 51);
    tft.print(IR_Sense());;
    }
  }
  #endif
  #ifdef LIVE_DEBUG
  if(MCU.Sonar.SONAR_READY == true){
  tft.setCursor(65, 100);
  tft.print(MCU.Sonar.SONAR_DISTANCE[0]);
  tft.print("     ");
  tft.setCursor(65, 117);
  tft.print(MCU.Sonar.SONAR_DISTANCE[1]);
  tft.print("     ");
  tft.setCursor(65, 134);
  tft.print(MCU.Sonar.SONAR_DISTANCE[2]);
  tft.print("     ");
  tft.setCursor(140, 151);
  tft.print(IR_Sense());
  tft.print("     ");
  tft.setCursor(140, 168);
  tft.print(analogRead(LED_DETECT_FWD));
  tft.print("     ");

  
  }
  #endif
}

//Checks rear sonars for potential targets/attacks
int AmbientSense(){
  if((MCU.Sonar.SONAR_DISTANCE[1] >= 20) && (MCU.Sonar.SONAR_DISTANCE[2] >= 20)){
    return 0;
  }
  else if((MCU.Sonar.SONAR_DISTANCE[1] < 8) && (MCU.Sonar.SONAR_DISTANCE[2] >= 15)){
    return 1;
  }
  else if((MCU.Sonar.SONAR_DISTANCE[1] >= 15) && (MCU.Sonar.SONAR_DISTANCE[2] < 8)){
    return 2;
  }
  else if((MCU.Sonar.SONAR_DISTANCE[1] < Sonar_Distance_EVADE) && (MCU.Sonar.SONAR_DISTANCE[2] < Sonar_Distance_EVADE)){
    return 3;
  }
}

//As the name suggests, logic behind the sumobot
void Drive_Logic(){
  String F = "Drive Logic";
  static String UART_SEND;
  static String PREV_UART_SEND;
  static bool TurnComplete = false;
  Sonar_Array_Sync();
  UART_SEND = HIM_INTF + ':' + Motor_Driver + ':';
  if(MCU.Sonar.SONAR_READY == true){
    switch(AmbientSense()){
      case 0:
      if(MCU.Sonar.SONAR_DISTANCE[0] >= Sonar_Distance_CLEAR){
        UART_SEND = UART_SEND + LFT_HRD + ':' + String(Drive_Speed_BASE) + ':' + String(Drive_Speed_BASE ) + ':';
        #ifdef DEBUG_DRIVE
        MCU.Debug.DebugUtil(0, F, String("Searching "), 0, 'I');
        #endif
      }
      else if(MCU.Sonar.SONAR_DISTANCE[0] <= Sonar_Distance_WARN){
        UART_SEND = UART_SEND + FWD + ':' + String(Drive_Speed_MED) + ':' + String(Drive_Speed_MED) + ':';
        #ifdef DEBUG_DRIVE
        MCU.Debug.DebugUtil(0, F, String("Potential Target! "), 0, 'I');
        #endif
      }
      else if(MCU.Sonar.SONAR_DISTANCE[0] <= Sonar_Distance_ATTACK){
        UART_SEND = UART_SEND + FWD + ':' + String(Drive_Speed_MAX) + ':' + String(Drive_Speed_MAX) + ':';
        #ifdef DEBUG_DRIVE
        MCU.Debug.DebugUtil(0, F, String("Target Locked! "), 0, 'I');
        #endif
      }
      break;
      case 1:
      //if(Timer(1500) == false){
      UART_SEND = UART_SEND + LFT_HRD + ':' + String(Drive_Speed_MED) + ':' + String(Drive_Speed_MED) + ':';
      #ifdef DEBUG_DRIVE
      MCU.Debug.DebugUtil(0, F, String("Target located in rear left! "), 0, 'I');
      #endif
      /*
      for(int x = 0; x < Drive_Speed_Rotate_TIMER_90; x++){
        Drive_Logic();
        delay(1);
      }
      */
      //}
      break;

      case 2:
      //if(Timer(1500) == false){
      UART_SEND = UART_SEND + RGT_HRD + ':' + String(Drive_Speed_MED) + ':' + String(Drive_Speed_MED) + ':';
      #ifdef DEBUG_DRIVE
      MCU.Debug.DebugUtil(0, F, String("Target located in rear right! "), 0, 'I');
      #endif
      /*
      for(int x = 0; x < Drive_Speed_Rotate_TIMER_90; x++){
        Drive_Logic();
        delay(1);
      }
      */
      //}
      break;

      case 3:
      //if(Timer(1500) == false){
      UART_SEND = UART_SEND + FWD + ':' + String(Drive_Speed_MAX) + ':' + String(Drive_Speed_MAX) + ':';
      #ifdef DEBUG_DRIVE
      MCU.Debug.DebugUtil(0, F, String("RUNNING AWAY"), 0, 'I');
      #endif
      /*
      for(int x = 0; x < Drive_Speed_Rotate_TIMER_90; x++){
        Drive_Logic();
        delay(1);
      }
      */
      //}
      break;
    }
    if(UART_SEND != PREV_UART_SEND){
    //Don't send bare commands. 
      if(UART_SEND.length() > (HIM_INTF + ':' + Motor_Driver + ':').length()){
        MCU.UART_INTF.UART_DATA(1, UART_SEND);
        PREV_UART_SEND = UART_SEND;
        //Dont flood uart with useless bs if it's the same command.
      }
    }
  }
}

//Info display mode
void Info(){
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(0, 0);
  tft.loadFont(AA_FONT_SMALL);
  tft.print("Build Date: ");
  tft.println(__DATE__);
  tft.print("Developed By: ");
  tft.println("Geel Sevathean");
}

/*
timer function to avoid complete system halts and
allows for simultaneous delays.
1000UL = 1s

if inputted time has passed, returns 1, if not, 0
*/
static unsigned long PrevTime = 0UL;

bool Timer(int TimerDelay){
  #ifdef DEBUG_TIMER
  String F = "Timer";
  #endif
  unsigned long CurrTime = millis();
  if(CurrTime - PrevTime >= TimerDelay){
    PrevTime = CurrTime;
    #ifdef DEBUG_TIMER
    MCU.Debug.DebugUtil(1, F, String("Timer Called: "), TimerDelay, 'I');
    #endif
    return true;
  }
  else{
    return false;  
  }
}