#include <ArduinoHardware.h>
#include <ros.h>
#include <Wire.h>    //required by Omni3MD.cpp
#include <Omni3MD.h>
#include <TimerOne.h>
#include <Servo.h> 
#include <EEPROM.h>

// Messages
#include "minho_team_ros/hardwareInfo.h"
#include "minho_team_ros/controlInfo.h"
#include "minho_team_ros/teleop.h"
#include "minho_team_ros/imuConfig.h"
#include "minho_team_ros/omniConfig.h"
// Services
#include "minho_team_ros/requestResetEncoders.h"
#include "minho_team_ros/requestResetIMU.h"
#include "minho_team_ros/requestSetIMULinTable.h"
#include "minho_team_ros/requestSetOmniProps.h"

using minho_team_ros::requestResetEncoders;
using minho_team_ros::requestResetIMU;
using minho_team_ros::requestSetOmniProps;
using minho_team_ros::requestSetIMULinTable;

Servo myservo;  // create servo object to control a servo 

/////////////////////////////MOTORS VARS//////////////////////////
#define DRIBLLER1 5  
#define MM1 4 
#define DRIBLLER2 6                      
#define MM2 7 
#define KICKPIN 8
#define BALLPIN A3
#define SERRVOPIN 9
//////////////////////////////////////////////////////////////////
#define Buzzer 53
#define Red 23
#define Green 24
#define Blue 25
//////////////////////////////////////////////////////////////////
#define CamBattery A0
#define PcBattery A1
#define FreeWheelButton A2
////////////////////////////OMNI3MD VARS//////////////////////////

//constants definitions
#define OMNI3MD_ADDRESS 0x30        //default factory address
#define BROADCAST_ADDRESS 0x00      //i2c broadcast address
#define M1  1            //Motor1
#define M2  2            //Motor2
#define M3  3            //Motor3
////////////////////////////EEPROM VARS//////////////////////////
#define PMSB 0
#define PLSB 1
#define IMSB 2
#define ILSB 3
#define DMSB 4
#define DLSB 5
#define BMSB 6
#define BLSB 7
#define NMSB 8
#define NLSB 9
#define MMSB 10
#define MLSB 11
#define IMULINSTEP 12
// EEPROM bytes from addr 12 to 360/read(IMULINSTEP) are reserved
void readEncoders();
void setupOmni();
void setupMotorsBoard();
void writePIDtoEEPROM();
void readPIDfromEEPROM();
void controlInfoCallback(const minho_team_ros::controlInfo& msg);
void teleopCallback(const minho_team_ros::teleop& msg);
void resetEncodersService(const requestResetEncoders::Request &req, requestResetEncoders::Response &res);
void resetIMUReferenceService(const requestResetIMU::Request &req, requestResetIMU::Response &res);
void setOmniPropsService(const requestSetOmniProps::Request &req, requestSetOmniProps::Response &res);
void setIMUTableService(const requestSetIMULinTable::Request &req, requestSetIMULinTable::Response &res);
Omni3MD omni;                       //declaration of object variable to control the Omni3MD

//Variables to read from Omni3MD
float temperature=0;   // temperature reading
byte firm_int=0;       // the firmware version of your Omni3MD board (visit http://www.botnroll.com/omni3md/ to check if your version is updated)
byte firm_dec=0;       // the firmware version of your Omni3MD board (visit http://www.botnroll.com/omni3md/ to check if your version is updated)
byte firm_dev=0;       // the firmware version of your Omni3MD board (visit http://www.botnroll.com/omni3md/ to check if your version is updated)
byte ctrl_rate=0;      // the control rate for your motors defined at calibration (in times per second)
int enc1_max;          // maximum count for encoder 1 at calibration, for the defined control rate
int enc2_max;          // maximum count for encoder 2 at calibration, for the defined control rate
int enc3_max;          // maximum count for encoder 3 at calibration, for the defined control rate

int P = 851;
int I = 71;
int D = 101;
int B = 700;
int N = 1300;
int M = 351;

//400,1300,0
int enc1_old = 0;
int enc2_old = 0;
int enc3_old = 0;

int baterryLimitTime = 3000;
unsigned long baterryTimeStamp = 0;
boolean batteryLow = false;
int baterryLowLimitTime = 250;
unsigned long baterryLowTimeStamp = 0;
///////////////////////END OMNI3MD VARS///////////////////////////

float bussolaValorNew = 0,bussolaValor = 0,bussolaValorOld = 0;
int dataSendLimitTime = 20;
unsigned long dataSendTimeStamp = 0;
int encoderLimitTime = 20;
unsigned long encoderTimeStamp = 0;
int kickTime = 0;
int maxKick = 35;

unsigned long loopTime = 0;

float maxVoltsBatteryPC = 13,maxVoltsBatteryCam = 8;
float maxBatteryPC = 567,maxBatteryCam = 537;

boolean batteryLowPC = false,batteryLowCam = false;

int baterryLowPCLimitTime = 450;
unsigned long baterryLowPCTimeStamp = 0;

int baterryLowCamLimitTime = 550;
unsigned long baterryLowCamTimeStamp = 0;

int comunicationTimeOutLimitTime = 200;
unsigned long comunicationTimeOutTimeStamp = 0;

int servo01 = 180,servo02 = 120;
bool teleop_active = false;
// ######### ROS DATA ######### 
ros::NodeHandle  nh;
minho_team_ros::hardwareInfo hwinfo_msg;
ros::Publisher hardware_info_pub("hardwareInfo", &hwinfo_msg);
ros::Subscriber<minho_team_ros::controlInfo> control_info_sub("controlInfo" , controlInfoCallback);
ros::Subscriber<minho_team_ros::teleop> teleop_info_sub("teleop" , teleopCallback);

ros::ServiceServer<requestResetEncoders::Request, requestResetEncoders::Response> server_resetEnc("requestResetEncoders",&resetEncodersService);
ros::ServiceServer<requestResetIMU::Request, requestResetIMU::Response> server_resetIMU("requestResetIMU",&resetIMUReferenceService);

void setup() {
  setupOmni();
  setupMotorsBoard();

  pinMode(KICKPIN, OUTPUT);
  digitalWrite(KICKPIN, LOW);
  
  Timer1.attachInterrupt( timerIsr ); // Timer to kick
  Timer1.stop();

  Serial.begin(57600);
  Serial.flush();
  Serial1.begin(9600);
  Serial1.flush();
  
  resetEncoders();
  readPIDfromEEPROM();
  
  tone(Buzzer, 4000,50);
  delay(150);
  tone(Buzzer, 4000,50);
  delay(300);
  tone(Buzzer, 4000,100);
  delay(300);
  tone(Buzzer, 4000,100);
  
  nh.initNode();
  nh.advertise(hardware_info_pub);
  nh.subscribe(control_info_sub);
  nh.subscribe(teleop_info_sub);
  nh.advertiseService(server_resetEnc);
  nh.advertiseService(server_resetIMU);
}

void loop() {
  
  // Detect free_wheel button
  if(analogRead(FreeWheelButton)>500 && !hwinfo_msg.free_wheel_activated){
    omni.stop_motors();
    hwinfo_msg.free_wheel_activated = true;
  } else if(analogRead(FreeWheelButton)<500 && hwinfo_msg.free_wheel_activated) hwinfo_msg.free_wheel_activated = false;
  // Safety timeout
  if(millis()-comunicationTimeOutTimeStamp>comunicationTimeOutLimitTime){
    omni.stop_motors();
    comunicationTimeOutTimeStamp = millis();
  }
  // Read IMU Value
  if(Serial1.available()>0){
    String S1 = Serial1.readStringUntil('\n');
    char buffer[10];
    S1.toCharArray(buffer, 10);
    hwinfo_msg.imu_value = atof(buffer);
  }
  // Read Batteries
  if(millis()-baterryTimeStamp>baterryLimitTime){
    hwinfo_msg.battery_camera = analogRead(CamBattery) * maxVoltsBatteryCam / maxBatteryCam;
    hwinfo_msg.battery_pc = analogRead(PcBattery) * maxVoltsBatteryPC / maxBatteryPC;
    
    hwinfo_msg.battery_main = omni.read_battery();
    hwinfo_msg.battery_main -=3;//Erro de leitura
    
    if(hwinfo_msg.battery_main<20 && hwinfo_msg.battery_main>4) batteryLow = true;
    else batteryLow = false;
    
    if(hwinfo_msg.battery_pc<10.5 && hwinfo_msg.battery_pc>5) batteryLowPC = true;
    else batteryLowPC = false;
    
    if(hwinfo_msg.battery_camera<5.5 && hwinfo_msg.battery_camera>3) batteryLowCam = true;
    else batteryLowCam = false;
    
    baterryTimeStamp = millis();
  }
  
  // Read Encoders
  if(millis()-encoderTimeStamp>encoderLimitTime){
    readEncoders();
    encoderTimeStamp = millis();
  }
  
  // Publish Data
  if(millis()-dataSendTimeStamp>dataSendLimitTime){
    if(analogRead(BALLPIN)>500) hwinfo_msg.ball_sensor = 1;
    else hwinfo_msg.ball_sensor = 0;
    sendDados();
    dataSendTimeStamp = millis();
  }
  
  // Master Battery low level sound warning
  if(batteryLow){
    if(millis()-baterryLowTimeStamp>baterryLowLimitTime){
      tone(Buzzer, 4000,baterryLowLimitTime/2);
      baterryLowTimeStamp = millis();
    }
  }
  
  // PC Battery low level sound warning
  if(batteryLowPC){
    if(millis()-baterryLowPCTimeStamp>baterryLowPCLimitTime){
      tone(Buzzer, 4500,baterryLowPCLimitTime/2);
      baterryLowPCTimeStamp = millis();
    }
  }
  
  // Cam Battery low level sound warning
  if(batteryLowCam){
    if(millis()-baterryLowCamTimeStamp>baterryLowCamLimitTime){
      tone(Buzzer, 5000,baterryLowCamLimitTime/2);
      baterryLowCamTimeStamp = millis();
    }
  }
  
  //Update ROS queues
  nh.spinOnce();
}

void timerIsr()
{
    digitalWrite(KICKPIN, LOW);
    Timer1.stop();
}

void resetEncoders()
{
   omni.set_enc_value(M1,0); // resets to zero the encoder value [byte encoder, word encValue]
   delay(1); // waits 1ms for Omni3MD to process information
   omni.set_enc_value(M2,0);
   delay(1); // waits 1ms for Omni3MD to process information
   omni.set_enc_value(M3,0);
   delay(1); // waits 1ms for Omni3MD to process information
}

// DataSend.ino
void sendDados()
{
   hardware_info_pub.publish(&hwinfo_msg);
   enc1_old = hwinfo_msg.encoder_1;
   enc2_old = hwinfo_msg.encoder_2;
   enc3_old = hwinfo_msg.encoder_3;
}

// Encoders.ino

void readEncoders()
{
  omni.read_encoders(&hwinfo_msg.encoder_1,&hwinfo_msg.encoder_2,&hwinfo_msg.encoder_3);
}

// setupMotorsBoard.ino
void setupMotorsBoard()
{
  //Setup Hardware control Pins
  pinMode(MM1, OUTPUT);   
  pinMode(MM2, OUTPUT); 
  pinMode(DRIBLLER1,OUTPUT);
  pinMode(DRIBLLER2,OUTPUT);

  pinMode(BALLPIN,INPUT);
  pinMode(PcBattery,INPUT);
  pinMode(CamBattery,INPUT);
  pinMode(FreeWheelButton,INPUT);
  
  digitalWrite(MM1,LOW);   
  digitalWrite(MM2, LOW);  
  
  analogWrite(DRIBLLER1, 0);   //PWM Speed Control
  analogWrite(DRIBLLER2, 0);   //PWM Speed Control

  myservo.attach(SERRVOPIN);  // attaches the servo on pin 22 to the servo object
  myservo.write(180);
}

// SetupOmni.ino
void setupOmni()
{
    omni.i2c_connect(OMNI3MD_ADDRESS);  // set i2c connection
    delay(10);                          // pause 10 milliseconds
    omni.stop_motors();                 // stops all motors
    delay(10);
    omni.read_firmware(&firm_int,&firm_dec,&firm_dev); // read firmware version value
    delay(5);
    ctrl_rate=omni.read_control_rate();   // read the control rate value
    delay(5);
    enc1_max=omni.read_enc1_max();        // read encoder1 maximum value at calibration (usefull for detecting a faulty encoder)
    delay(5);
    enc2_max=omni.read_enc2_max();        // read encoder1 maximum value at calibration (usefull for detecting a faulty encoder)
    delay(5);
    enc3_max=omni.read_enc3_max();        // read encoder1 maximum value at calibration (usefull for detecting a faulty encoder)
    delay(5);
    
    omni.set_PID(P,I,D); // Adjust paramenters for PID control [word Kp, word Ki, word Kd]
    delay(15);                 // 15ms pause required for Omni3MD eeprom writing

    omni.set_ramp(B,N,M);   // set acceleration ramp and limiar take off parameter gain[word ramp_time, word slope, word Kl] 
    delay(15);                 // 10ms pause required for Omni3MD eeprom writing
    delay(1000);
}

void writePIDtoEEPROM()
{
  EEPROM.write(PMSB,P>>8);
  EEPROM.write(PLSB,P);
  EEPROM.write(IMSB,I>>8);
  EEPROM.write(ILSB,I);
  EEPROM.write(DMSB,D>>8);
  EEPROM.write(DLSB,D);
  EEPROM.write(BMSB,B>>8);
  EEPROM.write(BLSB,B);
  EEPROM.write(NMSB,N>>8);
  EEPROM.write(NLSB,N);
  EEPROM.write(MMSB,M>>8);
  EEPROM.write(MLSB,M);
}

void readPIDfromEEPROM()
{
  P = (EEPROM.read(PMSB)<<8)|(EEPROM.read(PLSB)); 
  I = (EEPROM.read(IMSB)<<8)|(EEPROM.read(ILSB)); 
  D = (EEPROM.read(DMSB)<<8)|(EEPROM.read(DLSB));
  B = (EEPROM.read(BMSB)<<8)|(EEPROM.read(BLSB)); 
  N = (EEPROM.read(NMSB)<<8)|(EEPROM.read(NLSB)); 
  M = (EEPROM.read(MMSB)<<8)|(EEPROM.read(MLSB));
}
void controlInfoCallback(const minho_team_ros::controlInfo& msg)
{
  if((teleop_active && msg.is_teleop)||(!teleop_active && !msg.is_teleop)){
    comunicationTimeOutTimeStamp = millis();

    int movement_dir = 360-msg.movement_direction; //Ao contrario do simulador
    if(!hwinfo_msg.free_wheel_activated) omni.mov_omni(msg.linear_velocity
                                                      ,msg.angular_velocity
                                                      ,movement_dir);
    else omni.stop_motors();
    
    //dribler1
    if(msg.dribbler_on) { digitalWrite(MM1,HIGH); digitalWrite(MM2,HIGH); }
    else { digitalWrite(MM1,LOW); digitalWrite(MM2,LOW); }
    // Velocities to apply to dribblers
    //if(direct_dribler1!="2") analogWrite(DRIBLLER1, vel_dribler1.toInt());     //PWM Speed Control  
    //if(direct_dribler2!="2") analogWrite(DRIBLLER2, vel_dribler2.toInt());     //PWM Speed Control  
    
    int kickTime = (msg.kick_strength*maxKick)/100;
    if(kickTime>0)
    {
      if(kickTime>maxKick)kickTime = maxKick;
      if(kickTime<0)kickTime = 0;
      
      digitalWrite(KICKPIN, HIGH);
      Timer1.initialize(kickTime*1000);
    }
      
  }  
}

void teleopCallback(const minho_team_ros::teleop& msg)
{
  teleop_active = msg.set_teleop;  
}


void resetEncodersService(const requestResetEncoders::Request &req, requestResetEncoders::Response &res)
{
  omni.set_enc_value(M1,0); // resets to zero the encoder value [byte encoder, word encValue]
  delay(1); // waits 1ms for Omni3MD to process information
  omni.set_enc_value(M2,0);
  delay(1); // waits 1ms for Omni3MD to process information
  omni.set_enc_value(M3,0);
  delay(1); // waits 1ms for Omni3MD to process information   
} 

void resetIMUReferenceService(const requestResetIMU::Request &req, requestResetIMU::Response &res)
{
  Serial1.write("r");
  delay(10);
  Serial1.write("r");
}

void setOmniPropsService(const requestSetOmniProps::Request &req, requestSetOmniProps::Response &res)
{
  P = req.omniConf.P; I = req.omniConf.I; D = req.omniConf.D;
  B = req.omniConf.B; N = req.omniConf.N; M = req.omniConf.M;
  
  omni.set_PID(P,I,D); // Adjust paramenters for PID control [word Kp, word Ki, word Kd]
  delay(15);                 // 15ms pause required for Omni3MD eeprom writing
  omni.set_ramp(B,N,M);   // set acceleration ramp and limiar take off parameter gain[word ramp_time, word slope, word Kl] 
  delay(15);                 // 10ms pause required for Omni3MD eeprom writing  
  
  writePIDtoEEPROM();
  delay(500);
}

void setIMUTableService(const requestSetIMULinTable::Request &req, requestSetIMULinTable::Response &res)
{
  
}
/* TODO: Implement ROS service/message for:
 - Servo
 - Write/Read Omni Config to EEPROM
 - 
*/
