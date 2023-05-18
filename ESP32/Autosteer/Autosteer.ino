#include <Arduino.h>
#include <TaskScheduler.h>
#include <WiFiManager.h>
#include <AsyncUDP.h>
#include <elapsedMillis.h>
#include <EEPROM.h>
#include <Wire.h>
#include "zNMEAParser.h"
/*  PWM Frequency ->
     490hz (default) = 0
     122hz = 1
     3921hz = 2
*/
#define PWM_Frequency 0

/////////////////////////////////////////////

// if not in eeprom, overwrite
#define EEP_Ident 2400

//   ***********  Motor drive connections  **************888
//Connect ground only for cytron, Connect Ground and +5v for IBT2

//Dir1 for Cytron Dir, Both L and R enable for IBT2
#define DIR1_RL_ENABLE 4

//PWM1 for Cytron PWM, Left PWM for IBT2
#define PWM1_LPWM 2

//Not Connected for Cytron, Right PWM for IBT2
#define PWM2_RPWM 3

//--------------------------- Switch Input Pins ------------------------
#define STEERSW_PIN 32
#define WORKSW_PIN 34
#define REMOTE_PIN 37

#define CONST_180_DIVIDED_BY_PI 57.2957795130823

//How many degrees before decreasing Max PWM
#define LOW_HIGH_DEGREES 3.0

IPAddress myip;

Scheduler ts;

void imuTask();

void gpsStream();

void inputHandler();

Task t1(TASK_IMMEDIATE, TASK_FOREVER, &imuTask, &ts, true);

Task t2(TASK_IMMEDIATE, TASK_FOREVER, &gpsStream, &ts, true);

Task t3(20, TASK_FOREVER, &inputHandler, &ts, true);

//loop time variables in microseconds
const uint16_t LOOP_TIME = 25;  //40Hz
uint32_t autsteerLastTime = LOOP_TIME;
uint32_t currentTime = LOOP_TIME;
const uint16_t WATCHDOG_THRESHOLD = 100;
const uint16_t WATCHDOG_FORCE_VALUE = WATCHDOG_THRESHOLD + 2;  // Should be greater than WATCHDOG_THRESHOLD
uint8_t watchdogTimer = WATCHDOG_FORCE_VALUE;

uint8_t aog2Count = 0;
float sensorReading;
float sensorSample;

elapsedMillis gpsSpeedUpdateTimer = 0;

//EEPROM
int16_t EEread = 0;

//Relays
bool isRelayActiveHigh = true;
uint8_t relay = 0, relayHi = 0, uTurn = 0;
uint8_t tram = 0;

//Switches
uint8_t remoteSwitch = 0, workSwitch = 0, steerSwitch = 1, switchByte = 0;

//On Off
uint8_t guidanceStatus = 0;
uint8_t prevGuidanceStatus = 0;
bool guidanceStatusChanged = false;

//speed sent as *10
float gpsSpeed = 0;
bool GGA_Available = false;  //Do we have GGA on correct port?

const bool invertRoll = true;  //Used for IMU with dual antenna


#define REPORT_INTERVAL 20  //BNO report time, we want to keep reading it quick & offen. Its not timmed to anything just give constant data.

//pwm variables
int16_t pwmDrive = 0, pwmDisplay = 0;
float pValue = 0;
float errorAbs = 0;
float highLowPerDeg = 0;

//steering variables
float steerAngleActual = 0;
float steerAngleSetPoint = 0;  //the desired angle from AgOpen
int16_t steeringPosition = 0;  //from steering sensor
float steerAngleError = 0;     //setpoint - actual
int16_t helloSteerPosition = 0;

//Steer switch button  ***********************************************************************************************************
uint8_t currentState = 1, reading, previous = 0;
uint8_t pulseCount = 0;  // Steering Wheel Encoder
bool encEnable = false;  //debounce flag
uint8_t thisEnc = 0, lastEnc = 0;

unsigned int portMy = 5120;  // port of this module

//Variables for settings
struct Storage {
  uint8_t Kp = 40;      // proportional gain
  uint8_t lowPWM = 10;  // band of no action
  int16_t wasOffset = 0;
  uint8_t minPWM = 9;
  uint8_t highPWM = 60;  // max PWM value
  float steerSensorCounts = 30;
  float AckermanFix = 1;  // sent as percent
};
Storage steerSettings;  // 11 bytes

//Variables for settings - 0 is false
struct Setup {
  uint8_t InvertWAS = 0;
  uint8_t IsRelayActiveHigh = 0;  // if zero, active low (default)
  uint8_t MotorDriveDirection = 0;
  uint8_t SingleInputWAS = 1;
  uint8_t CytronDriver = 1;
  uint8_t SteerSwitch = 0;  // 1 if switch selected
  uint8_t SteerButton = 0;  // 1 if button selected
  uint8_t ShaftEncoder = 0;
  uint8_t PressureSensor = 0;
  uint8_t CurrentSensor = 0;
  uint8_t PulseCountMax = 5;
  uint8_t IsDanfoss = 0;
  uint8_t IsUseY_Axis = 0;  //Set to 0 to use X Axis, 1 to use Y avis
};
Setup steerConfig;  // 9 bytes

bool Autosteer_running = true;  //Auto set off in autosteer setup

void steerSettingsInit() {
  // for PWM High to Low interpolator
  highLowPerDeg = ((float)(steerSettings.highPWM - steerSettings.lowPWM)) / LOW_HIGH_DEGREES;
}

void steerConfigInit() {
  if (steerConfig.CytronDriver) {
    pinMode(PWM2_RPWM, OUTPUT);
  }
}

void autosteerSetup() {
  //PWM rate settings. Set them both the same!!!!
  /*  PWM Frequency ->
       490hz (default) = 0
       122hz = 1
       3921hz = 2
  */
  if (PWM_Frequency == 0) {
    ledcSetup(PWM1_LPWM, 490, 8);
    ledcSetup(PWM2_RPWM, 490, 8);
  } else if (PWM_Frequency == 1) {
    ledcSetup(PWM1_LPWM, 122, 8);
    ledcSetup(PWM2_RPWM, 122, 8);
  } else if (PWM_Frequency == 2) {
    ledcSetup(PWM1_LPWM, 3921, 8);
    ledcSetup(PWM2_RPWM, 3921, 8);
  }

  //keep pulled high and drag low to activate, noise free safe
  pinMode(WORKSW_PIN, INPUT_PULLUP);
  pinMode(STEERSW_PIN, INPUT_PULLUP);
  pinMode(REMOTE_PIN, INPUT_PULLUP);
  pinMode(DIR1_RL_ENABLE, OUTPUT);

  // Disable digital inputs for analog input pins
  //pinMode(CURRENT_SENSOR_PIN, INPUT_DISABLE);
  //pinMode(PRESSURE_SENSOR_PIN, INPUT_DISABLE);

  EEPROM.get(0, EEread);  // read identifier

  if (EEread != EEP_Ident)  // check on first start and write EEPROM
  {
    EEPROM.put(0, EEP_Ident);
    EEPROM.put(10, steerSettings);
    EEPROM.put(40, steerConfig);
  } else {
    EEPROM.get(10, steerSettings);  // read the Settings
    EEPROM.get(40, steerConfig);
  }

  steerSettingsInit();

  if (Autosteer_running) {
    Serial.println("Autosteer running, waiting for AgOpenGPS");
    // Autosteer Led goes Red if ADS1115 is found
    //digitalWrite(AUTOSTEER_ACTIVE_LED, 0);
    //digitalWrite(AUTOSTEER_STANDBY_LED, 1);
  } else {
    Autosteer_running = false;  //Turn off auto steer if no ethernet (Maybe running T4.0)
                                //    if(!Ethernet_running)Serial.println("Ethernet not available");
    Serial.println("Autosteer disabled, GPS only mode");
    return;
  }
}  // End of Setup

void setup() {
  // Setup Serial Monitor
  Serial.begin(115200);
  Serial2.begin(115200);

  initWifi();

  initHandler();

  initIMU();

  autosteerSetup();
}

void loop() {

  ts.execute();
}