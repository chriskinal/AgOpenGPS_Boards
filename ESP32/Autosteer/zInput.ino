#include <SimpleKalmanFilter.h>

//Define sensor pin for current or pressure sensor
#define LOAD_SENSOR_PIN 39
#define WAS_SENSOR_PIN 36

float rawSensor, rawWAS;

SimpleKalmanFilter sensorFilter(2, 2, 0.01);

SimpleKalmanFilter wasFilter(5, 5, 0.01);

int value = 0;


void initInput() {
}

void inputHandler() {

  //WAS sensore
  value = analogRead(WAS_SENSOR_PIN);
  rawWAS = wasFilter.updateEstimate(value);

  // Load sensor?
  if (steerConfig.PressureSensor || steerConfig.CurrentSensor) {
    value = analogRead(LOAD_SENSOR_PIN);
    rawSensor = sensorFilter.updateEstimate(value);
  }
}


void calcSteerAngle() {
  steeringPosition = rawWAS;
  steeringPosition = (steeringPosition << 2);  //bit shift by 2  0 to 13610 is 0 to 5v
  helloSteerPosition = steeringPosition - 6800;
  //DETERMINE ACTUAL STEERING POSITION

  //convert position to steer angle. 32 counts per degree of steer pot position in my case
  //  ***** make sure that negative steer angle makes a left turn and positive value is a right turn *****
  if (steerConfig.InvertWAS) {
    steeringPosition = (steeringPosition - 6805 - steerSettings.wasOffset);  // 1/2 of full scale
    steerAngleActual = (float)(steeringPosition) / -steerSettings.steerSensorCounts;
  } else {
    steeringPosition = (steeringPosition - 6805 + steerSettings.wasOffset);  // 1/2 of full scale
    steerAngleActual = (float)(steeringPosition) / steerSettings.steerSensorCounts;
  }

  //Ackerman fix
  if (steerAngleActual < 0) steerAngleActual = (steerAngleActual * steerSettings.AckermanFix);
}
