#include <EEPROM.h>
#include <QTRSensors.h>

#define LED_BUILTIN 2
#define STRT_BTN 23 // Pin for the start button
#define CAL_BTN 22 // Pin for the calibration button

// Sensor array
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

// Motors
#define AIN1 16  // Motor A input 1
#define AIN2 17  // Motor A input 2
#define BIN1 2   // Motor B input 1
#define BIN2 15  // Motor B input 2
#define STBY 4   // Standby pin

// PID parameters
double Kp = 0.015;  // Start with conservative values
double Ki = 0.0;
double Kd = 0.001;

float Pvalue;
float Ivalue;
float Dvalue;

uint16_t position;
int P, D, I, previousError, PIDvalue, error;
int lsp, rsp;
int baseSpeed = 50;  // Default speed
bool robotEnabled = false;  // Robot state
bool Calibration = false;  

void setup() {
  pinMode(STRT_BTN, INPUT_PULLUP);
  pinMode(CAL_BTN, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);

  delay(500);
  
  // configure the sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){ 26, 25, 33, 32, 35, 34, 39, 36}, SensorCount);
  qtr.setEmitterPin(27);
  Serial.begin(9600);
  
  analogReadResolution(12);  // Set ADC resolution to 12 bits
  analogSetAttenuation(ADC_11db);  // Set ADC attenuation for higher voltage range

  // calibrate sensors min max
  calibrateSensors();
}

void calibrateSensors() {
  Serial.println("Calibration mode activated");
  digitalWrite(STBY, LOW); // Disable motors 
  digitalWrite(LED_BUILTIN, HIGH);
  for (uint16_t i = 0; i < 200; i++) qtr.calibrate();
  digitalWrite(LED_BUILTIN, LOW);
  Serial.println("Calibration complete");
  
  // print calibration values
  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();
  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  delay(1000);
}

void loop() {
  if (digitalRead(CAL_BTN) != HIGH || Calibration) {
    Calibration = false;
    calibrateSensors();
  }
  
  if (digitalRead(STRT_BTN) != HIGH) {
    Serial.println("Start button pressed");
    robotEnabled = !robotEnabled;
    while(digitalRead(STRT_BTN) != HIGH) {}
  }

  position = qtr.readLineBlack(sensorValues);
  for (uint8_t i = 0; i < SensorCount; i++) {
    if (sensorValues[i] > 800) {
      Serial.print("#");
    } else if(sensorValues[i] > 200) {
      Serial.print("-");
    } else {
      Serial.print("_");
    }
  }
  Serial.print('\t');
  Serial.println(position);

  error = 3500 - position; 
  PID_Linefollow(error);
}

void PID_Linefollow(int error) {
  P = error;
  I = I + error;
  D = error - previousError;
  
  Pvalue = Kp * P;
  Ivalue = Ki * I;
  Dvalue = Kd * D; 

  float PIDvalue = Pvalue + Ivalue + Dvalue;
  previousError = error;

  lsp = baseSpeed + PIDvalue;
  rsp = baseSpeed - PIDvalue;
  
  lsp = constrain(lsp, -255, 255);
  rsp = constrain(rsp, -255, 255);

  motor_drive(lsp, rsp);
}

void motor_drive(int left, int right) {
  digitalWrite(STBY, robotEnabled ? HIGH : LOW);

  // Left motor
  if (left > 0) {
    analogWrite(AIN1, left);
    analogWrite(AIN2, 0);
  } else {
    analogWrite(AIN1, 0);
    analogWrite(AIN2, abs(left));
  }
  
  // Right motor
  if (right > 0) {
    analogWrite(BIN1, right);
    analogWrite(BIN2, 0);
  } else {
    analogWrite(BIN1, 0);
    analogWrite(BIN2, abs(right));
  }
}