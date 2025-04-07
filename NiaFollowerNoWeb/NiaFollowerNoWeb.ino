#include <EEPROM.h>
#include <QTRSensors.h>

#define LED_BUILTIN 2
#define STRT_BTN 23 // Pin for the start button
#define LED_BTN 22 // Pin for the led button

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
double Kp = 0.02;  // Start with conservative values
double Ki = 0;
double Kd = 0.03;

// Serial command buffer
String inputString = "";
bool stringComplete = false;

uint16_t position;
int P, D, I, previousError, PIDvalue, error;
int lsp, rsp;
int baseSpeed = 50;  // Default speed
bool robotEnabled = false;  // Robot state

void setup() {
  pinMode(STRT_BTN, INPUT_PULLUP);
  pinMode(LED_BTN, INPUT_PULLUP);
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
  Serial.begin(500000);
  Serial.println("PID Line Follower Control");
  processCommand("h"); // Show help on startup
  }

void loop() {
  // Process serial commands
  if (stringComplete) {
    processCommand(inputString);
    inputString = "";
    stringComplete = false;
  }

  // Read serial data
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }

  if (digitalRead(LED_BTN) != HIGH ) {
  }
  
  if (digitalRead(STRT_BTN) != HIGH) {
    Serial.println("Start button pressed");
    robotEnabled = !robotEnabled;
    while(digitalRead(STRT_BTN) != HIGH) {}
  }

  position = readLine(position, 4090);

  error = 3500 - position; 
  PID_Linefollow(error);
}

void processCommand(String command) {
  Serial.println("Command: "+command);
  if (command.length() < 1) return;
  
  char cmd = command.charAt(0);
  float value = 0;
  
  if (command.length() > 1) {
    value = command.substring(1).toFloat();
  }
  
  switch (cmd) {
    case 'p':
      Kp = value;
      Serial.println("Kp set to: "+String(Kp,4));
      break;
    case 'i':
      Ki = value;
      Serial.println("Ki set to: "+String(Ki,4));
      break;
    case 'd':
      Kd = value;
      Serial.println("Kd set to: "+String(Kd,4));
      break;
    case 's':
      baseSpeed = (int)value;
      Serial.println("Base speed set to: "+String(baseSpeed));
      break;
    case 'h':
      Serial.println("Commands:");
      Serial.println("p = "+String(Kp,4)+" // Set Kp");
      Serial.println("i = "+String(Ki,4)+" // Set Ki");
      Serial.println("d = "+String(Kd,4)+" // Set Kd");
      Serial.println("s = "+String(baseSpeed)+" // Set base speed");
      Serial.println("h - Show this help");
      break;
  }
}

void PID_Linefollow(int error) {
  P = error;
  I = I + error;
  D = error - previousError;
  
  float PIDvalue = Kp * P + Ki * I + Kd * D;
  previousError = error;
  
  lsp = constrain(baseSpeed + PIDvalue, -255, 255);
  rsp = constrain(baseSpeed - PIDvalue, -255, 255);

  motor_drive(lsp, rsp);
}

int readLine(int lastposition, int blackLineValue){
  qtr.read(sensorValues);

  int position = 0;
  int count = 0;
  for (uint8_t i = 0; i < SensorCount; i++){
    if (sensorValues[i] > blackLineValue) {
      count++;
      position += i * 1000; // multiply by 1000 to avoid float division
    }
  }

  // calculate the average position
  if (count == 8) {
    robotEnabled = false;
  } else if (count != 0) {
    position = position / count; // average the position
  } else {
    if (lastposition > 3500) {
      position = 7000; // no sensors detected, set position to last known
    } else {
      position = 0; // no sensors detected, set position to 0
    }
  }

  // for (uint8_t i = 0; i < SensorCount; i++){
  //   if (sensorValues[i] > blackLineValue) {
  //     Serial.print("#");
  //   } else {
  //     Serial.print("_");
  //   }
  // }
  // Serial.print("\t position: " );
  // Serial.println(position);
  
  return position;
}

void motor_drive(int left, int right) {
  digitalWrite(STBY, robotEnabled ? HIGH : LOW);

  // Left motor
  if (position < 7000) {
    if (left > 0) {
      analogWrite(AIN1, left);
      analogWrite(AIN2, 0);
    } else {
      analogWrite(AIN1, 0);
      analogWrite(AIN2, abs(left));
    }
  } else {
    analogWrite(AIN1, 0);
    analogWrite(AIN2, 30);
  }
  
  // Right motor
  if (position > 0) {
    if (right > 0) {
      analogWrite(BIN1, right);
      analogWrite(BIN2, 0);
    } else {
      analogWrite(BIN1, 0);
      analogWrite(BIN2, abs(right));
    }
  } else {
    analogWrite(BIN1, 0);
    analogWrite(BIN2, 30);
  }
}