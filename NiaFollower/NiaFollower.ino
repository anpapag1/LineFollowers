#include <EEPROM.h>
#include <QTRSensors.h>
#include "BluetoothSerial.h" // Add this at the top

// Check if Bluetooth is available
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to enable it
#endif

// Create BluetoothSerial instance
BluetoothSerial SerialBT;

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
double Kp = 0.015;  // Start with conservative values
double Ki = 0;
double Kd = 0.027;

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

  // Initialize Bluetooth Serial
  SerialBT.begin("NiaFollower"); // Bluetooth device name
  
  delay(500);
  
  // configure the sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){ 26, 25, 33, 32, 35, 34, 39, 36}, SensorCount);
  qtr.setEmitterPin(27);
  
  SerialBT.println("PID Line Follower Control");
  processCommand("h"); // Show help on startup
}

void loop() {
  // Process serial commands
  if (stringComplete) {
    processCommand(inputString);
    inputString = "";
    stringComplete = false;
  }

  // Read Bluetooth serial data
  while (SerialBT.available()) {
    char inChar = (char)SerialBT.read();
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
  if (command.length() < 1) return;
  
  char cmd = command.charAt(0);
  float value = 0;
  
  if (command.length() > 1) {
    value = command.substring(1).toFloat();
  }
  
  switch (cmd) {
    case 'p':
      Kp = value;
      break;
    case 'i':
      Ki = value;
      break;
    case 'd':
      Kd = value;
      break;
    case 's':
      baseSpeed = (int)value;
      break;
    case 't':
      robotEnabled = bool(value);
      break;
    case 'h':
      SerialBT.println(String(Kp,4)+" "+String(Ki,4)+" "+String(Kd,4)+" "+String(baseSpeed));
      break;
  }
}

void PID_Linefollow(int error) {
  P = error;
  I = I + error;
  D = error - previousError;
  
  float PIDvalue = Kp * P + Ki * I + Kd * D;
  previousError = error;
  
  rsp = constrain(baseSpeed + PIDvalue, -255, 255);
  lsp = constrain(baseSpeed - PIDvalue, -255, 255);

  motor_drive(rsp, lsp);
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

void motor_drive(int right, int left) {
  digitalWrite(STBY, robotEnabled ? HIGH : LOW);

  // Right motor
  if (position < 7000) {
    if (right > 0) {
      analogWrite(AIN1, right);
      analogWrite(AIN2, 0);
    } else {
      analogWrite(AIN1, 0);
      analogWrite(AIN2, abs(right));
    }
  } else {
    analogWrite(AIN1, 0);
    analogWrite(AIN2, 30);
  }
  
  // Left motor
  if (position > 0) {
    if (left > 0) {
      analogWrite(BIN1, left);
      analogWrite(BIN2, 0);
    } else {
      analogWrite(BIN1, 0);
      analogWrite(BIN2, abs(left));
    }
  } else {
    analogWrite(BIN1, 0);
    analogWrite(BIN2, 30);
  }
}