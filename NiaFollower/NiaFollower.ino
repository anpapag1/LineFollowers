#include <EEPROM.h>
#include <QTRSensors.h>
#include "BluetoothSerial.h" // Add this at the top
#include <FastLED.h>

// Check if Bluetooth is available
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to enable it
#endif

// Create BluetoothSerial instance
BluetoothSerial SerialBT;

#define EEPROM_SIZE 512
#define SETTINGS_ADDRESS 0

struct Settings {
  double Kp;
  double Ki;
  double Kd;
  int baseSpeed;
};

// LED strip definitions
#define LED_PIN     18  // Connect your LED strip data pin to GPIO 13
#define NUM_LEDS    10   // Number of LEDs in your strip
#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB

// Create LED array
CRGB leds[NUM_LEDS];

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
double Kp ;  // Start with conservative values
double Ki ;
double Kd ;

// Serial command buffer
String inputString = "";
bool stringComplete = false;

uint16_t position;
int P, D, I, previousError, PIDvalue, error;
int lsp, rsp;
int baseSpeed = 50;  // Default speed
bool robotEnabled = false;  // Robot state
bool manualControl = false; // Manual control state
int direction = 0; // Direction of the robot manual controll
uint8_t ledState = 0;  // LED state

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
  Serial.begin(500000); // Initialize Serial Monitor

  if (!EEPROM.begin(EEPROM_SIZE)) {
    SerialBT.println("Failed to initialize EEPROM");
    return;
  }
  loadSettings();  // Load saved settings on startup
  
  delay(500);
  
  // Initialize LED strip
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(50); // Set initial brightness (0-255)
  
  // Show startup animation
  fill_solid(leds, NUM_LEDS, CRGB::Blue);
  FastLED.show();
  delay(500);
  
  // configure the sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){ 26, 25, 33, 32, 35, 34, 39, 36}, SensorCount);
  qtr.setEmitterPin(27);
  
  SerialBT.println("PID Line Follower Control");
  processCommand("h"); // Show help on startup
}

void loop() {
  updateLEDs(ledState); // Update LED strip with rainbow effect
  
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
    ledState = (ledState + 1) % 3; // Cycle through LED states (0, 1, 2)
    Serial.println(ledState);
    while (digitalRead(LED_BTN) != HIGH) {} // Wait for button release
  }
  
  if (digitalRead(STRT_BTN) != HIGH) {
    Serial.println("Start button pressed");
    robotEnabled = !robotEnabled;
    while(digitalRead(STRT_BTN) != HIGH) {}
  }

  if (manualControl) {
    motor_drive(baseSpeed - direction, baseSpeed + direction); // Manual control
  } else {
    position = readLine(position, 4090);
    error = 3500 - position; 
    PID_Linefollow(error);
  }
}

void processCommand(String command) {
  if (command.length() < 1) return;
  
  char cmd = command.charAt(0);
  float value = 0;
  
  if (command.length() > 1) {
    value = command.substring(1).toFloat();
  }
  
  switch (cmd) {
    case 'p': // proportional value for PID
      Kp = value;
      Serial.println("Kp set to " + String(Kp, 4));
      break;
    case 'i': // integral value for PID
      Ki = value;
      Serial.println("Ki set to " + String(Ki, 4));
      break;
    case 'd': // derivative value for PID
      Kd = value;
      Serial.println("Kd set to " + String(Kd, 4));
      break;
    case 's': // base speed
      baseSpeed = (int)value;
      Serial.println("Base speed set to " + String(baseSpeed));
      break;
    case 't': // robot state change
      robotEnabled = bool(value);
      Serial.println("Robot " + String(robotEnabled ? "enabled" : "disabled"));
      break;
    case 'w': // save settings to EEPROM
      saveSettings();
      break;
    case 'm': // manual control
      manualControl = bool(value);
      Serial.println("Manual control " + String(manualControl ? "enabled" : "disabled"));
      break;
    case 'd': // the app will send a sring of caracters example: d020005 where the 020 is the speed and the 005 is the direction
      baseSpeed = command.substring(1, 4).toInt();
      baseSpeed -= 150; // Adjust base speed to be between -150 and 150
      direction = command.substring(4, 7).toInt();
      direction -= 130; // Adjust direction to be between -130 and 130
      break;
    case 'l': // LED state change
      ledState = (ledState + 1) % 3; // Cycle through LED states (0, 1, 2)
      Serial.println("LED state changed to " + String(ledState));
      break;
    case 'h':
      SerialBT.println(String(Kp,4)+" "+String(Ki,4)+" "+String(Kd,4)+" "+String(baseSpeed));
      break;
  }
}

void saveSettings() {
  Settings settings = {
    Kp,
    Ki,
    Kd,
    baseSpeed
  };
  EEPROM.put(SETTINGS_ADDRESS, settings);
  EEPROM.commit();
  SerialBT.println("Settings saved to EEPROM");
  Serial.println("Settings saved to EEPROM");
  Serial.println("Kp: " + String(Kp, 4) + ", Ki: " + String(Ki, 4) + ", Kd: " + String(Kd, 4) + ", Base Speed: " + String(baseSpeed));
}

void loadSettings() {
  Settings settings;
  EEPROM.get(SETTINGS_ADDRESS, settings);
  Kp = settings.Kp;
  Ki = settings.Ki;
  Kd = settings.Kd;
  baseSpeed = settings.baseSpeed;
  SerialBT.println("Settings loaded from EEPROM");
  Serial.println("Settings loaded from EEPROM");
  Serial.println("Kp: " + String(Kp, 4) + ", Ki: " + String(Ki, 4) + ", Kd: " + String(Kd, 4) + ", Base Speed: " + String(baseSpeed));
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
  if (position < 7000 || PIDvalue == 0) {
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
  if (position > 0 || PIDvalue == 0) {
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

void updateLEDs(uint8_t state) {
  // make LED have 3 modes
  // 0: off, 1: solid color, 2: rainbow
  if (state == 0) {
    fill_solid(leds, NUM_LEDS, CRGB::Black);
  } else if (state == 1) {
    if (robotEnabled){
      if (0 < position && position < 7000){
        fill_solid(leds, NUM_LEDS, CRGB::Green);
      } else {
        fill_solid(leds, NUM_LEDS, CRGB::Red);
      }
    } else {
      fill_solid(leds, NUM_LEDS, CRGB::Orange);
    }
  } else if (state == 2) {
    // Rainbow effect
    static uint8_t hue = 0;
    for (int i = 0; i < NUM_LEDS; i++) {
      leds[i] = CHSV(hue + (i * 255 / NUM_LEDS), 255, 255);
    }
    hue++;
  } 
  FastLED.show();
}