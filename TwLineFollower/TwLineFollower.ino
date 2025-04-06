#include <QTRSensors.h>
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#define LED_BUILTIN 2

#define AIN1 16  // Left Motor
#define AIN2 17
#define BIN1 2   // Right Motor
#define BIN2 15
#define STBY 4   // Standby pin

// Optional button pins (if you want to implement them)
#define CALIBRATE_BTN 22
#define START_BTN 23

const int offsetA = 1;
const int offsetB = 1;

// Initializing motors.  The library will allow you to initialize as many
// motors as you have memory for.  If you are using functions like forward
// that take 2 motors as arguements you can either write new functions or
// call the function more than once.

QTRSensors qtr;
BluetoothSerial SerialBT;

// Update sensor count constant
const uint8_t SensorCount = 8;  // Changed from 5 to 8 sensors
uint16_t sensorValues[SensorCount];
int threshold[SensorCount];

float Kp = 40;
float Ki = 0;
float Kd = 0;

uint8_t multiP = 1;
uint8_t multiI  = 1;
uint8_t multiD = 1;
uint8_t Kpfinal;
uint8_t Kifinal;
uint8_t Kdfinal;
float Pvalue;
float Ivalue;
float Dvalue;

boolean onoff = false;

int val, cnt = 0, v[3];

uint16_t position;
int P, D, I, previousError, PIDvalue, error;
int lsp, rsp;
int lfspeed = 50;

void setup()
{
  // configure the sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){26, 25, 33, 32, 35, 34, 39, 36}, SensorCount); // Changed to 8 sensors
  qtr.setEmitterPin(27); // Add emitter pin configuration

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode
  
  Serial.begin(115200);
  SerialBT.begin();
  Serial.println("Bluetooth Started! Ready to pair...");
  // analogRead() takes about 0.1 ms on an AVR.
  // 0.1 ms per sensor * 4 samples per sensor read (default) * 6 sensors
  // * 10 reads per calibrate() call = ~24 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on
  
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    threshold[i] = (qtr.calibrationOn.minimum[i] + qtr.calibrationOn.maximum[i])/2;
    Serial.print(threshold[i]);
    Serial.print("  ");
  }
  Serial.println();

  delay(1000);
  pinMode(STBY, OUTPUT);
}

void loop()
{
  if (SerialBT.available()){
    while(SerialBT.available() == 0);
    valuesread();
    processing();
  }
  if (onoff == true){
    digitalWrite(STBY, HIGH);
  }
  else if(onoff == false){
    digitalWrite(STBY, LOW); // Disable standby to turn off motors
  }
  robot_control();
}
void robot_control() {
  position = qtr.readLineBlack(sensorValues);
  error = 3500 - position;  // Changed from 2000 to 3500 for 8 sensors
  
  // Update all-black line detection condition for 8 sensors
  while(sensorValues[0]>=980 && sensorValues[1]>=980 && 
        sensorValues[2]>=980 && sensorValues[3]>=980 && 
        sensorValues[4]>=980 && sensorValues[5]>=980 && 
        sensorValues[6]>=980 && sensorValues[7]>=980) {
    if(previousError>0){       //Turn left if the line was to the left before
      motor_drive(-230,230);
    }
    else{
      motor_drive(230,-230); // Else turn right
    }
    position = qtr.readLineBlack(sensorValues);
  }
  
  PID_Linefollow(error);
}
void PID_Linefollow(int error){
    P = error;
    I = I + error;
    D = error - previousError;
    
    Pvalue = (Kp/pow(10,multiP))*P;
    Ivalue = (Ki/pow(10,multiI))*I;
    Dvalue = (Kd/pow(10,multiD))*D; 

    float PIDvalue = Pvalue + Ivalue + Dvalue;
    previousError = error;

    lsp = lfspeed + PIDvalue;
    rsp = lfspeed - PIDvalue;
    
    lsp = constrain(lsp, -255, 255);
    rsp = constrain(rsp, -255, 255);

    motor_drive(lsp , rsp);

    Serial.print("Left Speed: ");
    Serial.print(lsp);
    Serial.print("\t  Right Speed: ");
    Serial.print(rsp);
    Serial.print("\t Error: ");
    Serial.println(error);

  }
//This void delimits each instruction.
//The  Arduino knows that for each instruction it will receive 2 bytes.
void valuesread()  {
  val = SerialBT.read();
  cnt++;
  v[cnt] = val;
  if (cnt == 2)
    cnt = 0;
}

//In this void the the 2 read values are assigned.
void  processing() {
  int a = v[1];
  if (a == 1) {
    Kp = v[2];
    Serial.print("Kp: ");
    Serial.println(Kp);
  }
  if (a == 2) {
    multiP = v[2];
    Serial.print("multiP: ");
    Serial.println(multiP);
  }
  if (a == 3) {
    Ki = v[2];
    Serial.print("Ki: ");
    Serial.println(Ki);
  }
  if (a == 4) {
    multiI = v[2];
    Serial.print("multiI: ");
    Serial.println(multiI);
  }
  if (a == 5) {
    Kd  = v[2];
    Serial.print("Kd: ");
    Serial.println(Kd);
  }
  if (a == 6) {
    multiD = v[2];
    Serial.print("multiD: ");
    Serial.println(multiD);
  }
  if (a == 7)  {
    onoff = v[2];
    Serial.print("onoff: ");
    Serial.println(onoff);
  }
}
void motor_drive(int left, int right){
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