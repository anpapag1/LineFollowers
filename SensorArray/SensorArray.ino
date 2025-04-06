#include <QTRSensors.h>

QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

void setup()
{
  // configure the sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){ 26, 25, 33, 32, 35, 34, 39, 36}, SensorCount);
  qtr.setEmitterPin(27);

  Serial.begin(9600);
}


void loop()
{
  position = readLine(position);
  Serial.println(position);
}

int readLine(int lastposition){
  qtr.read(sensorValues);

  int position = 0;
  int count = 0;
  for (uint8_t i = 0; i < SensorCount; i++){
    if (sensorValues[i] > 4000) {
      count++;
      position += i * 1000; // multiply by 1000 to avoid float division
    }
  }

  // calculate the average position
  if (count != 0) {
    position = position / count; // average the position
  } else {
    position = lastposition; // no sensors detected, set position to 0
  }

  // for (uint8_t i = 0; i < SensorCount; i++){
  //   if (sensorValues[i] > 4000) {
  //     Serial.print("#");
  //   } else {
  //     Serial.print("_");
  //   }
  // }
  Serial.print("\t position: " );
  return position;
}