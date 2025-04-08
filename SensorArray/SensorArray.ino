// Sensor array pins
const uint8_t sensorPins[] = { 26, 25, 33, 32, 35, 34, 39, 36 };
const uint8_t SensorCount = 8;
uint8_t sensorValues[SensorCount];  // Changed to uint8_t since digital reads are 0 or 1
uint16_t position;

void setup() {
  Serial.begin(9600);
  // Set all sensor pins as inputs
  for(uint8_t i = 0; i < SensorCount; i++) {
    pinMode(sensorPins[i], INPUT);
  }
}

int readLine(int lastposition) {
  // Read all sensors digitally
  for(uint8_t i = 0; i < SensorCount; i++) {
    sensorValues[i] = digitalRead(sensorPins[i]);
  }

  int position = 0;
  int count = 0;
  
  // Calculate position
  for(uint8_t i = 0; i < SensorCount; i++) {
    if(sensorValues[i] == HIGH) {  // Changed from analog threshold to digital HIGH
      count++;
      position += i * 1000;
    }
  }

  // Calculate average position
  if(count != 0) {
    position = position / count;
  } else {
    position = lastposition;
  }

  // Debug output
  for(uint8_t i = 0; i < SensorCount; i++) {
    if(sensorValues[i] == HIGH) {
      Serial.print("#");
    } else {
      Serial.print("_");
    }
    Serial.print("\t");
  }
  Serial.print("position: ");
  
  return position;
}

void loop() {
  position = readLine(position);
  Serial.println(position);
  delay(100);
}