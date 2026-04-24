#define WHEEL_COUNT 3

int pwmPins[WHEEL_COUNT] = {3, 6, 9};
int dir1Pins[WHEEL_COUNT] = {4, 7, 10};
int dir2Pins[WHEEL_COUNT] = {5, 8, 11};

void setup() {
  Serial.begin(115200);
  for(int i=0;i<WHEEL_COUNT;i++){
    pinMode(pwmPins[i], OUTPUT);
    pinMode(dir1Pins[i], OUTPUT);
    pinMode(dir2Pins[i], OUTPUT);
  }
}

void setMotor(int motor, int speed){
  if(speed > 0){
    digitalWrite(dir1Pins[motor], HIGH);
    digitalWrite(dir2Pins[motor], LOW);
    analogWrite(pwmPins[motor], speed);
  } else if(speed < 0){
    digitalWrite(dir1Pins[motor], LOW);
    digitalWrite(dir2Pins[motor], HIGH);
    analogWrite(pwmPins[motor], -speed);
  } else {
    analogWrite(pwmPins[motor], 0);
  }
}

void loop() {
  if(Serial.available() >= 3){
    int speeds[WHEEL_COUNT];
    for(int i=0;i<WHEEL_COUNT;i++){
      speeds[i] = Serial.read() - 128; // map 0-255 to -127 to 127
      setMotor(i, speeds[i]);
    }
  }
}
