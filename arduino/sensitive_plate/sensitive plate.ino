#include <stdint.h>
#include "TouchScreen.h"
#include <Servo.h>

#define YP A2
#define XM A3
#define YM 8
#define XP 9

float setpointX = 0;
float setpointY = 0;
const float width = 140;
const float height = 250;
float time, timePrev;
float errorX, errorY, previousErrorX, previousErrorY;
TSPoint p;
float Px, Ix, Dx, Py, Iy, Dy;
int numValidPoints = 0;
int numInvalidPoints = 0;

const double Kpx = .5;
const double Kix = 0.04;
const double Kdx = .3;

const double Kpy = .4;
const double Kiy = 0.05;
const double Kdy = .2;

const int xServoPin = 10;
const int yServoPin = 11;
Servo xServo;
Servo yServo;
const int flatXAngle = 80;
const int flatYAngle = 90;

TouchScreen ts = TouchScreen(XP, YP, XM, YM, 500);

const int pointsPerCycle = 150;
float radialVelocity = 1;
int index = 0;
float trajectoryUpdateTime, lastTrajectoryUpdateTime;

const int inputWindowSize = 10;
float filteredX = 0;
float filteredY = 0;
float sumX = 0;
float sumY = 0;
float readingsX[inputWindowSize];
float readingsY[inputWindowSize];

int mode = 0;

void setup() {
  Serial.begin(9600);
  xServo.attach(xServoPin);
  yServo.attach(yServoPin);
  xServo.write(flatXAngle);
  yServo.write(flatYAngle);
  time = millis();
  lastTrajectoryUpdateTime = millis();
}

void loop() {
  time = millis();
  float dt = (time - timePrev) / 1000;
  updateSetpoint();

  if (dt > 0.02) {
    timePrev = time;
    p = ts.getPoint();

    if(p.z == 0) {
      numValidPoints = 0;
      numInvalidPoints++;
    } else {
      numValidPoints++;
      numInvalidPoints = 0;
    }

    if(numInvalidPoints >= 100) {
      xServo.write(flatXAngle);
      yServo.write(flatYAngle);
      Ix = 0;
      Iy = 0;
      return;
    }

    if(numInvalidPoints >= 300) {
      xServo.detach();
      yServo.detach();
    }

    if(numValidPoints < 3) {
      return;
    }

    if (p.z >= 10) {
      float x = map(p.x, 0, 1024, -82, 82);
      float y = map(p.y, 0, 1024, 75.5, -75.5);

      sumX = sumX - readingsX[0];
      for(int i = 0; i< inputWindowSize - 1; i++) {
        readingsX[i] = readingsX[i+1];
      }
      readingsX[inputWindowSize -1] = x;
      sumX = sumX + x;
      filteredX = sumX/inputWindowSize;

      sumY = sumY - readingsY[0];
      for(int i = 0; i< inputWindowSize - 1; i++) {
        readingsY[i] = readingsY[i+1];
      }
      readingsY[inputWindowSize -1] = y;
      sumY = sumY + y;
      filteredY = sumY/inputWindowSize;

      errorX = setpointX - filteredX;
      errorY = setpointY - filteredY;

      Px = Kpx*errorX;
      Ix += Kix*errorX*dt;
      Ix = clip2(Ix, -10, 10);
      Dx = Kdx*(errorX-previousErrorX)/dt;
      float PIDx = Px+Ix+Dx;

      Py = Kpy*errorY;
      Iy += Kiy*errorY*dt;
      Iy = clip2(Iy, -height/2, height/2);
      Dy = Kdy*(errorY-previousErrorY)/dt;
      float PIDy = Py+Iy+Dy;

      int xOutput = int(round(map(PIDx, -width/2, width/2, -50, 50)));
      int yOutput = int(round(map(PIDy, -height/2, height/2, -40, 40)));
      xOutput = clip(xOutput,-50,50);
      yOutput = clip(yOutput,-40,40);

      xServo.write(flatXAngle + xOutput);
      yServo.write(flatYAngle + yOutput);

      previousErrorX = errorX;
      previousErrorY = errorY;
    }
  }
}

void circle(float radius, int i) {
    float angle = float(i)/pointsPerCycle * M_PI * 2;
    setpointX = radius * cos(angle);
    setpointY = radius * sin(angle);
}

void ellipse(float a, float b, int i) {
    float angle = float(i)/pointsPerCycle * M_PI * 2;
    setpointX = a * cos(angle);
    setpointY = b * sin(angle);
}

void line(float length, int i) {
  if (i < pointsPerCycle/2) {
    setpointX = index/length/2;
  } else {
    setpointX = -index/length/2;
  }
  setpointY = 0;
}

int cornerIndex = 1;
void fourCorners(float l) {
  float w = 32;
  float h = 13;
  switch(cornerIndex) {
    case 1:
      setpointX = w;
      setpointY = h;
      break;
    case 2:
      setpointX = -w;
      setpointY = h;
      break;
    case 3:
      setpointX = -w;
      setpointY = -h;
      break;
    case 4:
      setpointX = w;
      setpointY = -h;
      break;
  }
  cornerIndex++;
  if(cornerIndex > 4) cornerIndex = 1;
}

void updateSetpoint() {
  float dt = (time - lastTrajectoryUpdateTime)/1000;
  float updateIncrement = 1/radialVelocity/pointsPerCycle;

  switch(mode) {
    case 0:
      setpointX = 0;
      setpointY = 0;
      break;
    case 1:
      if (dt > updateIncrement) {
        circle(10, index);
        lastTrajectoryUpdateTime = time;
        index+=int(round(dt/updateIncrement));
        if (index > pointsPerCycle) {
          index = 0;
        }
      }
      break;
    case 2:
      if(dt > 2) {
        fourCorners(30);
        lastTrajectoryUpdateTime = time;
      }
      break;
    case 3:
      if (dt > 1/radialVelocity/pointsPerCycle) {
        ellipse(15,10, index);
        lastTrajectoryUpdateTime = time;
        index+=int(round(dt/updateIncrement));
        if (index > pointsPerCycle) {
          index = 0;
        }
      }
      break;
    default:
      setpointX = 0;
      setpointY = 0;
      break;
  }
}

int clip(int value, int minimum, int maximum) {
  if (value > maximum) return maximum;
  if (value < minimum) return minimum;
  return value;
}

float clip2(float value, float minimum, float maximum) {
  if (value > maximum) return maximum;
  if (value < minimum) return minimum;
  return value;
}
