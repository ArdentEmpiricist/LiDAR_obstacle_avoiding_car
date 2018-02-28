#include <Servo.h>

#include "Adafruit_VL53L0X.h"

Adafruit_VL53L0X lox = Adafruit_VL53L0X();
VL53L0X_RangingMeasurementData_t measure;
Servo myservo;

int pos = 60;
const int stopDist = 200;
int rightDistance = 0, leftDistance = 0;

#define ENA 5
#define ENB 6
#define IN1 7
#define IN2 8
#define IN3 9
#define IN4 11
#define carSpeed 90
#define carSpeedTurn 140


void forward() {
  analogWrite(ENA, carSpeed);
  analogWrite(ENB, carSpeed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  Serial.println("Forward");
}

void back() {
  analogWrite(ENA, carSpeedTurn);
  analogWrite(ENB, carSpeedTurn);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  Serial.println("Back");
}

void left() {
  analogWrite(ENA, carSpeedTurn);
  analogWrite(ENB, carSpeedTurn);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  Serial.println("Left");
}

void right() {
  analogWrite(ENA, carSpeedTurn);
  analogWrite(ENB, carSpeedTurn);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  Serial.println("Right");
}

void stop() {
  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);
  Serial.println("Stop!");
}

int distanceTest() {
  lox.rangingTest(&measure, false);
  delayMicroseconds(10);
  float Fdistance = measure.RangeMilliMeter;
  delayMicroseconds(10);
  return (int)Fdistance;
}

/*int checkWay() {
  myservo.write(10);
  delay(1000);
  rightDistance = distanceTest();

  delay(500);
  myservo.write(90);
  delay(1000);
  myservo.write(180);
  delay(1000);
  leftDistance = distanceTest();

  delay(500);
  myservo.write(90);
  delay(2000);

  if (rightDistance > leftDistance) {
    right();
    delay(600);
  }
  else if (rightDistance < leftDistance) {
    left();
    delay(600);
  }
  else if ((rightDistance <= 20) || (leftDistance <= 20)) {
    back();
    delay(180);
  }
  else {
    forward();
  }
  }*/


void setup() {

  Serial.begin(115200);

  // wait until serial port opens for native USB devices
  while (! Serial) {
    delay(1);
  }

  Serial.println("VL53L0X");
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while (1);
  }
  // power
  Serial.println(F("VL53L0X"));

  myservo.attach(3, 1000, 2000);
  myservo.write(60);
}

void loop()
{


  for (pos = 60; pos <= 120; pos += 1)  {
    myservo.write(pos);
    //  Serial.print("Reading a measurement... ");
    lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

    if (measure.RangeStatus != 4) {  // phase failures have incorrect data
      Serial.print("Position: "); Serial.print(pos); Serial.print("   "); Serial.print("Distance (mm): "); Serial.println(measure.RangeMilliMeter);
    } else {
      Serial.println(" out of range ");
    }
    if (measure.RangeMilliMeter <= stopDist) {
      stop();
      
      delay(500);
      
      myservo.write(20);
      delay(1000);
      int rightDistance = distanceTest();
      Serial.println(rightDistance);

      delay(500);
     
      myservo.write(160);
      delay(1000);
      int leftDistance = distanceTest();
      Serial.println(leftDistance);

      delay(500);
     
      if (rightDistance > leftDistance) {
        right();
        Serial.println("Right");
        delay(600);
      }
      else if (rightDistance < leftDistance) {
        left();
        Serial.println("Left");
        delay(600);
      }
      else if ((rightDistance <= 20) || (leftDistance <= 20)) {
        back();
        Serial.println("Back");
        delay(180);
      }
      else {
        forward();
      }
    }
    else {
      forward();
    }
  }

  for (pos = 120; pos >= 60; pos -= 1)  {
    myservo.write(pos);
    //   Serial.print("Reading a measurement... ");
    lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

    if (measure.RangeStatus != 4) {  // phase failures have incorrect data
      Serial.print("Position: "); Serial.print(pos); Serial.print("   "); Serial.print("Distance (mm): "); Serial.println(measure.RangeMilliMeter);
    } else {
      Serial.println(" out of range ");
    }
    if (measure.RangeMilliMeter <= stopDist) {
      stop();
      
      delay(500);
      
      myservo.write(20);
      delay(1000);
      int rightDistance = distanceTest();
      Serial.println(rightDistance);

      delay(500);
     
      myservo.write(160);
      delay(1000);
      int leftDistance = distanceTest();
      Serial.println(leftDistance);

      delay(500);
     
      if (rightDistance > leftDistance) {
        right();
        Serial.println("Right");
        delay(600);
      }
      else if (rightDistance < leftDistance) {
        left();
        Serial.println("Left");
        delay(600);
      }
      else if ((rightDistance <= 20) || (leftDistance <= 20)) {
        back();
        Serial.println("Back");
        delay(180);
      }
      else {
        forward();
      }
    }
    else {
      forward();
    }
  }
}




