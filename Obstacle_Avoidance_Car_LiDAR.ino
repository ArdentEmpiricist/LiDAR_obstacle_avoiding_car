//Includes code from Elegoo (www.elegoo.com) and Adafruit (www.adafruit.com)

#include <Servo.h>

#include "Adafruit_VL53L0X.h"

Adafruit_VL53L0X lox = Adafruit_VL53L0X();
VL53L0X_RangingMeasurementData_t measure;
Servo myservo;

int pos = 50;
const int stopDist = 350;
int rightDistance = 0, leftDistance = 0;
#define HIGH_SPEED
#define ENA 5
#define ENB 6
#define IN1 7
#define IN2 8
#define IN3 9
#define IN4 11
#define carSpeed 85
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
  delay(1);
  float Fdistance = measure.RangeMilliMeter;
  delay(1);
  return (int)Fdistance;
}

int checkWay() {
  myservo.write(10);
  delay(1000);
  int rightDistance = distanceTest();
  Serial.println(rightDistance);

  delay(500);

  myservo.write(170);
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
  else if ((rightDistance <= 150) || (leftDistance <= 150)) {
    back();
    Serial.println("Back");
    delay(180);
  }
  else if (rightDistance == leftDistance) {
    back();
    Serial.println("Back");
    delay(180);
  }
  else {
    forward();
  }
}


void setup() {

  Serial.begin(115200);

  while (! Serial) {
    delay(1);
  }

  Serial.println("VL53L0X");
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while (1);
  }

  Serial.println(F("VL53L0X"));

  myservo.attach(3, 1000, 2000);
  myservo.write(50);
}

void loop()
{


  for (pos = 50; pos <= 130; pos += 3)  {
    myservo.write(pos);

    lox.rangingTest(&measure, false); 

    if (measure.RangeStatus != 4) {  
      Serial.print("Position: "); Serial.print(pos); Serial.print("   "); Serial.print("Distance (mm): "); Serial.println(measure.RangeMilliMeter);
    }
    else {
      Serial.println(" out of range ");
    }
    if (measure.RangeMilliMeter <= stopDist) {
      stop();

      delay(500);

      checkWay();
    }

    else {
      forward();
    }
  }

  for (pos = 130; pos >= 50; pos -= 3)  {
    myservo.write(pos);
   
    lox.rangingTest(&measure, false);

    if (measure.RangeStatus != 4) {  
      Serial.print("Position: "); Serial.print(pos); Serial.print("   "); Serial.print("Distance (mm): "); Serial.println(measure.RangeMilliMeter);
    }
    else {
      Serial.println(" out of range ");
    }
    if (measure.RangeMilliMeter <= stopDist) {
      stop();

      delay(500);

      checkWay();
    }

    else {
      forward();
    }
  }
}



