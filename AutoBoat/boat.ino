/*
Adafruit Arduino - Lesson 13. DC Motor
*/
 
#include "Boat.h"

Boat my_boat(5,3);

void setup() 
{   
  Serial.begin(9600);
  while (! Serial);
  Serial.println("Motor setup");
  
} 
 
 
void loop() 
{ 
  /*if (Serial.available())
  {
    int speed = Serial.parseInt();
    if (speed >= 0 && speed <= 255)
    {
      analogWrite(motorPinLeft, speed);
      analogWrite(motorPinRight, speed);
    }
  }*/

  Serial.println("boat go");
  my_boat.go();
  delay(10000);
  Serial.println("boat stop");
  my_boat.stop();
  delay(5000);
  Serial.println("boat turn 20 degree");
  my_boat.turn(20);
  delay(5000);
  Serial.println("boat turn 50 degree");
  my_boat.turn(50);
  delay(5000);

  while(1) { }
} 









