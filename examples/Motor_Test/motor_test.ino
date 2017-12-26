/*
Adafruit Arduino - Lesson 13. DC Motor
*/
 
#include "Boat.h"
#define NUM_SAMPLES 5

Boat my_boat(5,3,100);

void setup() 
{   
  Serial.begin(9600);
  while (! Serial);
  Serial.println("Motor setup");
  
} 
  
 
float read_voltage_on_bat()
{    
    int sum = 0,sample_count =0;
    float voltage;
    // take a number of analog samples and add them up
    while (sample_count < NUM_SAMPLES) {
        sum += analogRead(A0);
        sample_count++;
        delay(1);
    }
    // calculate the voltage
    // use 5.0 for a 5.0V ADC reference voltage
    // 5.015V is the calibrated reference voltage
    voltage = ((float)sum / (float)NUM_SAMPLES * 5) / 1024.0;
    return voltage;
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

  while(1){
  Serial.println(read_voltage_on_bat());

  Serial.println("boat go");
  my_boat.go();
  delay(1000);
  }

  /*
  Serial.println("boat stop");
  my_boat.stop();
  delay(5000);
  Serial.println("boat turn 20 degree");
  my_boat.turn(20);
  delay(5000);
  Serial.println("boat turn 50 degree");
  my_boat.turn(50);
  delay(5000);
  */
} 









