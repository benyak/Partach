/***************************************************
  This is an example for our Adafruit FONA Cellular Module

  Designed specifically to work with the Adafruit FONA
  ----> http://www.adafruit.com/products/1946
  ----> http://www.adafruit.com/products/1963
  ----> http://www.adafruit.com/products/2468
  ----> http://www.adafruit.com/products/2542

  These cellular modules use TTL Serial to communicate, 2 pins are
  required to interface
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include "Adafruit_FONA.h"

#define FONA_RX 2
#define FONA_TX 3
#define FONA_RST 4

// this is a large buffer for replies
char replybuffer[255];

// We default to using software serial. If you want to use hardware serial
// (because softserial isnt supported) comment out the following three lines 
// and uncomment the HardwareSerial line
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <LiquidCrystal.h>

SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;

/* Assign a unique ID to this compass sensor at the same time */
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);

// Use this for FONA 800 and 808s
Adafruit_FONA fona = Adafruit_FONA(FONA_RST);
// Use this one for FONA 3G
//Adafruit_FONA_3G fona = Adafruit_FONA_3G(FONA_RST);

uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);
uint8_t type;

// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
const int rs = 12, en = 11, d4 = 8, d5 = 7, d6 = 6, d7 = 5;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

void setup() 
{
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  
  while (!Serial);

  Serial.begin(115200);
  Serial.println(F("Looking for FONA808"));
  Serial.println(F("Initializing....(May take 3 seconds)"));

  fonaSerial->begin(4800);
  if (! fona.begin(*fonaSerial)) {
    Serial.println(F("Couldn't find FONA"));
    while (1);
  }
  type = fona.type();
  Serial.println(F("FONA is OK"));
  Serial.print(F("Found "));
  switch (type) {
    case FONA800L:
      Serial.println(F("FONA 800L")); break;
    case FONA800H:
      Serial.println(F("FONA 800H")); break;
    case FONA808_V1:
      Serial.println(F("FONA 808 (v1)")); break;
    case FONA808_V2:
      Serial.println(F("FONA 808 (v2)")); break;
    case FONA3G_A:
      Serial.println(F("FONA 3G (American)")); break;
    case FONA3G_E:
      Serial.println(F("FONA 3G (European)")); break;
    default: 
      Serial.println(F("???")); break;
  }
  
  // Print module IMEI number.
  char imei[16] = {0}; // MUST use a 16 character buffer for IMEI!
  uint8_t imeiLen = fona.getIMEI(imei);
  if (imeiLen > 0) {
    Serial.print("Module IMEI: "); Serial.println(imei);
  }

  // turn GPS on
  if (!fona.enableGPS(true))
    Serial.println(F("Failed to turn on"));
  else
    Serial.println(F("GPS turned on"));

  // Initialise the compass
  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
  else
    Serial.println("LSM303 compass detected.");
}

void loop() 
{
  ////////////// LCD 
  // set the cursor to column 0, line 1
  // (note: line 1 is the second row, since counting begins with 0):
  lcd.setCursor(0, 0);
  
  float latitude, longitude, speed_kph, heading, speed_mph, altitude;

  // if you ask for an altitude reading, getGPS will return false if there isn't a 3D fix
  boolean gps_success = fona.getGPS(&latitude, &longitude, &speed_kph, &heading, &altitude);

  if (gps_success) 
  {

//  Serial.print("GPS lat:");
//  Serial.println(latitude, 6);
//  Serial.print("GPS long:");
//  Serial.println(longitude, 6);
//  Serial.print("GPS speed KPH:");
//  Serial.println(speed_kph);
//  Serial.print("GPS speed MPH:");
//  speed_mph = speed_kph * 0.621371192;
//  Serial.println(speed_mph);
    Serial.print("GPS heading:");
    Serial.println(heading);
//  Serial.print("GPS altitude:");
//  Serial.println(altitude);
  } 
  else 
  {
    Serial.println("Waiting for FONA GPS 3D fix...");
    // print NO GPS on LCD
    lcd.print("No GPS Fix )-:");
    return;
  }

  //Get the compass heading
  heading = calcCompassHeading() ;
  
  float lat2 = 32.376948F ;//32.370282F;
  float lon2 = 34.863807F ;//34.861653F;
  int bearing = CalcBearing(latitude, longitude, lat2, lon2);
  Serial.print("Bearing to office: ");
  Serial.print(bearing);
  Serial.println(" degrees");
  
  long distance = CalcDistance(latitude, longitude, lat2, lon2) ;
  Serial.print("Distance to office: ");
  Serial.print(distance);
  Serial.println(" meters");

  int turn = bearing - heading ;
  Serial.print("Turn: ");
  Serial.print(turn);
  if(turn < 0)
    Serial.println(" degrees LEFT");
  else
    Serial.println(" degrees RIGHT");

  // Command the Servo to make the actual turn
  turnTo(turn) ;

  // Print data to LCD
  lcd.clear();
  lcd.print(latitude);
  lcd.print(" : ");
  lcd.print(longitude);
  lcd.setCursor(0, 1);
  lcd.print("H:");
  lcd.print(bearing);
  lcd.print(" T:");
  lcd.print(turn);
  lcd.print(" D:");
  lcd.print(distance);

  delay(5000);
}

char readBlocking() 
{
  while (!Serial.available());
  return Serial.read();
}

uint16_t readnumber() 
{
  uint16_t x = 0;
  char c;
  while (! isdigit(c = readBlocking())) 
  {
    //Serial.print(c);
  }
  Serial.print(c);
  x = c - '0';
  while (isdigit(c = readBlocking())) 
  {
    Serial.print(c);
    x *= 10;
    x += c - '0';
  }
  return x;
}

uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout) 
{
  uint16_t buffidx = 0;
  boolean timeoutvalid = true;
  if (timeout == 0) timeoutvalid = false;

  while (true) {
    if (buffidx > maxbuff) 
    {
      //Serial.println(F("SPACE"));
      break;
    }

    while (Serial.available()) 
    {
      char c =  Serial.read();

      //Serial.print(c, HEX); Serial.print("#"); Serial.println(c);

      if (c == '\r') continue;
      if (c == 0xA) {
        if (buffidx == 0)   // the first 0x0A is ignored
          continue;

        timeout = 0;         // the second 0x0A is the end of the line
        timeoutvalid = true;
        break;
      }
      buff[buffidx] = c;
      buffidx++;
    }

    if (timeoutvalid && timeout == 0) 
    {
      //Serial.println(F("TIMEOUT"));
      break;
    }
    delay(1);
  }
  buff[buffidx] = 0;  // null term
  return buffidx;
}


/////////////////////////////////////
//                                 //
//         GPS Code                //
//                                 //
/////////////////////////////////////

//convert degrees to radians
double dtor(double fdegrees)
{
  return(fdegrees * PI / 180);
}

//Convert radians to degrees
double rtod(double fradians)
{
  return(fradians * 180.0 / PI);
}

//Calculate distance form lat1/lon1 to lat2/lon2 using haversine formula
//Note lat1/lon1/lat2/lon2 must be in radians
//Returns distance in feet
long CalcDistance(double lat1, double lon1, double lat2, double lon2)
{
  double dlon, dlat, a, c;
  double dist = 0.0;
  dlon = dtor(lon2 - lon1);
  dlat = dtor(lat2 - lat1);
  a = pow(sin(dlat/2),2) + cos(dtor(lat1)) * cos(dtor(lat2)) * pow(sin(dlon/2),2);
  c = 2 * atan2(sqrt(a), sqrt(1-a));
  
////  dist = 20925656.2 * c;  //radius of the earth (6378140 meters) in feet 20925656.2
  dist = 6371000 * c;

  return( (long) dist + 0.5);
}

//Calculate bearing from lat1/lon1 to lat2/lon2
//Note lat1/lon1/lat2/lon2 must be in radians
//Returns bearing in degrees
int CalcBearing(double lat1, double lon1, double lat2, double lon2)
{
  //determine angle
  double bearing = atan2(sin(lon2-lon1)*cos(lat2), (cos(lat1)*sin(lat2))-(sin(lat1)*cos(lat2)*cos(lon2-lon1)));
  //convert to degrees
  bearing = rtod(bearing);
  //use mod to turn -90 = 270
  bearing = fmod((bearing + 360.0), 360);
  return (int) bearing + 0.5;
}

void ComputeDestPoint(double lat1, double lon1, int iBear, int iDist, double *lat2, double *lon2)
{
  double bearing = dtor((double) iBear);
  double dist = (double) iDist / 6371000;
  lat1 = dtor(lat1);
  lon1 = dtor(lon1);
  *lat2 = asin(sin(lat1)* cos(dist)+ cos(lat1)* sin(dist)*cos(bearing));
  *lon2 = lon1 + atan2(sin(bearing)*sin(dist)*cos(lat1), cos(dist)-sin(lat1)*sin(*lat2));
  *lon2 = fmod( *lon2 + 3 * PI, 2*PI )- PI;
  *lon2 = rtod( *lon2);
  *lat2 = rtod( *lat2);
}

void turnTo(int degrees)
{

}

float calcCompassHeading(void) 
{
  /* Get a new sensor event */ 
  sensors_event_t event; 
  mag.getEvent(&event);
  
  float Pi = 3.14159;
  
  // Calculate the angle of the vector y,x
  float heading = (atan2(event.magnetic.y,event.magnetic.x) * 180) / Pi;
  
  // Normalize to 0-360
  if (heading < 0)
  {
    heading = 360 + heading;
  }
  Serial.print("Compass Heading: ");
  Serial.println(heading);

  return heading;
}

