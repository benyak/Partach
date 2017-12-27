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
#include <LSM303.h> // Using the Pololu library for the Compass due to lack of calibration code on Adafruit

#include "Boat.h"

// Init the Boat object for controlling the engines.
Boat myBoat(5,6) ;

SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;
LSM303 compass; 

// In STANDBY mode we are not moving. Once we get coordinated we start 
// navigating until the state is changed to STANDBY.
enum state {STANDBY=0, NAVIGATE=1} boatState ;

// Destination coordinates
float dstLat = 0.00F ;//32.376948F;
float dstLon = 0.00F ;//34.863807F;

// Use this for FONA 800 and 808s
Adafruit_FONA fona = Adafruit_FONA(FONA_RST);
// Use this one for FONA 3G
//Adafruit_FONA_3G fona = Adafruit_FONA_3G(FONA_RST);

boolean readSMS(char* sms_buffer);

uint8_t type;

void setup() 
{
  // Start in STANDBY mode
  boatState = STANDBY ;
  
  while (!Serial);

  Serial.begin(115200);
  Serial.println(F("Looking for FONA808"));
  Serial.println(F("Initializing....(May take 3 seconds)"));

  fonaSerial->begin(57600);
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
  if (imeiLen > 0) 
  {
    Serial.print("Module IMEI: "); Serial.println(imei);
  }
  //set up the FONA to send a +CMTI notification when an SMS is received
  fonaSerial->print("AT+CNMI=2,1\r\n"); 

  // turn GPS on
  if (!fona.enableGPS(true))
    Serial.println(F("Failed to turn on"));
  else
    Serial.println(F("GPS turned on"));

  // Initialise the compass
  Wire.begin();
  compass.init();
  compass.enableDefault();
  
  /*
  Calibration values; the default values of +/-32767 for each axis
  lead to an assumed magnetometer bias of 0. Use the Calibrate example
  program to determine appropriate values for your particular unit.
  */
  compass.m_min = (LSM303::vector<int16_t>){-452,   -562,   -384};
  compass.m_max = (LSM303::vector<int16_t>){+572,   +331,   +553};
}

char smsBuffer[250];

void loop() 
{
  //look for a new SMS command
  if(readSMS(smsBuffer) == true)
  {
      Serial.println("Got SMS command:");
      Serial.println(smsBuffer);

      // Try to figure out what we got in the SMS.
      // Can be new coordinates that will kickout a rescue,
      // or a standby command that will stop navigation.
      char lonStr[10], latStr[10];
      int result = sscanf(smsBuffer, "%[^','],%s", latStr, lonStr);

      if(result ==2) // Found 2 coordinates, navigate to it
      {
        boatState = NAVIGATE ;
        // Update destination coordinates
        dstLat = atof(latStr);
        dstLon  = atof(lonStr);
        Serial.print("New Waypoint: ");Serial.print(dstLat,8);Serial.print(" ");Serial.print(dstLon,8);Serial.println(" 0");
      }
      else
      {
        // We stop navigating
        boatState = STANDBY ;
      }   
  }

 /*********************************************************************************************/
  
  float latitude, longitude, speed_kph, heading, altitude;

  // if you ask for an altitude reading, getGPS will return false if there isn't a 3D fix
  boolean gps_success = fona.getGPS(&latitude, &longitude, &speed_kph, &heading, &altitude);
  if (!gps_success) 
  {
    Serial.println("Waiting for FONA GPS 3D fix...");
    delay(1000);

    return;
  }

  //Get the compass heading
  compass.read();
  
  /* 
  When given no arguments, the heading() function returns the angular
  difference in the horizontal plane between a default vector and
  north, in degrees.
  Use the +X axis as a reference. 
  */
  heading = compass.heading((LSM303::vector<int>){1, 0, 0});
  Serial.print("Compass Heading: ");
  Serial.println(heading);
  
  int bearing = CalcBearing(dtor(latitude), dtor(longitude), dtor(dstLat), dtor(dstLon));
  Serial.print("Bearing to destination: ");
  Serial.print(bearing);
  Serial.println(" degrees");
  
  long distance = CalcDistance(latitude, longitude, dstLat, dstLon) ;
  Serial.print("Distance to destination: ");
  Serial.print(distance);
  Serial.println(" meters");

  Serial.print("State: "); Serial.println(boatState);
  Serial.print("Destination: "); Serial.print(dstLat); Serial.print(",");Serial.println(dstLon);
  
  // TODO - stop????
  if(distance < 20 )
  {
    if(boatState != STANDBY)
    {
      Serial.print("Tracking: ");Serial.print(latitude,4);Serial.print(" ");Serial.print(longitude,4);Serial.print(" ");Serial.println("1");
    }
    boatState = STANDBY ;
  }
  else
  {
    Serial.print("Tracking: ");Serial.print(latitude,4);Serial.print(" ");Serial.print(longitude,4);Serial.print(" ");Serial.println("0");
  }

  // Normalize turn
  int turn = bearing - heading ;
  if(turn < 0)
    turn += 360 ;

  Serial.print("Turn: ");
  Serial.println(turn);

  // Command the Servo to make the actual turn
  if(boatState == STANDBY)
    myBoat.stop() ;
  else
    myBoat.turn(turn) ;

  delay(2000);
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
  
  dist = 6371000 * c; //radius of the earth (6378000 meters)

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


void turnTo(int degrees)
{

}


boolean readSMS(char* sms_buffer)
{
    
    //look for sms message on slot #1, and delete  
    //read the first slot SMS
    uint16_t smslen;
    if (fona.readSMS(1, sms_buffer, 250, &smslen)) 
    { 
      // pass in buffer and max len!
      Serial.println(F("********* Got an SMS message ******* "));
      Serial.println(smsBuffer);
      char callerIDbuffer[32];  //we'll store the SMS sender number in here
      
      // Retrieve SMS sender address/phone number.
      if (! fona.getSMSSender(1, callerIDbuffer, 31)) 
      {
        Serial.println("Didn't find SMS message!");
      }
      else
      {
        Serial.print(F("FROM: ")); Serial.println(callerIDbuffer);
      }

      //delete the message
      if (fona.deleteSMS(1)) 
      {
        Serial.println(F("Delete OK!"));
      } 
      else 
      {
        Serial.println(F("Couldn't delete SMS")); 
      }

      return true;
    }

    //no messages
    return false;
}
