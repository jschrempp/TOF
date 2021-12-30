/*
  Read an 8x8 array of distances from the VL53L5CX
  By: Nathan Seidle
  SparkFun Electronics
  Date: October 26, 2021
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example shows how to read all 64 distance readings at once.

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/18642

*/

#include <Wire.h>

#include <SparkFun_VL53L5CX_Library.h> //http://librarymanager/All#SparkFun_VL53L5CX

#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm_; 

// XXX use D7 LED for status
const int LED_PIN = D7;

SparkFun_VL53L5CX myImager;
VL53L5CX_ResultsData measurementData; // Result data class structure, 1356 byes of RAM

int imageResolution = 0; //Used to pretty print output
int imageWidth = 0; //Used to pretty print output

// XXX enable the system thread to make sure that loop() does not block for cloud ops
//SYSTEM_THREAD(ENABLED);

const int EYESERVOX = 0;
const int EYESERVOY = 1;
const int LIDLEFTTOPSERVO = 2;
const int LIDLEFTBOTSERVO = 3;
const int LIDRIGHTTOPSERVO = 4;
const int LIDRIGHTBOTSERVO = 5;

const int EYEMINX = -121;
const int EYEMAXX = 141;
const int EYEMINY = -82;
const int EYEMAXY = 98;
const int LIDLEFTTOPOPEN = 287;
const int LIDLEFTBOTOPEN = 450;
const int LIDRIGHTTOPOPEN = 469;
const int LIDRIGHTBOTOPEN = 265;

void moveEyes (int x, int y){

    double xPos = map(x, 0, 100, EYEMINX, EYEMAXX);
    double yPos = map(y, 0, 100, EYEMINY, EYEMAXY);

    pwm_.setPWM(EYESERVOX, 0, xPos);
    pwm_.setPWM(EYESERVOY, 0, yPos);   

}


void setup()
{
  // XXX turn on D7 LED to indicate that we are in setup()
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  
  Serial.begin(115200);
  delay(1000);
  Serial.println("SparkFun VL53L5CX Imager Example");

  Wire.begin(); //This resets to 100kHz I2C
  Wire.setClock(400000); //Sensor has max I2C freq of 400kHz 
  
  Serial.println("Initializing sensor board. This can take up to 10s. Please wait.");
  if (myImager.begin() == false)
  {
    Serial.println(F("Sensor not found - check your wiring. Freezing"));
    while (1) ;
  }
  
  myImager.setResolution(8*8); //Enable all 64 pads

  // XXX try 4x4 to see if it makes a difference
  //myImager.setResolution(4*4); //Enable all 64 pads
  
  imageResolution = myImager.getResolution(); //Query sensor for current resolution - either 4x4 or 8x8
  imageWidth = sqrt(imageResolution); //Calculate printing width

  // XXX debug print statement - are we communicating with the module
  String theResolution = "Resolution = ";
  theResolution += String(imageResolution);
  Serial.println(theResolution);

  myImager.startRanging();

    // set up eyes and have the lids open
    pwm_ = Adafruit_PWMServoDriver();
    pwm_.begin(); 
    pwm_.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
    pwm_.setPWM(LIDLEFTBOTSERVO, 0, LIDLEFTBOTOPEN);
    pwm_.setPWM(LIDRIGHTBOTSERVO, 0, LIDRIGHTBOTOPEN);
    pwm_.setPWM(LIDLEFTTOPSERVO, 0, LIDLEFTTOPOPEN);
    pwm_.setPWM(LIDRIGHTTOPSERVO, 0, LIDRIGHTTOPOPEN);
    moveEyes(50,50);

  // XXX indicate that setup() is complete
  digitalWrite(LED_PIN, LOW);
}

void loop()
{
  //Poll sensor for new data
  if (myImager.isDataReady() == true)
  {
    if (myImager.getRangingData(&measurementData)) //Read distance data into array
    {
      //The ST library returns the data transposed from zone mapping shown in datasheet
      //Pretty-print data with increasing y, decreasing x to reflect reality
      for (int y = 0 ; y <= imageWidth * (imageWidth - 1) ; y += imageWidth)
      {
        for (int x = imageWidth - 1 ; x >= 0 ; x--)
        {
          Serial.print("\t");
          Serial.print(measurementData.distance_mm[x + y]);
        }
        Serial.println();
      }
      Serial.println();
    }
  }

    
  //decide where to point the eyes
  // x,y 0-100
  moveEyes(50,50);

  //delay(5); //Small delay between polling
  // XXX add in larger delay to allow data to be visualized
  delay(4000);  // large delay between polling
}


