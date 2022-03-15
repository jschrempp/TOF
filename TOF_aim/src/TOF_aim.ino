/*
  TOF_aim:  determine position in an 8x8 matrix to aim animatronic eyes.

  The basic idea is to calibrate the room in setup() by storing an array of
  8x8 ranging values.  Then in loop():

  1.  read out an array of measured values.
  2.  compare each value to the calibration.  Ignore if close to the
    calibration value at that location.  Otherwise, save the position.
  3.  Compare each new position to save with any reviously saved position
    and save only the position of the nearest (to the sensor) value.
  4.   Serial display the results to see how well the algorithm performs.
    Possibility to play with sharpening, etc. to improve the performance.

  This firmware is based upon the example 1 code in the Sparkfun library.    
  
  Author: Bob Glicksman, Jim Schrempp
  Date: 2/26/22

  rev 2.0   expanded to use zones 0 and 7 
            and 0-100 instead of 20-80
  rev 1.9   moved wire initialization from ttp_tof to the .ino file setup()
  rev 1.8   moved TOF code into a class module
  rev 1.7   resolved bugs in avg and score array transversal
  rev 1.6   smooth eye movement
  rev 1.5   eyes now move to position with a number of steps
  rev 1.4   eye lids now open when there is a focus and close otherwise
  rev 1.3   merged in eyeServo
  rev 1.2   averagedistZone function added. Also moved processMeasuredData to its own function
  rev 1.1   used map function for x/y to eye position
  rev 1.0.  Second table print is our internal zone score
  rev 0.9.  Added Eye Servo control. Code still runs without eye servo board.
  rev 0.8.  Filter out spurious data readings by making sure that adjacent pixel values
      are valid data. Also fixed the reported smallest range coordinates to correspond
      to the prettyPrint() display coordinates.
  rev 0.7.  Add support for changing target order and sharpening. Also changed format of
      decision printout to make it cleaner.
  rev 0.6.  Adds second data table to continuous output
  rev 0.5.  Added code to overwrite data table
  rev 0.4.  Determine minimum valid range value and the coordinates of that value
    (note: unsure of coordinate system origin - 0,0?)
  rev 0.3.  Filter out bad measurements based upon target_status info
  rev 0.2.  Calibrate background and eliminate background measurements
  rev 0.1.  Restructure example code to put more stuff into functions.

*/

#include <Wire.h>

#include <TPP_TOF.h>
TPP_TOF theTOF;

#include <Adafruit_PWMServoDriver.h>
#include <eyeservosettings.h>

Adafruit_PWMServoDriver pwm_; 
// Servo Numbers for the Servo Driver board
#define X_SERVO 0
#define Y_SERVO 1
#define L_UPPERLID_SERVO 2
#define L_LOWERLID_SERVO 3
#define R_UPPERLID_SERVO 4
#define R_LOWERLID_SERVO 5

// use D7 LED for status
const int LED_PIN = D7;

/* ------------------------------ */
/* ------------------------------ */
void setup(){
    
    // turn on D7 LED to indicate that we are in setup()
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    Wire.begin(); //This resets to 100kHz I2C
    Wire.setClock(400000); //Sensor has max I2C freq of 400kHz 
    
    theTOF.initTOF();
    
    // set up eyes and have the lids open
    pwm_ = Adafruit_PWMServoDriver();
    pwm_.begin(); 
    pwm_.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
    pwm_.setPWM(L_LOWERLID_SERVO, 0, LEFT_LOWER_OPEN);
    pwm_.setPWM(R_LOWERLID_SERVO, 0, RIGHT_LOWER_OPEN);
    pwm_.setPWM(L_UPPERLID_SERVO, 0, LEFT_UPPER_OPEN);
    pwm_.setPWM(R_UPPERLID_SERVO, 0, RIGHT_UPPER_OPEN);
    moveEyeLids(100);
    moveEyes(0,0);
    delay(500);
    moveEyes(50,50);
    delay(500);
    moveEyes(100,100);
    delay (500);
    moveEyeLids(0);


    // indicate that setup() is complete
    digitalWrite(LED_PIN, LOW);
}

/* ------------------------------ */
/* ------------------------------ */
void loop() {
    int32_t smallestValue; 
    static int32_t focusX = -255;
    static int32_t  focusY = -255;
    static long lastEyeUpdateMS = 0;
   
    pointOfInterest thisPOI;

    theTOF.getPOI(&thisPOI);
    focusX = thisPOI.x;
    focusY = thisPOI.y;
    smallestValue = thisPOI.distanceMM;

    //decide where to point the eyes
    if (millis() - lastEyeUpdateMS > 10){
            lastEyeUpdateMS = millis();
            // x,y 0-100
            if ((focusX >= 0) && (focusY >= 0)) {
                static int xCurrentPos = 50;
                static int yCurrentPos = 50;
                int xPos = map(focusX,0,7, 0,100);   
                int yPos = map(focusY,0,7, 100,0);
                xCurrentPos = xCurrentPos + (0.1 * (xPos - xCurrentPos));
                yCurrentPos = yCurrentPos + (0.1 * (yPos - yCurrentPos));
                moveEyeLids(100);  
                moveEyes(xCurrentPos, yCurrentPos);
            } else {
                moveEyeLids(0);
                moveEyes(50,50);
            }
    }
    delay(5); //Small delay between polling
}


/* ------------------------------ */
void moveEyes (int x, int y){

    double xPos = map(x, 0, 100, X_POS_MID + X_POS_LEFT_OFFSET, X_POS_MID + X_POS_RIGHT_OFFSET);
    double yPos = map(y, 0, 100, Y_POS_MID + Y_POS_DOWN_OFFSET, Y_POS_MID + Y_POS_UP_OFFSET);

    pwm_.setPWM(X_SERVO, 0, xPos);
    pwm_.setPWM(Y_SERVO, 0, yPos);   

}

/* ------------------------------ */
void moveEyeLids(int openPct){

    float leftUpperPos = map(openPct, 0,100, LEFT_UPPER_CLOSED, LEFT_UPPER_OPEN  );
    float leftLowerPos = map(openPct, 0,100, LEFT_LOWER_CLOSED, LEFT_LOWER_OPEN  );
    float rightUpperPos = map(openPct, 0,100, RIGHT_UPPER_CLOSED, RIGHT_UPPER_OPEN  );
    float rightLowerPos = map(openPct, 0,100, RIGHT_LOWER_CLOSED, RIGHT_LOWER_OPEN );

    pwm_.setPWM(L_LOWERLID_SERVO, 0, leftLowerPos);
    pwm_.setPWM(L_UPPERLID_SERVO, 0, leftUpperPos);
    pwm_.setPWM(R_LOWERLID_SERVO, 0, rightLowerPos);
    pwm_.setPWM(R_UPPERLID_SERVO, 0, rightUpperPos);  
}
