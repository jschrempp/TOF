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
  Date: 1/21/22

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

#include <SparkFun_VL53L5CX_Library.h> //http://librarymanager/All#SparkFun_VL53L5CX

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

// noise range in measured data.  Anything within +/- 50 of the calibrations is noise
const uint16_t NOISE_RANGE = 50;
const uint16_t MAX_CALIBRATION = 2000;  // anything greater is set to 2000 mm

SparkFun_VL53L5CX myImager;
VL53L5CX_ResultsData measurementData; // Result data class structure, 1356 byes of RAM

// declare 8x8 array of calibration values
int32_t calibration[64];

int imageResolution = 0; // read this back from the sensor
int imageWidth = 0; // read this back from the sensor


/* ------------------------------ */
/* ------------------------------ */
void setup(){
    
    // turn on D7 LED to indicate that we are in setup()
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);
    
    Serial.begin(115200);
    delay(1000);
    moveTerminalCursorDown(20);
    Serial.println("SparkFun VL53L5CX Imager Example");

    Wire.begin(); //This resets to 100kHz I2C
    Wire.setClock(400000); //Sensor has max I2C freq of 400kHz 

    
    Serial.println("Initializing sensor board. This can take up to 10s. Please wait.");
    if (myImager.begin() == false) {
        Serial.println(F("Sensor not found - check your wiring. Freezing"));
        while (1) ;
    }
    
    myImager.setResolution(64); //Enable all 64 pads - 8 x 8 array of readings
    
    imageResolution = myImager.getResolution(); //Query sensor for current resolution - either 4x4 or 8x8
    imageWidth = sqrt(imageResolution); //Calculate printing width

    // debug print statement - are we communicating with the module
    String theResolution = "Resolution = ";
    theResolution += String(imageResolution);
    Serial.println(theResolution);

    // XXX test out target order and sharpener changes
    // myImager.setSharpenerPercent(20);
    // myImager.setTargetOrder(SF_VL53L5CX_TARGET_ORDER::CLOSEST);
    // myImager.setTargetOrder(SF_VL53L5CX_TARGET_ORDER::STRONGEST);

    myImager.setRangingFrequency(8);

    myImager.startRanging();

    // fill in the calibration data array

    // wait for data to be ready
    do {
        // do nothing here, wait for data to be ready
        delay(5); //Small delay between polling
    } while(myImager.isDataReady() != true);

    // data is now ready
    if (myImager.getRangingData(&measurementData)) { //Read distance data into array
    
        // read out the measured data into an array
        for(int i = 0; i < 64; i++) {
        
            calibration[i] = measurementData.distance_mm[i];

            // adjust for calibration values being 0 or too long for measurement
            if( (calibration[i] == 0) || (calibration[i] > MAX_CALIBRATION) ) {
                calibration[i] = MAX_CALIBRATION;
            }

        }
        Serial.println("Calibration data:");
        prettyPrint(calibration);
        Serial.println("End of calibration data\n");
    }
    
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
    int32_t smallestValue, focusX, focusY;
    int32_t adjustedData[imageResolution];
    int32_t secondTable[imageResolution];   // second table to print out
    String secondTableTitle = ""; // will hold title of second table 

    // initialize second table
    for (int i = 0; i<imageResolution; i++) {
        secondTable[i] = 0;
    }
  
    //Poll sensor for new data.  Adjust if close to calibration value
    
    if (myImager.isDataReady() == true) {
    
        if (myImager.getRangingData(&measurementData)) { //Read distance data into ST driver array
       
            // initialize findings
            smallestValue = MAX_CALIBRATION; // start with the max allowed
            focusX = -255;  // code for no focus determined
            focusY = -255;  // code for no focus determined

            // process the measured data
            processMeasuredData(measurementData, adjustedData);
            
            prettyPrint(adjustedData);

            // XXXX New criteria (v 0.8+ for establishing the smallest valid distance)
            //  Walk through the adjustedData array except for the edges.  For each possible
            //    smallest value found, check that surrounding values asre valid.

            secondTableTitle = "average distance";
            // do not process the edges: x, y == 0 or x,y == 7  
            for (int y = 1; y < imageWidth-1; y++) {
                for (int x = 1; x < imageWidth-1; x++) {

                    int thisZone = y*imageWidth + x;

                    // Get the average distance of this zone
                    int avgDistThisZone = avgdistZone(thisZone, adjustedData);

                    secondTable[thisZone] = avgDistThisZone; 

                    int score = scoreZone(thisZone, adjustedData);


                    // test for the smallest value that is a significant zone
                    if( (adjustedData[thisZone] > 0) && (adjustedData[thisZone] < smallestValue) &&
                        (validate(score) == true) ) {

                        focusX = x;
                        focusY = y;
                        smallestValue = adjustedData[thisZone];

                    }
                }
            }

            // print out focus value found
            Serial.print("\nFocus on x = ");
            Serial.printf("%-5ld", focusX);
            Serial.print(" y = ");
            Serial.printf("%-5ld", focusY);
            Serial.print(" range = ");
            Serial.printf("%-5ld", smallestValue);
            Serial.println();
            Serial.println();
            Serial.println();

            Serial.println(secondTableTitle);
            prettyPrint(secondTable);
            Serial.println();

            // XXX overwrite the previous display
            moveTerminalCursorUp(22);

            //decide where to point the eyes
            // x,y 0-100
            if ((focusX > 0) && (focusY > 0)) {
                int xPos = map(focusX,1,6,0,100);   
                int yPos = map(focusY,1,6,100,0);
                moveEyeLids(100);  
                moveEyes(xPos  , yPos);
            } else {
                moveEyeLids(0);
                moveEyes(50,50);
            }
        }
    }
    delay(5); //Small delay between polling
    // delay(8000);  // longer delay to ponder results
}

/* ------------------------------ */
// function to pretty print data to serial port
void prettyPrint(int32_t dataArray[]) {
    //The ST library returns the data transposed from zone mapping shown in datasheet
    //Pretty-print data with increasing y, decreasing x to reflect reality 

    for(int y = 0; y <= imageWidth * (imageWidth - 1) ; y += imageWidth)  {
        for (int x = imageWidth - 1 ; x >= 0 ; x--) {
            Serial.print("\t");
            Serial.printf("%-5ld", dataArray[x + y]);
        }
        Serial.println();
    } 
}

/* ------------------------------ */
// function to move the terminal cursor back up to overwrite previous data printout
void moveTerminalCursorUp(int numlines) {
    String cursorUp = String("\033[") + String(numlines) + String("A");
    Serial.print(cursorUp);
    Serial.print("\r");
}

/* ------------------------------ */
// function to move the terminal cursor down to get past previous data printout - used on startup
void moveTerminalCursorDown(int numlines) {
    String cursorUp = String("\033[") + String(numlines) + String("B");
    Serial.print(cursorUp);
    Serial.print("\r");
}

/* ------------------------------ */
// function to validate that a value is surrounded by valid values
int scoreZone(int location, int32_t dataArray[]){
    int score = 0;
    int locX, locY, loc;
    locY = location/imageWidth;
    locX = location % imageWidth;

    if ((locX == 0) || (locX == imageWidth) || (locY == 0) || (locY == imageWidth)) {
        // we don't handle the edges of the matrix
        return 0;
    }

    for(int yIndex = -1; yIndex <= 1; yIndex++) {
        for(int xIndex = -1; xIndex <= 1; xIndex++) {

            // determine the location in the dataArray of value to test for validity
            loc = ((locY + yIndex) * imageWidth) + (locX + xIndex);

            if(dataArray[loc] > 0) { // valid value
                score++;
            }
        }
    }
    return score;
}

/* ------------------------------ */
// function to validate that a value is surrounded by valid values
int avgdistZone(int location, int32_t distance[]){
    int totalDist = 0;
    int numZones = 0;
    int avgDist = 0;
    int locX, locY, loc;
    locY = location/imageWidth;
    locX = location % imageWidth;

    if ((locX == 0) || (locX == imageWidth) || (locY == 0) || (locY == imageWidth)) {
        // we don't handle the edges of the matrix
        return distance[location];
    }

    avgDist = distance[location];
    if (distance[location] > 0) { 
        for(int yIndex = -1; yIndex <= 1; yIndex++) {
            for(int xIndex = -1; xIndex <= 1; xIndex++) {

                // determine the location in the dataArray of value to test for validity
                loc = ((locY + yIndex) * imageWidth) + (locX + xIndex);
                if (distance[loc] > 0 ) {
                    totalDist += distance[loc] ;
                    numZones++;
                }
            }
        }
        avgDist = totalDist / numZones;
    } 
    return avgDist;
}

/* ------------------------------ */
// function to decide if a zone is good enough for focus
bool validate(int score) {
    const int VALID_SCORE_MINIMUM = 6;
    
    if(score >= VALID_SCORE_MINIMUM) {
        return true;  
    } else {
        return false;
    }
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

/* ------------------------------ */
// process the measured data
void processMeasuredData(VL53L5CX_ResultsData measurementData, int32_t adjustedData[]) { 

    int statusCode = 0;
    int measuredData = 0;
    int32_t temp = 0;

    for(int i = 0; i < imageResolution; i++) {
      
        // process the status code, only good data if status code is 5 or 9
        statusCode = measurementData.target_status[i];
        measuredData = measurementData.distance_mm[i];

        if( (statusCode != 5) && (statusCode != 9) && (statusCode != 6)) { // TOF measurement is bad
            
            adjustedData[i] = -1;

        } else if ( (measuredData == 0) || (measuredData > MAX_CALIBRATION) ) { 
         //data out of range
                
            adjustedData[i] = -2;  // indicate out of range data

        } else  {
            // data is good and in range, check if background
          
            // check new data against calibration value
            temp = measuredData - calibration[i];
            
            // take the absolute value
            if(temp < 0) {
                temp = -temp;
            }

            if(temp <= NOISE_RANGE) { 
                    // zero out noise  
                
                    adjustedData[i] = -3; // data is background; ignore
            } 
            else { 
            
                    adjustedData[i] = (int16_t) measuredData;
            }

        }
        
    }
} 