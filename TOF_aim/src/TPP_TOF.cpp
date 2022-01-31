/*
    TPPTOF.cpp

    Team Practical Project Time of Flight sensor

    This library implements a TOF interface that selects points of interest within the
    TOF field of view. As objects in the field of view change, the points of interest
    will change. Our intent is to direct an animatronic eye mechanism with points it 
    should consider looking at. We leave the decision about how long to look at any 
    given point up to the caller.

    This firmware is based upon the example 1 code in the Sparkfun library.    
  
    Author: Bob Glicksman, Jim Schrempp
    (c) Copyright 2022 Bob Glicksman and Jim Schrempp

    This work is licensed under a Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
*/

#include <TPP_TOF.h>

SparkFun_VL53L5CX myImager;
VL53L5CX_ResultsData measurementData; // Result data class structure, 1356 byes of RAM

// noise range in measured data.  Anything within +/- 50 of the calibrations is noise
const uint16_t NOISE_RANGE = 50;
const uint16_t MAX_CALIBRATION = 2000;  // anything greater is set to 2000 mm

// declare 8x8 array of calibration values
int32_t calibration[64];

int imageResolution; // read this back from the sensor
int imageWidth; // read this back from the sensor

// -------- initTOF ----------
// called once to initialize the sensor
// may take up to 10 seconds to return
// returns True if sensor was initialized; False for error
void TPP_TOF::initTOF(){

    imageResolution = 0; // read this back from the sensor
    imageWidth = 0; // read this back from the sensor

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

    myImager.setRangingFrequency(4);

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

}


/* ------------------------------ */
// process the measured data
void TPP_TOF::processMeasuredData(VL53L5CX_ResultsData measurementData, int32_t adjustedData[]) { 

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


/* ------------------------------ */
// function to validate that a value is surrounded by valid values
int TPP_TOF::scoreZone(int location, int32_t dataArray[]){
    int score = 0;
    int locX, locY, loc;
    int locYInit = location/imageWidth;
    int locXInit = location % imageWidth;

    for(int yIndex = -1; yIndex <= 1; yIndex++) {
        for(int xIndex = -1; xIndex <= 1; xIndex++) {

            locX = locXInit+ xIndex;
            locY = locYInit + yIndex;

            if ((locX >= 0) && (locX < imageWidth) && (locY >= 0) && (locY < imageWidth)) {

                // determine the location in the dataArray of value to test for validity
                loc = (locY * imageWidth) + locX;

                if(dataArray[loc] > 0) { // valid value
                    score++;
                }
            }
        }
    }
    return score;
}

/* ------------------------------ */
// function to validate that a value is surrounded by valid values
int TPP_TOF::avgdistZone(int location, int32_t distance[]){
    int totalDist = 0;
    int numZones = 0;
    int avgDist = 0;
    int locX, locY, loc;
    int locYInit = location/imageWidth;
    int locXInit = location % imageWidth;


    avgDist = distance[location];
    if (distance[location] > 0) { 
        for(int yIndex = -1; yIndex <= 1; yIndex++) {
            for(int xIndex = -1; xIndex <= 1; xIndex++) {

                locX = locXInit + xIndex;
                locY = locYInit + yIndex;

                if ((locX >= 0) && (locX < imageWidth) && (locY >= 0) && (locY < imageWidth)) {

                    // determine the location in the dataArray of value to test for validity
                    loc = (locY * imageWidth) + locX;
                    if (distance[loc] > 0 ) {
                        totalDist += distance[loc] ;
                        numZones++;
                    }
                }
            }
        }
        avgDist = totalDist / numZones;
    } 
    return avgDist;
}

/* ------------------------------ */
// function to decide if a zone is good enough for focus
bool TPP_TOF::validate(int score) {
    const int VALID_SCORE_MINIMUM = 3;
    
    if(score >= VALID_SCORE_MINIMUM) {
        return true;  
    } else {
        return false;
    }
}


/* ------------------------------ */
// function to move the terminal cursor back up to overwrite previous data printout
void TPP_TOF::moveTerminalCursorUp(int numlines) {
    String cursorUp = String("\033[") + String(numlines) + String("A");
    Serial.print(cursorUp);
    Serial.print("\r");
}

/* ------------------------------ */
// function to move the terminal cursor down to get past previous data printout - used on startup
void TPP_TOF::moveTerminalCursorDown(int numlines) {
    String cursorUp = String("\033[") + String(numlines) + String("B");
    Serial.print(cursorUp);
    Serial.print("\r");
}



// -------- getPOI ------------
// called anytime to have sensor read and interpret its zone data
// returns the current Point Of Interest
void TPP_TOF::getPOI(pointOfInterest *pPOI){

    int32_t smallestValue = MAX_CALIBRATION; 
    static int32_t focusX = -255;
    static int32_t  focusY = -255;
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
            
#ifdef CONTINUOUS_DEBUG_DISPLAY
            prettyPrint(adjustedData);
#endif
            // XXXX New criteria (v 0.8+ for establishing the smallest valid distance)
            //  Walk through the adjustedData array except for the edges.  For each possible
            //    smallest value found, check that surrounding values asre valid.

            secondTableTitle = "avgDistThisZone";
            // do not process the edges: x, y == 0 or x,y == 7  
            for (int y = 0; y < imageWidth; y++) {
                for (int x = 0; x < imageWidth; x++) {

                    int thisZone = y*imageWidth + x;

                    // Get the average distance of this zone
                    int avgDistThisZone = avgdistZone(thisZone, adjustedData);


                    int score = scoreZone(thisZone, adjustedData);


                    secondTable[thisZone] = avgDistThisZone; 


                    // test for the smallest value that is a significant zone
                    if( (avgDistThisZone > 0) && (avgDistThisZone < smallestValue) &&
                        (validate(score) == true) ) {

                        focusX = x;
                        focusY = y;
                        smallestValue = avgDistThisZone;

                    }
                }
            }

#ifdef CONTINUOUS_DEBUG_DISPLAY
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
#endif
            
        }
    }

    pPOI->detectedAtMS = millis();
    pPOI->distanceMM = smallestValue;
    pPOI->x = focusX;
    pPOI->y = focusY;

}




/* ------------------------------ */
// function to pretty print data to serial port
void TPP_TOF::prettyPrint(int32_t dataArray[]) {
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