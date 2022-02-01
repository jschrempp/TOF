/*
    TPPTOF.h

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

#ifndef _TPP_TOF_H
#define _TPP_TOF_H

//#define CONTINUOUS_DEBUG_DISPLAY

#include <SparkFun_VL53L5CX_Library.h> //http://librarymanager/All#SparkFun_VL53L5CX
#include <Wire.h>

extern SparkFun_VL53L5CX myImager;
extern VL53L5CX_ResultsData measurementData; // Result data class structure, 1356 byes of RAM

typedef struct {
    unsigned long detectedAtMS;
    long distanceMM;
    int x;
    int y;
} pointOfInterest ;

/*!
 *  @brief  Class that stores state and functions for interacting with the VL53L5CX
 * Time of Flight (TOF) sensor
 */
class TPP_TOF {
public:
    void initTOF();
    void getPOI(pointOfInterest *pPOI);

private:
    void prettyPrint(int32_t dataArray[]);
    void processMeasuredData(VL53L5CX_ResultsData measurementData, int32_t adjustedData[]);
    int  scoreZone(int location, int32_t dataArray[]);
    int  avgdistZone(int location, int32_t distance[]);
    bool validate(int score);
    void moveTerminalCursorUp(int numlines);
    void moveTerminalCursorDown(int numlines);

};


#endif