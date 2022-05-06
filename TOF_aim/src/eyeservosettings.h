/*
 * eyeservosettings
 * Part of the animatronic exploration of Team Practical Projects
 * https://github.com/TeamPracticalProjects
 *
 * These are the settings that are used to position the eye control
 * servos. They depend on your particular eye mechanism, the servos you
 * use and how the horns are attached to the servo shafts.
 *
 * You should run the AnimatronicEyesCalibration.ino firmware to determine
 * the values to put in this file.
 *
 * You can select the set of values to use by uncommenting a #define
 *
 */

//#define SETTINGS_JIMS
#define SETTINGS_BOBG


// Servo Positions
// Use the calibration test code to figure the correct values for each.
// THE CALIBRATION TEST WILL PRINT A SET OF DEFINES THAT YOU CAN COPY
// AND PASTE OVER THE ONES BELOW.
// -------------------------------------------------------------------
// -------------------------------------------------------------------

#ifdef SETTINGS_JIMS

  #define X_POS_MID 400
  #define X_POS_LEFT_OFFSET 141
  #define X_POS_RIGHT_OFFSET -121

  #define Y_POS_MID 407
  #define Y_POS_UP_OFFSET 98
  #define Y_POS_DOWN_OFFSET -82

  #define LEFT_UPPER_CLOSED 486
  #define LEFT_UPPER_OPEN 287

  #define LEFT_LOWER_CLOSED 258
  #define LEFT_LOWER_OPEN 450

  #define RIGHT_UPPER_CLOSED 261
  #define RIGHT_UPPER_OPEN 469

  #define RIGHT_LOWER_CLOSED 509
  #define RIGHT_LOWER_OPEN 256

#endif

#ifdef SETTINGS_BOBG

#define X_POS_MID 518
#define X_POS_LEFT_OFFSET 98
#define X_POS_RIGHT_OFFSET -90

#define Y_POS_MID 332
#define Y_POS_UP_OFFSET 95
#define Y_POS_DOWN_OFFSET -42

#define LEFT_UPPER_CLOSED 462
#define LEFT_UPPER_OPEN 327

#define LEFT_LOWER_CLOSED 299
#define LEFT_LOWER_OPEN 451

#define RIGHT_UPPER_CLOSED 300
#define RIGHT_UPPER_OPEN 491

#define RIGHT_LOWER_CLOSED 514
#define RIGHT_LOWER_OPEN 288

#endif



// -------------------------------------------------------------------
// -------------------------------------------------------------------