/*
MAKE SURE TO ADD THIS IN YOUR ADDTIONAL BOARD MANAGER URL IF NEVER USED AN ESP 32 
FILE--->PREFENCES--->SETTINGS
https://dl.espressif.com/dl/package_esp32_index.json
https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json

_____________________________________________________________________________________________
Imported library need for this project, make sure to download all the libraries before using (On the left side tabs)

*BOARD MANAGER INSTALL*
-esp32 by Espressif Systems

------------------------------------------
*LIBRARY MANAGER INTSTALL*
-AccelStepper by Mike

--------------------------------------------
#include Month.h imports a class made in the second file called "Month.h" 
*/
//_________________________________________________________________________________________________________________________________________________________________________________________________

#include "Month.h"
#include "Arduino.h"
#include <WiFi.h>
#include <Stepper.h>
#undef OCT
#undef DEC

#define uS_TO_S_FACTOR 1000000ULL  /* Conversion factor for micro seconds to seconds */
#define inverteral_TIME 3600  // 1 hour

//------------------------------------------------------------------------
//You can change the numbers depending on your motor
// Motor Step Constants
const float A_motor_step = 1.8;       //This motor moves 1.8 degrees every step
const byte ledPin = 2; // Replace with your chosen GPIO pin
const int stepsPerRevolution = 200;   //Won't be used 

//------------------------------------------------------------------------
RTC_DATA_ATTR int day = 0;  // Day counter, starts from 0
RTC_DATA_ATTR int timer = 0; // Timer counter, starts from 0

// Define pins for stepper motors
const int motorX_pin1 = 5;  // Set the GPIO pins for Stepper Motor X
const int motorX_pin2 = 18; 
const int motorY_pin1 = 19; // Set the GPIO pins for Stepper Motor Y
const int motorY_pin2 = 21;

// Create Stepper motor objects
Stepper myStepperX(stepsPerRevolution, motorX_pin1, motorX_pin2);
Stepper myStepperY(stepsPerRevolution, motorY_pin1, motorY_pin2);

//_________________________________________________________________________________________________________________________________________________________________________________________________
//HERE'S WHERE YOU CAN PUT YOUR DATA FOR EACH MONTH INTO 
//ROUNDING TO EVEN NUMBER MADE EASIER FOR THE MOTOR-Y TO MOVE BACK TO ORGIN (0)
//------------------------------------------------------------
// Define Months with sunrise, sunset, and hours of daylight
Month JAN(12, 31, 12, 123.6 , 240.3, 29.7);
Month FEB(14, 59, 10, 116.8 , 245.1, 45.2);
Month MAR(16, 90, 8, 100.3 , 251.9, 58.1);      // 15 round up to 16
Month APR(18, 120, 7, 90.5 , 260.4, 68.3);      // 17 round uo to 18
Month MAY(18, 151, 6, 80.2 , 268.7, 73.8);
Month JUNE(18, 181, 6, 70.1 , 275.9, 76.5);
Month JULY(18, 212, 6, 72.3 , 270.5, 75.2);
Month AUG(16, 243, 8, 85.6, 261.8, 69.4);
Month SEP(16, 273, 8, 98.7, 254.2, 60.7);       // 15 round up to 16
Month OCT(14, 304, 10, 112.4, 247.6, 50.9);
Month NOV(12, 334, 12, 111.56, 248.85, 36.10);
Month DEC(8, 365, 16, 125.32, 232.59, 28.73);
//----------------------------------------------------------------------------------------------------------------------

//THIS DOES THE MATH AND ROUND THE NUMBER TO WHOLE A NUMBER FOR THE STEPS
//THE ERROR IS +- 1.8 DEGREES FOR SUN-SET OR SUNRSIE  
// Stepper Motor Arrays
//This was used to round to get rid of decimal cause motor can't move fractions of step
const byte Peak[12] = {(JAN.hours / 2), (FEB.hours / 2), (MAR.hours / 2), (APR.hours / 2), (MAY.hours / 2), 
                        (JUNE.hours / 2), (JULY.hours / 2), (AUG.hours / 2), (SEP.hours / 2), (OCT.hours / 2),
                        (NOV.hours / 2), (DEC.hours / 2)};

const byte Set_Array[12] = {((JAN.Set - JAN.Rise) / A_motor_step), ((FEB.Set - FEB.Rise) / A_motor_step), 
                            ((MAR.Set - MAR.Rise) / A_motor_step), ((APR.Set - APR.Rise) / A_motor_step), 
                            ((MAY.Set - MAY.Rise) / A_motor_step), ((MAY.Set - MAY.Rise) / A_motor_step),
                            ((JUNE.Set - JUNE.Rise) / A_motor_step), ((JULY.Set - JULY.Rise) / A_motor_step), 
                            ((AUG.Set - AUG.Rise) / A_motor_step), ((SEP.Set - SEP.Rise) / A_motor_step), 
                            ((OCT.Set - OCT.Rise) / A_motor_step), ((NOV.Set - NOV.Rise) / A_motor_step)};

//_________________________________________________________________________________________________________________________________________________________________________________________________

//FUNCTIONS THAT HANDLES THE MOVEMENT 
// Function to move motors to position
void moveMotorToPosition(float position, int motor) {
  Serial.print("Moving motor ");
  Serial.print(motor);
  Serial.print(" to position: ");
  Serial.println(position);
  
  if (motor == 1) {
    myStepperX.step(position);  // Move motor X
  } else if (motor == 2) {
    myStepperY.step(position);  // Move motor Y
  }
}

// Function to handle motor movements based on the current month and timer
void handleMonthMovement(Month month, int monthIndex) {
  if (timer == 0) {
    Serial.println("Moving Motors to origin");
    moveMotorToPosition(month.Rise, 1);  // Example for motor 1
    moveMotorToPosition(0, 2);  // Example for motor 2
    timer++;
  } else if (timer > 0 && timer < month.hours) {
    timer++;
    Serial.println("Moving Motor X");
    moveMotorToPosition(Set_Array[monthIndex] / month.hours, 1);  // Adjust for each motor
    if (timer < Peak[monthIndex] || timer == Peak[monthIndex]) {
      Serial.println("Moving Motor Y");
      moveMotorToPosition((month.Elevation- 45) / month.hours, 2);  // Adjust for each motor  MINUS BY 45 ANGLE OF SOLAR PANEL
    } else {
      Serial.println("Moving Motor Y back");
      moveMotorToPosition(-((month.Elevation -45) / month.hours), 2);  // Adjust for each motor
    }
  } else if (timer == month.hours) {
    timer = 0;
    day++;
    Serial.println("Moving Motor X to origin");
    moveMotorToPosition(0, 1);  // Return motor to origin
    esp_sleep_enable_timer_wakeup(month.sleep * uS_TO_S_FACTOR);
    esp_deep_sleep_start();
  }
}
//_________________________________________________________________________________________________________________________________________________________________________________________________

void setup() {
  Serial.begin(115200);
  btStop();//TURN OFF WIFI AND BLUETOOTH
  delay(2000);  // Give serial monitor time to connect
  pinMode(ledPin, OUTPUT);  // Set LED pin

  Serial.println("Sunflower Solar Panel System Started");

  // Check for the current month and handle movement
  if (day <= JAN.days) {
    handleMonthMovement(JAN, 0);
  } else if (JAN.days < day && day <= FEB.days) {
    handleMonthMovement(FEB, 1);
  } else if (FEB.days < day && day <= MAR.days) {
    handleMonthMovement(MAR, 2);
  } else if (MAR.days < day && day <= APR.days) {
    handleMonthMovement(APR, 3);
  } else if (APR.days < day && day <= MAY.days) {
    handleMonthMovement(MAY, 4);
  } else if (MAY.days < day && day <= JUNE.days) {
    handleMonthMovement(JUNE, 5);
  } else if (JUNE.days < day && day <= JULY.days) {
    handleMonthMovement(JULY, 6);
  } else if (JULY.days < day && day <= AUG.days) {
    handleMonthMovement(AUG, 7);
  } else if (AUG.days < day && day <= SEP.days) {
    handleMonthMovement(SEP, 8);
  } else if (SEP.days < day && day <= OCT.days) {
    handleMonthMovement(OCT, 9);
  } else if (OCT.days < day && day <= NOV.days) {
    handleMonthMovement(NOV, 10);
  } else if (NOV.days < day && day <= DEC.days) {
    handleMonthMovement(DEC, 11);
  }

  // Debug Information
  Serial.println("Time");
  Serial.println(timer);
  Serial.println("Day");
  Serial.println(day);
  
  // Deep sleep for power saving
  Serial.println("Entering deep sleep mode...");
  esp_sleep_enable_timer_wakeup(inverteral_TIME * uS_TO_S_FACTOR);
  esp_deep_sleep_start();
}

void loop() {
  Serial.println("This should never print after deep sleep");
  delay(1000);
}
