/****************************************************************************
* Copyright (C) 2011 - 2014 Bosch Sensortec GmbH
*
* Motion.ino
* Date: 2014/09/09
* Revision: 2.0 $
*
* Usage:        Example code of a game to demonstrate the Any motion 
*                  and No motion Interrupt features
*
****************************************************************************
/***************************************************************************
* License:
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*   Redistributions of source code must retain the above copyright
*   notice, this list of conditions and the following disclaimer.
*
*   Redistributions in binary form must reproduce the above copyright
*   notice, this list of conditions and the following disclaimer in the
*   documentation and/or other materials provided with the distribution.
*
*   Neither the name of the copyright holder nor the names of the
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
*
* The information provided is believed to be accurate and reliable.
* The copyright holder assumes no responsibility for the consequences of use
* of such information nor for any infringement of patents or
* other rights of third parties which may result from its use.
* No license is granted by implication or otherwise under any patent or
* patent rights of the copyright holder.
*/

#include "NAxisMotion.h"        //Contains the bridge code between the API and the Arduino Environment
#include <Wire.h>

NAxisMotion mySensor;   //Object that for the sensor
bool intDetected = false;       //Flag to indicate if an interrupt was detected
int threshold = 5;              //At a Range of 4g, the threshold is set at 39.05mg or 0.3830m/s2. This Range is the default for NDOF Mode
int duration = 1;               //At a filter Bandwidth of 62.5Hz, the duration is 8ms. This Bandwidth is the default for NDOF Mode
bool anyMotion = true;          //To know which interrupt was triggered

void setup() //This code is executed once
{
  //Peripheral Initialization
  Serial.begin(115200);           //Initialize the Serial Port to view information on the Serial Monitor
  I2C.begin();                    //Initialize I2C communication to the let the library communicate with the sensor.
  //Sensor Initialization
  Serial.println("Please wait. Initialization in process.");
  mySensor.initSensor();          //The I2C Address can be changed here inside this function in the library
  mySensor.setOperationMode(OPERATION_MODE_NDOF);   //Can be configured to other operation modes as desired
  mySensor.setUpdateMode(MANUAL);	//The default is AUTO. Changing to manual requires calling the relevant update functions prior to calling the read functions
  //Setting to MANUAL  requires lesser reads to the sensor

  attachInterrupt(INT_PIN, motionISR, RISING);  //Attach the interrupt to the Interrupt Service Routine for a Rising Edge. Change the interrupt pin depending on the board

  //Setup the initial interrupt to trigger at No Motion
  mySensor.resetInterrupt();
  mySensor.enableSlowNoMotion(threshold, duration, NO_MOTION);
  anyMotion = false;
  mySensor.accelInterrupts(ENABLE, ENABLE, ENABLE);  //Accelerometer interrupts can be triggered from all 3 axes
  Serial.println("This is a game to test how steady you can move an object with one hand. \nKeep the device on a table and mark 2 points.");
  Serial.println("Move the Device from one place to another without triggering the Any Motion Interrupt.\n\n");
  delay(1000); //Delay for the player(s) to read
  Serial.println("Move the device around and then place it at one position.\nChange the threshold and duration to increase the difficulty level.");
  Serial.println("Have fun!\n\n"); 
}

void loop() //This code is looped forever
{
  if (intDetected)
  {
    if (anyMotion)
    {
      Serial.println("You moved!! Try again. Keep the Device at one place.\n");
      intDetected = false;
      mySensor.resetInterrupt();          //Reset the interrupt line
      mySensor.disableAnyMotion();        //Disable the Any motion interrupt
      mySensor.enableSlowNoMotion(threshold, duration, NO_MOTION);  //Enable the No motion interrupt (can also use the Slow motion instead)
      anyMotion = false;
    }
    else
    {
      Serial.println("Device is not moving. You may start again.\n\n\n");
      intDetected = false;
      mySensor.resetInterrupt();          //Reset the interrupt line
      mySensor.disableSlowNoMotion();     //Disable the Slow or No motion interrupt
      mySensor.enableAnyMotion(threshold, duration);   //Enable the Any motion interrupt
      anyMotion = true;
    }
  }
}

//Interrupt Service Routine when the sensor triggers an Interrupt
void motionISR()
{
  intDetected = true;
}