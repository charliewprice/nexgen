/*
 * TdynDutyCycleControl.cpp
 *
 *  Created on: Nov 18, 2023
 *      Author: Charlie
 */

#include "TdynDutyCycleControl.h"

TdynDutyCycleControl::TdynDutyCycleControl() {
	
}

TdynDutyCycleControl::~TdynDutyCycleControl() {
	// TODO Auto-generated destructor stub
}

void TdynDutyCycleControl::setCycleTimeSecs(uint16_t secs) {
  cycletimesecs = secs;
  onTicks = cycletimesecs;
}

void TdynDutyCycleControl::setFunction(float a[4][3]) {
	//make a copy
	for (uint8_t row=0; row<4; row++) {
	  for (uint8_t col=0; col<3; col++) {
		fn[row][col] = a[row][col];
	  }
	}
}

void TdynDutyCycleControl::setOutput(uint16_t pin) {
	onpin = pin;
	onTicks = 0;
	lastOutput = 0x0000;
}

void TdynDutyCycleControl::setDeviceName(const char *s) {
  uint8_t n=0;
  char c;
  do {
	  c = s[n];
	  name[n] = c;
	  n += 1;
  } while (c!=0);
}

float TdynDutyCycleControl::getDutyFactor() {
	return dutyFactor;
}

void TdynDutyCycleControl::reset() {
  onTicks = 0;
}

float TdynDutyCycleControl::calculateDutyFactor(float x) {
  float dutyFactor = 0.0;
  for(uint8_t n=0; n<4; n++) {
    if (x > fn[n][0]) {
      dutyFactor += fn[n][1]; // the step
      dutyFactor += (x-fn[n][0]) * fn[n][2]; 
    }
  }
  return dutyFactor;
}

uint16_t TdynDutyCycleControl::poll(float tempF) {
	uint16_t rc = lastOutput;	
	if (onTicks == 0) {
	  // new cycle is starting, calculate dutyFactor
	  //SerialDebug.print(" zero ");
	  dutyFactor = calculateDutyFactor(tempF);
	  //SerialDebug.println(dutyFactor); //SerialDebug.print(" ");
	  onTicks = cycletimesecs;
	  if (dutyFactor>0.0) {
	    rc = onpin;
	    lastOutput = rc;
	  } else {
        rc = 0x0000;
	    lastOutput = rc;
	  }
	} else if (onTicks <= (uint16_t) (cycletimesecs - ((dutyFactor * cycletimesecs)/100))) {
	  rc = 0x0000;
	  lastOutput = rc;
	} 

    #if defined(DEBUG_MESSAGES)
	char s[64];
	sprintf(s,"%s temp=%d df=%d cyc=%i on=%i, rc=0x%x", name, (int)tempF, (int)dutyFactor, cycletimesecs, onTicks, rc);
	SerialDebug.println(s);
	#endif
    
	onTicks -= 1;
	return rc;
}



