/*
 * TdynDutyCycleControl.cpp
 *
 *  Created on: Nov 18, 2023
 *      Author: Charlie
 */

#include "TdynDutyCycleControl.h"

TdynDutyCycleControl::TdynDutyCycleControl() {
	onTicks = 0;
}

TdynDutyCycleControl::~TdynDutyCycleControl() {
	// TODO Auto-generated destructor stub
}

void TdynDutyCycleControl::setCycleTimeSecs(uint16_t secs) {
  cycletimesecs = secs;
}

void TdynDutyCycleControl::setFunction(float a[4][3]) {
	//make a copy
	for (uint8_t row=0; row<4; row++) {
	  for (uint8_t col=0; col<3; col++) {
		fn[row][col] = a[row][col];
	  }
	}
}

void TdynDutyCycleControl::setOutput(uint8_t pin) {
	ssrPin = pin;
	pinMode(ssrPin, OUTPUT);
	digitalWrite(ssrPin, HIGH);
	onTicks = 0;
	lastOutput = HIGH;
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

uint8_t TdynDutyCycleControl::poll(float tempF) {
	onTicks += 1;
	if (onTicks >= cycletimesecs) {
	  // new cycle is starting, calculate dutyFactor
	  dutyFactor = calculateDutyFactor(tempF);
	  onTicks = 0;
	  if (dutyFactor>0.0) {
	    digitalWrite(ssrPin, LOW);
	    lastOutput = LOW;
	  } else {
        digitalWrite(ssrPin, HIGH);
	    lastOutput = HIGH;
	  }
	} else if (onTicks >= ((dutyFactor * cycletimesecs)/100)) {
	  digitalWrite(ssrPin, HIGH);
	  lastOutput = HIGH;
	}	
	return !lastOutput;
}



