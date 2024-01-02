/*
 * TstatOnOffControl.cpp
 *
 *  Created on: Oct 4, 2023
 *      Author: Charlie
 */
#include "TstatOnOffControl.h"
#include <Arduino.h>

// Constructor/Destructor

TstatOnOffControl::TstatOnOffControl() {
	mode = MODE_HEAT;
}

TstatOnOffControl::~TstatOnOffControl() {

}

void TstatOnOffControl::setOutput(uint8_t pin) {
	ssrPin = pin;
	pinMode(ssrPin, OUTPUT);
	digitalWrite(ssrPin, HIGH);
	lastOutput = HIGH;
}

void TstatOnOffControl::setMode(uint8_t mo) {
	mode = mo;
}

void TstatOnOffControl::set(float setPointF, float idlebandF) {
  setpoint = setPointF;
  idleband = idlebandF;
  /*
  char display[32];
  sprintf(display,"%s sp=%u ib=%u",name,(uint8_t) setpoint,(uint8_t) idleband);
  Serial.println(display);
  */
}

void TstatOnOffControl::dumpConfig() {
  Serial.print(name); Serial.print(" SP="); Serial.print(setpoint); Serial.print(" IB="); Serial.println(idleband);
}

void TstatOnOffControl::setDeviceName(const char *s) {
  uint8_t n=0;
  char c;
  do {
	  c = s[n];
	  name[n] = c;
	  n += 1;
  } while (c!=0);
}

float TstatOnOffControl::getLastTemp() {
  return lastTemp;
}

float TstatOnOffControl::getSetPoint() {
  return setpoint;
}

uint8_t TstatOnOffControl::getStatus() {
  uint8_t rc;
  if (lastOutput == HIGH)
	rc = SSR_OFF;
  else
	rc = SSR_ON;

  return rc;
}

void TstatOnOffControl::test() {
  if (lastOutput == HIGH)
	digitalWrite(ssrPin, LOW);
  else
	digitalWrite(ssrPin, HIGH);

  lastOutput = !lastOutput;
}

uint8_t TstatOnOffControl::poll(float tempF) {
  //Serial.print(name); Serial.print("-> "); Serial.print(tempF); Serial.print(" -> ");
  lastTemp = tempF;
  if (tempF < (setpoint-idleband/2)) {
	if (mode==MODE_HEAT) {
	  //Serial.println("ON ");
	  digitalWrite(ssrPin, LOW);
	  lastOutput = LOW;
	} else {
	  //Serial.println("OFF ");
	  digitalWrite(ssrPin, HIGH);
	  lastOutput = HIGH;
	}
  } else if (tempF > (setpoint+idleband/2)) {
	if (mode==MODE_HEAT) {
	  //Serial.println("OFF ");
	  digitalWrite(ssrPin, HIGH);
	  lastOutput = HIGH;
	} else {
	  //Serial.println("ON ");
	  digitalWrite(ssrPin, LOW);
	  lastOutput = LOW;
	}
  } else {	
	//Serial.println("SOAK");
	digitalWrite(ssrPin, lastOutput);
  }

  return !lastOutput;

}





