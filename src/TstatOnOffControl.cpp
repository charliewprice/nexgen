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

void TstatOnOffControl::setOutput(uint16_t pin) {
	onpin = pin;
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
  SerialDebug.println(display);
  */
}

void TstatOnOffControl::dumpConfig() {
  SerialDebug.print(name); SerialDebug.print(" SP="); SerialDebug.print(setpoint); SerialDebug.print(" IB="); SerialDebug.println(idleband);
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
/*
void TstatOnOffControl::test() {
  if (lastOutput == 0x0000HIGH)
	digitalWrite(ssrPin, LOW);
  else
	digitalWrite(ssrPin, HIGH);

  lastOutput = !lastOutput;
}
*/

uint16_t TstatOnOffControl::poll(float tempF) {
  uint16_t rc = 0x0000;
  //SerialDebug.print(name); SerialDebug.print("-> "); SerialDebug.print(tempF); SerialDebug.print(" -> ");
  lastTemp = tempF;
  if (tempF < (setpoint-idleband/2)) {
	if (mode==MODE_HEAT) {
	  //SerialDebug.println("ON ");
	  rc = onpin;
	  lastOutput = rc;
	} else {
	  //SerialDebug.println("OFF ");
	  rc = 0x0000;
	  lastOutput = rc;
	}
  } else if (tempF > (setpoint+idleband/2)) {
	if (mode==MODE_HEAT) {
	  //SerialDebug.println("OFF ");
	  rc = 0x0000;
	  lastOutput = rc;
	} else {
	  //SerialDebug.println("ON ");
	  rc = onpin;
	  lastOutput = rc;
	}
  } else {	
	//SerialDebug.println("SOAK");
	rc = lastOutput;
  }

  return rc;

}





