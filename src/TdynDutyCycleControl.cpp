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

void TdynDutyCycleControl::setpoint(float min, float max, float onset, float thresh) {
  df_minimum = min;
  df_maximum = max;
  temp_rampOnset = onset;
  temp_rampThresh = thresh;
}

void TdynDutyCycleControl::setOutput(uint8_t pin) {
	ssrPin = pin;
	pinMode(ssrPin, OUTPUT);
	digitalWrite(ssrPin, HIGH);
	onTicks = 0;
	lastOutput = HIGH;
}

void TdynDutyCycleControl::setDeviceName(char *s) {
  uint8_t n=0;
  char c;
  do {
	  c = s[n];
	  name[n] = c;
	  n += 1;
  } while (c!=0);
}

uint8_t TdynDutyCycleControl::poll(float tempF) {
	onTicks += 1;
	if (onTicks >= PERIOD_TICKS) {
	  onTicks = 0;
	  digitalWrite(ssrPin, LOW);
	  lastOutput = LOW;
	} else if (onTicks >= (dutyFactor * PERIOD_TICKS)) {
	  digitalWrite(ssrPin, HIGH);
	  lastOutput = HIGH;
	}
	/*
	Serial.print(name); Serial.print("->");
	if (lastOutput==0)
	  Serial.println("ON");
	else
	  Serial.println("OFF");
    */
	return lastOutput;
}



