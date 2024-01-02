/*
 * DutyCycleControl.cpp
 *
 *  Created on: Nov 17, 2023
 *      Author: Charlie
 */
#include "DutyCycleControl.h"
#include <Arduino.h>

// Constructor/Destructor

DutyCycleControl::DutyCycleControl() {

}

DutyCycleControl::~DutyCycleControl() {

}

void DutyCycleControl::setOutput(uint8_t pin) {
	ssrPin = pin;
	pinMode(ssrPin, OUTPUT);
	digitalWrite(ssrPin, HIGH);
	onTicks = 0;
	lastOutput = HIGH;
}

void DutyCycleControl::setDutyFactor(float df) {
  dutyFactor = df;
}

void DutyCycleControl::test() {

}

uint8_t DutyCycleControl::clock(float tempF) {
	onTicks += 1;
	if (onTicks >= PERIOD_TICKS) {
	  onTicks = 0;
	  digitalWrite(ssrPin, LOW);
	  lastOutput = LOW;
	} else if (onTicks >= (dutyFactor * PERIOD_TICKS)) {
	  digitalWrite(ssrPin, HIGH);
	  lastOutput = HIGH;
	}
	return !lastOutput;
}










