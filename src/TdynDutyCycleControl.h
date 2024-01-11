/*
 * TdynDutyCycleControl.h
 *
 *  Created on: Nov 18, 2023
 *      Author: Charlie
 */

#ifndef TDYNDUTYCYCLECONTROL_H_
#define TDYNDUTYCYCLECONTROL_H_

#include <Arduino.h>

#include "common.h"

#define PERIOD_TICKS 20

class TdynDutyCycleControl {
public:
	TdynDutyCycleControl();
	virtual ~TdynDutyCycleControl();
	void setCycleTimeSecs(uint16_t cycletimesecs);
	void setFunction(float a[4][3]);
	uint16_t poll(float temp);
	void setOutput(uint16_t p);
	void setDeviceName(const char *s);
	float getDutyFactor();
	void reset();

private:
    float calculateDutyFactor(float x);
	float fn[4][3];
	char name[16];
	uint16_t onpin;
	//float df_minimum;
	//float df_maximum;
	//float temp_rampOnset;
	//float temp_rampThresh;
	float dutyFactor;
	uint16_t cycletimesecs;
	uint16_t lastOutput;
	uint16_t onTicks;
};

#endif /* TDYNDUTYCYCLECONTROL_H_ */
