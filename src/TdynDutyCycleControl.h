/*
 * TdynDutyCycleControl.h
 *
 *  Created on: Nov 18, 2023
 *      Author: Charlie
 */

#ifndef TDYNDUTYCYCLECONTROL_H_
#define TDYNDUTYCYCLECONTROL_H_

#include <Arduino.h>

#define PERIOD_TICKS 20

class TdynDutyCycleControl {
public:
	TdynDutyCycleControl();
	virtual ~TdynDutyCycleControl();
	void setpoint(float min, float max, float onset, float thresh);
	uint8_t poll(float temp);
	void setOutput(uint8_t p);
	void setDeviceName(char *s);

private:
	char name[16];
	uint8_t ssrPin;
	float df_minimum;
	float df_maximum;
	float temp_rampOnset;
	float temp_rampThresh;
	float dutyFactor;
	uint8_t lastOutput;
	uint16_t onTicks;
};

#endif /* TDYNDUTYCYCLECONTROL_H_ */
