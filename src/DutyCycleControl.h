/*
 * DutyCycleControl.h
 *
 *  Created on: Nov 17, 2023
 *      Author: Charlie
 */

#ifndef DUTYCYCLECONTROL_H_

#include <Arduino.h>

#define OFF 0
#define ON  1

#define PERIOD_TICKS 100

class DutyCycleControl {
  public:
	DutyCycleControl();
    ~DutyCycleControl();
    void setOutput(uint8_t pin);
    uint8_t clock(float temp);
    void setDutyFactor(float df);
    void test();
  private:
    uint8_t ssrPin;
    float dutyFactor;
    uint16_t onTicks;
    uint8_t lastOutput;

};

#define DUTYCYCLECONTROL_H_





#endif /* DUTYCYCLECONTROLLER_H_ */
