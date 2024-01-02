/*
 * TstatOnOffControl.h
 *
 *  Created on: Oct 4, 2023
 *      Author: Charlie
 */
#include <Arduino.h>

#ifndef TstatOnOffControl_H_
#define TstatOnOffControl_H_

#define SSR_OFF 0
#define SSR_ON  1

#define MODE_HEAT 0
#define MODE_COOL 1

class TstatOnOffControl {
  public:
    TstatOnOffControl();
    ~TstatOnOffControl();
    void setMode(uint8_t mode);
    void setOutput(uint8_t pin);
    uint8_t poll(float tempF);
    void set(float setPointF, float hysteresisF);
    void test();
    uint8_t getStatus();
    float getLastTemp();
    float getSetPoint();
    void  setDeviceName(const char *s);
    void  dumpConfig();
  private:
    char name[16];
    uint8_t ssrPin;
    float setpoint, idleband, lastTemp;
    uint8_t lastOutput;
    uint8_t mode;

};

#endif /* TstatOnOffControl_H_ */
