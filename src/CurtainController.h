// CurtainController
// (c) charlie
// Code under MIT License

#ifndef CurtainController__h
#define CurtainController__h

#define _IDLE      0
#define _CLOSING   1
#define _CLOSESOAK 2
#define _OPENING   3
#define _OPENSOAK  4

#define OUTPUT_ON   LOW
#define OUTPUT_OFF  HIGH

#define TEMP_BUF_SIZE 16

class CurtainController {
  public:
    CurtainController();
    uint8_t poll(float tempF);
    void clock();
    void setOutput(uint8_t openpin, uint8_t closepin);
    void set(uint16_t openPeriodSecs, uint16_t openOnSecs,
             uint16_t closePeriodSecs,uint16_t closeOnSecs,
	         double setpointDegrees, double idlebandDegrees);
    void test();
    uint8_t getStatus();
    void setDeviceName(char *s);

  private:
    char name[16];
    uint8_t openpin;
    uint8_t closepin;
    uint16_t openPeriodSecs;
    uint16_t openOnSecs;
    uint16_t closePeriodSecs;
    uint16_t closeOnSecs;
    uint16_t cycleSecsRemaining;
    uint8_t status, lastStatus;
    long openTimeSecs, closeTimeSecs;
	float setpointDegrees;
	float idlebandDegrees;

};
#endif

