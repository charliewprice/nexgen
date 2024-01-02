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
#define _SOAKING   5

#define OUTPUT_ON   LOW
#define OUTPUT_OFF  HIGH

#define TEMP_BUF_SIZE 16

class CurtainController {
  public:
    CurtainController();
    uint8_t poll(float tempF);
    void setOutput(uint8_t openpin, uint8_t closepin);
    void set(uint16_t periodSecs, uint16_t onSecs,
	           double setpointDegrees, double idlebandDegrees);
    void test();
    void dumpConfig();
    uint8_t getStatus();
    void setDeviceName(const char *s);

  private:
    char name[16];
    uint8_t openpin;
    uint8_t closepin;
    uint16_t periodSecs;
    uint16_t onSecs;
    uint16_t cycleSecsRemaining;
    uint8_t status, lastStatus;
    long openTimeSecs, closeTimeSecs;
	  float setpointDegrees;
	  float idlebandDegrees;

};
#endif

