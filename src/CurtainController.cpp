// CurtainController Library
// (c) charlie

#include "Arduino.h"
#include "CurtainController.h"

CurtainController* ctl;

CurtainController::CurtainController() {
  status = _IDLE;
  cycleSecsRemaining = periodSecs;
}

void CurtainController::set(uint16_t pSecs, uint16_t oSecs,
	                        double setpTDegrees, double idlebndDegrees) {
	periodSecs = pSecs;
	onSecs = oSecs;	
	setpointDegrees = setpTDegrees;
	idlebandDegrees = idlebndDegrees;
	status = _IDLE;
	cycleSecsRemaining = periodSecs;
}

void CurtainController::dumpConfig() {
	char buf[64];
	sprintf(buf,"%s psecs %u osecs %u",name,periodSecs,onSecs);
	SerialDebug.println(buf);
	SerialDebug.print(setpointDegrees); SerialDebug.print(" "); SerialDebug.println(idlebandDegrees);
}

void CurtainController::setOutput(uint16_t op, uint16_t cp) {
	openpin = op;
	closepin = cp;
	lastRc = 0x0000;
}

void CurtainController::setDeviceName(const char *s) {
  uint8_t n=0;
  char c;
  do {
	  c = s[n];
	  name[n] = c;
	  n += 1;
  } while (c!=0);
}

uint8_t CurtainController::getStatus() {
  return status;
}

void CurtainController::reset() {
	cycleSecsRemaining = 0;
}

uint16_t CurtainController::poll(float temperature) {
  uint16_t rc = 0x0000;  

  if (cycleSecsRemaining == 0) {	
	cycleSecsRemaining = periodSecs;
	if (temperature<(setpointDegrees-(idlebandDegrees/2))) {
	   status = _CLOSING;
	   rc = closepin;
	   lastRc = rc;
	   //SerialDebug.print("CLOSE ");
	} else if (temperature>(setpointDegrees+(idlebandDegrees/2))) {
	   status = _OPENING;
	   rc = openpin;
	   lastRc = rc;
	   //SerialDebug.print("OPEN ");
	} else {
	   status = _IDLE;
	   rc = 0x0000;
	   lastRc = rc;
	   //SerialDebug.print("IDLE ");
	}  
  } else if (cycleSecsRemaining < (periodSecs - onSecs)) {
	  rc = 0x0000;
	  lastRc = rc;
	  status = _SOAKING;
	  //SerialDebug.print("SOAKING ");
  } else {
	rc = lastRc;
  }
  //SerialDebug.println(cycleSecsRemaining);
  cycleSecsRemaining -= 1;
  return rc;
}


