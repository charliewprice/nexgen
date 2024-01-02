// CurtainController Library
// (c) charlie

#include "Arduino.h"
#include "CurtainController.h"

CurtainController* ctl;

CurtainController::CurtainController() {
  status = _IDLE;
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
	Serial.println(buf);
	Serial.print(setpointDegrees); Serial.print(" "); Serial.println(idlebandDegrees);
}

void CurtainController::setOutput(uint8_t op, uint8_t cp) {
	openpin = op;
	closepin = cp;
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

uint8_t CurtainController::poll(float temperature) {
  //Serial.print(cycleSecsRemaining); Serial.print(" temp "); Serial.print(temperature);
  if (cycleSecsRemaining == 0) {	
	//Serial.println(" CYC ");
	cycleSecsRemaining = periodSecs;
  } else if (cycleSecsRemaining==periodSecs) {	
	if (temperature<(setpointDegrees-(idlebandDegrees/2))) {
	   status = _CLOSING;
	} else if (temperature>(setpointDegrees+(idlebandDegrees/2))) {
	   status = _OPENING;
	} else {
	   status = _IDLE;
	}

	//Serial.print(" "); Serial.print(status);

	switch (status) {
		case _CLOSING: digitalWrite(closepin, OUTPUT_ON); break; //Serial.println(" CLOSING"); break;
		case _OPENING: digitalWrite(openpin, OUTPUT_ON);  break; //Serial.println(" OPENING"); break;
	    default: break; //Serial.println(" IDLE"); break; 
	}  
	cycleSecsRemaining -= 1;
  } else if (cycleSecsRemaining < (periodSecs - onSecs)) {
	  //Serial.println(" SOAKING ");
	  digitalWrite(closepin, OUTPUT_OFF);
	  digitalWrite(openpin, OUTPUT_OFF);
	  status = _SOAKING;
	  cycleSecsRemaining -= 1;
  } else {
	//Serial.println("");
	cycleSecsRemaining -= 1;
  }
  return status;
}


