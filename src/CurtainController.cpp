// CurtainController Library
// (c) charlie

#include "Arduino.h"
#include "CurtainController.h"

CurtainController* ctl;

CurtainController::CurtainController() {
  status = _IDLE;
}

void CurtainController::set(uint16_t opPeriodSecs, uint16_t opOnSecs,
                            uint16_t clPeriodSecs,uint16_t clOnSecs,
	                        double setpTDegrees, double idlebndDegrees) {
	openPeriodSecs = opPeriodSecs;
	openOnSecs = opOnSecs;
	closePeriodSecs = clPeriodSecs;
	closeOnSecs = clOnSecs;
	setpointDegrees = setpTDegrees;
	idlebandDegrees = idlebndDegrees;


	//char display[32];
	//sprintf(display,"%s sp=%u ib=%u",name,(uint8_t) setpointDegrees,(uint8_t) idlebndDegrees);
	//Serial.println(display);
}

void CurtainController::setOutput(uint8_t op, uint8_t cp) {
	openpin = op;
	closepin = cp;
}

void CurtainController::setDeviceName(char *s) {
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

void CurtainController::clock() {
  cycleSecsRemaining -= 1;
  if(cycleSecsRemaining==0) {
    digitalWrite(closepin, OUTPUT_OFF);
    digitalWrite(openpin, OUTPUT_OFF);
    status = _IDLE;
  } else {
	  if (status == _CLOSING) {
		if(cycleSecsRemaining<(closePeriodSecs-closeOnSecs)) {
		  digitalWrite(closepin, OUTPUT_OFF);
		  status = _CLOSESOAK;
		} else
		  digitalWrite(closepin, OUTPUT_ON);
	  } else if (status == _OPENING) {
		if(cycleSecsRemaining<(openPeriodSecs-openOnSecs)) {
		  digitalWrite(openpin, OUTPUT_OFF);
		  status = _OPENSOAK;
		} else
		  digitalWrite(openpin, OUTPUT_ON);
	  }
  }

}

uint8_t CurtainController::poll(float temperature) {
  if (temperature<(setpointDegrees-(idlebandDegrees/2))) {
	  //initiate new CLOSE/HEAT cycle
	  status = _CLOSING;
	  cycleSecsRemaining = closePeriodSecs;
  } else  if (temperature>(setpointDegrees+(idlebandDegrees/2))) {
	  //initiate new OPEN/COOL cycle
	  status = _OPENING;
	  cycleSecsRemaining = openPeriodSecs;
  }
  /*
  Serial.print(name); Serial.print("->");
  switch(status) {
  case _IDLE:       Serial.println("IDLE"); break;
  case _CLOSING:      Serial.println("CLOSING"); break;
  case _CLOSESOAK: Serial.println("CLOSE SOAK"); break;
  case _OPENING:       Serial.println("OPENING"); break;
  case _OPENSOAK:  Serial.println("OPEN SOAK"); break;
  }
  */

  return status;
}


