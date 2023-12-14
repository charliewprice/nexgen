#include <Arduino.h>

///#define _TEST_WATCHDOG		//used to determine the actual watchdog timeout
//#define WAIT_FOR_SERIALDEBUG
//#define _DEBUG_SERIAL

#define CONFIGURATION_ID 9000
/********************************************************************************************************************************************
 *
 * AgriMote (c)  2020, 2021, 2022, 2023 Charlie Price
 *
 * This uses OTAA (Over-the-air activation), where where a DevEUI and
 * application key is configured, which are used in an over-the-air
 * activation procedure where a DevAddr and session keys are
 * assigned/generated for use with all further communication.
 *
 * 11/01/23 automatic bare-metal bild based on CONFIGURATION_ID
 *******************************************************************************************************************************************/

#include <Wire.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <RTCZero.h>
#include <Adafruit_NeoPixel.h>

#define HEATPAD_OUTPUT A2
#define SPACEHEATER_OUTPUT 5
#define STIRFAN_OUTPUT 9
#define CURTAIN_OPENOUTPUT A4
#define CURTAIN_CLOSEOUTPUT A3
#define VENTILATION_OUTPUT 10

//#define BTSERIALTERMINAL

#if defined(BTSERIALTERMINAL)
#include "ErriezSerialTerminal.h"
char newlineChar = '\n';
char delimiterChar = ' ';
SerialTerminal term(newlineChar, delimiterChar);
#endif

#define PIXELPIN       11
#define NUMPIXELS      2

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIXELPIN, NEO_GRB + NEO_KHZ800);

#include "TstatOnOffControl.h"
TstatOnOffControl heatPadController;
uint8_t heatPad_OP;

TstatOnOffControl spaceHeaterController;
uint8_t spaceHeater_OP;

TstatOnOffControl stirFanController;
uint8_t stirFan_OP;

#include "CurtainController.h";
CurtainController curtainController;
uint8_t curtain_OP;

#include "TdynDutyCycleControl.h";
TdynDutyCycleControl ventilationController;
uint8_t ventilation_OP;

#include "Eeprom.h"
Eeprom eeprom;
#define _EEPROMUPDATE_MILLIS 	30000
boolean updateEeprom;

#define SerialTerminal Serial1
#define SerialDebug Serial

#define _WATCHDOG_DONE 12
#define _TESTBED_PIN    5

#include "project_config/lmic_project_config.h"
#define _ACK_PACKETS		// #16 low SNR/RSSI
#define _DONT_SLEEP
#define _DUMP_KEYS
#define _BLINK_MILLIS 2000
#define _SEND_MILLIS 60000
#define _EEPROM_IC2_ADDR 	0x50		//the I2C bus address for the EEPROM
#define _DEVEUI_ADDR 		0xF8		//the location in the EEPROM for the DEVEUI
//special pins for all motes
#define _BATTERY_PIN A7					// divider circuit internally wired here
#define _WATCHDOG_DONE 12				// pulsed once on each LoRa TX_COMPLETE event
#define _LOOP_PULSE_PIN 5				// pulsed once on each iteration of loop()
#define _BUZZER_PIN A3					// used for _LIQUID_LEVEL and _GEOLOCATION sensortypes only
#define _SEND_INTERVAL_SECONDS 15
#define TX_INTERVAL_SECONDS 15        	// Schedule the interval(seconds) at which data is sent
static osjob_t sendjob;
boolean loraJoined;

//#14 reset the processor when frame count reaches a threshold
unsigned int frameCount;
//#define RESET_ON_FRAMECOUNT
#define _FRAMECOUNT_MAX 5000				// will reset when this count is reached
boolean ackRequested;
boolean ackReceived;

/*
 * LoRa Radio Pin Mappings
 */
const lmic_pinmap lmic_pins = {
  .nss = 8,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 4,
  .dio = {3,6},
}; //.dio = {3,6,11},

RTCZero rtc;

/* EEPROM I2C Device address calculated as follows:
 * 101 0[A2][A1][A0]
 *
 * DEVEUI the EUI-64 address is stored in the EEPROM at 0xF8
 * APPKEY is constructed from the APP_KEY_ROOT with the 3LSB replaced by the 3LSB of the DEVEUI
 *                                            s     e     n     s     e     i     0     0
 */
static const u1_t  	APPKEY_ROOT[16] 	= {0x73, 0x65, 0x6E, 0x73, 0x65, 0x69, 0x30, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static const u1_t 	PROGMEM APPEUI[8]  	= {0x22, 0xf6, 0xec, 0x6b, 0x9a, 0x9a, 0x62, 0xdf};

u1_t devEui[8];
u1_t appKey[16];

#include <DS18B20.h>
DS18B20 ds_pad(A0);
DS18B20 ds_room(A1);

/*
 * Mote Record Structure
 */
#define _DEVICE_PROFILE 41

struct moterecord {
	uint8_t curtain_setpoint;
	uint8_t curtain_idleband;
	uint16_t curtain_upcyclesecs;
	uint16_t curtain_uponsecs;
	uint16_t curtain_dncyclesecs;
	uint16_t curtain_dnonsecs;
	uint8_t heatpad_setpoint;
	uint8_t heatpad_idleband;
	uint8_t spaceheater_setpoint;
	uint8_t spaceheater_idleband;
	uint8_t stirfan_setpoint;
	uint8_t stirfan_idleband;
	uint16_t ventilation_cyclesecs;
	uint8_t  ventilation_minpcton;
	uint8_t  ventilation_tramponset;
	uint8_t  ventilation_trampfinal;

	uint8_t  heatpad_temp;
	uint8_t  room_temp;
	uint16_t circuitflags;

	uint8_t  lastackreceived;
};
typedef struct moterecord Record;
Record moteRec;
char b[sizeof(moteRec)];

#define _PROCESS_TIME_MILLIS 1000
long upTimeSecs;
long lastProcessMillis;
long heatingTimeSecs;

struct ctl_parms {
	int16_t configuration_id;
	int16_t channel;
	int16_t zone;
	char bt_name[16];
	char bt_pin[6];

	float curtain_setpoint;
	float curtain_idleband;
	uint16_t curtain_upcyclesecs;
	uint16_t curtain_uponsecs;
	uint16_t curtain_dncyclesecs;
	uint16_t curtain_dnonsecs;

	float heatpad_setpoint;
	float heatpad_idleband;

	float stirfan_setpoint;
	float stirfan_idleband;

	float spaceheater_setpoint;
	float spaceheater_idleband;

	uint16_t  ventilation_cyclesecs;
	float  ventilation_minpcton;
	float  ventilation_tramponset;
	float  ventilation_trampfinal;

};

typedef struct ctl_parms Config;
Config cfg;

void bareMetal(byte* b) {
  SerialDebug.println("Bare Metal");
  Config cfg;

  sprintf(cfg.bt_name,"APL\0");
  sprintf(cfg.bt_pin,"0000\0");
  cfg.configuration_id = CONFIGURATION_ID;
  cfg.channel = 14;
  cfg.zone = 100;

  cfg.curtain_setpoint = 70.0;
  cfg.curtain_idleband = 6.0;
  cfg.curtain_upcyclesecs = 300;
  cfg.curtain_uponsecs = 60;
  cfg.curtain_dncyclesecs = 300;
  cfg.curtain_dnonsecs = 60;

  cfg.heatpad_setpoint = 100.0;
  cfg.heatpad_idleband = 8.0;

  cfg.stirfan_setpoint = 80.0;
  cfg.stirfan_idleband = 4.0;

  cfg.spaceheater_setpoint = 60.0;
  cfg.spaceheater_idleband = 8.0;

  cfg.ventilation_cyclesecs = 600;
  cfg.ventilation_minpcton = .010;
  cfg.ventilation_tramponset = 70.0;
  cfg.ventilation_trampfinal = 85.0;

  memcpy(b, &cfg, sizeof(cfg));
}

void initialize(byte* b) {
  SerialDebug.println("initializing...");
  memcpy(&cfg, b, sizeof(cfg));
}
/*
float getHeatPadTemp() {
	float tempF;
	while (ds_pad.selectNext()) {
	  tempF = ds_pad.getTempF();
	}
	return tempF;
}

float getRoomTemp() {
	float tempF;
	while (ds_room.selectNext()) {
	  tempF = ds_room.getTempF();
	}
	return tempF;
}
*/
float getHeatPadTemp() {
	ds_pad.selectNext();
	float tempF = ds_pad.getTempF();
	return tempF;
}

float getRoomTemp() {
	ds_room.selectNext();
	float tempF = ds_room.getTempF();
	return tempF;
}

#if defined(BTSERIALTERMINAL)
void receiveSerialTerminal(char* s) {
	for(uint8_t k=0; k<16; k++) {
		s[k] = 0;
	}
	uint8_t n=0;
	while( (SerialTerminal.available()==0) && n<5) {
	  n += 1;
	  delay(50);
	}
	int8_t nchars = SerialTerminal.available();
	for (uint8_t m=0; m<nchars; m++) {
	  s[m] = SerialTerminal.read();
	}
}

void cmdGetInfo() {
	 char durations[64];
	 long seconds;
	 double heat_pct;
     seconds = upTimeSecs;
     heat_pct=(100.0*heatingTimeSecs)/upTimeSecs;
     SerialTerminal.println(F("\n~~Heat Pad Status~~~~~~~~~~~~~~~~"));
     SerialTerminal.print("upTime   : ");
	 int hrs = seconds/3600;                                                        //Number of seconds in an hour
	 int mins = (seconds-hrs*3600)/60;                                     //Remove the number of hours and calculate the minutes.
	 int secs = seconds-hrs*3600-mins*60;                                //Remove the number of hours and minutes, leaving only seconds.
	 sprintf(durations, "%ih %im %is  [%i%%] ON", hrs, mins, secs, (int)heat_pct);
     SerialTerminal.println(durations);
     SerialTerminal.print("Current temperature "); SerialTerminal.print(heatPadController.getLastTemp()); SerialTerminal.println(" F");
     if (heatPadController.getStatus()== SSR_ON)
       SerialTerminal.println("The heat pads are ON now.");
     else
       SerialTerminal.println("The heat pads are OFF now.");
     SerialTerminal.println("==================");

     char hexdigit[2];

     SerialTerminal.print("coreId (DB): ");
     for(int n=0; n<8; n++) {
       sprintf(hexdigit,"%02x",devEui[n]);
       SerialTerminal.print(hexdigit);
     }

     SerialTerminal.println("\ndevEui (Chirpstack format): ");
     for(int n=7; n>=0; n--) {
       sprintf(hexdigit,"%02x",devEui[n]);
       SerialTerminal.print(hexdigit);
     }

     SerialTerminal.println("\nappKey: ");
     for(uint8_t n=0; n<16; n++) {
       sprintf(hexdigit,"%02x",appKey[n]);
       SerialTerminal.print(hexdigit);
     }
     SerialTerminal.println("\n==================");

}


//the cmdSet function will enforce these setting values:
#define MAX_HEATPAD_TEMP 120.0
#define MIN_HEATPAD_TEMP 75.0
#define MIN_HEATPAD_IDLEBAND 4.0
#define MAX_HEATPAD_IDLEBAND 16.0

void cmdSetHeatpad() {
  float setTemp = atof(term.getNext());
  float idleBand = atof(term.getNext());
  if ((setTemp>=MIN_HEATPAD_TEMP)
		  && (setTemp<=MAX_HEATPAD_TEMP)
		  && (idleBand>=MIN_HEATPAD_IDLEBAND)
		  && (idleBand<=MAX_HEATPAD_IDLEBAND)) {
    cfg.heatpad_setpoint = setTemp;
    cfg.heatpad_idleband = idleBand;
    heatPadController.set(cfg.heatpad_setpoint, cfg.heatpad_idleband);
    SerialTerminal.println("Temperature is set.");
  } else {
	SerialTerminal.println("Temperature could not be set.");
  }
}



#define MAX_SPACEHEAT_TEMP 120.0
#define MIN_SPACEHEAT_TEMP 75.0
#define MIN_SPACEHEAT_IDLEBAND 4.0
#define MAX_SPACEHEAT_IDLEBAND 16.0
void cmdSetSpaceHeat() {
    //setpoint and idleband
	float setTemp = atof(term.getNext());
	float idleBand = atof(term.getNext());
	if ((setTemp>=MIN_SPACEHEAT_TEMP)
	  && (setTemp<=MAX_SPACEHEAT_TEMP)
	  && (idleBand>=MIN_SPACEHEAT_IDLEBAND)
	  && (idleBand<=MAX_SPACEHEAT_IDLEBAND)) {
	  cfg.spaceheater_setpoint = setTemp;
	  cfg.spaceheater_idleband = idleBand;
	  spaceHeaterController.set(cfg.heatpad_setpoint, cfg.heatpad_idleband);
	  SerialTerminal.println("Space Heater Temperature is set.");
	} else {
	  SerialTerminal.println("Space Heater  could not be set.");
	}
}
#define MAX_STIRFAN_TEMP 120.0
#define MIN_STIRFAN_TEMP 75.0
#define MIN_STIRFAN_IDLEBAND 4.0
#define MAX_STIRFAN_IDLEBAND 16.0
void cmdSetStirFan() {
    //setpoint and idleband
	float setTemp = atof(term.getNext());
	float idleBand = atof(term.getNext());
	if ((setTemp>=MIN_STIRFAN_TEMP)
	  && (setTemp<=MAX_STIRFAN_TEMP)
	  && (idleBand>=MIN_STIRFAN_IDLEBAND)
	  && (idleBand<=MAX_STIRFAN_IDLEBAND)) {
	  cfg.stirfan_setpoint = setTemp;
	  cfg.stirfan_idleband = idleBand;
	  stirFanController.set(cfg.heatpad_setpoint, cfg.heatpad_idleband);
	  SerialTerminal.println("Stir Fan Temperature is set.");
	} else {
	  SerialTerminal.println("Stir Fan could not be set.");
	}
}
#define MAX_CURTAIN_TEMP 120.0
#define MIN_CURTAIN_TEMP 75.0
#define MIN_CURTAIN_IDLEBAND 4.0
#define MAX_CURTAIN_IDLEBAND 16.0
void cmdSetCurtain() {
    //setpoint and idleband
	float setTemp = atof(term.getNext());
	float idleBand = atof(term.getNext());
	if ((setTemp>=MIN_CURTAIN_TEMP)
	  && (setTemp<=MAX_CURTAIN_TEMP)
	  && (idleBand>=MIN_CURTAIN_IDLEBAND)
	  && (idleBand<=MAX_CURTAIN_IDLEBAND)) {
	  cfg.curtain_setpoint = setTemp;
	  cfg.curtain_idleband = idleBand;
	  curtainController.set(cfg.curtain_dncyclesecs,
	  		                cfg.curtain_dnonsecs,
	  						cfg.curtain_upcyclesecs,
	  						cfg.curtain_uponsecs,
	  						cfg.curtain_setpoint,
	  						cfg.curtain_idleband);
	  SerialTerminal.println("Curtain Temperature is set.");
	} else {
	  SerialTerminal.println("Curtain could not be set.");
	}
}
#define MIN_VENTSYS_ONSETTEMP 50.0
#define MAX_VENTSYS_ONSETTEMP 75.0
#define MIN_VENTSYS_FINALTEMP 80.0
#define MAX_VENTSYS_FINALTEMP 95.0
#define MIN_VENTSYS_DF 0.1
void cmdSetVentSys() {
    //setpoint and idleband
	float minVent = atof(term.getNext());
	float rampOnset = atof(term.getNext());
	float rampFinal = atof(term.getNext());
	if ((rampOnset>=MIN_VENTSYS_ONSETTEMP)
	  && (rampOnset<=MAX_VENTSYS_ONSETTEMP)
	  && (rampFinal>=MIN_VENTSYS_FINALTEMP)
	  && (rampFinal<=MAX_VENTSYS_FINALTEMP)
	  && (minVent>=MIN_VENTSYS_DF)) {
	  cfg.ventilation_minpcton = minVent;
	  cfg.ventilation_tramponset = rampOnset;
	  cfg.ventilation_trampfinal = rampFinal;
	  ventilationController.setpoint(cfg.ventilation_minpcton,
			                         100.0,
									 cfg.ventilation_tramponset,
									 cfg.ventilation_trampfinal);
	  SerialTerminal.println("Ventilation is set.");
	} else {
	  SerialTerminal.println("Ventilation could not be set.");
	}
}

void cmdSetChannel() {
  uint8_t channel = atoi(term.getNext());
  cfg.channel = channel;
  SerialTerminal.println(F("Channel set"));
}

void cmdHelp() {
	SerialTerminal.println(F("\n~~HeatBot Commands~~~~~~~~~~~~"));
	SerialTerminal.println(F("  help     Print usage info"));
	SerialTerminal.println(F("  set      <center> <idleband>"));
	SerialTerminal.println(F("  config   Show configuration"));
	SerialTerminal.println(F("  save     Save configuration"));
	SerialTerminal.println(F("  reset    Reset controller"));
	SerialTerminal.println(F("  info     Current status"));
	SerialTerminal.println(F("  btname   Set BT name"));
	SerialTerminal.println(F("  padheat  sp ib"));
	SerialTerminal.println(F("  spaceheat sp ib"));
	SerialTerminal.println(F("  stirfan  sp ib"));
	SerialTerminal.println(F("  curtain  sp ib"));
	SerialTerminal.println(F("  ventsys  min ro rf"));
	SerialTerminal.println("==================");
}

void cmdSetBtName() {
  char* bt_name = term.getNext();
  char c;
  uint8_t n=0;
  do {
    c = bt_name[n];
    cfg.bt_name[n] = c;
    n++;
  } while (c!=0);
  SerialTerminal.println("BT name changed.");
}

void cmdSave() {
	uint8_t size = sizeof(ctl_parms);
	byte b[size];
	memcpy(b, &cfg, size);
	eeprom.saveDeviceConfig(b, EEPROM_CONFIG_ADDRESS,size);
	SerialTerminal.println("Configuration saved.");
}

void cmdReboot() {
	SerialTerminal.println(F("Resetting the hardware..."));
	delay(500);
	NVIC_SystemReset();
}

void cmdShow() {
	float tempF = getHeatPadTemp();
	SerialTerminal.println(F("\n~~Heat Pad Configuration~~~~~~~~~"));
	SerialTerminal.print("ConfigId : "); SerialTerminal.println(cfg.configuration_id);
	SerialTerminal.print("BT Name  : "); SerialTerminal.println(cfg.bt_name);
	SerialTerminal.print("BT PIN   : "); SerialTerminal.println(cfg.bt_pin);
	SerialTerminal.print("Channel  : "); SerialTerminal.println(cfg.channel);
	SerialTerminal.print("Zone     : "); SerialTerminal.println(cfg.zone);
    SerialTerminal.println("-----------------");

    SerialTerminal.print("heatpad SP: "); SerialTerminal.print(cfg.heatpad_setpoint); SerialTerminal.println(" F");
    SerialTerminal.print("        IB: "); SerialTerminal.print(cfg.heatpad_idleband);  SerialTerminal.println(" F");

    SerialTerminal.print("spaceheater SP: "); SerialTerminal.print(cfg.spaceheater_setpoint); SerialTerminal.println(" F");
    SerialTerminal.print("        IB: "); SerialTerminal.print(cfg.spaceheater_idleband);  SerialTerminal.println(" F");

    SerialTerminal.print("stirfan SP: "); SerialTerminal.print(cfg.stirfan_setpoint); SerialTerminal.println(" F");
    SerialTerminal.print("        IB: "); SerialTerminal.print(cfg.stirfan_idleband);  SerialTerminal.println(" F");

    SerialTerminal.print("curtain SP: "); SerialTerminal.print(cfg.curtain_setpoint); SerialTerminal.println(" F");
    SerialTerminal.print("        IB: "); SerialTerminal.print(cfg.curtain_idleband);  SerialTerminal.println(" F");

    SerialTerminal.print("ventsys MIN: "); SerialTerminal.print(cfg.ventilation_minpcton); SerialTerminal.println(" F");
    SerialTerminal.print("        TRO: "); SerialTerminal.print(cfg.ventilation_tramponset);  SerialTerminal.println(" F");
    SerialTerminal.print("        TRF: "); SerialTerminal.print(cfg.ventilation_trampfinal);  SerialTerminal.println(" F");

	//SerialTerminal.print("On  below "); SerialTerminal.print(cfg.heatPad_SP-cfg.heatPad_IB/2); SerialTerminal.println(" F");
	//SerialTerminal.print("Off above "); SerialTerminal.print(cfg.heatPad_SP+cfg.heatPad_IB/2); SerialTerminal.println(" F");
	//SerialTerminal.print("Current T "); SerialTerminal.print(tempF); SerialTerminal.println(" F");
	SerialTerminal.println("==================");
}

void setBtParms() {
  //SerialTerminal.begin(38400);
  char atcommand[32];
  char recv[16];
  sprintf(atcommand,"AT");
  SerialDebug.println(atcommand);
  SerialTerminal.println(atcommand);
  receiveSerialTerminal(recv);
  SerialDebug.print("AT response: "); SerialDebug.println(recv);
  SerialDebug.println("End of AT response");
  //want to know if we received an OK response
  if (strncmp (recv,"OK",2) == 0) {
	SerialDebug.println("BT in command mode, setting parms now.");
	delay(1000);
    sprintf(atcommand,"AT+NAME=%s",cfg.bt_name);
	SerialDebug.println(atcommand);
	SerialTerminal.println(atcommand);
	receiveSerialTerminal(recv);
	SerialDebug.println(recv);
	delay(1000);
	sprintf(atcommand,"AT+PSWD=%s",cfg.bt_pin);
	SerialDebug.println(atcommand);
	SerialTerminal.println(atcommand);
	receiveSerialTerminal(recv);
	SerialDebug.println(recv);
	delay(1000);
	sprintf(atcommand,"AT+RESET");
	SerialDebug.println(atcommand);
	SerialTerminal.println(atcommand);
	delay(2000);
	SerialDebug.println("Bluetooth parameters set.");
  } else {
	SerialDebug.println("BT not in command mode.");
  }
  //SerialTerminal.begin(9600);
}
void unknownCommand(const char *command) {
    SerialTerminal.print(F("Unknown command: "));
    SerialTerminal.println(command);
}
#endif

void i2c_eeprom_read_buffer( int deviceaddress, unsigned int eeaddress, byte *buffer, int length ) {
    Wire.beginTransmission(deviceaddress);
    Wire.write(eeaddress);
    Wire.endTransmission();
    Wire.requestFrom(deviceaddress,length);
    int c = 0;
    for ( c = 0; c < length; c++ )
       if (Wire.available())
         buffer[c] = Wire.read();
    Wire.endTransmission();
}

// These methods are called by the LMIC code after the JOIN
void os_getArtEui (u1_t* buf) {
  memcpy_P(buf, APPEUI, 8);
}

void os_getDevEui (u1_t* buf) {
  memcpy_P(buf, devEui, 8);
}

void os_getDevKey (u1_t* buf) {
  memcpy_P(buf, appKey, 16);
}

void buildMoteRec() {
  moteRec.heatpad_temp = (uint8_t) getHeatPadTemp();
  moteRec.room_temp = (uint8_t) getRoomTemp();

  moteRec.heatpad_setpoint = cfg.spaceheater_setpoint;
  moteRec.heatpad_idleband = cfg.heatpad_idleband;
  moteRec.curtain_setpoint = cfg.curtain_setpoint;
  moteRec.curtain_idleband = cfg.curtain_idleband;
  moteRec.curtain_upcyclesecs = cfg.curtain_upcyclesecs;
  moteRec.curtain_uponsecs = cfg.curtain_uponsecs;
  moteRec.curtain_dncyclesecs = cfg.curtain_dncyclesecs;
  moteRec.curtain_dnonsecs = cfg.curtain_dnonsecs;
  moteRec.heatpad_setpoint = cfg.heatpad_setpoint;
  moteRec.heatpad_idleband = cfg.heatpad_idleband;

  moteRec.stirfan_setpoint = cfg.stirfan_setpoint;
  moteRec.stirfan_idleband = cfg.stirfan_idleband;

  moteRec.spaceheater_setpoint = cfg.spaceheater_setpoint;
  moteRec.spaceheater_idleband = cfg.spaceheater_idleband;

  moteRec.ventilation_cyclesecs = cfg.ventilation_cyclesecs;
  moteRec.ventilation_minpcton = cfg.ventilation_minpcton;
  moteRec.ventilation_tramponset = cfg.ventilation_tramponset;
  moteRec.ventilation_trampfinal = cfg.ventilation_trampfinal;
}

void processLoop() {
  SerialDebug.print(".");

  moteRec.circuitflags = 0x0000;
  float padTempF = 90.0; //getHeatPadTemp();
  //Serial.println(padTempF);
  heatPad_OP = heatPadController.poll(padTempF);
  moteRec.circuitflags |= heatPad_OP;
  //upTimeSecs +=  (millis() - lastProcessMillis)/1000;
  //if  (heatPadController.getStatus()== SSR_ON)
  //  heatingTimeSecs +=  (millis() - lastProcessMillis)/1000;

  float roomTempF = 72.9; //getRoomTemp();
  //SerialDebug.print("roomT="); SerialDebug.println(roomTempF);
  spaceHeater_OP = spaceHeaterController.poll(roomTempF);
  moteRec.circuitflags |= (spaceHeater_OP << 1);
  curtainController.clock();
  curtain_OP = curtainController.poll(padTempF); //roomTempF);
  if (curtain_OP == _CLOSING)
    moteRec.circuitflags |= (0x04);
  if (curtain_OP == _OPENING)
    moteRec.circuitflags |= (0x08);

  stirFan_OP = stirFanController.poll(padTempF); //roomTempF);
  moteRec.circuitflags |= (stirFan_OP << 4);
  ventilation_OP = ventilationController.poll(roomTempF);
  moteRec.circuitflags |= (ventilation_OP << 5);
}

void scheduleNextSend(void);			//predeclared here
/*
 * do_send() is called periodically to send sensor data
 */
void do_send(osjob_t* j) {
	if (LMIC.opmode & OP_TXRXPEND) {
	  // Check if there is a current TX/RX job running
    } else {
      buildMoteRec();
      if (ackReceived)
        moteRec.lastackreceived = (uint8_t) 1;
      else
        moteRec.lastackreceived = (uint8_t) 0;

      memcpy(b, &moteRec, sizeof(moteRec));

      if ((frameCount%10)==0) {
        LMIC_setTxData2(1, (byte*) b, sizeof(moteRec), 1);	// queue the packet, ACK
        ackRequested = true;
        ackReceived = false;
      } else {
        LMIC_setTxData2(1, (byte*) b, sizeof(moteRec), 0);	// queue the packet, NO ACK
        ackRequested = false;
        ackReceived = false;
      }
      scheduleNextSend();
    }
}

void sitUbu() {
  #if not defined(_TEST_WATCHDOG)
  digitalWrite(_WATCHDOG_DONE, HIGH);				// rising edge pulse on DONE to keep the watchdog happy!
  delay(2);											// pet the dog before going to sleep.
  digitalWrite(_WATCHDOG_DONE, LOW);
  #endif
}

/*
 * alarmMatch() is invoked when the rtcZero clock alarms.
 */
void alarmMatch()  {
  //the following call attempts to avoid collisions with other motes by randomizing the send time
  os_setTimedCallback(&sendjob, os_getTime()+ms2osticks(random(200)), do_send);
  frameCount++;
  scheduleNextSend();
}

void scheduleNextSend() {
  // set alarm for TX_INTERVAL_SECONDS from now
  rtc.setAlarmEpoch( rtc.getEpoch() + TX_INTERVAL_SECONDS);
  rtc.enableAlarm(rtc.MATCH_YYMMDDHHMMSS);
  rtc.attachInterrupt(alarmMatch);
}

void onEvent (ev_t ev) {
  #ifdef _DEBUG_SERIAL
  Serial.print(os_getTime());
  Serial.print(": ["); Serial.print(ev); Serial.print("] ");
  #endif
  switch(ev) {
    /*
    case EV_SCAN_TIMEOUT:
      #ifdef _DEBUG_SERIAL
      Serial.println(F("SCAN_TIMEOUT"));
      #endif
      break;
    case EV_BEACON_FOUND:
      #ifdef _DEBUG_SERIAL
      Serial.println(F("BEACON_FOUND"));
      #endif
      break;
    case EV_BEACON_MISSED:
      #ifdef _DEBUG_SERIAL
      Serial.println(F("BEACON_MISSED"));
      #endif
      break;
    case EV_BEACON_TRACKED:
      #ifdef _DEBUG_SERIAL
      Serial.println(F("BEACON_TRACKED"));
      #endif
      break;
    */
    case EV_JOINING:
      #ifdef _DEBUG_SERIAL
      Serial.println(F("JOINING"));
      #endif
      break;
    case EV_JOINED: {
      u4_t netid = 0;
      devaddr_t devaddr = 0;
      u1_t nwkKey[16];
      u1_t artKey[16];
      LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
      loraJoined = true;
      #ifdef _DEBUG_SERIAL
      Serial.println(F("JOINED"));
      /*
      Serial.print("netid: ");
      Serial.println(netid, DEC);
      Serial.print("devaddr: "); // @suppress("Method cannot be resolved")
      Serial.println(devaddr, HEX);
      Serial.print("artKey: ");
      for (int i=0; i<sizeof(artKey); ++i) {
        if (i != 0)
          Serial.print("-");
        Serial.print(artKey[i], HEX);
      }
      Serial.println("");
      Serial.print("nwkKey: ");
      for (int i=0; i<sizeof(nwkKey); ++i) {
        if (i != 0)
          Serial.print("-");
        Serial.print(nwkKey[i], HEX);
      }
      Serial.println("");
      */
      #endif
      LMIC_setLinkCheckMode(0);
      break;
    }
    /*
    case EV_JOIN_FAILED:
      #ifdef _DEBUG_SERIAL
      Serial.println(F("JOIN_FAIL"));
      #endif
      loraJoined = false;
      scheduleNextSend(); //goToSleep();
      break;
    case EV_REJOIN_FAILED:
      #ifdef _DEBUG_SERIAL
      Serial.println(F("REJOIN_FAIL"));
      #endif
      loraJoined = false;
      scheduleNextSend(); //goToSleep();
      break;
    */
    case EV_TXCOMPLETE:
      digitalWrite(LED_BUILTIN, LOW);     // LED Off
      #ifdef _DEBUG_SERIAL
      //Serial.println(F("TXCOMPLETE"));
      #endif
      if ((ackRequested) && (LMIC.txrxFlags & TXRX_ACK)) {
        // if confirmation was requested and is received - cycle the WATCHDOG bit
        ackReceived = true;
        #ifdef _DEBUG_SERIAL
        //Serial.println(F("Ack"));
        #endif
      }
      // Check if we have a downlink on either Rx1 or Rx2 windows
      if ((LMIC.txrxFlags & ( TXRX_DNW1 | TXRX_DNW2 )) != 0) {
#ifdef _DEBUG_SERIAL
    	  /*
      	if ((LMIC.txrxFlags & TXRX_DNW1) != 0)
       	  Serial.print(F("DRx1-"));
       	else
       	  Serial.print(F("DRx2-"));
       	  */
#endif
       	if (LMIC.dataLen) {
       	  #ifdef _DEBUG_SERIAL
       	  Serial.print(LMIC.dataLen);
       	  // Receive bytes in LMIC.frame are the B64 decode of the
       	  // 'data' field sent by ChirpStack
       	  for (int i = 0; i < LMIC.dataLen; i++) {
       	    Serial.print(LMIC.frame[LMIC.dataBeg + i], HEX);
       	  }
       	  Serial.println();
       	  #endif

       	}
      }
      break;
      /*
    case EV_LOST_TSYNC:
      #ifdef _DEBUG_SERIAL
      Serial.println(F("EV_LOST_TSYNC"));
      #endif
      break;
    case EV_RESET:
      #ifdef _DEBUG_SERIAL
      Serial.println(F("EV_RESET"));
      #endif
      loraJoined = false;
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      #ifdef _DEBUG_SERIAL
      Serial.println(F("EV_RXCOMPLETE"));
      #endif
      break;
    case EV_LINK_DEAD:
      //no confirmation has been received for an extended perion
      #ifdef _DEBUG_SERIAL
      Serial.println(F("EV_LINK_DEAD"));
      #endif
      loraJoined = false;
      scheduleNextSend(); //goToSleep();
      break;
    case EV_LINK_ALIVE:
      #ifdef _DEBUG_SERIAL
      Serial.println(F("EV_LINK_ALIVE"));
      #endif
      break;
      */
    case EV_TXSTART:
      #ifdef _DEBUG_SERIAL
      Serial.println(F("EV_TXSTART"));
      #endif
      //digitalWrite(LED_BUILTIN, LOW);
      break;
    default:
      #ifdef _DEBUG_SERIAL
      //Serial.print(F("Unknown event: "));
      //Serial.println((unsigned) ev);
      #endif
      break;
  }
}



void setup() {
  // Resets occurring ~immediately after a successful SEND #10, is this the Brown Out Detector firing?
  // Configure the regulator to run in normal mode when in standby mode
  // Otherwise it defaults to low power mode and can only supply 50 uA
  SYSCTRL->VREG.bit.RUNSTDBY = 1;

  pinMode(_WATCHDOG_DONE, OUTPUT);					 // Watchdog Done pin
  digitalWrite(_WATCHDOG_DONE, LOW);

  SerialDebug.begin(115200);

#if defined (WAIT_FOR_SERIALDEBUG)
  while(!SerialDebug) {

  }
#endif

  SerialDebug.println(F("Starting"));
  SerialDebug.println("Reading EEPROM...");
  Wire.begin();
  Wire.setClock(50000UL);			// lowering the clock frequency to see if reliability improves
  uint8_t size = sizeof(ctl_parms);
  byte b[size];
  //new approach to BARE METAL CONFIGURATION
  //if the configuration_id stored in EEPROM doesn't match
  //the CONFIGURATION_ID in this file, we'll do the bare metal build
  eeprom.loadDeviceConfig(b, EEPROM_CONFIG_ADDRESS, size);
  initialize(b);
  while (cfg.configuration_id != CONFIGURATION_ID) {
	  bareMetal(b);
	  eeprom.saveDeviceConfig(b, EEPROM_CONFIG_ADDRESS, size);
	  delay(1000);
	  eeprom.loadDeviceConfig(b, EEPROM_CONFIG_ADDRESS, size);
	  initialize(b);
	  SerialDebug.println("BARE_METAL_BUILD");
  }
  //cmdShow();
  heatPadController.setDeviceName("heat pad");
  heatPadController.setOutput(HEATPAD_OUTPUT);
  heatPadController.set(cfg.heatpad_setpoint, cfg.heatpad_idleband);

  spaceHeaterController.setDeviceName("gas heat");
  spaceHeaterController.setOutput(SPACEHEATER_OUTPUT);
  spaceHeaterController.set(cfg.spaceheater_setpoint, cfg.spaceheater_idleband);

  stirFanController.setDeviceName("stir fan");
  stirFanController.setOutput(STIRFAN_OUTPUT);
  stirFanController.setMode(MODE_COOL);
  stirFanController.set(cfg.stirfan_setpoint, cfg.stirfan_idleband);

  curtainController.setDeviceName("curtain");
  curtainController.setOutput(CURTAIN_OPENOUTPUT, CURTAIN_CLOSEOUTPUT);
  curtainController.set(cfg.curtain_dncyclesecs,
		                cfg.curtain_dnonsecs,
						cfg.curtain_upcyclesecs,
						cfg.curtain_uponsecs,
						cfg.curtain_setpoint,
						cfg.curtain_idleband);

  ventilationController.setDeviceName("vent");
  ventilationController.setOutput(VENTILATION_OUTPUT);
  ventilationController.setpoint(cfg.ventilation_minpcton,
		                         100.0,
								 cfg.ventilation_tramponset,
								 cfg.ventilation_trampfinal);

  //get the DEVEUI from the EEPROM
  i2c_eeprom_read_buffer(_EEPROM_IC2_ADDR, _DEVEUI_ADDR, devEui, 8);
  memcpy_P(appKey, APPKEY_ROOT, 16);
  //modify the appKey
  for (uint8_t n=8; n<16; n++) {
    appKey[n] = devEui[n-8];
  }
  #if defined(_DUMP_KEYS)
  delay(1000);
  SerialDebug.println("EEPROM read complete.");
  char hexdigit[2];
  SerialDebug.print("coreId (DB): ");
  for(int n=0; n<8; n++) {
    sprintf(hexdigit,"%02x",devEui[n]);
    SerialDebug.print(hexdigit);
  }
  SerialDebug.print("\n\rdevEui (Chirpstack format): ");
  for(int n=7; n>=0; n--) {
    sprintf(hexdigit,"%02x",devEui[n]);
    SerialDebug.print(hexdigit);
  }
  SerialDebug.print("\n\rappKey: ");
  for(uint8_t n=0; n<16; n++) {
    sprintf(hexdigit,"%02x",appKey[n]);
    SerialDebug.print(hexdigit);
  }
  SerialDebug.println();
  #endif
  rtc.begin();                                        // Initialize RTC
  rtc.setEpoch(0);                                    // Use RTC as a second timer instead of calendar
  os_init();                                               // LMIC init
  LMIC_reset();                                            // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_setClockError(MAX_CLOCK_ERROR * 10 / 100);

  if (cfg.channel!=255) {
    for(uint8_t c=0; c<72; c++) {
  	  if (c!=cfg.channel)
        LMIC_disableChannel(c);
  	  else
  	    LMIC_enableChannel(c);
    }
  } else {
      LMIC_selectSubBand(1);
  }

  LMIC_setLinkCheckMode(0);
  LMIC_setDrTxpow(DR_SF7,14);

  #ifdef _DEBUG_SERIAL
  SerialDebug.println("Setup completed.");
  #endif

  //randomSeed(analogRead(1));	//seed the random number generator
  frameCount=0;

#if defined(BTSERIALTERMINAL)
  //we can't start Serial1 until now.
  SerialTerminal.begin(38400);
  delay(5000);
  setBtParms();				//if the BT module is in AT command mode we'll configure it
  SerialTerminal.begin(9600);

  term.addCommand("help", cmdHelp);
  term.addCommand("heatpad", cmdSetHeatpad);
  term.addCommand("spaceheat", cmdSetSpaceHeat);
  term.addCommand("stirfan", cmdSetStirFan);
  term.addCommand("curtain", cmdSetCurtain);
  term.addCommand("ventsys", cmdSetVentSys);
  term.addCommand("save", cmdSave);
  term.addCommand("show", cmdShow);
  term.addCommand("reset", cmdReboot);
  term.addCommand("info", cmdGetInfo);
  term.addCommand("btname", cmdSetBtName);
  term.addCommand("channel", cmdSetChannel);

  cmdHelp();
#endif
  loraJoined = false;
  do_send(&sendjob);
}

void loop() {
  os_runloop_once();
  sitUbu();

  if (loraJoined && ( (millis() - lastProcessMillis) > _PROCESS_TIME_MILLIS )) {
  	//processLoop();
#if defined(BTSERIALTERMINAL)
  	term.readSerial();
#endif
  	//sitUbu();
  	lastProcessMillis = millis();
  }
}
