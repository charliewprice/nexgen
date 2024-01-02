#include <Arduino.h>

///#define _TEST_WATCHDOG		//used to determine the actual watchdog timeout
//#define WAIT_FOR_SERIALDEBUG
//#define _DEBUG_SERIAL

#define CONFIGURATION_ID 9008
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
#define FOGGER_OUTPUT 10

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

#include "CurtainController.h"
CurtainController curtainController;
uint8_t curtain_OP;

#include "TdynDutyCycleControl.h"
TdynDutyCycleControl ventilationController;
uint8_t ventilation_OP;

TdynDutyCycleControl foggerController;
uint8_t fogger_OP;


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
#define TX_INTERVAL_SECONDS 5        	// Schedule the interval(seconds) at which data is sent
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
DS18B20 ds_pad(A1);
DS18B20 ds_room(A0);

boolean useTestTemperature = false;
float testTemperature;

/*
 * Mote Record Structure
 */
#define _DEVICE_PROFILE 41

struct ctl_parms {
	float curtain_setpoint_temp;
	float curtain_idleband_degrees;
#define curtain_cyclesecs 30
#define curtain_onsecs 6
	float heatpad_setpoint_temp;
	float heatpad_idleband_degrees;
	float stirfan_setforward_degrees;
#define stirfan_idleband_degrees 4.0
	float spaceheater_setback_degrees;
	float spaceheater_idleband_degrees;
#define ventilation_cyclesecs 30
	float  ventilation_setback_degrees;
	float  ventilation_df_min;
  float  ventilation_df_max;
#define fogger_cyclesecs 30
	float  fogger_setforward_degrees;
  float  fogger_rampup_degrees;
	float  fogger_df_min;
	float  fogger_df_max;	
  uint16_t configuration_id;
};

typedef struct ctl_parms Config;
Config cfg;

struct teststruct {
   float roomTemperature;
};
typedef struct teststruct TestStruct;
TestStruct test;

#define MODE_NORMAL 0
#define MODE_TEST   1

/*
 * this is the record structure that is sent over LoRa
 * received by the LoRa gateway and relayed to the 
 * Chirpstack interface.
*/
struct moterecord {
  uint8_t  config[sizeof(cfg)];
  float  heatpad_temp;
	float  room_temp;
  float  evac_pct;
  float  fogg_pct;
	uint16_t switches;
	uint8_t  lastackreceived;
  uint8_t  mode;
};
typedef struct moterecord Record;
Record moteRec;
char b[sizeof(moteRec)];

/*
 * This is the message that is sent via RS485 serial
 * down to the switch box
 */
struct rs485packet {
  uint8_t  command;
  uint8_t  cksum;
  uint16_t switches;
};

typedef struct rs485packet Rs485Packet;
Rs485Packet rPacket;
char rsbytes[sizeof(rPacket)];

uint16_t lastSwitches = 0;

#define _PROCESS_TIME_MILLIS 1000
long upTimeSecs;
long lastProcessMillis;
long heatingTimeSecs;

void bareMetal(byte* b) {
  SerialDebug.println("Bare Metal");
  Config cfg;
  cfg.configuration_id = CONFIGURATION_ID;
  cfg.curtain_setpoint_temp = 60.0;         //p[0]
	cfg.curtain_idleband_degrees = 6.0;       //p[1]
	cfg.heatpad_setpoint_temp = 93.0;         //p[2]
	cfg.heatpad_idleband_degrees = 4.0;       //p[3]
	cfg.stirfan_setforward_degrees = 15.0;    //p[4]
	cfg.spaceheater_setback_degrees = 25.0;   //p[5]
	cfg.spaceheater_idleband_degrees = 4.0;   //p[6]
  cfg.ventilation_setback_degrees = 20.0;   //p[7]
	cfg.ventilation_df_min = 20.0;            //p[8]
  cfg.ventilation_df_max = 100.0;           //p[9]
	cfg.fogger_setforward_degrees = 5.0;      //p[10]
  cfg.fogger_rampup_degrees = 10.0;         //p[11]
	cfg.fogger_df_min = 20.0;                 //p[12]
	cfg.fogger_df_max = 80.0;                 //p[13]

	memcpy(b, &cfg, sizeof(cfg));
}

void initialize(byte* b) {
  SerialDebug.println(F("Initializing..."));
  memcpy(&cfg, b, sizeof(cfg));
}

void storeConfig() {
	uint8_t size = sizeof(ctl_parms);
	byte b[size];
	memcpy(b, &cfg, size);
	eeprom.saveDeviceConfig(b, EEPROM_CONFIG_ADDRESS,size);
	SerialDebug.println(F("Configuration saved."));
}

void reboot() {
	SerialDebug.println(F("Rebooting..."));
	delay(500);
	NVIC_SystemReset();
}

float getHeatPadTemp() {
  if (!useTestTemperature) {
	  ds_pad.selectNext();
	  float tempF = ds_pad.getTempF();
	  return tempF;
  } else {
    return test.roomTemperature;
  }
}

float getRoomTemp() {
  if (!useTestTemperature) {
	  ds_room.selectNext();
	  float tempF = ds_room.getTempF();
	  return tempF;
  } else {
    return test.roomTemperature;
  }
}

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
  moteRec.heatpad_temp = getHeatPadTemp();
  moteRec.room_temp = getRoomTemp();
  moteRec.evac_pct = ventilationController.getDutyFactor();
  moteRec.fogg_pct = foggerController.getDutyFactor();
  memcpy(moteRec.config, &cfg, sizeof(cfg));
  //Serial.print("room "); Serial.println(moteRec.room_temp);
  //Serial.print("pads "); Serial.println(moteRec.heatpad_temp);
}

void processLoop() {
  //SerialDebug.print("."); SerialDebug.println(sizeof(cfg));
  moteRec.switches = 0x0000;
  float padTempF = moteRec.heatpad_temp; //getHeatPadTemp();
  
  heatPad_OP = heatPadController.poll(padTempF);
  moteRec.switches |= heatPad_OP;
  
  float roomTempF = moteRec.room_temp; //getRoomTemp();
  //moteRec.heatpad_temp = padTempF;
  //moteRec.room_temp = roomTempF;
  //SerialDebug.print("roomT="); SerialDebug.println(roomTempF);
  //SerialDebug.print("hpadT="); SerialDebug.println(padTempF);
  
  spaceHeater_OP = spaceHeaterController.poll(roomTempF);
  moteRec.switches |= (spaceHeater_OP << 1);
  
  curtain_OP = curtainController.poll(roomTempF); //roomTempF);
  if (curtain_OP == _CLOSING)
    moteRec.switches |= (0x04);
  if (curtain_OP == _OPENING)
    moteRec.switches |= (0x08);

  stirFan_OP = stirFanController.poll(padTempF); //roomTempF);
  moteRec.switches |= (stirFan_OP << 4);
  
  ventilation_OP = ventilationController.poll(roomTempF);
  moteRec.switches |= (ventilation_OP << 5);

  fogger_OP = foggerController.poll(roomTempF);
  moteRec.switches |= (fogger_OP << 6);

  if (moteRec.switches != lastSwitches) {
    rPacket.command = 0xD3;
    uint16_t bitmask = 0x0001;
    uint8_t cksum = 0;
    for (uint8_t n=0; n<16; n++) {
      if ((moteRec.switches & bitmask) == bitmask)
        cksum += 1;
      bitmask = bitmask << 1;
    }
    rPacket.cksum = cksum;
    rPacket.switches = moteRec.switches;
    memcpy(rsbytes, &rPacket, sizeof(rPacket));
    SerialTerminal.write(rsbytes);
    lastSwitches = moteRec.switches;
  }
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
       	  //Serial.print(LMIC.dataLen);
       	  // Receive bytes in LMIC.frame are the B64 decode of the
       	  // 'data' field sent by ChirpStack
          
       	  for (int i = 0; i < LMIC.dataLen; i++) {
            if (LMIC.frame[LMIC.dataBeg + i]<16)
              Serial.print("0");
       	    Serial.print(LMIC.frame[LMIC.dataBeg + i], HEX);
       	  }
       	  Serial.println();
       	  #endif
          uint8_t cksum = 0;
          for (int i = 2; i < LMIC.dataLen; i++) {
            cksum += LMIC.frame[LMIC.dataBeg + i];
          }
          if (cksum == LMIC.frame[LMIC.dataBeg + 1]) {
            if (LMIC.frame[LMIC.dataBeg]==0xCF) {
              Serial.println("Charlie Foxtrot");
              uint16_t config_id = cfg.configuration_id;
              initialize(LMIC.frame + LMIC.dataBeg + 2);
              cfg.configuration_id = config_id;
              storeConfig();
              reboot();
            } else if (LMIC.frame[LMIC.dataBeg]==0xC7) { 
              Serial.println("Charlie Seven");
              memcpy(&test, LMIC.frame + LMIC.dataBeg + 2, sizeof(test));
              useTestTemperature = true;
              moteRec.mode = MODE_TEST;
            } else if (LMIC.frame[LMIC.dataBeg]==0xC5) { 
              Serial.println("Charlie Five");
              reboot();
            }
          } else {
            Serial.print("ckSum bad: "); 
            Serial.print(cksum); Serial.print(" should be "); Serial.println(LMIC.frame[LMIC.dataBeg + 1]);
            for (int i = 0; i < LMIC.dataLen; i++) {
              if (LMIC.frame[LMIC.dataBeg + i]<16)
                Serial.print("0");
       	      Serial.print(LMIC.frame[LMIC.dataBeg + i], HEX);
       	    }
       	    Serial.println();
          }          
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
  
   for(uint8_t n=0; n<size; n++) {
    if (b[n]<16)
      SerialDebug.print("0");
    SerialDebug.print(b[n],HEX);
  }

  SerialDebug.println();

  float evacfunc[4][3] = {
  {0.0,cfg.ventilation_df_min,0.0},
  {(cfg.curtain_setpoint_temp-cfg.ventilation_setback_degrees),0.0,((cfg.ventilation_df_max-cfg.ventilation_df_min)/ (cfg.ventilation_setback_degrees-(cfg.curtain_idleband_degrees/2.0))) },
  {(cfg.curtain_setpoint_temp-(cfg.curtain_idleband_degrees/2.0)),0.0,((cfg.ventilation_df_min-cfg.ventilation_df_max)/ (cfg.ventilation_setback_degrees-(cfg.curtain_idleband_degrees/2.0))) },
  {(cfg.curtain_setpoint_temp+(cfg.curtain_idleband_degrees/2.0)),-cfg.ventilation_df_max,0.0}
};

  float foggfunc[4][3] = {
    {(cfg.curtain_setpoint_temp+cfg.stirfan_setforward_degrees+(stirfan_idleband_degrees/2)+cfg.fogger_setforward_degrees),cfg.fogger_df_min,0.0},
    {(cfg.curtain_setpoint_temp+cfg.stirfan_setforward_degrees+(stirfan_idleband_degrees/2)+cfg.fogger_setforward_degrees),0.0,((cfg.fogger_df_max-cfg.fogger_df_min)/cfg.fogger_rampup_degrees)},
    {(cfg.curtain_setpoint_temp+cfg.stirfan_setforward_degrees+(stirfan_idleband_degrees/2)+cfg.fogger_setforward_degrees+cfg.fogger_rampup_degrees),0.0,((cfg.fogger_df_min-cfg.fogger_df_max)/cfg.fogger_rampup_degrees)},
    {1000.0,0.0,0.0}
  };
    

  heatPadController.setDeviceName("heat pad");
  heatPadController.setOutput(HEATPAD_OUTPUT);
  heatPadController.set(cfg.heatpad_setpoint_temp, cfg.heatpad_idleband_degrees);
  heatPadController.setMode(MODE_HEAT);
  heatPadController.dumpConfig();

  spaceHeaterController.setDeviceName("gas heat");
  spaceHeaterController.setOutput(SPACEHEATER_OUTPUT);
  spaceHeaterController.set(cfg.curtain_setpoint_temp-cfg.spaceheater_setback_degrees, cfg.spaceheater_idleband_degrees);
  spaceHeaterController.dumpConfig();
  spaceHeaterController.setMode(MODE_HEAT);

  stirFanController.setDeviceName("stir fan");
  stirFanController.setOutput(STIRFAN_OUTPUT);
  stirFanController.setMode(MODE_COOL);
  stirFanController.set(cfg.curtain_setpoint_temp+cfg.stirfan_setforward_degrees, stirfan_idleband_degrees);

  curtainController.setDeviceName("curtain");
  curtainController.setOutput(CURTAIN_OPENOUTPUT, CURTAIN_CLOSEOUTPUT);
  curtainController.set(curtain_cyclesecs,
		                curtain_onsecs,
						        cfg.curtain_setpoint_temp,
						        cfg.curtain_idleband_degrees);
  curtainController.dumpConfig();
 
  ventilationController.setDeviceName("vent");
  ventilationController.setOutput(VENTILATION_OUTPUT);
  ventilationController.setCycleTimeSecs(ventilation_cyclesecs);
  ventilationController.setFunction(evacfunc);

  foggerController.setDeviceName("fogger");
  foggerController.setOutput(FOGGER_OUTPUT);
  foggerController.setCycleTimeSecs(fogger_cyclesecs);
  foggerController.setFunction(foggfunc);

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
  char hexdigit[3];
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
  os_init();                                          // LMIC init
  LMIC_reset();                                       // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_setClockError(MAX_CLOCK_ERROR * 10 / 100);

  LMIC_selectSubBand(1);

  LMIC_setLinkCheckMode(0);
  LMIC_setDrTxpow(DR_SF7,14);

  #ifdef _DEBUG_SERIAL
  SerialDebug.println("Setup completed.");
  #endif

  //randomSeed(analogRead(1));	//seed the random number generator
  frameCount=0;
  SerialTerminal.begin(115200);
  moteRec.mode = MODE_NORMAL;
  loraJoined = true;
  do_send(&sendjob);
}

void loop() {
  os_runloop_once();

  if (loraJoined && ( (millis() - lastProcessMillis) > _PROCESS_TIME_MILLIS )) {
  	processLoop();
#if defined(BTSERIALTERMINAL)
  	term.readSerial();
#endif
  	sitUbu();
  	lastProcessMillis = millis();
  }
}
