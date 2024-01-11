#include <Arduino.h>
#include "wiring_private.h" // pinPeripheral() function

///#define _TEST_WATCHDOG		//used to determine the actual watchdog timeout
#define WAIT_FOR_SERIALDEBUG
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

#include "common.h"

#define HEATPAD_OUTPUT          0x0100
#define FOGGER_OUTPUT           0x0200
#define STIRFAN_OUTPUT          0x0400
#define CURTAIN_OPENOUTPUT      0x0800
#define CURTAIN_CLOSEOUTPUT     0x1000
#define EVAC_OUTPUT             0x2000
#define SPACEHEATER_OUTPUT      0x4000  
  
#define RS485_DIR 13

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
TstatOnOffControl spaceHeaterController;
TstatOnOffControl stirFanController;

#include "CurtainController.h"
CurtainController curtainController;

#include "TdynDutyCycleControl.h"
TdynDutyCycleControl evacfanController;
TdynDutyCycleControl foggerController;

#include "Eeprom.h"
Eeprom eeprom;
#define _EEPROMUPDATE_MILLIS 	30000
boolean updateEeprom;

//Uart Serial2 (&sercom1, 11, 10, SERCOM_RX_PAD_0, UART_TX_PAD_2);
#define rs485Serial Serial1

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
//#define _SEND_INTERVAL_SECONDS 15
#define LORA_TX_INTERVAL_SECONDS 5       	// Schedule the interval(seconds) at which data is sent

static osjob_t sendLoRaJob;
static osjob_t sendRS485Job;
static osjob_t processJob;

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

#define _DEVICE_PROFILE 41

struct ctl_parms {
	float curtain_setpoint_temp;
	float curtain_idleband_degrees;
#define CURTAIN_CYCLESECS 300
#define CURTAIN_ONSECS 60
	float heatpad_setpoint_temp;
	float heatpad_idleband_degrees;
	float stirfan_setforward_degrees;
#define stirfan_idleband_degrees 4.0
	float spaceheater_setback_degrees;
	float spaceheater_idleband_degrees;
#define EVACFAN_CYCLESECS 30
	float  evacfan_setback_degrees;
	float  evacfan_df_min;
  float  evacfan_df_max;
#define FOGGER_CYCLESECS 300
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

#define MOTECMD_CONFIGURE 0xCF
#define MOTECMD_TESTTEMP  0xC7
#define MOTECMD_RESET     0xC5

const byte numBytes = 32;
byte receivedBytes[numBytes];
byte numReceived = 0;

boolean newData = false;

struct __attribute__((__packed__)) rs485packet {
  uint8_t  command;
  uint16_t switches;
};

uint16_t lastSwitches;

typedef struct rs485packet Rs485Packet;
Rs485Packet rPacket;
byte rsbytes[sizeof(rPacket) + 3];

#define START_MARKER      (uint8_t) 0x3C
#define END_MARKER        (uint8_t) 0x3E

#define CMD_SENDSWITCHES  (uint8_t) 0xD3
#define ACK_SENDSWITCHES  (uint8_t) 0xD4

#define _PROCESS_TIME_MILLIS 1000
long upTimeSecs;
long lastProcessMillis;

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
  cfg.evacfan_setback_degrees = 20.0;       //p[7]
	cfg.evacfan_df_min = 20.0;                //p[8]
  cfg.evacfan_df_max = 100.0;               //p[9]
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

void scheduleNextRS485Send(void);			//predeclared here
void scheduleNextLoRaSend(void);			//predeclared here

uint8_t calculateChecksum(byte* b, uint16_t size) {
  uint16_t cksum = 0;
  for (uint8_t n=0; n<size; n++) {
    cksum += b[n];
  }
  return (cksum%256);
}
/*
 * calculate checksum, add start and end-markers and send it
*/
void sendRS485(osjob_t* j) {
  rsbytes[0] = START_MARKER;
  memcpy(rsbytes+1,&rPacket,sizeof(rPacket));
  rsbytes[sizeof(rPacket)+1] = calculateChecksum(rsbytes+1,sizeof(rPacket));
  rsbytes[sizeof(rPacket)+2] = END_MARKER;
  #if defined(RS485_MESSAGES)
  SerialDebug.print("sendRS485: ");
  for (uint8_t n=0; n<sizeof(rPacket)+3; n++) {
    if (rsbytes[n]<16)
      SerialDebug.print("0");
    SerialDebug.print(rsbytes[n],HEX);
  }
  SerialDebug.println();
  #endif
  digitalWrite(RS485_DIR, HIGH);        //transmit
  delay(100);
  rs485Serial.write(rsbytes, sizeof(rPacket)+3);
  delay(100);  //rs485Serial.flush();                 //block until buffer empty
  digitalWrite(RS485_DIR, LOW);        //receive
  //scheduleNextLoRaSend();
}

void buildMoteRec() {
  moteRec.heatpad_temp = getHeatPadTemp();
  moteRec.room_temp = getRoomTemp();
  moteRec.evac_pct = evacfanController.getDutyFactor();
  moteRec.fogg_pct = foggerController.getDutyFactor();
  memcpy(moteRec.config, &cfg, sizeof(cfg));
}

void recvBytesWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;    
    byte rb;   

    while (rs485Serial.available() > 0 && newData == false) {
        rb = rs485Serial.read();
        /*
        if (rb<16)
          Serial.print("0");
        Serial.print(rb, HEX);
        */
        if (recvInProgress == true) {
            if (rb != END_MARKER) {
                receivedBytes[ndx] = rb;
                ndx++;
                if (ndx >= numBytes) {
                    ndx = numBytes - 1;
                }
            }
            else {
                receivedBytes[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                numReceived = ndx;  // save the number for use when printing
                ndx = 0;
                newData = true;
            }
        }

        else if (rb == START_MARKER) {
            recvInProgress = true;
        }
    }
}

void parseSerialData() {
    if (newData == true) {
        if (receivedBytes[sizeof(rPacket)] == calculateChecksum(receivedBytes,sizeof(rPacket))) {
          memcpy(&rPacket, receivedBytes, sizeof(rPacket));
          switch(rPacket.command) {
            case ACK_SENDSWITCHES: {
               #if defined(RS485_MESSAGES)
               SerialDebug.println("RS485 ACK_SENDSWITCHES received");
               #endif          
              lastSwitches = moteRec.switches;
              break;
            } 
            default: SerialDebug.println("Unrecognized command."); break;
          }
        } else {
          #if defined(RS485_MESSAGES)
          SerialDebug.print("RS485 cksum bad "); SerialDebug.println(calculateChecksum(receivedBytes,sizeof(rPacket)), HEX);
          #endif
        }
        newData = false;
    }
}

void processLoop() {
  SerialDebug.print(".");
  moteRec.switches = 0x0000;
  float padTempF = moteRec.heatpad_temp; //getHeatPadTemp();
  float roomTempF = moteRec.room_temp;   //getRoomTemp();

  moteRec.switches = moteRec.switches | heatPadController.poll(padTempF);      
  moteRec.switches = moteRec.switches | spaceHeaterController.poll(roomTempF);  
  moteRec.switches = moteRec.switches | curtainController.poll(roomTempF);
  moteRec.switches = moteRec.switches | stirFanController.poll(roomTempF);  
  moteRec.switches = moteRec.switches | evacfanController.poll(roomTempF);  
  moteRec.switches = moteRec.switches | foggerController.poll(roomTempF);

  #if defined(DEBUG_PROCESS_MESSAGES)
  char s[64];
  sprintf(s,"switches=0x%X lastSwitches=0x%X", moteRec.switches, lastSwitches);
  SerialDebug.println(s);
  #endif

  if (moteRec.switches != lastSwitches) {
    rPacket.command = CMD_SENDSWITCHES;
    rPacket.switches = moteRec.switches;
    os_setCallback(&sendRS485Job, sendRS485);
    #if defined(DEBUG_PROCESSLOOP_MESSAGES)
    SerialDebug.println("RS485 send queued");
    #endif
  }   
}

void testRS485(osjob_t* j) {
  static uint16_t switches = 0x8000;
  rPacket.command = 0xD3;
  rPacket.switches = switches;
  switches = switches >> 1;
  if (switches == 0x0000)
    switches = 0x8000;
  os_setCallback(&sendRS485Job, sendRS485);
}

/*
 * do_send() is called periodically to send sensor data
 */
void do_send(osjob_t* j) {
  if (LMIC.opmode & OP_TXRXPEND) {    
	  // Check if there is a current TX/RX job running
    } else {
      #if defined(DEBUG_LORA_SEND)
      SerialDebug.println("do_send");
      #endif
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
      
      scheduleNextLoRaSend();
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
void loraSendAlarmMatch()  {
  //the following call attempts to avoid collisions with other motes by randomizing the send time
  os_setTimedCallback(&sendLoRaJob, os_getTime()+ms2osticks(random(200)), do_send);
  frameCount++;
  //scheduleNextLoRaSend();
}

void scheduleNextLoRaSend() {
  // set alarm for LORA_TX_INTERVAL_SECONDS from now
  rtc.setAlarmEpoch( rtc.getEpoch() + LORA_TX_INTERVAL_SECONDS);
  rtc.enableAlarm(rtc.MATCH_YYMMDDHHMMSS);
  rtc.attachInterrupt(loraSendAlarmMatch);
}

void onEvent (ev_t ev) {
  #ifdef DEBUG_LORA_EVENTS
  SerialDebug.print(os_getTime());
  SerialDebug.print(": ["); SerialDebug.print(ev); SerialDebug.print("] ");
  #endif

  switch(ev) {
    case EV_JOINING:
      #ifdef _DEBUG_SERIAL
      SerialDebug.println(F("JOINING"));
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
      SerialDebug.println(F("JOINED"));
      /*
      SerialDebug.print("netid: ");
      SerialDebug.println(netid, DEC);
      SerialDebug.print("devaddr: "); // @suppress("Method cannot be resolved")
      SerialDebug.println(devaddr, HEX);
      SerialDebug.print("artKey: ");
      for (int i=0; i<sizeof(artKey); ++i) {
        if (i != 0)
          SerialDebug.print("-");
        SerialDebug.print(artKey[i], HEX);
      }
      SerialDebug.println("");
      SerialDebug.print("nwkKey: ");
      for (int i=0; i<sizeof(nwkKey); ++i) {
        if (i != 0)
          SerialDebug.print("-");
        SerialDebug.print(nwkKey[i], HEX);
      }
      SerialDebug.println("");
      */
      #endif
      LMIC_setLinkCheckMode(0);
      break;
    }
    
    case EV_TXCOMPLETE:
      digitalWrite(LED_BUILTIN, LOW);     // LED Off
      #ifdef _DEBUG_SERIAL
      //SerialDebug.println(F("TXCOMPLETE"));
      #endif
      if ((ackRequested) && (LMIC.txrxFlags & TXRX_ACK)) {
        // if confirmation was requested and is received - cycle the WATCHDOG bit
        ackReceived = true;
        #if defined(DEBUG_MESSAGES)
        SerialDebug.println(F("LoRa Ack"));
        #endif
      }
      // Check if we have a downlink on either Rx1 or Rx2 windows
      if ((LMIC.txrxFlags & ( TXRX_DNW1 | TXRX_DNW2 )) != 0) {
        #ifdef DEBUG_LORA_DOWNLINK
    	  if ((LMIC.txrxFlags & TXRX_DNW1) != 0)
       	  SerialDebug.print(F("DRx1-"));
       	else
       	  SerialDebug.print(F("DRx2-"));
        #endif

       	if (LMIC.dataLen) {
                   
       	  #ifdef DEBUG_LORA_DOWNLINK
       	  SerialDebug.print(LMIC.dataLen);
       	  // Receive bytes in LMIC.frame are the B64 decode of the
       	  // 'data' field sent by ChirpStack
          
       	  for (int i = 0; i < LMIC.dataLen; i++) {
            if (LMIC.frame[LMIC.dataBeg + i]<16)
              SerialDebug.print("0");
       	    SerialDebug.print(LMIC.frame[LMIC.dataBeg + i], HEX);
       	  }
       	  SerialDebug.println();
       	  #endif
          uint8_t cksum = 0;
          for (int i = 2; i < LMIC.dataLen; i++) {
            cksum += LMIC.frame[LMIC.dataBeg + i];
          }
          if (cksum == LMIC.frame[LMIC.dataBeg + 1]) {
            if (LMIC.frame[LMIC.dataBeg]==0xCF) {
              #if defined(DEBUG_LORA_DOWNLINK_COMMANDS)
              SerialDebug.println("Charlie Foxtrot - configure");
              #endif
              uint16_t config_id = cfg.configuration_id;
              initialize(LMIC.frame + LMIC.dataBeg + 2);
              cfg.configuration_id = config_id;
              storeConfig();
              reboot();
            } else if (LMIC.frame[LMIC.dataBeg]==0xC7) {
              #if defined(DEBUG_LORA_DOWNLINK_COMMANDS) 
              SerialDebug.println("Charlie Seven - set test temperature");
              #endif
              memcpy(&test, LMIC.frame + LMIC.dataBeg + 2, sizeof(test));
              useTestTemperature = true;
              moteRec.mode = MODE_TEST;
              evacfanController.reset();
              foggerController.reset();
              curtainController.reset();
            } else if (LMIC.frame[LMIC.dataBeg]==0xC5) {
              #if defined(DEBUG_LORA_DOWNLINK_COMMANDS) 
              SerialDebug.println("Charlie Five - reset");
              #endif
              reboot();
            }
          } else {
            #if defined(DEBUG_LORA_DOWNLINK)
              char s[64];
              sprintf(s,"cksum calc=%X actual=%X", cksum, LMIC.frame[LMIC.dataBeg + 1]);
              SerialDebug.println(s);
              for (int i = 0; i < LMIC.dataLen; i++) {
              if (LMIC.frame[LMIC.dataBeg + i]<16)
                SerialDebug.print("0");
       	      SerialDebug.print(LMIC.frame[LMIC.dataBeg + i], HEX);
       	    }
       	    SerialDebug.println();
            #endif            
          }          
       	}
      }
      break;
    #if defined(DEBUG_LORA_EVENTS)
    case EV_SCAN_TIMEOUT:
      #ifdef _DEBUG_SERIAL
      SerialDebug.println(F("SCAN_TIMEOUT"));
      #endif
      break;
    case EV_BEACON_FOUND:
      #ifdef _DEBUG_SERIAL
      SerialDebug.println(F("BEACON_FOUND"));
      #endif
      break;
    case EV_BEACON_MISSED:
      #ifdef _DEBUG_SERIAL
      SerialDebug.println(F("BEACON_MISSED"));
      #endif
      break;
    case EV_BEACON_TRACKED:
      #ifdef _DEBUG_SERIAL
      SerialDebug.println(F("BEACON_TRACKED"));
      #endif
      break;
    case EV_JOIN_FAILED:
      #ifdef _DEBUG_SERIAL
      SerialDebug.println(F("JOIN_FAIL"));
      #endif
      loraJoined = false;
      scheduleNextLoRaSend(); //goToSleep();
      break;
    case EV_REJOIN_FAILED:
      #ifdef _DEBUG_SERIAL
      SerialDebug.println(F("REJOIN_FAIL"));
      #endif
      loraJoined = false;
      scheduleNextLoRaSend(); //goToSleep();
      break;
    case EV_LOST_TSYNC:
      SerialDebug.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      SerialDebug.println(F("EV_RESET"));
      loraJoined = false;
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      SerialDebug.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      //no confirmation has been received for an extended perion
      SerialDebug.println(F("EV_LINK_DEAD"));
      loraJoined = false;
      scheduleNextLoRaSend(); //goToSleep();
      break;
    case EV_LINK_ALIVE:
      SerialDebug.println(F("EV_LINK_ALIVE"));
      break;
    case EV_TXSTART:
      SerialDebug.println(F("EV_TXSTART"));
      //digitalWrite(LED_BUILTIN, LOW);
      break;
    default:
      SerialDebug.print(F("Unknown event: "));
      SerialDebug.println((unsigned) ev);
      break;
    #endif  
  }
}

void setup() {
  // Resets occurring ~immediately after a successful SEND #10, is this the Brown Out Detector firing?
  // Configure the regulator to run in normal mode when in standby mode
  // Otherwise it defaults to low power mode and can only supply 50 uA
  SYSCTRL->VREG.bit.RUNSTDBY = 1;

  pinMode(_WATCHDOG_DONE, OUTPUT);					 // Watchdog Done pin
  digitalWrite(_WATCHDOG_DONE, LOW);

  pinMode(RS485_DIR, OUTPUT);
  digitalWrite(RS485_DIR, LOW);
  
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
  {0.0,cfg.evacfan_df_min,0.0},
  {(cfg.curtain_setpoint_temp-cfg.evacfan_setback_degrees),0.0,((cfg.evacfan_df_max-cfg.evacfan_df_min)/ (cfg.evacfan_setback_degrees-(cfg.curtain_idleband_degrees/2.0))) },
  {(cfg.curtain_setpoint_temp-(cfg.curtain_idleband_degrees/2.0)),0.0,((cfg.evacfan_df_min-cfg.evacfan_df_max)/ (cfg.evacfan_setback_degrees-(cfg.curtain_idleband_degrees/2.0))) },
  {(cfg.curtain_setpoint_temp+(cfg.curtain_idleband_degrees/2.0)),-cfg.evacfan_df_max,0.0}
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
  curtainController.set(CURTAIN_CYCLESECS,
		                CURTAIN_ONSECS,
						        cfg.curtain_setpoint_temp,
						        cfg.curtain_idleband_degrees);
  curtainController.dumpConfig();
 
  evacfanController.setDeviceName("evac");
  evacfanController.setOutput(EVAC_OUTPUT);
  evacfanController.setCycleTimeSecs(EVACFAN_CYCLESECS);
  evacfanController.setFunction(evacfunc);

  foggerController.setDeviceName("fogger");
  foggerController.setOutput(FOGGER_OUTPUT);
  foggerController.setCycleTimeSecs(FOGGER_CYCLESECS);
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

  frameCount=0;
  rs485Serial.begin(9600);
  digitalWrite(RS485_DIR, HIGH);
  // Assign pins 10 & 11 SERCOM functionality
  //pinPeripheral(10, PIO_SERCOM);
  //pinPeripheral(11, PIO_SERCOM);

  lastSwitches = 0xFFFF;

  moteRec.mode = MODE_NORMAL;
  loraJoined = true;
  do_send(&sendLoRaJob);
  lastProcessMillis = millis(); // + 5 * _PROCESS_TIME_MILLIS;

}

uint16_t switches = 0x8000;

void loop() {
  os_runloop_once();
  recvBytesWithStartEndMarkers();
  parseSerialData();
  if (loraJoined && ( (millis() - lastProcessMillis) > _PROCESS_TIME_MILLIS )) {
  	processLoop();  
    //os_setCallback(&processJob, processLoop);  
  	sitUbu();
  	lastProcessMillis = millis();
  }
}
