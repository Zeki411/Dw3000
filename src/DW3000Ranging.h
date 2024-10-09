#ifndef __DW3000RANGING_H__
#define __DW3000RANGING_H__

#include <Arduino.h>
#include <dw3000.h>
#include "DW3000Device.h"
#include "DW3000Mac.h"

#define RANGING_DEBUG 0

#define DEFAULT_RST_PIN 27
#define DEFAULT_SPI_SS_PIN 4
#define DEFAULT_IRQ_PIN 34

/* Delay between frames, in UWB microseconds. */
#define POLL_TX_TO_RESP_RX_DLY_UUS 100
#define RESP_RX_TIMEOUT_UUS 500
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385

#define MAX_COUNT_TO_BLINK 1000

// messages used in the ranging protocol
#define TWR_POLL 0
#define TWR_POLL_ACK 1
#define TWR_RANGE 2
#define TWR_RANGE_REPORT 3
#define TWR_RANGE_FAILED 255
#define TWR_BLINK 4
#define TWR_RANGING_INIT 5

#define TWR_LEN_DATA 90

#define TWR_MAX_DEVICES 10

//Default value
//in ms
#define TWR_DEFAULT_RESET_PERIOD 200
//in us
#define TWR_DEFAULT_REPLY_DELAY_TIME 7000

//default timer delay
#define TWR_DEFAULT_TIMER_DELAY 80

//sketch type (anchor or tag)
#define TWR_TAG 0
#define TWR_ANCHOR 1

class DW3000RangingClass {
public:

    static uint8_t s_dataBuffer[TWR_LEN_DATA];
    // DW3000Ranging();
    // ~DW3000Ranging();

    static void     initCommunication(uint8_t myRST = DEFAULT_RST_PIN, uint8_t mySS = DEFAULT_SPI_SS_PIN, uint8_t myIRQ = DEFAULT_IRQ_PIN);
    static void     configureNetwork(uint16_t deviceAddress, uint16_t networkId, dwt_config_t config);
    static void     generalStart();
    static void     startAsTag(char t_address[], uint16_t t_networkId, dwt_config_t t_config, const bool t_randomShortAddress = false);
    static void     startAsAnchor(char t_address[], uint16_t t_networkId, dwt_config_t t_config, const bool t_randomShortAddress = false);

    static int16_t  detectMessageType(uint8_t t_data[]);

    static boolean  addNetworkDevices(DW3000Device* device);
    static boolean  addNetworkDevices(DW3000Device* t_device, boolean t_shortAddress);

    static void     removeNetworkDevices(int16_t index);

    // ranging functions
    static void loop();

    // dw3000 additional functions
    static void     setTxAntennaDelay(uint16_t txAntennaDelay);
    static void     setRxAntennaDelay(uint16_t rxAntennaDelay);

    // Handler
    static void attachNewRangeEventHandler(void (* t_handleNewRangeEvent)(void)) { s_handleNewRangeEvent = t_handleNewRangeEvent; };
    static void attachNewDeviceEventHandler(void (* t_handleNewDeviceEvent)(DW3000Device*)) { s_handleNewDeviceEvent = t_handleNewDeviceEvent; };
    static void attachInactiveDeviceEventHandler(void (* t_handleInactiveDeviceEvent)(DW3000Device*)) { s_handleInactiveDeviceEvent = t_handleInactiveDeviceEvent; };

private:
    //other devices in the network
    static DW3000Device s_networkDevices[TWR_MAX_DEVICES];
    static uint8_t s_currentAddress[8];
    static uint8_t s_currentShortAddress[2];
    static volatile uint8_t DRAM_ATTR s_networkDevicesNumber;
	static int16_t  s_lastDistantDevice;
	static uint8_t  s_lastSentToShortAddress[2];
	static DW3000Mac s_globalMac;
    // timer and counter for blink
    static int32_t  s_lastTickTime;
	static uint16_t  s_counterForBlink;

    // handlers
    static void (* s_handleNewRangeEvent)(void);
    static void (* s_handleNewDeviceEvent)(DW3000Device* device);
    static void (* s_handleInactiveDeviceEvent)(DW3000Device* device);


	//sketch type (tag or anchor)
	static uint8_t s_type; //0 for tag and 1 for anchor

	// message flow state
	static volatile uint8_t DRAM_ATTR s_expectedMsgId;
	// message sent/received state
	static volatile boolean DRAM_ATTR s_txDone;
	static volatile boolean DRAM_ATTR s_rxDone;
    
    // reset line to the chip
    static uint8_t _RST; // Reset pin
    static uint8_t _SS; // Slave Select pin

	// watchdog and reset period
	static uint32_t s_lastActivity;
	static uint32_t s_resetPeriod;
	// reply times (same on both sides for symm. ranging)
	static uint16_t s_replyDelayTimeUS;

	// timer Tick delay
	static uint16_t s_timerDelay;

    static uint16_t _txAntennaDelay;
    static uint16_t _rxAntennaDelay;

    /* Arduino interrupt handler */
	static void IRAM_ATTR handleInterrupt();

    // ranging counter (per second)
    static uint32_t s_rangingCountPeriod;


    // methods
    static void handleTxDoneEvent();
	static void handleRxDoneEvent();
	static void noteActivity();
    static void resetInactive();

    // global functions
    static void checkForReset();
    static void checkForInactiveDevices();
    static void copyShortAddress(uint8_t ts_destAddr[], uint8_t t_srcAddr[]);

    //for ranging protocole (ANCHOR)
    static void startReceive();

    static void initTransmit();
    static void transmitBlink();
    static void transmitRangingInit(DW3000Device* t_networkDevices);


    static void tickTimer();

};

extern DW3000RangingClass dw3000ranging;

#endif // __DW3000RANGING_H__