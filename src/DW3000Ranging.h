#ifndef __DW3000RANGING_H__
#define __DW3000RANGING_H__

#include <Arduino.h>
#include <dw3000.h>

#define RANGING_DEBUG 0

#define DEFAULT_RST_PIN 27
#define DEFAULT_SPI_SS_PIN 4
#define DEFAULT_IRQ_PIN 34

/* Delay between frames, in UWB microseconds. */
#define POLL_TX_TO_RESP_RX_DLY_UUS 100
#define RESP_RX_TIMEOUT_UUS 500
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385

//sketch type (anchor or tag)
#define TAG 0
#define ANCHOR 1

class DW3000RangingClass {
public:
    // DW3000Ranging();
    // ~DW3000Ranging();

    static void     initCommunication(uint8_t myRST = DEFAULT_RST_PIN, uint8_t mySS = DEFAULT_SPI_SS_PIN, uint8_t myIRQ = DEFAULT_IRQ_PIN);
    static void     configureNetwork(uint16_t deviceAddress, uint16_t networkId, dwt_config_t config);
    static void     generalStart();
    static void     startAsTag(char address[], dwt_config_t config, const bool randomShortAddress = true);
    static void     startAsAnchor(char address[], dwt_config_t config, const bool randomShortAddress = true);

    // ranging functions
    static void loop();

    // dw3000 additional functions
    static void     setTxAntennaDelay(uint16_t txAntennaDelay);
    static void     setRxAntennaDelay(uint16_t rxAntennaDelay);




private:
    //other devices in the network
    static uint8_t _currentAddress[8];
    static uint8_t _currentShortAddress[2];


	//sketch type (tag or anchor)
	static int16_t  _type; //0 for tag and 1 for anchor

    
    static uint8_t _RST; // Reset pin
    static uint8_t _SS; // Slave Select pin

    static uint16_t _txAntennaDelay;
    static uint16_t _rxAntennaDelay;

    /* Arduino interrupt handler */
	static void IRAM_ATTR handleInterrupt();

    // ranging counter (per second)
    static uint32_t _rangingCountPeriod;

  
};

extern DW3000RangingClass dw3000ranging;

#endif // __DW3000RANGING_H__