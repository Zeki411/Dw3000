#include "dw3000.h"
#include "DW3000Ranging.h"

DW3000RangingClass dw3000ranging;

uint8_t DW3000RangingClass::_RST;
uint8_t DW3000RangingClass::_SS;

uint16_t DW3000RangingClass::_txAntennaDelay = TX_ANT_DLY;
uint16_t DW3000RangingClass::_rxAntennaDelay = RX_ANT_DLY;

uint32_t DW3000RangingClass::_rangingCountPeriod = 0;

extern dwt_txconfig_t txconfig_options;

static uint8_t nibbleFromChar(char c);
static void convertToByte(char string[], byte* bytes);



void IRAM_ATTR DW3000RangingClass::handleInterrupt() {

    // uint32_t status_reg = dwt_read32bitreg(SYS_STATUS_ID);

    // /* Handle Events */

    // //check if TX done
    // if (status_reg & SYS_STATUS_TXFRS_BIT_MASK) {
    //     // Clear TXFRS event
    //     dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
    // }

    //check if RX done

}

void DW3000RangingClass::initCommunication(uint8_t myRST, uint8_t mySS, uint8_t myIRQ) {
    // Set pin values
    _RST = myRST;
    _SS = mySS;

    // Set the interrupt handler for DW IC IRQ
    port_set_dwic_isr(&handleInterrupt);
    // Initialize SPI
    spiBegin(myIRQ, _RST);
    spiSelect(_SS);

    delay(200); // Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC, or could wait for SPIRDY event)

    while (!dwt_checkidlerc()) // Need to make sure DW IC is in IDLE_RC before proceeding 
    {
        // ESP_LOGE(TAG_TWR_LOG_TAG, "IDLE FAILED");
        while (1) ;
    }

    dwt_softreset();
    delay(200);

    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
    {
        // ESP_LOGE(TAG_TWR_LOG_TAG, "INIT FAILED");
        while (1) ;
    }
}

void DW3000RangingClass::configureNetwork(uint16_t deviceAddress, uint16_t networkId, dwt_config_t config) {
    dwt_setdwstate(DWT_DW_IDLE); // Set IDLE state

    if(dwt_configure(&config)) // if the dwt_configure returns DWT_ERROR either the PLL or RX calibration has failed the host should reset the device
    {
        // ESP_LOGE(TAG_TWR_LOG_TAG, "CONFIG FAILED");
        while (1) ;
    }

    /* Set expected response's delay and timeout.
     * As this example only handles one incoming frame with always the same delay and timeout, those values can be set here once for all. */
    // dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    // dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);

    // Set device address
    dwt_setpanid(networkId);
    dwt_setaddress16(deviceAddress);

    /* Configure the TX spectrum parameters (power, PG delay and PG count) */
    dwt_configuretxrf(&txconfig_options);


    /* Apply default antenna delay value. */
    // dwt_setrxantennadelay(RX_ANT_DLY);
    // dwt_settxantennadelay(TX_ANT_DLY);
}

void DW3000RangingClass::generalStart() {


    // for first time ranging frequency computation
	_rangingCountPeriod = millis();
}

void DW3000RangingClass::startAsTag(char address[], dwt_config_t config, const bool randomShortAddress) {
    convertToByte(address, _currentAddress); // convert address string to byte array

    //set EUI
    dwt_seteui(_currentAddress);

    //TODO: print EUI

    //set short address
	if (randomShortAddress) {
		//we need to define a random short address:
		randomSeed(analogRead(0));
		_currentShortAddress[0] = random(0, 256);
		_currentShortAddress[1] = random(0, 256);
	}
	else {
		// we use first two bytes in addess for short address
		_currentShortAddress[0] = _currentAddress[0];
		_currentShortAddress[1] = _currentAddress[1];
	}

    configureNetwork(_currentShortAddress[0]*256+_currentShortAddress[1], 0xDECA, config);

    generalStart();

    _type = TAG;
}

void DW3000RangingClass::startAsAnchor(char address[], dwt_config_t config, const bool randomShortAddress) {
    convertToByte(address, _currentAddress); // convert address string to byte array

    //set EUI
    dwt_seteui(_currentAddress);

    //set short address
	if (randomShortAddress) {
		//we need to define a random short address:
		randomSeed(analogRead(0));
		_currentShortAddress[0] = random(0, 256);
		_currentShortAddress[1] = random(0, 256);
	}
	else {
		// we use first two bytes in addess for short address
		_currentShortAddress[0] = _currentAddress[0];
		_currentShortAddress[1] = _currentAddress[1];
	}

    configureNetwork(_currentShortAddress[0]*256+_currentShortAddress[1], 0xDECA, config);

    generalStart();

    _type = ANCHOR;
}

void DW3000RangingClass::loop() {

}



void DW3000RangingClass::setTxAntennaDelay(uint16_t txAntennaDelay) {
    _txAntennaDelay = txAntennaDelay;
}

void DW3000RangingClass::setRxAntennaDelay(uint16_t rxAntennaDelay) {
    _rxAntennaDelay = rxAntennaDelay;
}


static uint8_t nibbleFromChar(char c) {
	if(c >= '0' && c <= '9') {
		return c-'0';
	}
	if(c >= 'a' && c <= 'f') {
		return c-'a'+10;
	}
	if(c >= 'A' && c <= 'F') {
		return c-'A'+10;
	}
	return 255;
}

static void convertToByte(char string[], byte* bytes) {
	uint8_t eui_byte[8]; // LEN_EUI
	// we fill it with the char array under the form of "AA:FF:1C:...."
	for(uint16_t i = 0; i < 8; i++) {
		eui_byte[i] = (nibbleFromChar(string[i*3]) << 4)+nibbleFromChar(string[i*3+1]);
	}
	memcpy(bytes, eui_byte, 8);
}

