#include <esp_log.h>
#include <esp_intr_alloc.h>

#include "dw3000.h"
#include "DW3000Ranging.h"

#define DW3000RANGING_LOG_TAG "DW3000RANGING"
#define DW3000RANGING_LOG_LEVEL ESP_LOG_INFO

DW3000RangingClass dw3000ranging;

DW3000Device DW3000RangingClass::s_networkDevices[TWR_MAX_DEVICES];
uint8_t DW3000RangingClass::s_currentAddress[8];
uint8_t DW3000RangingClass::s_currentShortAddress[2];
volatile uint8_t DRAM_ATTR DW3000RangingClass::s_networkDevicesNumber = 0; // TODO short, 8bit?
int16_t DW3000RangingClass::s_lastDistantDevice = 0;
uint8_t DW3000RangingClass::s_lastSentToShortAddress[2];

DW3000Mac DW3000RangingClass::s_globalMac;

uint8_t DW3000RangingClass::s_dataBuffer[TWR_LEN_DATA];

//module type (anchor or tag)
uint8_t DW3000RangingClass::s_type;

// message flow state
volatile uint8_t DRAM_ATTR DW3000RangingClass::s_expectedMsgId;

// message sent/received state
volatile boolean DRAM_ATTR DW3000RangingClass::s_txDone;
volatile boolean DRAM_ATTR DW3000RangingClass::s_rxDone;

// timestamps to remember
int32_t DW3000RangingClass::s_lastTickTime    = 0;
uint16_t DW3000RangingClass::s_counterForBlink = 0; // TODO 8 bit?
// reset line to the chip
uint8_t DW3000RangingClass::_RST;
uint8_t DW3000RangingClass::_SS;
// watchdog and reset period
uint32_t  DW3000RangingClass::s_lastActivity;
uint32_t  DW3000RangingClass::s_resetPeriod;
// reply times (same on both sides for symm. ranging)
uint16_t  DW3000RangingClass::s_replyDelayTimeUS;
//timer delay
uint16_t  DW3000RangingClass::s_timerDelay;

uint16_t DW3000RangingClass::_txAntennaDelay = TX_ANT_DLY;
uint16_t DW3000RangingClass::_rxAntennaDelay = RX_ANT_DLY;

uint32_t DW3000RangingClass::s_rangingCountPeriod = 0;

extern dwt_txconfig_t txconfig_options;

// Here event handlers
void (* DW3000RangingClass::s_handleNewRangeEvent)(void) = NULL;
void (* DW3000RangingClass::s_handleNewDeviceEvent)(DW3000Device* device) = NULL;
void (* DW3000RangingClass::s_handleInactiveDeviceEvent)(DW3000Device* device) = NULL;

boolean DRAM_ATTR interrupt_raised = false;

static uint8_t nibbleFromChar(char c);
static void convertToByte(char string[], byte* bytes);

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

void IRAM_ATTR DW3000RangingClass::handleInterrupt() {

    interrupt_raised = true;

    // uint32_t status_reg = dwt_read32bitreg(SYS_STATUS_ID);

    /* Handle Events */

    // check if TX done
    // if (status_reg & SYS_STATUS_TXFRS_BIT_MASK) {
    //     // Clear TXFRS event
    //     dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
        
    //     // Handle TX done event
    //     handleTxDoneEvent();
    // }

    // //check if RX done
    // if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) {
    //     // Clear RXFCG event
    //     dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

    //     // Handle RX done event
    //     handleRxDoneEvent();
    // }

}

void DW3000RangingClass::initCommunication(uint8_t myRST, uint8_t mySS, uint8_t myIRQ) {
    // Set pin values
    _RST = myRST;
    _SS = mySS;

    s_resetPeriod      = TWR_DEFAULT_RESET_PERIOD;
	// reply times (same on both sides for symm. ranging)
	s_replyDelayTimeUS = TWR_DEFAULT_REPLY_DELAY_TIME;
	//we set our timer delay
	s_timerDelay       = TWR_DEFAULT_TIMER_DELAY;

    esp_log_level_set(DW3000RANGING_LOG_TAG, DW3000RANGING_LOG_LEVEL);

    // Set the interrupt handler for DW IC IRQ
    port_set_dwic_isr(&handleInterrupt);
    // Initialize SPI
    spiBegin(myIRQ, _RST);
    spiSelect(_SS);

    delay(200); // Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC, or could wait for SPIRDY event)

    while (!dwt_checkidlerc()) // Need to make sure DW IC is in IDLE_RC before proceeding 
    {
        ESP_LOGE(DW3000RANGING_LOG_TAG, "IDLE FAILED");
        while (1) ;
    }

    dwt_softreset();
    delay(200);

    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
    {
        ESP_LOGE(DW3000RANGING_LOG_TAG, "INIT FAILED");
        while (1) ;
    }
}

void DW3000RangingClass::configureNetwork(uint16_t deviceAddress, uint16_t networkId, dwt_config_t config) {
    // dwt_setdwstate(DWT_DW_IDLE); // Set IDLE state

    if(dwt_configure(&config)) // if the dwt_configure returns DWT_ERROR either the PLL or RX calibration has failed the host should reset the device
    {
        ESP_LOGE(DW3000RANGING_LOG_TAG, "CONFIG FAILED");
        while (1) ;
    }

    // Set device address
    dwt_setpanid(networkId);
    dwt_setaddress16(deviceAddress);
}

void DW3000RangingClass::generalStart() {

    /* Configure the TX spectrum parameters (power, PG delay and PG count) */
    dwt_configuretxrf(&txconfig_options);

    /* Apply default antenna delay value. */
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);
    
    // for first time ranging frequency computation
	s_rangingCountPeriod = millis();
}

void DW3000RangingClass::startAsTag(char t_address[], uint16_t t_networkId, dwt_config_t t_config, const bool t_randomShortAddress) {
    convertToByte(t_address, s_currentAddress); // convert address string to byte array

    //set EUI
    dwt_seteui(s_currentAddress);

    //TODO: print EUI

    //set short address
	if (t_randomShortAddress) {
		//we need to define a random short address:
		randomSeed(analogRead(0)); // TODO:
		s_currentShortAddress[0] = random(0, 256);
		s_currentShortAddress[1] = random(0, 256);
	}
	else {
		// we use first two bytes in addess for short address
		s_currentShortAddress[0] = s_currentAddress[0];
		s_currentShortAddress[1] = s_currentAddress[1];
	}

    configureNetwork(s_currentShortAddress[0]*256+s_currentShortAddress[1], t_networkId, t_config);

    generalStart();

    /* Set expected response's delay and timeout.
     * As this example only handles one incoming frame with always the same delay and timeout, those values can be set here once for all. */
    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);

    /* Configure the IRQ events */
    dwt_setinterrupt(\
        (uint32_t) (
                // RX events
                SYS_ENABLE_LO_RXFCG_ENABLE_BIT_MASK | SYS_ENABLE_LO_RXFTO_ENABLE_BIT_MASK | SYS_ENABLE_LO_RXPTO_ENABLE_BIT_MASK |\
                SYS_ENABLE_LO_RXPHE_ENABLE_BIT_MASK | SYS_ENABLE_LO_RXFCE_ENABLE_BIT_MASK | SYS_ENABLE_LO_RXFSL_ENABLE_BIT_MASK |\
                SYS_ENABLE_LO_RXSTO_ENABLE_BIT_MASK |\
                // TX events
                SYS_ENABLE_LO_TXFRS_ENABLE_BIT_MASK |\
                // other events
                SYS_ENABLE_LO_ARFE_ENABLE_BIT_MASK | SYS_ENABLE_LO_CIAERR_ENABLE_BIT_MASK\
            ),\
        (uint32_t) (0x00),\
        DWT_ENABLE_INT_ONLY
    );

    s_type = TWR_TAG;
}

void DW3000RangingClass::startAsAnchor(char t_address[], uint16_t t_networkId, dwt_config_t t_config, const bool t_randomShortAddress) {
    convertToByte(t_address, s_currentAddress); // convert address string to byte array

    // set EUI
    dwt_seteui(s_currentAddress);

    // set short address
	if (t_randomShortAddress) {
		//we need to define a random short address:
		randomSeed(analogRead(0)); //TODO
		s_currentShortAddress[0] = random(0, 256);
		s_currentShortAddress[1] = random(0, 256);
	}
	else {
		// we use first two bytes in addess for short address
		s_currentShortAddress[0] = s_currentAddress[0];
		s_currentShortAddress[1] = s_currentAddress[1];
	}

    configureNetwork(s_currentShortAddress[0]*256+s_currentShortAddress[1], t_networkId, t_config);

    generalStart();

    /* Configure the IRQ events */
    dwt_setinterrupt(\
        (uint32_t) (
                // RX events
                SYS_ENABLE_LO_RXFCG_ENABLE_BIT_MASK | SYS_ENABLE_LO_RXFTO_ENABLE_BIT_MASK | SYS_ENABLE_LO_RXPTO_ENABLE_BIT_MASK |\
                SYS_ENABLE_LO_RXPHE_ENABLE_BIT_MASK | SYS_ENABLE_LO_RXFCE_ENABLE_BIT_MASK | SYS_ENABLE_LO_RXFSL_ENABLE_BIT_MASK |\
                SYS_ENABLE_LO_RXSTO_ENABLE_BIT_MASK |\
                // TX events
                SYS_ENABLE_LO_TXFRS_ENABLE_BIT_MASK |\
                // other events
                SYS_ENABLE_LO_ARFE_ENABLE_BIT_MASK | SYS_ENABLE_LO_CIAERR_ENABLE_BIT_MASK\
            ),\
        (uint32_t) (0x00),\
        DWT_ENABLE_INT_ONLY
    );

    s_type = TWR_ANCHOR;
}

void DW3000RangingClass::handleTxDoneEvent() {
	// status change on sent success
	s_txDone = true;
}

void DW3000RangingClass::handleRxDoneEvent() {
	// status change on received success
	s_rxDone = true;
}

void DW3000RangingClass::startReceive() {

    /* clear RX status */
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);

    /* Activate reception immediately. */
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

void DW3000RangingClass::noteActivity() {
	// update activity timestamp, so that we do not reach "resetPeriod"
	s_lastActivity = millis();
}

void DW3000RangingClass::resetInactive() {
	//if inactive
	if(s_type == TWR_ANCHOR) {
		s_expectedMsgId = TWR_POLL;
        startReceive();
	}
	noteActivity();
}

void DW3000RangingClass::tickTimer() {
    // If this device is a TWR_TAG
    if (s_type == TWR_TAG) {
        if (s_counterForBlink == 0) {
            // At blink interval, send a blink message
            transmitBlink();
        } else if (s_networkDevicesNumber > 0) {
            // If there are devices in the network, send a poll
            s_expectedMsgId = TWR_POLL_ACK;

            // Send a broadcast poll message
            // transmitPoll(nullptr);
        }
    }

    // Check for inactive devices at blink interval for both anchors and tags
    if (s_counterForBlink == 0) {
        checkForInactiveDevices();
    }

    // Increment and reset the blink counter as necessary
    s_counterForBlink++;
    if (s_counterForBlink > MAX_COUNT_TO_BLINK) {
        s_counterForBlink = 0;
    }
}

void DW3000RangingClass::checkForReset() {
	uint32_t current_timestamp = millis();
	if(!s_txDone && !s_rxDone) {
		// check if inactive
		if(current_timestamp - s_lastActivity > s_resetPeriod) {
			resetInactive();
		}
		return; // TODO cc
	}
}

void DW3000RangingClass::removeNetworkDevices(int16_t index) {
	//if we have just 1 element
	if(s_networkDevicesNumber == 1) {
		s_networkDevicesNumber = 0;
	}
	else if(index == s_networkDevicesNumber-1) //if we delete the last element
	{
		s_networkDevicesNumber--;
	}
	else {
		//we translate all the element wich are after the one we want to delete.
		for(int16_t i = index; i < s_networkDevicesNumber-1; i++) { // TODO 8bit?
			memcpy((uint8_t *)&s_networkDevices[i], &s_networkDevices[i+1], sizeof(DW3000Device));  //3_16_24 pointer cast sjr
			s_networkDevices[i].setIndex(i);
		}
		s_networkDevicesNumber--;
	}

    ESP_LOGI(DW3000RANGING_LOG_TAG, "Device removed");
}

void DW3000RangingClass::checkForInactiveDevices() {
	for(uint8_t i = 0; i < s_networkDevicesNumber; i++) {
		if(s_networkDevices[i].isInactive()) {
			if(s_handleInactiveDeviceEvent != 0) {
				(*s_handleInactiveDeviceEvent)(&s_networkDevices[i]);
			}
			//we need to delete the device from the array:
			removeNetworkDevices(i);
		}
	}
}

boolean DW3000RangingClass::addNetworkDevices(DW3000Device* t_device, boolean t_shortAddress) {
	boolean   addDevice = true;
	//we test our network devices array to check
	//we don't already have it
	for(uint8_t i = 0; i < s_networkDevicesNumber; i++) {
		if(s_networkDevices[i].isAddressEqual(t_device) && !t_shortAddress) {
			//the device already exists
			addDevice = false;
			return false;
		}
		else if(s_networkDevices[i].isShortAddressEqual(t_device) && t_shortAddress) {
			//the device already exists
			addDevice = false;
			return false;
		}
		
	}
	
	if(addDevice) {
		t_device->setRange(0);
		memcpy((uint8_t *)&s_networkDevices[s_networkDevicesNumber], t_device, sizeof(DW3000Device)); //3_16_24 add pointer cast sjr
		s_networkDevices[s_networkDevicesNumber].setIndex(s_networkDevicesNumber);
		s_networkDevicesNumber++;
		return true;
	}
	
	return false;
}



boolean DW3000RangingClass::addNetworkDevices(DW3000Device* t_device) {
	boolean addDevice = true;
	//we test our network devices array to check
	//we don't already have it
	for(uint8_t i = 0; i < s_networkDevicesNumber; i++) {
		if(s_networkDevices[i].isAddressEqual(t_device) && s_networkDevices[i].isShortAddressEqual(t_device)) {
			//the device already exists
			addDevice = false;
			return false;
		}
	}
	if(addDevice) {
        //TODO: release the limit
		if(s_type == TWR_ANCHOR) //for now let's start with 1 TAG
		{
			s_networkDevicesNumber = 0;
		}
		memcpy((uint8_t *)&s_networkDevices[s_networkDevicesNumber], t_device, sizeof(DW3000Device));  //3_16_24 pointer cast sjr
		s_networkDevices[s_networkDevicesNumber].setIndex(s_networkDevicesNumber);
		s_networkDevicesNumber++;
		return true;
	}
	
	return false;
}

void DW3000RangingClass::copyShortAddress(uint8_t ts_destAddr[], uint8_t t_srcAddr[]) {
	*ts_destAddr     = *t_srcAddr;
	*(ts_destAddr+1) = *(t_srcAddr+1);
}

void DW3000RangingClass::initTransmit() {
    //TODO
}

void DW3000RangingClass::transmitBlink() {
    //TODO: initTransmit(); // change from other state to TX ??
    s_globalMac.generateBlinkFrame(s_dataBuffer, s_currentAddress, s_currentShortAddress);
    dwt_writetxdata(sizeof(s_dataBuffer), s_dataBuffer, 0); /* Zero offset in TX buffer. */
    dwt_writetxfctrl(sizeof(s_dataBuffer), 0, 1); /* Zero offset in TX buffer, ranging. */
    // dwt_starttx(DWT_START_TX_IMMEDIATE);
    dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
}

void DW3000RangingClass::transmitRangingInit(DW3000Device* t_networkDevices) {
    //TODO: rx --> tx

    //we generate the mac frame for a ranging init message
	s_globalMac.generateLongMACFrame(s_dataBuffer, s_currentShortAddress, t_networkDevices->getByteAddress());
	//we define the function code
	s_dataBuffer[LONG_MAC_LEN] = TWR_RANGING_INIT;
	
	copyShortAddress(s_lastSentToShortAddress, t_networkDevices->getByteShortAddress());

    //we send the message
    dwt_writetxdata(sizeof(s_dataBuffer), s_dataBuffer, 0); /* Zero offset in TX buffer. */
    dwt_writetxfctrl(sizeof(s_dataBuffer), 0, 1); /* Zero offset in TX buffer, ranging. */
    dwt_starttx(DWT_START_TX_IMMEDIATE);
}


void DW3000RangingClass::setTxAntennaDelay(uint16_t txAntennaDelay) {
    _txAntennaDelay = txAntennaDelay;
}

void DW3000RangingClass::setRxAntennaDelay(uint16_t rxAntennaDelay) {
    _rxAntennaDelay = rxAntennaDelay;
}

int16_t DW3000RangingClass::detectMessageType(uint8_t t_data[]) {
	if(t_data[0] == FC_1_BLINK) {
		return TWR_BLINK;
	}
	else if(t_data[0] == FC_1 && t_data[1] == FC_2) {
		//we have a long MAC frame message (ranging init)
		return t_data[LONG_MAC_LEN];
	}
	else if(t_data[0] == FC_1 && t_data[1] == FC_2_SHORT) {
		//we have a short mac frame message (poll, range, range report, etc..)
		return t_data[SHORT_MAC_LEN];
	}
}

void DW3000RangingClass::loop() {

    // Handle events from interrupt
    if (interrupt_raised) {
        interrupt_raised = false;

        uint32_t status_reg = dwt_read32bitreg(SYS_STATUS_ID);

        // check if TX done
        if (status_reg & SYS_STATUS_TXFRS_BIT_MASK) {
            // Clear TXFRS event
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);

            // Handle TX done event
            handleTxDoneEvent();
        }

        //check if RX done
        if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) {
            // Clear RXFCG event
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

            // Handle RX done event
            handleRxDoneEvent();
        }

        // Clear all other events
        dwt_write32bitreg(SYS_STATUS_ID, 0xFFFFFFFF);
    }

	//we check if needed to reset !
	checkForReset();
	uint32_t current_timestamp = millis(); // TODO other name - too close to "timer"
	if(current_timestamp - s_lastTickTime > s_timerDelay) {
		s_lastTickTime = current_timestamp;
		tickTimer();
	}

    /**************************************************************************/

    /* Code Block for TWR_TAG */
    if (s_type == TWR_TAG && s_txDone) {
        // Clear flag
        s_txDone = false;

        int16_t msgType = detectMessageType(s_dataBuffer);

        if (msgType == TWR_BLINK) {
            // ESP_LOGI(DW3000RANGING_LOG_TAG, "Blink message sent");
            // dwt_rxenable(DWT_START_RX_IMMEDIATE);
            // dwt_setinterrupt(\
            //     (uint32_t) (
            //             // RX events
            //             SYS_ENABLE_LO_RXFCG_ENABLE_BIT_MASK | SYS_ENABLE_LO_RXFTO_ENABLE_BIT_MASK | SYS_ENABLE_LO_RXPTO_ENABLE_BIT_MASK |\
            //             SYS_ENABLE_LO_RXPHE_ENABLE_BIT_MASK | SYS_ENABLE_LO_RXFCE_ENABLE_BIT_MASK | SYS_ENABLE_LO_RXFSL_ENABLE_BIT_MASK |\
            //             SYS_ENABLE_LO_RXSTO_ENABLE_BIT_MASK |\
            //             // TX events
            //             SYS_ENABLE_LO_TXFRS_ENABLE_BIT_MASK |\
            //             // other events
            //             SYS_ENABLE_LO_ARFE_ENABLE_BIT_MASK | SYS_ENABLE_LO_CIAERR_ENABLE_BIT_MASK\
            //         ),\
            //     (uint32_t) (0x00),\
            //     DWT_ENABLE_INT_ONLY
            // );
            // ESP_LOGI(DW3000RANGING_LOG_TAG, "Listening for response");
        }

    }

    if (s_type == TWR_TAG && s_rxDone) {

        ESP_LOGI(DW3000RANGING_LOG_TAG, "TAG Received message");

        // Clear flag
        s_rxDone = false;

        uint16_t rx_frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_BIT_MASK;
        if (rx_frame_len > TWR_LEN_DATA) {
            // TODO: error
            return;
        }

        dwt_readrxdata(s_dataBuffer, rx_frame_len - FCS_LEN, 0); /* No need to read the FCS/CRC. */

        int16_t msgType = detectMessageType(s_dataBuffer);

        if (msgType == TWR_RANGING_INIT) {

            ESP_LOGI(DW3000RANGING_LOG_TAG, "Ranging init message detected");

            uint8_t srcShortAddress[2];
            s_globalMac.decodeLongMACFrame(s_dataBuffer, srcShortAddress);

            DW3000Device rangeInitAnchor(srcShortAddress);

            // add device use only short address
			if(addNetworkDevices(&rangeInitAnchor, true)) {
				if(s_handleNewDeviceEvent != 0) {
					(*s_handleNewDeviceEvent)(&rangeInitAnchor);
				}

                ESP_LOGI(DW3000RANGING_LOG_TAG, "New device added to network");
			}

            noteActivity();
        }
    }

    /**************************************************************************/

    /* Code Block for anchor */
    if (s_type == TWR_ANCHOR && s_txDone) {
        // Clear flag
        s_txDone = false;
    }

    if (s_type == TWR_ANCHOR && s_rxDone) {
        // Clear flag
        s_rxDone = false;
        
        uint16_t rx_frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_BIT_MASK;
        if (rx_frame_len > TWR_LEN_DATA) {
            // TODO: error
            return;
        }

        dwt_readrxdata(s_dataBuffer, rx_frame_len - FCS_LEN, 0); /* No need to read the FCS/CRC. */

        int16_t msgType = detectMessageType(s_dataBuffer);

        if(msgType == TWR_BLINK) {
            ESP_LOGI(DW3000RANGING_LOG_TAG, "Blink message detected");

            uint8_t srcAddress[8];
            uint8_t srcShortAddress[2];
            s_globalMac.decodeBlinkFrame(s_dataBuffer, srcAddress, srcShortAddress);

            DW3000Device blinkTag(srcAddress, srcShortAddress);

            if(addNetworkDevices(&blinkTag)) {

                ESP_LOGI(DW3000RANGING_LOG_TAG, "New device added to network");

				if(s_handleNewDeviceEvent != 0) {
					(*s_handleNewDeviceEvent)(&blinkTag);
				}
				//we reply by the transmit ranging init message
				transmitRangingInit(&blinkTag);
				noteActivity();
			}

        }
    }


    
    
}