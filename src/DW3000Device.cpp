#include <string.h>
#include "DW3000Device.h"


DW3000Device::DW3000Device() {
    randomShortAddress();
}

DW3000Device::DW3000Device(uint8_t t_shortAddress[]) {
	setAddress(t_shortAddress);
}

DW3000Device::DW3000Device(uint8_t t_deviceAddress[], uint8_t t_shortAddress[]) {
	//we have a 8 bytes address
	setAddress(t_deviceAddress);
	//we set the 2 bytes address
	setShortAddress(t_shortAddress);
}

DW3000Device::~DW3000Device() {
}


// setters
void DW3000Device::setAddress(uint8_t address[]) {
    memcpy(m_ownAddress, address, DW3000_DEVICE_ADDRESS_SIZE);
}

void DW3000Device::setShortAddress(uint8_t address[]) {
    memcpy(m_shortAddress, address, DW3000_DEVICE_SHORT_ADDRESS_SIZE);
}


boolean DW3000Device::isAddressEqual(DW3000Device* device) {
	return memcmp(this->getByteAddress(), device->getByteAddress(), DW3000_DEVICE_ADDRESS_SIZE) == 0;
}

boolean DW3000Device::isShortAddressEqual(DW3000Device* device) {
    return memcmp(this->getByteShortAddress(), device->getByteShortAddress(), DW3000_DEVICE_SHORT_ADDRESS_SIZE) == 0;
}

void DW3000Device::setRange(float range) { m_range = round(range*100); }

// getters
uint8_t* DW3000Device::getByteAddress() { return m_ownAddress; }

uint8_t* DW3000Device::getByteShortAddress() { return m_shortAddress; }

float DW3000Device::getRange() { return float(m_range)/100.0f; }


// others
void DW3000Device::randomShortAddress() {
	m_shortAddress[0] = random(0, 256);
	m_shortAddress[1] = random(0, 256);
}

boolean DW3000Device::isInactive() {
	//One second of inactivity
	if( millis() - m_lastActiveTimestamp > DW3000_DEVICE_INACTIVITY_TIMEOUT) {
		m_lastActiveTimestamp = millis();
		return true;
	}
	return false;
}

