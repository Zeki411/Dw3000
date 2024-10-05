#include <string.h>
#include "DW3000Device.h"


DW3000Device::DW3000Device() {
}


void DW3000Device::setAddress(uint8_t address[]) {
    memcpy(_ownAddress, address, DW3000_DEVICE_ADDRESS_SIZE);
}

void DW3000Device::setShortAddress(uint8_t address[]) {
    memcpy(_shortAddress, address, DW3000_DEVICE_SHORT_ADDRESS_SIZE);
}



