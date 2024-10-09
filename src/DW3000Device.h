#ifndef __DW3000DEVICE_H__
#define __DW3000DEVICE_H__

#include <stdint.h>
#include "DW3000Mac.h"

#define DW3000_DEVICE_ADDRESS_SIZE 8
#define DW3000_DEVICE_SHORT_ADDRESS_SIZE 2
#define DW3000_DEVICE_INACTIVITY_TIMEOUT 1000

class DW3000Device {
public:
    DW3000Device();
    DW3000Device(uint8_t t_shortAddress[]);
    DW3000Device(uint8_t t_deviceAddress[], uint8_t t_shortAddress[]);
    ~DW3000Device();

    //setters
	void setAddress(uint8_t* address);
	void setShortAddress(uint8_t address[]);
    void setIndex(int8_t index) { m_index = index; }

    void setRange(float range);

    //getters
    uint8_t* getByteAddress();
    uint8_t* getByteShortAddress();

    float getRange();


	boolean isAddressEqual(DW3000Device* device);
	boolean isShortAddressEqual(DW3000Device* device);
    boolean isInactive();

    void randomShortAddress();

private:
    
    uint8_t m_ownAddress[8];
    uint8_t m_shortAddress[2];
    int32_t m_lastActiveTimestamp;
    int8_t m_index; // not used

    int16_t m_range;

};





#endif // __DW3000DEVICE_H__