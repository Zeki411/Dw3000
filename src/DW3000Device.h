#ifndef __DW3000DEVICE_H__
#define __DW3000DEVICE_H__

#include <stdint.h>


#define DW3000_DEVICE_ADDRESS_SIZE 8
#define DW3000_DEVICE_SHORT_ADDRESS_SIZE 2


class DW3000Device {
public:
    DW3000Device();
    // DW3000Device(uint8_t deviceAddress[], uint8_t shortAddress[]);
    ~DW3000Device();

    //setters
	void setAddress(uint8_t* address);
	void setShortAddress(uint8_t address[]);

    //getters

private:
    
    uint8_t _ownAddress[8];
    uint8_t _shortAddress[2];


};





#endif // __DW3000DEVICE_H__