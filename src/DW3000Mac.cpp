#include "DW3000Mac.h"


//Constructor and destructor

DW3000Mac::DW3000Mac() {
	m_seqNumber = 0;
}


DW3000Mac::~DW3000Mac() {
}

//for poll message we use just 2 bytes address
//total=12 bytes
void DW3000Mac::generateBlinkFrame(uint8_t frame[], uint8_t sourceAddress[], uint8_t sourceShortAddress[]) {
	//Frame Control
	*frame     = FC_1_BLINK;
	//sequence number
	*(frame+1) = m_seqNumber;
	//tag 64 bit ID (8 bytes address) -- reverse
	uint8_t sourceAddressReverse[8];
	reverseArray(sourceAddressReverse, sourceAddress, 8);
	memcpy(frame+2, sourceAddressReverse, 8);
	
	//tag 2bytes address:
	uint8_t sourceShortAddressReverse[2];
	reverseArray(sourceShortAddressReverse, sourceShortAddress, 2);
	memcpy(frame+10, sourceShortAddressReverse, 2);
	
	//we increment seqNumber
	incrementSeqNumber();
}

//the short fram usually for Resp, Final, or Report
//2 bytes for Desination Address and 2 bytes for Source Address
//total=9 bytes
void DW3000Mac::generateShortMACFrame(uint8_t frame[], uint8_t sourceShortAddress[], uint8_t destinationShortAddress[]) {
	//Frame controle
	*frame     = FC_1;
	*(frame+1) = FC_2_SHORT;
	//sequence number (11.3) modulo 256
	*(frame+2) = m_seqNumber;
	//PAN ID
	*(frame+3) = 0xCA;
	*(frame+4) = 0xDE;
	
	
	//destination address (2 bytes)
	uint8_t destinationShortAddressReverse[2];
	reverseArray(destinationShortAddressReverse, destinationShortAddress, 2);
	memcpy(frame+5, destinationShortAddressReverse, 2);
	
	//source address (2 bytes)
	uint8_t sourceShortAddressReverse[2];
	reverseArray(sourceShortAddressReverse, sourceShortAddress, 2);
	memcpy(frame+7, sourceShortAddressReverse, 2);
	
	
	//we increment seqNumber
	incrementSeqNumber();
}

//the long frame for Ranging init
//8 bytes for Destination Address and 2 bytes for Source Address
//total=15
void DW3000Mac::generateLongMACFrame(uint8_t frame[], uint8_t sourceShortAddress[], uint8_t destinationAddress[]) {
	//Frame controle
	*frame     = FC_1;
	*(frame+1) = FC_2;
	//sequence number
	*(frame+2) = m_seqNumber;
	//PAN ID (0xDECA)
	*(frame+3) = 0xCA;
	*(frame+4) = 0xDE;
	
	//destination address (8 bytes) - we need to reverse the uint8_t array
	uint8_t destinationAddressReverse[8];
	reverseArray(destinationAddressReverse, destinationAddress, 8);
	memcpy(frame+5, destinationAddressReverse, 8);
	
	//source address (2 bytes)
	uint8_t sourceShortAddressReverse[2];
	reverseArray(sourceShortAddressReverse, sourceShortAddress, 2);
	memcpy(frame+13, sourceShortAddressReverse, 2);
	
	//we increment seqNumber
	incrementSeqNumber();
}


void DW3000Mac::decodeBlinkFrame(uint8_t t_frame[], uint8_t t_srcAddress[], uint8_t t_srcShortAddress[]) {
	//we save the long address of the sender into the device. -- reverse direction
	uint8_t reverseAddress[8];
	memcpy(reverseAddress, t_frame+2, 8);
	reverseArray(t_srcAddress, reverseAddress, 8);
	
	uint8_t reverseShortAddress[2];
	memcpy(reverseShortAddress, t_frame+10, 2);
	reverseArray(t_srcShortAddress, reverseShortAddress, 2);
}

void DW3000Mac::decodeShortMACFrame(uint8_t t_frame[], uint8_t t_srcShortaddress[]) {
	uint8_t reverseAddress[2];
	memcpy(reverseAddress, t_frame+7, 2);
	reverseArray(t_srcShortaddress, reverseAddress, 2);
	//we grab the destination address for the mac frame
	//uint8_t destinationAddress[2];
	//memcpy(destinationAddress, frame+5, 2);
}

void DW3000Mac::decodeLongMACFrame(uint8_t t_frame[], uint8_t t_srcShortaddress[]) {
	uint8_t reverseAddress[2];
	memcpy(reverseAddress, t_frame+13, 2);
	reverseArray(t_srcShortaddress, reverseAddress, 2);
	//we grab the destination address for the mac frame
	//uint8_t destinationAddress[8];
	//memcpy(destinationAddress, frame+5, 8);
}


void DW3000Mac::incrementSeqNumber() {
	// normally overflow of uint8 automatically resets to 0 if over 255
	// but if-clause seems safer way
	if(m_seqNumber == 255)
		m_seqNumber = 0;
	else
		m_seqNumber++;
}

void DW3000Mac::reverseArray(uint8_t t_to[], uint8_t t_from[], uint16_t t_size) {
	for(uint16_t i = 0; i < t_size; i++) {
		*(t_to+i) = *(t_from+t_size-i-1);
	}
}
