// file crc16.h
#include <stdint.h>

//
// is a character based protocol
// using CCITT-16 CRC to protect the data
// which most definitely is coped from the meerstetter implementation
//

extern "C"
{
  int16_t calcCRC16(uint8_t* buff, uint8_t len);
  

//
// given an array of uin8_t bytes, calculate the CRC16 for it
//

}
