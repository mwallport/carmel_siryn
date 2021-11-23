// file crc16.cpp

#include "crc16_modbus.h"

extern "C"
{


// calculates proper modbus crc according to : https://www.lammertbies.nl/comm/info/crc-calculation
int16_t calcCRC16(uint8_t* buff, uint8_t len)
{
  unsigned int temp, temp2, flag;
  temp = 0xFFFF;
  for (unsigned char i = 0; i < len; i++)
  {   
    temp = temp ^ buff[i];
    for (unsigned char j = 1; j <= 8; j++)
    {
      flag = temp & 0x0001;
      temp >>=1;
      if (flag)
        temp ^= 0xA001;
    }
  }   
  // Reverse byte order.
  temp2 = temp >> 8;
  temp = (temp << 8) | temp2;
  temp &= 0xFFFF;
  // the returned value is already swapped
  // crcLo byte is first & crcHi byte is last
  return temp;
}

}
