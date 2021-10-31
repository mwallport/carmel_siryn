// file crc16.cpp

#include "crc16.h"

uint16_t getCRC16(uint16_t CRC, uint8_t byte)
{
    CRC = ( (CRC % 256) << 8 ) ^ ( CRC16_table_C[ (CRC >> 8) ^ byte ] );
    return (CRC);
}


uint16_t calcCRC16(uint8_t* pBuff, uint16_t length)
{
    uint16_t    CRC = 0;


    for(uint16_t i = 0; i < length; i++)
    {
        CRC = getCRC16(CRC, pBuff[i]);
    }

    return(CRC);
}


