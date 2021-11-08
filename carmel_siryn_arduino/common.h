//defines and such that are common to many compilation units
#ifndef __COMMON_HEADER__
#define __COMMON_HEADER__


// process defines
#define _READ_TIME_OUT_    3000

// there are 6 bytes that must be in a Rx'ed read response packet
// id(1) + r/w func(1) + byte count (2) + crc(2) = 6 bytes
#define _PKT_BYTS_EXCEPT_BYTE_CNT_BYTES_  6 


// need the htons, et al..
#define htons(x) ( ((x)<< 8 & 0xFF00) | \
                ((x)>> 8 & 0x00FF) )

#define ntohs(x) htons(x)

#define htonl(x) ( ((x)<<24 & 0xFF000000UL) | \
                ((x)<< 8 & 0x00FF0000UL) | \
                ((x)>> 8 & 0x0000FF00UL) | \
                ((x)>>24 & 0x000000FFUL) )
                
#define ntohl(x) htonl(x)




// debug defines
// dump MODBUS packets Tx and Rx
#define __DEBUG_MODBUS_CMDS__
#define __DEBUG_MODBUS_TXRX__

// these need to match the actual physical assignments
enum Rs485BusId
{
  ASIC  = 1,
  DDR   = 2,
  RTD   = 3,
  MAX_RS485_ID
};




#endif
