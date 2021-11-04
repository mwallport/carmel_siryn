//defines and such that are common to many compilation units
#ifndef __COMMON_HEADER__
#define __COMMON_HEADER__


// process defines
#define _READ_TIME_OUT_    3000


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
