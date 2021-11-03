#ifndef __CTRL_COMMANDS__
#define __CTRL_COMMANDS__


//
// temp controller min Rx'ed packet size
//
// TODO: check what the error pkt looks like from the temp controller
static const uint8_t  CTRLR_MIN_RX_PKT_LEN  = 8;
static const uint8_t  CTRLR_ID_OFFSET       = 0;
static const uint8_t  CTRLR_FUNC_OFFSET     = 1;  // read 0x03 or write 0x06
static const uint8_t  CTRLR_PARAM_ADDR      = 2;  // register address
static const uint8_t  CTRLR_WRITE_DATA_OFFSET     = 4;
static const uint8_t  CTRLR_READ_DATA_CNT_OFFSET  = 4;
static const uint8_t  CTRLR_READ_BYTE_CNT_OFFSET  = 2;
static const uint8_t  CTRLR_CRC_OFFSET      = 6;
//
// temp controller modbus register addresses
//

// read only commands
static const uint16_t PVPVOF    = 0x1000;


// read write commands
static const uint16_t SV        = 0x0000;


//
// RTD controller modbus register addresses
//

#endif
