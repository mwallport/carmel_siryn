#ifndef __CTRL_COMMANDS__
#define __CTRL_COMMANDS__
#include <inttypes.h>

//
// temp controller min Rx'ed packet size
//
// TODO: check what the error pkt looks like from the temp controller
static const uint8_t  CTRLR_MIN_RX_PKT_LEN  = 8;
static const uint8_t  CTRLR_ID_OFFSET     = 0;
static const uint8_t  CTRLR_FUNC_OFFSET   = 1;  // read 0x03 or write 0x06
static const uint8_t  CTRLR_PARAM_ADDR    = 2;  // register address
static const uint8_t  CTRLR_WRITE_DATA_OFFSET   = 4;
static const uint8_t  CTRLR_READ_DATA_CNT_OFFSET  = 4;
static const uint8_t  CTRLR_READ_BYTE_CNT_OFFSET  = 2;
static const uint8_t  CTRLR_CRC_OFFSET    = 6;

// values for ENAB register
static const uint8_t  OFF = 0x0000;   // turn off output
static const uint8_t  AT1 = 0x0001;   // auto-tune at SV
static const uint8_t  AT2 = 0x0002;   // auto-tune at 90% of SV
static const uint8_t  MPWR  = 0x0003; // manual set duty cycle
static const uint8_t  SPON  = 0x0004; // single temp point ctrl
static const uint8_t  PROG  = 0x0005; // run programmable temp profile
static const uint8_t  HOLD  = 0x0006; // hold temp durin gprog profile run

//
// temp controller modbus register addresses
//

// read only commands
static const uint16_t PVPVOF  = 0x1000;
static const uint16_t SVSVOF  = 0x1001;
static const uint16_t OUTL    = 0x1002;
static const uint16_t WKERNO  = 0x1003;
static const uint16_t RAMP_TL   = 0x1004;
static const uint16_t RAMP_TH   = 0x1005;
static const uint16_t ALM1_TL   = 0x1006;
static const uint16_t ALM1_TH   = 0x1007;
static const uint16_t SV0     = 0x1008;
static const uint16_t PV0     = 0x1009;
static const uint16_t PV1     = 0x100A;
static const uint16_t PV2     = 0x100B;
static const uint16_t ET0     = 0x100C;
static const uint16_t ET1     = 0x100D;
static const uint16_t ET2     = 0x100E;
static const uint16_t Px    = 0x100F;
static const uint16_t Ix    = 0x1010;
static const uint16_t Dx    = 0x1011;
static const uint16_t MRx     = 0x1012;
static const uint16_t ARx     = 0x1013;
static const uint16_t Pout    = 0x1014;
static const uint16_t Iout    = 0x1015;
static const uint16_t Dout    = 0x1016;
static const uint16_t Pband   = 0x1017;
static const uint16_t ARW     = 0x1018;
static const uint16_t LEVEL   = 0x1019;
static const uint16_t AD0     = 0x101A;
static const uint16_t AD1     = 0x101B;
static const uint16_t VER     = 0x1F00;
static const uint16_t SERIAL_NH = 0x1F01;
static const uint16_t SERIAL_NL = 0x1f02;

// read 0x03 and write 0x06 parameter table
static const uint16_t SV    = 0x0000;
static const uint16_t OUTLrw  = 0x0001;
static const uint16_t ENAB    = 0x0002;
static const uint16_t PB1x    = 0x0003;
static const uint16_t TI1     = 0x0004;
static const uint16_t TD1     = 0x0005;
static const uint16_t MR1     = 0x0006;
static const uint16_t AR1     = 0x0007;
static const uint16_t ASP1    = 0x0008;
static const uint16_t PB2x    = 0x0009;
static const uint16_t TI2     = 0x000A;
static const uint16_t TD2     = 0x000B;
static const uint16_t MR2     = 0x000C;
static const uint16_t AR2     = 0x000D;
static const uint16_t ASP2    = 0x000E;
static const uint16_t PB3x    = 0x000F;
static const uint16_t TI3     = 0x0010;
static const uint16_t TD3     = 0x0011;
static const uint16_t MR3     = 0x0012;
static const uint16_t AR3     = 0x0013;
static const uint16_t ASP3    = 0x0014;
static const uint16_t PB4x    = 0x0015;
static const uint16_t TI4     = 0x0016;
static const uint16_t TD4     = 0x0017;
static const uint16_t MR4     = 0x0018;
static const uint16_t AR4     = 0x0019;
static const uint16_t A1SP    = 0x001A;
static const uint16_t A1HY    = 0x001B;
static const uint16_t A1FU    = 0x001C;
static const uint16_t A1MD    = 0x001D;
static const uint16_t A1DT    = 0x001E;
static const uint16_t A1AB    = 0x001F;
static const uint16_t A1ER    = 0x0020;
static const uint16_t A2SP    = 0x0021;
static const uint16_t A2HY    = 0x0022;
static const uint16_t A2FU    = 0x0023;
static const uint16_t A2MD    = 0x0024;
static const uint16_t A2DT    = 0x0025;
static const uint16_t A2AB    = 0x0026;
static const uint16_t A2ER    = 0x0027;
static const uint16_t TYPE    = 0x0028;
static const uint16_t UNIT    = 0x0029;
static const uint16_t DP    = 0x002A;
static const uint16_t DIR     = 0x002B;
static const uint16_t LOLT    = 0x002C;
static const uint16_t HILT    = 0x002D;
static const uint16_t TUNT    = 0x002E;
static const uint16_t EROP    = 0x002F;
static const uint16_t SPOF    = 0x0030;
static const uint16_t PVOF    = 0x0031;
static const uint16_t FILT    = 0x0032;
static const uint16_t ID    = 0x0033;
static const uint16_t STAT    = 0x0034;
static const uint16_t STAR    = 0x0035;
static const uint16_t BAND    = 0x0036;
static const uint16_t RT1     = 0x0037;
static const uint16_t SP1     = 0x0038;
static const uint16_t ST1     = 0x0039;
static const uint16_t SF1     = 0x003A;
static const uint16_t LN1     = 0x003B;
static const uint16_t RT2     = 0x003C;
static const uint16_t SP2     = 0x003D;
static const uint16_t ST2     = 0x003E;
static const uint16_t SF2     = 0x003F;
static const uint16_t LN2     = 0x0040;
static const uint16_t RT3     = 0x0041;
static const uint16_t SP3     = 0x0042;
static const uint16_t ST3     = 0x0043;
static const uint16_t SF3     = 0x0044;
static const uint16_t LN3     = 0x0045;
static const uint16_t RT4     = 0x0046;
static const uint16_t SP4     = 0x0047;
static const uint16_t ST4     = 0x0048;
static const uint16_t SF4     = 0x0049;
static const uint16_t LN4     = 0x004A;
static const uint16_t RT5     = 0x004B;
static const uint16_t SP5     = 0x004C;
static const uint16_t ST5     = 0x004D;
static const uint16_t SF5     = 0x004E;
static const uint16_t LN5     = 0x004F;
static const uint16_t RT6     = 0x0050;
static const uint16_t SP6     = 0x0051;
static const uint16_t ST6     = 0x0052;
static const uint16_t SF6     = 0x0053;
static const uint16_t LN6     = 0x0054;
static const uint16_t RT7     = 0x0055;
static const uint16_t SP7     = 0x0056;
static const uint16_t ST7     = 0x0057;
static const uint16_t SF7     = 0x0058;
static const uint16_t LN7     = 0x0059;
static const uint16_t RT8     = 0x005A;
static const uint16_t SP8     = 0x005B;
static const uint16_t ST8     = 0x005C;
static const uint16_t SF8     = 0x005D;
static const uint16_t LN8     = 0x005E;
static const uint16_t LOCK    = 0x005F;
static const uint16_t SCAL    = 0x0075;
static const uint16_t SCAH    = 0x0076;
static const uint16_t CUT     = 0x0077;


#endif
