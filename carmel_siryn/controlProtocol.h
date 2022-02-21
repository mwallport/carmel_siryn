#ifndef _CONTROL_PROTOCOL_
#define _CONTROL_PROTOCOL_

// debug - the Serial port must be started, else these will hange the Due
//#define __DEBUG_CTRL_PROTO__
//#define __DEBUG_CONTROL_PKT_TX__
//#define __DEBUG_CONTROL_PKT_RX__

// platform
//#define __USING_LINUX_USB__
//#define __USING_WINDOWS_USB__
#define __RUNNING_ON_CONTROLLINO__

//chiller
#define __USING_CHILLER__

// common
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "eventlog.h"


#ifdef __RUNNING_ON_CONTROLLINO__
#include "Arduino.h"
#include <HardwareSerial.h>
#define htons(x) ( ((x)<< 8 & 0xFF00) | ((x)>> 8 & 0x00FF) )
#define ntohs(x) htons(x)
#define htonl(x) ( ((x)<<24 & 0xFF000000UL) | \
                    ((x)<< 8 & 0x00FF0000UL) | \
                    ((x)>> 8 & 0x0000FF00UL) | \
                    ((x)>>24 & 0x000000FFUL) )
#define ntohl(x) htonl(x)
#endif

#ifdef __USING_LINUX_USB__
    #include <unistd.h>
    #include <string.h>
    #include <unistd.h>
    #include <arpa/inet.h>
    #include <termios.h>
    #include <fcntl.h>
#endif

#ifdef __USING_WINDOWS_USB__
    #include <winsock2.h>
    #include <windows.h>
#endif

#ifndef GET_LOW_NIBBLE
#define GET_LOW_NIBBLE
typedef union _int_byte 
{   
    int intVal;
    uint8_t byteVal[sizeof(int)];  
} int_byte;

// idea is to return the lowest nibble for Intel processor of arbitrary size int
uint8_t get_low_nibble(int); 
#endif


//
// is a character based protocol
// using CCITT-16 CRC to protect the data
// which most definitely is coped from the meerstetter implementation
//
// given an array of uin8_t bytes, calculate the CRC16 for it
//
const uint16_t CRC16_table_C[256] = {
    // CRC-CCIT calculated for every byte between 0x0000 and 0x00FF
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
    0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
    0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
    0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
    0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
    0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
    0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
    0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
    0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
    0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
    0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
    0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
    0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
    0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
    0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
    0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
    0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
    0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
    0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
    0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
    0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
    0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
    0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0
};


uint16_t getCRC16(uint16_t n, uint8_t m);
uint16_t calcCRC16(uint8_t* pBuff, uint16_t length);
// end of crc16.h


//
// loosely based off the Meerstetter protocol in which each frame has
//  - a start character, '#' for master and '!' for response from slave
//  - a sequence number to ensure the answer gotten is the answer to the
//      current request, i.e. the slave will echo back the sequence number
//  - some payload
//  - a 16 bit CRC field
//  - an end of frame character
//
const   uint8_t     MAX_CHILLER_TEMP_LENGH  = 8;    // i.e "-21.5"  or "+100.1" - sign and a float number
const   uint8_t     MAX_ACU_TEMP_LENGH      = 8;    // i.e "-21.5"  or "+100.1" - sign and a float number
const   uint8_t     MAX_HUMIDITY_LENGTH     = 8;    // "34.37" interpreted as percent
const   uint8_t     MAX_BUFF_LENGTH_CP      = 200;   // size of the work m_buffer
const   uint8_t     MAX_CHILLER_INFO_LENGTH = 20;   // same size as the name in the huber protocol
const   uint8_t     COMMAND                 = '#';  // start packet byte for commands
const   uint8_t     RESPONSE                = '!';  // start packet byte for responses
const   uint8_t     MSG_NUM_OFFSET          = 4;
const   uint16_t    EOP_VAL                 = 0x0D; // end of transmission val
const   uint16_t    COMM_TIMEOUT            = 10000; // milliseconds communication time out


typedef enum _msgID
{
    getStatusCmd,               // fetch the status of chiller, all ACUs, and humidity sensor
    getStatusResp,              // get status response
    setACUTemperature,          // target ACU m_address and temp
    setACUTemperatureResp,      // target ACU m_address and temp response
    getACUTemperature,          // target ACU m_address and temp
    getACUTemperatureResp,      // target ACU m_address and temp response
    getACUObjTemperature,       // target ACU m_address and object temp
    getACUObjTemperatureResp,   // target ACU m_address and object temp response
    getACUInfoMsg,              // get ACU info
    getACUInfoMsgResp,          // response ***
    enableACUs,                 // turn on all ACUs
    enableACUsResp,             // turn on all ACUs response
    disableACUs,                // turn off all ACUs
    disableACUsResp,            // turn off all ACUs response
#if defined(__USING_CHILLER__)
    setChillerTemperature,      // set chiller set point
    setChillerTemperatureResp,  // set chiller set point response
    getChillerTemperature,      // get chiller set point
    getChillerTemperatureResp,  // get chiller set point response
    getChillerObjTemperature,   // get chiller internal temp for now, don't know if model will have external 
    getChillerObjTemperatureResp,// get chiller internal temp response
    startChillerMsg,            // start the chiller  ***
    startChillerMsgResp,        // response ***
    stopChiller,                // stop the chiller  ***
    stopChillerResp,            // response ***
    getChillerInfo,             // get the name of the chiller  ***
    getChillerInfoResp,         // response ***
#endif
    startUpCmd,                 // start up
    startUpCmdResp,             // reponse
    startUpATCmd,               // start up
    startUpATCmdResp,           // reponse
    shutDownCmd,                // shutdown
    shutDownCmdResp,            // shutdown response
    setRTCCmd,                  // set RTC clock command
    setRTCCmdResp,              // set RTC clock response
    getRTCCmd,                  // get the RTC clock 
    getRTCCmdResp,              // get the RTC clock response
    clrEventLogCmd,             // clear the event log
    clrEventLogCmdResp,         // clear the event log response
    getEventLogCmd,             // get the eventlog
    getEventLogCmdResp,         // get the eventlog response
    setH20AlarmASIC,            // 
    setH20AlarmASICResp,        //
    setH20AlarmDDR,             // 
    setH20AlarmDDRResp,         //
    getH20AlarmASIC,            // 
    getH20AlarmASICResp,        //
    getH20AlarmDDR,             // 
    getH20AlarmDDRResp,         //
    NACK                        // command not supported
} msgID;


typedef struct _Address
{
    uint16_t    address;    // currently 0 for master control PC, 1 for Conrollino
} Address_t;


typedef uint16_t CRC;
typedef uint16_t EOP;


typedef struct _msgHeader
{
    uint8_t    control;        // '#' or '!' - character
    uint8_t    length;         // total packet length, byte 0 .. n
    Address_t  address;        // 0 for master, !0 for slave(s) - uint8_t
    uint8_t    seqNum;         // uint16_t
    uint8_t    msgNum;         // uint16_t - this will be the getStatusMsg message
} msgHeader_t;


typedef struct _getStatus
{
    msgHeader_t header;
    CRC         crc;            // 16 bit CRC over the packet
    EOP         eop;            // end of transmission character/byte
} getStatus_t;
// TODO: better way to exclude crc legnth?
const uint16_t len_getStatus_t    = sizeof(getStatus_t) - sizeof(CRC) - sizeof(EOP);


typedef struct _statusReport
{
    uint16_t    RTDsRunning;    // 0 - no enough are toast the system is shutdown ; 1 - yes
    uint16_t    ACUsRunning;    // 0 - no ; 1 - yes
    uint16_t    chillerOnLine;  // 0 - no ; 1 - yes
} statusReport_t;

typedef struct _getStatusResp
{
    msgHeader_t     header;
    statusReport_t  status;     // the status
    CRC             crc;        // 16 bit CRC over the packet
    EOP             eop;        // end of transmission character/byte
} getStatusResp_t;
const uint16_t len_getStatusResp_t  = sizeof(getStatusResp_t) - sizeof(CRC) - sizeof(EOP);


typedef struct _setACUTemperature
{
    msgHeader_t header;
    uint16_t    acu_address;    // uint16_6 - ACU address
    uint8_t     temperature[MAX_ACU_TEMP_LENGH];    // float in 32 bits
    CRC         crc;            // 16 bit CRC over the packet
    EOP         eop;            // end of transmission character/byte
} setACUTemperature_t;
const uint16_t len_setACUTemperature_t    = sizeof(setACUTemperature_t) - sizeof(CRC) - sizeof(EOP);


typedef struct _setACUTemperatureResp
{
    msgHeader_t header;
    uint16_t    acu_address;    // uint16_6 - ACU address
    uint16_t    result;         // 0 - fail ; 1 - success
    CRC         crc;            // 16 bit CRC over the packet
    EOP         eop;            // end of transmission character/byte
} setACUTemperatureResp_t;
const uint16_t len_setACUTemperatureResp_t    = sizeof(setACUTemperatureResp_t) - sizeof(CRC) - sizeof(EOP);


typedef struct _getACUTemperature
{
    msgHeader_t header;
    uint16_t    acu_address;    // uint16_6 - ACU address
    CRC         crc;            // 16 bit CRC over the packet
    EOP         eop;            // end of transmission character/byte
} getACUTemperature_t;
const uint16_t len_getACUTemperature_t    = sizeof(getACUTemperature_t) - sizeof(CRC) - sizeof(EOP);


typedef struct _getACUTemperatureResp
{
    msgHeader_t header;
    uint16_t    acu_address;    // uint16_6 - ACU address
    uint16_t    result;         // 0 - fail ; 1 - success
    uint8_t     temperature[MAX_ACU_TEMP_LENGH];    // float in 32 bits
    CRC         crc;            // 16 bit CRC over the packet
    EOP         eop;            // end of transmission character/byte
} getACUTemperatureResp_t;
const uint16_t len_getACUTemperatureResp_t    = sizeof(getACUTemperatureResp_t) - sizeof(CRC) - sizeof(EOP);


typedef struct _getACUObjTemperature
{
    msgHeader_t header;
    uint16_t    acu_address;    // uint16_6 - ACU address
    CRC         crc;            // 16 bit CRC over the packet
    EOP         eop;            // end of transmission character/byte
} getACUObjTemperature_t;
const uint16_t len_getACUObjTemperature_t    = sizeof(getACUObjTemperature_t) - sizeof(CRC) - sizeof(EOP);


typedef struct _getACUObjTemperatureResp
{
    msgHeader_t header;
    uint16_t    acu_address;    // uint16_6 - ACU address
    uint16_t    result;         // 0 - fail ; 1 - success
    uint8_t     temperature[MAX_ACU_TEMP_LENGH];    // float in 32 bits
    CRC         crc;            // 16 bit CRC over the packet
    EOP         eop;            // end of transmission character/byte
} getACUObjTemperatureResp_t;
const uint16_t len_getACUObjTemperatureResp_t    = sizeof(getACUObjTemperatureResp_t) - sizeof(CRC) - sizeof(EOP);


typedef struct _enableACUs
{
    msgHeader_t header;
    CRC         crc;            // 16 bit CRC over the packet
    EOP         eop;            // end of transmission character/byte
} enableACUs_t;
const uint16_t len_enableACUs_t    = sizeof(enableACUs_t) - sizeof(CRC) - sizeof(EOP);


typedef struct _enableACUsResp
{
    msgHeader_t header;
    uint16_t    result;         // 0 - fail ; 1 - success
    CRC         crc;            // 16 bit CRC over the packet
    EOP         eop;            // end of transmission character/byte
} enableACUsResp_t;
const uint16_t len_enableACUsResp_t    = sizeof(enableACUsResp_t) - sizeof(CRC) - sizeof(EOP);


typedef struct _disableACUs
{
    msgHeader_t header;
    CRC         crc;            // 16 bit CRC over the packet
    EOP         eop;            // end of transmission character/byte
} disableACUs_t;
const uint16_t len_disableACUs_t    = sizeof(disableACUs_t) - sizeof(CRC) - sizeof(EOP);


typedef struct _disableACUsResp
{
    msgHeader_t header;
    uint16_t    result;         // 0 - fail ; 1 - success
    CRC         crc;            // 16 bit CRC over the packet } disableACUsResp_t;
    EOP         eop;            // end of transmission character/byte
} disableACUsResp_t;
const uint16_t len_disableACUsResp_t    = sizeof(disableACUsResp_t) - sizeof(CRC) - sizeof(EOP);

typedef struct _setChillerTemperature
{
    msgHeader_t header;
    uint16_t    temperature[MAX_CHILLER_TEMP_LENGH];    // float in 32 bits
    CRC         crc;            // 16 bit CRC over the packet
    EOP         eop;            // end of transmission character/byte
} setChillerTemperature_t;
const uint16_t len_setChillerTemperature_t    = sizeof(setChillerTemperature_t) - sizeof(CRC) - sizeof(EOP);


typedef struct _setChillerTemperatureResp
{
    msgHeader_t header;
    uint16_t    result;         // 0 - fail ; 1 - success
    CRC         crc;            // 16 bit CRC over the packet
    EOP         eop;            // end of transmission character/byte
} setChillerTemperatureResp_t;
const uint16_t len_setChillerTemperatureResp_t = sizeof(setChillerTemperatureResp_t) - sizeof(CRC) - sizeof(EOP);


typedef struct _getChillerTemperature
{
    msgHeader_t header;
    CRC         crc;            // 16 bit CRC over the packet
    EOP         eop;            // end of transmission character/byte
} getChillerTemperature_t;
const uint16_t len_getChillerTemperature_t = sizeof(getChillerTemperature_t) - sizeof(CRC) - sizeof(EOP);


typedef struct _getChillerTemperatureResp
{
    msgHeader_t header;
    uint8_t     temperature[MAX_CHILLER_TEMP_LENGH];    // float in 32 bits
    CRC         crc;            // 16 bit CRC over the packet
    EOP         eop;            // end of transmission character/byte
} getChillerTemperatureResp_t;
const uint16_t len_getChillerTemperatureResp_t = sizeof(getChillerTemperatureResp_t) - sizeof(CRC) - sizeof(EOP);


typedef struct _getChillerObjTemperature
{
    msgHeader_t header;
    CRC         crc;            // 16 bit CRC over the packet
    EOP         eop;            // end of transmission character/byte
} getChillerObjTemperature_t;
const uint16_t len_getChillerObjTemperature_t = sizeof(getChillerObjTemperature_t) - sizeof(CRC) - sizeof(EOP);


typedef struct _getChillerObjTemperatureResp
{
    msgHeader_t header;
    uint8_t     temperature[MAX_CHILLER_TEMP_LENGH];    // float in 32 bits
    CRC         crc;            // 16 bit CRC over the packet
    EOP         eop;            // end of transmission character/byte
} getChillerObjTemperatureResp_t;
const uint16_t len_getChillerObjTemperatureResp_t = sizeof(getChillerObjTemperatureResp_t) - sizeof(CRC) - sizeof(EOP);


typedef struct _startChillerMsg
{
    msgHeader_t header;
    CRC         crc;            // 16 bit CRC over the packet
    EOP         eop;            // end of transmission character/byte
} startChillerMsg_t;
const uint16_t len_startChillerMsg_t = sizeof(startChillerMsg_t) - sizeof(CRC) - sizeof(EOP);


typedef struct _startChillerMsgResp
{
    msgHeader_t header;
    uint16_t    result;         // 0 - failed to set; 1 - successfully set
    CRC         crc;            // 16 bit CRC over the packet
    EOP         eop;            // end of transmission character/byte
} startChillerMsgResp_t;
const uint16_t len_startChillerMsgResp_t = sizeof(startChillerMsgResp_t) - sizeof(CRC) - sizeof(EOP);


typedef struct _stopChiller
{
    msgHeader_t header;
    CRC         crc;            // 16 bit CRC over the packet
    EOP         eop;            // end of transmission character/byte
} stopChiller_t;
const uint16_t len_stopChiller_t = sizeof(stopChiller_t) - sizeof(CRC) - sizeof(EOP);


typedef struct _stopChillerResp
{
    msgHeader_t header;
    uint16_t    result;         // 0 - failed to set; 1 - successfully set
    CRC         crc;            // 16 bit CRC over the packet
    EOP         eop;            // end of transmission character/byte
} stopChillerResp_t;
const uint16_t len_stopChillerResp_t = sizeof(stopChillerResp_t) - sizeof(CRC) - sizeof(EOP);


typedef struct _getChillerInfo
{
    msgHeader_t header;
    CRC         crc;            // 16 bit CRC over the packet
    EOP         eop;            // end of transmission character/byte
} getChillerInfo_t;
const uint16_t len_getChillerInfo_t = sizeof(getChillerInfo_t) - sizeof(CRC) - sizeof(EOP);


typedef struct _getChillerInfoResp
{
    msgHeader_t header;
    uint16_t    result;
    uint8_t     info[MAX_CHILLER_INFO_LENGTH];   // ASCII string data, i.e. name
    CRC         crc;                            // 16 bit CRC over the packet
    EOP         eop;                            // end of transmission character/byte
} getChillerInfoResp_t;
const uint16_t len_getChillerInfoResp_t = sizeof(getChillerInfoResp_t) - sizeof(CRC) - sizeof(EOP);


typedef struct _getACUInfoMsg
{
    msgHeader_t header;
    uint16_t    acu_address;    // uint16_6 - ACU address
    CRC         crc;            // 16 bit CRC over the packet
    EOP         eop;            // end of transmission character/byte
} getACUInfoMsg_t;
const uint16_t len_getACUInfoMsg_t = sizeof(getACUInfoMsg_t) - sizeof(CRC) - sizeof(EOP);


//typedef struct __attribute__((packed)) _getACUInfoMsgResp
typedef struct _getACUInfoMsgResp
{
    msgHeader_t header;
    uint16_t    result;         // 0 - failed to get; 1 - successfully set
    uint16_t    acu_address;    // uint16_6 - ACU address
    uint32_t    OutL;     // meerstetter device type, see MeCom Protocol Specification 5117C.pdf
    uint32_t    WkErno;      // meerstetter h/w version
    uint32_t    Ver;      // meerstetter f/w version
    #ifdef __RUNNING_ON_CONTROLLINO__
//    uint16_t    pad;            // some Arduino black magic here
    #endif
    uint32_t    SerialNo;   // meerstetter serial number
    CRC         crc;            // 16 bit CRC over the packet
    EOP         eop;            // end of transmission character/byte
} getACUInfoMsgResp_t;
const uint16_t len_getACUInfoMsgResp_t = sizeof(getACUInfoMsgResp_t) - sizeof(CRC) - sizeof(EOP);


typedef struct _startUpCmd
{
    msgHeader_t header;
    CRC         crc;            // 16 bit CRC over the packet
    EOP         eop;            // end of transmission character/byte
} startUpCmd_t;
const uint16_t len_startUpCmd_t = sizeof(startUpCmd_t) - sizeof(CRC) - sizeof(EOP);


typedef struct _startUpCmdResp
{
    msgHeader_t header;
    uint16_t    result;         // 0 - failed to set; 1 - successfully set
    CRC         crc;            // 16 bit CRC over the packet
    EOP         eop;            // end of transmission character/byte
} startUpCmdResp_t;
const uint16_t len_startUpCmdResp_t = sizeof(startUpCmdResp_t) - sizeof(CRC) - sizeof(EOP);


typedef struct _startUpATCmd
{
    msgHeader_t header;
    CRC         crc;            // 16 bit CRC over the packet
    EOP         eop;            // end of transmission character/byte
} startUpATCmd_t;
const uint16_t len_startUpATCmd_t = sizeof(startUpATCmd_t) - sizeof(CRC) - sizeof(EOP);


typedef struct _startUpATCmdResp
{
    msgHeader_t header;
    uint16_t    result;         // 0 - failed to set; 1 - successfully set
    CRC         crc;            // 16 bit CRC over the packet
    EOP         eop;            // end of transmission character/byte
} startUpATCmdResp_t;
const uint16_t len_startUpATCmdResp_t = sizeof(startUpATCmdResp_t) - sizeof(CRC) - sizeof(EOP);


typedef struct _shutDownCmd
{
    msgHeader_t header;
    CRC         crc;            // 16 bit CRC over the packet
    EOP         eop;            // end of transmission character/byte
} shutDownCmd_t;
const uint16_t len_shutDownCmd_t = sizeof(shutDownCmd_t) - sizeof(CRC) - sizeof(EOP);


typedef struct _shutDownCmdResp
{
    msgHeader_t header;
    uint16_t    result;         // 0 - failed to set; 1 - successfully set
    CRC         crc;            // 16 bit CRC over the packet
    EOP         eop;            // end of transmission character/byte
} shutDownCmdResp_t;
const uint16_t len_shutDownCmdResp_t = sizeof(shutDownCmdResp_t) - sizeof(CRC) - sizeof(EOP);


typedef struct _setRTCCmd
{
    msgHeader_t header;
    timeind  tv;       // the timeval payload to set the RTC on the Controllino
    CRC     crc;      // 16 bit CRC over the packet
    EOP     eop;      // end of transmission character/byte
} setRTCCmd_t;
const uint16_t len_setRTCCmd_t = sizeof(setRTCCmd_t) - sizeof(CRC) - sizeof(EOP);


typedef struct _setRTCCmdResp
{
    msgHeader_t header;
    uint16_t    result;         // 0 - failed to set; 1 - successfully set
    CRC         crc;            // 16 bit CRC over the packet
    EOP         eop;            // end of transmission character/byte
} setRTCCmdResp_t;
const uint16_t len_setRTCCmdResp_t = sizeof(setRTCCmdResp_t) - sizeof(CRC) - sizeof(EOP);


typedef struct _getRTCCmd
{
    msgHeader_t header;
    CRC     crc;      // 16 bit CRC over the packet
    EOP     eop;      // end of transmission character/byte
} getRTCCmd_t;
const uint16_t len_getRTCCmd_t = sizeof(getRTCCmd_t) - sizeof(CRC) - sizeof(EOP);


typedef struct _getRTCCmdResp
{
    msgHeader_t header;
    uint16_t    result; // 0 - failed to get; 1 - successfully get
    timeind     tv;     // the timeval payload to get the RTC on the Controllino
    CRC         crc;    // 16 bit CRC over the packet
    EOP         eop;    // end of transmission character/byte
} getRTCCmdResp_t;
const uint16_t len_getRTCCmdResp_t = sizeof(getRTCCmdResp_t) - sizeof(CRC) - sizeof(EOP);


typedef struct _clrEventLogCmd
{
    msgHeader_t header;
    CRC         crc;            // 16 bit CRC over the packet
    EOP         eop;            // end of transmission character/byte
} clrEventLogCmd_t;
const uint16_t len_clrEventLogCmd_t = sizeof(clrEventLogCmd_t) - sizeof(CRC) - sizeof(EOP);


typedef struct _clrEventLogCmdResp
{
    msgHeader_t header;
    uint16_t    result;         // 0 - failed to set; 1 - successfully set
    CRC         crc;            // 16 bit CRC over the packet
    EOP         eop;            // end of transmission character/byte
} clrEventLogCmdResp_t;
const uint16_t len_clrEventLogCmdResp_t = sizeof(clrEventLogCmdResp_t) - sizeof(CRC) - sizeof(EOP);


typedef struct _getEventLogCmd
{
    msgHeader_t header;
    CRC         crc;            // 16 bit CRC over the packet
    EOP         eop;            // end of transmission character/byte
} getEventLogCmd_t;
const uint16_t len_getEventLogCmd_t = sizeof(getEventLogCmd_t) - sizeof(CRC) - sizeof(EOP);


typedef struct _getEventLogCmdResp
{
    msgHeader_t header;
    uint16_t    result;         // 0 - failed to set; 1 - successfully set
    elogentry   eventlog[MAX_ELOG_ENTRY];       // the whole event log !
    CRC         crc;            // 16 bit CRC over the packet
    EOP         eop;            // end of transmission character/byte
} getEventLogCmdResp_t;
const uint16_t len_getEventLogCmdResp_t = sizeof(getEventLogCmdResp_t) - sizeof(CRC) - sizeof(EOP);


typedef struct _setH20AlarmASIC
{
    msgHeader_t header;
    uint16_t    temperature[MAX_CHILLER_TEMP_LENGH];    // float in 32 bits
    CRC         crc;            // 16 bit CRC over the packet
    EOP         eop;            // end of transmission character/byte
} setH20AlarmASIC_t;
const uint16_t len_setH20AlarmASIC_t    = sizeof(setH20AlarmASIC_t) - sizeof(CRC) - sizeof(EOP);


typedef struct _setH20AlarmASICResp
{
    msgHeader_t header;
    uint16_t    result;         // 0 - fail ; 1 - success
    CRC         crc;            // 16 bit CRC over the packet
    EOP         eop;            // end of transmission character/byte
} setH20AlarmASICResp_t;
const uint16_t len_setH20AlarmASICResp_t = sizeof(setH20AlarmASICResp_t) - sizeof(CRC) - sizeof(EOP);


typedef struct _setH20AlarmDDR
{
    msgHeader_t header;
    uint16_t    temperature[MAX_CHILLER_TEMP_LENGH];    // float in 32 bits
    CRC         crc;            // 16 bit CRC over the packet
    EOP         eop;            // end of transmission character/byte
} setH20AlarmDDR_t;
const uint16_t len_setH20AlarmDDR_t    = sizeof(setH20AlarmDDR_t) - sizeof(CRC) - sizeof(EOP);


typedef struct _setH20AlarmDDRResp
{
    msgHeader_t header;
    uint16_t    result;         // 0 - fail ; 1 - success
    CRC         crc;            // 16 bit CRC over the packet
    EOP         eop;            // end of transmission character/byte
} setH20AlarmDDRResp_t;
const uint16_t len_setH20AlarmDDRResp_t = sizeof(setH20AlarmDDRResp_t) - sizeof(CRC) - sizeof(EOP);


typedef struct _getH20AlarmASIC
{
    msgHeader_t header;
    CRC         crc;            // 16 bit CRC over the packet
    EOP         eop;            // end of transmission character/byte
} getH20AlarmASIC_t;
const uint16_t len_getH20AlarmASIC_t = sizeof(getH20AlarmASIC_t) - sizeof(CRC) - sizeof(EOP);


typedef struct _getH20AlarmASICResp
{
    msgHeader_t header;
    uint8_t     temperature[MAX_CHILLER_TEMP_LENGH];    // float in 32 bits
    CRC         crc;            // 16 bit CRC over the packet
    EOP         eop;            // end of transmission character/byte
} getH20AlarmASICResp_t;
const uint16_t len_getH20AlarmASICResp_t = sizeof(getH20AlarmASICResp_t) - sizeof(CRC) - sizeof(EOP);


typedef struct _getH20AlarmDDR
{
    msgHeader_t header;
    CRC         crc;            // 16 bit CRC over the packet
    EOP         eop;            // end of transmission character/byte
} getH20AlarmDDR_t;
const uint16_t len_getH20AlarmDDR_t = sizeof(getH20AlarmDDR_t) - sizeof(CRC) - sizeof(EOP);


typedef struct _getH20AlarmDDRResp
{
    msgHeader_t header;
    uint8_t     temperature[MAX_CHILLER_TEMP_LENGH];    // float in 32 bits
    CRC         crc;            // 16 bit CRC over the packet
    EOP         eop;            // end of transmission character/byte
} getH20AlarmDDRResp_t;
const uint16_t len_getH20AlarmDDRResp_t = sizeof(getH20AlarmDDRResp_t) - sizeof(CRC) - sizeof(EOP);


typedef struct _NACK
{
    msgHeader_t header;
    CRC         crc;            // 16 bit CRC over the packet
    EOP         eop;            // end of transmission character/byte
} NACK_t;
const uint16_t len_NACK_t = sizeof(NACK_t) - sizeof(CRC) - sizeof(EOP);


class controlProtocol
{
    public:
    //
    // define function pointers to handle USB and Serial interfaces
    // called like (this.*TxCommand)()
    //
    bool    (controlProtocol::*TxCommand)(uint16_t);
    bool    (controlProtocol::*RxResponse)(uint16_t);
    bool    (controlProtocol::*RxCommand)(uint16_t);
    bool    (controlProtocol::*TxResponse)(uint16_t);

    //
    // functions to hide the member function pointer syntax
    //
    bool    doRxCommand(uint16_t TimeoutMs);
    bool    doTxResponse(uint16_t length);
    bool    doTxCommand(uint16_t length) { return( (this->*TxCommand)(length) ); };
    bool    doRxResponse(uint16_t timeout) { return( (this->*RxResponse)(timeout) ); };

    #ifdef __RUNNING_ON_CONTROLLINO__
    controlProtocol(HardwareSerial&, uint16_t, uint16_t);       // serial : m_myAddress, m_peerAddress
    #else
    controlProtocol(uint16_t, uint16_t, const char*, uint32_t); // USB : m_myAddress, m_peerAddress, USB file
    #endif
    ~controlProtocol();
    
    bool    StartUpCmd(uint16_t);
    bool    StartUpATCmd(uint16_t);
    bool    ShutDownCmd(uint16_t);
    bool    GetStatus(uint16_t, uint16_t*, uint16_t*, uint16_t*);
    bool    SetACUTemperature(uint16_t, uint16_t, float);
    bool    GetACUTemperature(uint16_t, uint16_t, uint16_t*, float*);
    bool    GetACUObjTemperature(uint16_t, uint16_t, uint16_t*, float*);
    bool    StartChiller(uint16_t);
    bool    StopChiller(uint16_t);
    bool    GetChillerInfo(uint16_t, char*, uint8_t);
    bool    SetChillerTemperature(uint16_t, float);
    bool    GetChillerTemperature(uint16_t, float*);
    bool    GetChillerObjTemperature(uint16_t, float*);
    bool    EnableACUs(uint16_t);
    bool    DisableACUs(uint16_t);
    bool    GetACUInfo(uint16_t, uint16_t, uint32_t*, uint32_t*, uint32_t*, uint32_t*);
    bool    SetRTCCmd(uint16_t);
    bool    GetRTCCmd(uint16_t, struct tm*);
    bool    ClrEventLogCmd(uint16_t);
    bool    GetEventLogCmd(uint16_t, elogentry*);
    bool    SetH20AlarmASIC(uint16_t, float);
    bool    GetH20AlarmASIC(uint16_t, float*);
    bool    SetH20AlarmDDR(uint16_t, float);
    bool    GetH20AlarmDDR(uint16_t, float*);


    // master - control/test PC USB or serial interface
    bool        TxCommandUSB(uint16_t);    // uses m_buff and m_seqNum
    bool        RxResponseUSB(uint16_t);   // uses m_buff and m_seqNum, msec to wait for response
    // implement these later if needed
    //bool        TxCommandSerial();            // uses m_buff and m_seqNum
    //bool        RxResponseSerial(uint16_t);   // uses m_buff and m_seqNum, msec to wait for response

    // slave - Controllino uC 
    bool        RxCommandSerial(uint16_t);            // uses m_buff and m_seqNum
    bool        TxResponseSerial(uint16_t);           // uses m_buff and m_seqNum

    #ifdef __RUNNING_ON_CONTROLLINO__
    HardwareSerial& m_so;                       // serial object
    #endif
    uint16_t    m_seqNum;                       // current m_seqNum
    uint16_t    m_myAddress;                    // 'my' Address
    uint16_t    m_peerAddress;                  // peer address, 1 to 1 communication
    uint8_t     m_buff[MAX_BUFF_LENGTH_CP + 1]; // work m_buffer used by all functions
    
    #if defined(__USING_LINUX_USB__)
    int         m_fd;                           // for the USB port
    #endif
    
    #if defined(__USING_WINDOWS_USB__)
    HANDLE      m_fd;
    #endif

    bool        openUSBPort(const char*, uint32_t);
    bool        verifyMessage(uint16_t, uint16_t, uint16_t, EOP);
    bool        verifyMessage(uint16_t, uint16_t, EOP);
    bool        verifyMessageSeqNum(uint16_t, uint16_t);
    bool        verifyMessageCRC(uint16_t, uint16_t);
    bool        verifyMessageLength(EOP);
    uint16_t    getMsgId();

    uint16_t    Make_startUpCmd(uint16_t, uint8_t*);
    uint16_t    Make_startUpCmdResp(uint16_t, uint8_t*, uint16_t, uint16_t);
    void        Parse_startUpCmdResp(uint8_t*, uint16_t*, uint16_t*);

    uint16_t    Make_startUpATCmd(uint16_t, uint8_t*);
    uint16_t    Make_startUpATCmdResp(uint16_t, uint8_t*, uint16_t, uint16_t);
    void        Parse_startUpATCmdResp(uint8_t*, uint16_t*, uint16_t*);

    uint16_t    Make_shutDownCmd(uint16_t, uint8_t*);
    uint16_t    Make_shutDownCmdResp(uint16_t, uint8_t*, uint16_t, uint16_t);
    void        Parse_shutDownCmdResp(uint8_t*, uint16_t*, uint16_t*);

    uint16_t    Make_getStatus(uint16_t, uint8_t*);
    uint16_t    Make_getStatusResp(uint16_t, uint8_t*, uint16_t, uint16_t, uint16_t, uint16_t);
    void        Parse_getStatusResp(uint8_t*, uint16_t*, uint16_t*, uint16_t*, uint16_t*);

    uint16_t    Make_setACUTemperature(uint16_t, uint8_t*, uint16_t, float);
    uint16_t    Make_setACUTemperatureResp(uint16_t, uint8_t*, uint16_t, uint16_t, uint16_t);
    void        Parse_setACUTemperatureResp(uint8_t*, uint16_t*, uint16_t*);

    uint16_t    Make_getACUTemperature(uint16_t, uint8_t*, uint16_t);
    uint16_t    Make_getACUTemperatureResp(uint16_t, uint8_t*, uint16_t, uint16_t, float, uint16_t);
    void        Parse_getACUTemperatureResp(uint8_t*, uint16_t*, float*, uint16_t*);

    uint16_t    Make_getACUObjTemperature(uint16_t, uint8_t*, uint16_t);
    uint16_t    Make_getACUObjTemperatureResp(uint16_t, uint8_t*, uint16_t, uint16_t, float, uint16_t);
    void        Parse_getACUObjTemperatureResp(uint8_t*, uint16_t*, float*, uint16_t*);

    uint16_t    Make_getACUInfoMsg(uint16_t, uint8_t*, uint16_t);
    uint16_t    Make_getACUInfoMsgResp(uint16_t, uint8_t*, uint16_t, uint16_t, uint32_t, 
                                            uint32_t, uint32_t, uint32_t, uint16_t);
    void        Parse_getACUInfoMsgResp(uint8_t*, uint16_t*, uint32_t*, uint32_t*, uint32_t*,
                                                                uint32_t*, uint16_t*);

    uint16_t    Make_enableACUs(uint16_t, uint8_t*);
    uint16_t    Make_enableACUsResp(uint16_t, uint8_t*, uint16_t, uint16_t);
    void        Parse_enableACUsResp(uint8_t*, uint16_t*, uint16_t*);

    uint16_t    Make_disableACUs(uint16_t, uint8_t*);
    uint16_t    Make_disableACUsResp(uint16_t, uint8_t*, uint16_t, uint16_t);
    void        Parse_disableACUsResp(uint8_t*, uint16_t*, uint16_t*);

    uint16_t    Make_startChillerMsg(uint16_t, uint8_t*);
    uint16_t    Make_startChillerMsgResp(uint16_t, uint8_t*, uint16_t, uint16_t);
    void        Parse_startChillerMsgResp(uint8_t*, uint16_t*, uint16_t*);

    uint16_t    Make_stopChiller(uint16_t, uint8_t*);
    uint16_t    Make_stopChillerResp(uint16_t, uint8_t*, uint16_t, uint16_t);
    void        Parse_stopChillerResp(uint8_t*, uint16_t*, uint16_t*);

    uint16_t    Make_getChillerInfo(uint16_t, uint8_t*);
    uint16_t    Make_getChillerInfoResp(uint16_t, uint8_t*, uint16_t, uint8_t*, uint8_t, uint16_t);
    void        Parse_getChillerInfoResp(uint8_t*, uint16_t*, char*, uint8_t, uint16_t*);

    uint16_t    Make_setChillerTemperature(uint16_t, uint8_t*, float);
    uint16_t    Make_setChillerTemperatureResp(uint16_t, uint8_t*, uint16_t, uint16_t);
    void        Parse_setChillerTemperatureResp(uint8_t*, uint16_t*, uint16_t*);

    uint16_t    Make_getChillerTemperature(uint16_t, uint8_t*);
    uint16_t    Make_getChillerTemperatureResp(uint16_t, uint8_t*, float, uint16_t);
    void        Parse_getChillerTemperatureResp(uint8_t*, float*, uint16_t*);

    uint16_t    Make_getChillerObjTemperature(uint16_t, uint8_t*);
    uint16_t    Make_getChillerObjTemperatureResp(uint16_t, uint8_t*, float, uint16_t);
    void        Parse_getChillerObjTemperatureResp(uint8_t*, float*, uint16_t*);

    uint16_t    Make_setRTCCmd(uint16_t, uint8_t*, struct tm*);
    uint16_t    Make_setRTCCmdResp(uint16_t, uint8_t*, uint16_t, uint16_t);
    void        Parse_setRTCCmdResp(uint8_t*, uint16_t*, uint16_t*);

    uint16_t    Make_getRTCCmd(uint16_t, uint8_t*);
    uint16_t    Make_getRTCCmdResp(uint16_t, uint8_t*, timeind*, uint16_t, uint16_t);
    void        Parse_getRTCCmdResp(uint8_t*, uint16_t*, struct tm*, uint16_t*);

    uint16_t    Make_clrEventLogCmd(uint16_t, uint8_t*);
    uint16_t    Make_clrEventLogCmdResp(uint16_t, uint8_t*, uint16_t, uint16_t);
    void        Parse_clrEventLogCmdResp(uint8_t*, uint16_t*, uint16_t*);

    uint16_t    Make_getEventLogCmd(uint16_t, uint8_t*);
    uint16_t    Make_getEventLogCmdResp(uint16_t, uint8_t*, uint16_t, const elogentry*, uint16_t);
    void        Parse_getEventLogCmdResp(uint8_t*, uint16_t*, elogentry*, uint16_t*);

    uint16_t    Make_setH20AlarmASIC(uint16_t, uint8_t*, float);
    uint16_t    Make_setH20AlarmASICResp(uint16_t, uint8_t*, uint16_t, uint16_t);
    void        Parse_setH20AlarmASICResp(uint8_t*, uint16_t*, uint16_t*);

    uint16_t    Make_setH20AlarmDDR(uint16_t, uint8_t*, float);
    uint16_t    Make_setH20AlarmDDRResp(uint16_t, uint8_t*, uint16_t, uint16_t);
    void        Parse_setH20AlarmDDRResp(uint8_t*, uint16_t*, uint16_t*);

    uint16_t    Make_getH20AlarmASIC(uint16_t, uint8_t*);
    uint16_t    Make_getH20AlarmASICResp(uint16_t, uint8_t*, float, uint16_t);
    void        Parse_getH20AlarmASICResp(uint8_t*, float*, uint16_t*);

    uint16_t    Make_getH20AlarmDDR(uint16_t, uint8_t*);
    uint16_t    Make_getH20AlarmDDRResp(uint16_t, uint8_t*, float, uint16_t);
    void        Parse_getH20AlarmDDRResp(uint8_t*, float*, uint16_t*);

    uint16_t    Make_NACK(uint16_t, uint8_t*, uint16_t);    // always for command not supported/recognized
};

#endif
