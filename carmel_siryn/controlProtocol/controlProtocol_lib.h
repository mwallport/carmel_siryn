#pragma once

//#define __USING_LINUX_USB__
#define __USING_WINDOWS_USB__

#if defined __USING_WINDOWS_USB__
#ifdef CONTROLPROTOCOL_LIB_EXPORTS
#define CONTROLPROTOCOL_LIB_API __declspec(dllexport)
#else
#define CONTROLPROTOCOL_LIB_API __declspec(dllimport)
#endif
#endif

#include <inttypes.h>
#include "eventlog.h"

//
// RS485 bus Ids
//
#define   ASIC_RS485_ID   1
#define   DDR_RS485_ID    2


//
// get_Status() bit maps
//
#define ASIC_THERMAL_CTRL_OFFLINE       0x1000
#define ASIC_THERMAL_CTRL_NOT_RUNNING   0x0100
#define DDR_THERMAL_CTRL_OFFLINE        0x0010
#define DDR_THERMAL_CTRL_NOT_RUNNING    0x0001
#define ASIC_CHILLER_RTD_HAS_FAULTS     0x1000
#define DDR1_RTD_HAS_FAULTS             0x0100
#define DDR2_RTD_HAS_FAULTS             0x0010
#define DDR_CHILLER_RTD_HAS_FAULTS      0x0001


//
// the number of event log entries stored in the control unit (slave)
//
#define   MAX_ELOG_ENTRY  5

//
// event log constanst
// 
// fololowing constants are used to decode the eventlog that is stored on the control unit
//
// the low 16 bits is the event log number
// the high 16 bits is the instance, i.e. which RTD, which thermal control unit, etc.
//  i.e. is ACU 2 is not would be  instance 0x02 | event id ACUNotOnLine 0x0010 = 0x0210
//
const uint16_t    ACUNotOnLine = 0x0010; // ACUNotOnLine - parameter will indicate which one
const uint16_t    ACUNotRunning = 0x0011; // ACUNotRunning - paramater will indicate which one
const uint16_t    ACUIsMismatch = 0x0012; // ACUIsMismatch - parameter will indicate which one
const uint16_t    ASIC_RTDFault = 0x0020; // RTD has non-zero fault
const uint16_t    ASIC_Chiller_RTDFault = 0x0021; // RTD has non-zero fault
const uint16_t    DDR_RTDFault = 0x0022; // RTD has non-zero fault
const uint16_t    DDR_Chiller_RTDFault = 0x0023; // RTD has non-zero fault
const uint16_t    ASIC_Chiller_RTDHot = 0x0024; // ASIC chiller water too hot
const uint16_t    DDR_Chiller_RTDHot = 0x0025; // DDR chiller water too hot
const uint16_t    ChillerOffline = 0x0030; // ChillerOffline
const uint16_t    ChillerNotRunning = 0x0031; // ChillerNotRunning
const uint16_t    HumidityHigh = 0x0040; // high humidity
const uint16_t    HumiditySensorFail = 0x0041; // humiditiy sensor failed


/*
* set the host address - keep always 0 for now
*/
#if defined __USING_WINDOWS_USB__
extern "C" CONTROLPROTOCOL_LIB_API void set_MyAddress(uint16_t);
#else
extern "C" void set_MyAddress(uint16_t);
#endif

/*
* set the slave address - keep always as 1 for now
* 
* always 1 for now
* the slave is currently hard coded as Id 1 and exptects the master to be Id 0
*/
#if defined __USING_WINDOWS_USB__
extern "C" CONTROLPROTOCOL_LIB_API void set_PeerAddress(uint16_t);
#else
extern "C" void set_PeerAddress(uint16_t);
#endif

/*
* set the USB port for the master's connection to the slave
* 
* NOTE : on Windows, for COM port 10 and above, you need to open them with the
*   command \\.\\COMn, which corresponds to the C string \\\\.\\COMn, so
*    you would enter \\\\.\\COM11 for COM11
* 
* NOTE : on Linux typically this will be something like /dev/tty/USB0 or /dev/tty/USB1
* 
*/
#if defined __USING_WINDOWS_USB__
extern "C" CONTROLPROTOCOL_LIB_API void set_UsbPort(const char*);
#else
extern "C" void set_UsbPort(const char*);
#endif

/*
* USB port speed for the connection to the slave - keep alwasy as 19200 for now
* 
* the slave is programmed for 19200 BUAD N81
* 
*/
#if defined __USING_WINDOWS_USB__
extern "C" CONTROLPROTOCOL_LIB_API void set_Speed(uint32_t);
#else
extern "C" void set_Speed(uint32_t);
#endif

/*
* write the connection parameters to stdout
*/
#if defined __USING_WINDOWS_USB__
extern "C" CONTROLPROTOCOL_LIB_API void showConnParams(void);
#else
extern "C" void showConnParams(void);
#endif

/*
* get the the system status, returns the following
* 
* RTD faults for
* - DDR1 RTD
* - DDR2 RTD
* - DDR Chiller RTD
* 
* Thermal control
* - ASIC offlline or online, running or not running
* - DDR offline or online, running or not running
* 
*/
#if defined __USING_WINDOWS_USB__
extern "C" CONTROLPROTOCOL_LIB_API bool get_Status(uint16_t*, uint16_t*, uint16_t*);
#else
extern "C" bool get_Status(uint16_t*, uint16_t*, uint16_t*);
#endif

/*
*  start both thermal controllers, not auto tune mode
*/
#if defined __USING_WINDOWS_USB__
extern "C" CONTROLPROTOCOL_LIB_API bool startUp(void);
#else
extern "C" bool startUp(void);
#endif

/*
*  start both thermal controller, auto tune mode
*/
#if defined __USING_WINDOWS_USB__
extern "C" CONTROLPROTOCOL_LIB_API bool startUpAT(void);
#else
extern "C" bool startUpAT(void);
#endif

/*
*  stop the thermal controllers
*/
#if defined __USING_WINDOWS_USB__
extern "C" CONTROLPROTOCOL_LIB_API bool shutDown(void);
#else
extern "C" bool shutDown(void);
#endif

/*
*  set the set value for the ASIC thermal control, celcius
*/
#if defined __USING_WINDOWS_USB__
extern "C" CONTROLPROTOCOL_LIB_API bool set_SVASIC(float temp);
#else
extern "C" bool set_SVASIC(float temp);
#endif

/*
*  get the set value for the ASIC thermal control, celcius
*/
#if defined __USING_WINDOWS_USB__
extern "C" CONTROLPROTOCOL_LIB_API bool get_SVASIC(float* temp);
#else
extern "C" bool get_SVASIC(float* temp);
#endif

/*
*  set the set value for the DDR thermal control, celcius
*/
#if defined __USING_WINDOWS_USB__
extern "C" CONTROLPROTOCOL_LIB_API bool set_SVDDR(float temp);
#else
extern "C" bool set_SVDDR(float temp);
#endif

/*
*  get the set value for the DDR thermal control, celcius
*/
#if defined __USING_WINDOWS_USB__
extern "C" CONTROLPROTOCOL_LIB_API bool get_SVDDR(float* temp);
#else
extern "C" bool get_SVDDR(float* temp);
#endif

/*
*  get the process value, actual current temperatur of the ASIC thermal control, celcius
*/
#if defined __USING_WINDOWS_USB__
extern "C" CONTROLPROTOCOL_LIB_API bool get_PVASIC(float* temp);
#else
extern "C" bool get_PVASIC(float* temp);
#endif

/*
*  get the process value, actual current temperature of the DDR thermal control, celcius
*/
#if defined __USING_WINDOWS_USB__
extern "C" CONTROLPROTOCOL_LIB_API bool get_PVDDR(float* temp);
#else
extern "C" bool get_PVDDR(float* temp);
#endif

/*
*  start both thermal controllers, not auto-tune, puts the system in RUNNING state
* 
*  recommend using the startUp() or startUpAT() commands instead
*/
#if defined __USING_WINDOWS_USB__
extern "C" CONTROLPROTOCOL_LIB_API bool do_enableACUs(void);
#else
extern "C" bool do_enableACUs(void);
#endif

/*
*  stop both thermal control units, puts the system in READY state
* 
*  recommend using the shutDown() command instead
*/
#if defined __USING_WINDOWS_USB__
extern "C" CONTROLPROTOCOL_LIB_API bool do_disableACUs(void);
#else
extern "C" bool do_disableACUs(void);
#endif

/*
*  set the real time clock in the slave to the time on 'this' computer
*/
#if defined __USING_WINDOWS_USB__
extern "C" CONTROLPROTOCOL_LIB_API bool set_RTC(void);
#else
extern "C" bool set_RTC(void);
#endif

/*
*  get the real time clock on the slave
*/
#if defined __USING_WINDOWS_USB__
extern "C" CONTROLPROTOCOL_LIB_API bool get_RTC(struct tm* ltime);
#else
extern "C" bool get_RTC(struct tm* ltime);
#endif

/*
*  clear the event log
*/
#if defined __USING_WINDOWS_USB__
extern "C" CONTROLPROTOCOL_LIB_API bool clr_EventLog(void);
#else
extern "C" bool clr_EventLog(void);
#endif

/*
*  get the event log
*/
#if defined __USING_WINDOWS_USB__
extern "C" CONTROLPROTOCOL_LIB_API bool get_EventLog(char** eventlog);
#else
extern "C" bool get_EventLog(char** eventlog);
#endif

/*
*  set the alarm temperature for the ASIC thermal control
* 
*  if the actual temperature gets to this value, the system will go to SHUTDOWN
*  state and an event will be logged
*/
#if defined __USING_WINDOWS_USB__
extern "C" CONTROLPROTOCOL_LIB_API bool set_H20AlarmASIC(float temp);
#else
extern "C" bool set_H20AlarmASIC(float temp);
#endif

/*
*  get the alarm temperature for the ASIC thermal control
*
*/
#if defined __USING_WINDOWS_USB__
extern "C" CONTROLPROTOCOL_LIB_API bool get_H20AlarmASIC(float* temp);
#else
extern "C" bool get_H20AlarmASIC(float* temp);
#endif

/*
*  set the alarm temperature for the DDR thermal control
*
*  if the actual temperature gets to this value, the system will go to SHUTDOWN
*  state and an event will be logged
*/
#if defined __USING_WINDOWS_USB__
extern "C" CONTROLPROTOCOL_LIB_API bool set_H20AlarmDDR(float temp);
#else
extern "C" bool set_H20AlarmDDR(float temp);
#endif

/*
*  get the alarm temperature for the DDR thermal control
*
*/
#if defined __USING_WINDOWS_USB__
extern "C" CONTROLPROTOCOL_LIB_API bool get_H20AlarmDDR(float* temp);
#else
extern "C" bool get_H20AlarmDDR(float* temp);
#endif


