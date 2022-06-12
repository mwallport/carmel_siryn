#ifndef __EVENTS__
#define __EVENTS__

//
// event log
//
// the low 16 bits is the event log number
// the high 16 bits is the instance
//  i.e. is ACU 2 is not would be  instance 0x02 | event id ACUNotOnLine 0x0010 = 0x0210
//
const uint16_t    ACUNotOnLine            = 0x0010; // ACUNotOnLine - parameter will indicate which one
const uint16_t    ACUNotRunning           = 0x0011; // ACUNotRunning - paramater will indicate which one
const uint16_t    ACUIsMismatch           = 0x0012; // ACUIsMismatch - parameter will indicate which one
const uint16_t    ASIC_RTDFault           = 0x0020; // RTD has non-zero fault
const uint16_t    ASIC_Chiller_RTDFault   = 0x0021; // RTD has non-zero fault
const uint16_t    DDR_RTDFault            = 0x0022; // RTD has non-zero fault
const uint16_t    DDR_Chiller_RTDFault    = 0x0023; // RTD has non-zero fault
const uint16_t    ASIC_Chiller_RTDHot     = 0x0024; // ASIC chiller water too hot
const uint16_t    DDR_Chiller_RTDHot      = 0x0025; // DDR chiller water too hot
const uint16_t    ChillerOffline          = 0x0030; // ChillerOffline
const uint16_t    ChillerNotRunning       = 0x0031; // ChillerNotRunning
const uint16_t    HumidityHigh            = 0x0040; // high humidity
const uint16_t    HumiditySensorFail      = 0x0041; // humiditiy sensor failed

#endif
