#ifndef __MENU__
#define __MENU__
#include <unistd.h>
#include <iostream>
#include <string>
#include <cstdint>
#include "controlProtocol.h"
#include "events.h"


//#define __USING_CHILLER__
//#define __USING_HUMIDITY__
    

using namespace std;

typedef bool (controlProtocol::*pStartUpCmd_t)(uint16_t);
typedef bool (controlProtocol::*pStartUpATCmd_t)(uint16_t);
typedef bool (controlProtocol::*pShutDownCmd_t)(uint16_t);
typedef bool (controlProtocol::*pGetStatus_t)(uint16_t, uint16_t*, uint16_t*, uint16_t*);
typedef bool (controlProtocol::*pGetHumidity_t)(uint16_t, float*);
typedef bool (controlProtocol::*pSetHumidityThreshold_t)(uint16_t, uint16_t);
typedef bool (controlProtocol::*pGetHumidityThreshold_t)(uint16_t, uint16_t*);
typedef bool (controlProtocol::*pSetACUTemperature_t)(uint16_t, uint16_t, float);
typedef bool (controlProtocol::*pGetACUTemperature_t)(uint16_t, uint16_t, uint16_t*, float*);
typedef bool (controlProtocol::*pGetACUObjTemperature_t)(uint16_t, uint16_t, uint16_t*, float*);
typedef bool (controlProtocol::*pStartChiller_t)(uint16_t);
typedef bool (controlProtocol::*pStopChiller_t)(uint16_t);
typedef bool (controlProtocol::*pGetChillerInfo_t)(uint16_t, char*, uint8_t);
typedef bool (controlProtocol::*pSetChillerTemperature_t)(uint16_t, float);
typedef bool (controlProtocol::*pGetChillerTemperature_t)(uint16_t, float*);
typedef bool (controlProtocol::*pGetChillerObjTemperature_t)(uint16_t, float*);
typedef bool (controlProtocol::*pEnableACUs_t)(uint16_t);
typedef bool (controlProtocol::*pDisableACUs_t)(uint16_t);
typedef bool (controlProtocol::*pGetACUInfo_t)(uint16_t, uint16_t, uint32_t*, uint32_t*, uint32_t*, uint32_t*);
typedef bool (controlProtocol::*pSetRTCCmd_t)(uint16_t);
typedef bool (controlProtocol::*pGetRTCCmd_t)(uint16_t, struct tm*);
typedef bool (controlProtocol::*pClrEventLogCmd_t)(uint16_t);
typedef bool (controlProtocol::*pGetEventLogCmd_t)(uint16_t, elogentry*);
typedef bool (controlProtocol::*pSetH20AlarmASIC_t)(uint16_t, float);
typedef bool (controlProtocol::*pGetH20AlarmASIC_t)(uint16_t, float*);
typedef bool (controlProtocol::*pSetH20AlarmDDR_t)(uint16_t, float);
typedef bool (controlProtocol::*pGetH20AlarmDDR_t)(uint16_t, float*);
typedef bool (controlProtocol::*pGetTempCmd_t)(uint16_t, uint16_t*);


class menuItemBase
{
    public:
    string  m_name;			            // name for the command to list in a menu
    string  m_description;	            // breif description of the command
    uint16_t m_destId;

    menuItemBase(const string& name, const string& description, const uint16_t destId = 1)
        : m_name(name), m_description(description), m_destId(destId) {};
    virtual ~menuItemBase() {};
    virtual bool getParameters(void) { return(true); };    // prompt for parameters for cmd - derived as needed
    void executeTest(controlProtocol* pCP) {cout.flush(); cout << "Not implemented.\n"; cout.flush(); };
    virtual void execute(controlProtocol*) = 0;
    friend ostream& operator<<(ostream& str, const menuItemBase& item)
    {
        str << setw(30) << item.m_name << ":" << item.m_description;
        return(str);
    }
    
    private:
    menuItemBase();
    menuItemBase(const menuItemBase&);
    menuItemBase& operator=(const menuItemBase&);
};


// bool    StartUpCmd(uint16_t);
class menuStartUpCmd : public menuItemBase
{
    public:
    pStartUpCmd_t m_pStartUpCmd  = &controlProtocol::StartUpCmd;

    menuStartUpCmd()
        :   menuItemBase("startup system", "start temp control"),     // TODO: add chiller back for other
            m_pStartUpCmd(&controlProtocol::StartUpCmd) {}

    void execute(controlProtocol* pCP)
    {
        if( (pCP->*m_pStartUpCmd)(m_destId) )
            cout << "\nstart temp control successful" << endl;
        else
            cout << "\nstart temp control failed" << endl;
    }
    
    private:
    menuStartUpCmd(const menuItemBase&);
    menuStartUpCmd& operator=(const menuItemBase&);
};


// bool    StartUpCmd(uint16_t);
class menuStartUpATCmd : public menuItemBase
{
    public:
    pStartUpATCmd_t m_pStartUpATCmd  = &controlProtocol::StartUpATCmd;

    menuStartUpATCmd()
        :   menuItemBase("autotune", "start autotune temp control"),     // TODO: add chiller back for other
            m_pStartUpATCmd(&controlProtocol::StartUpATCmd) {}

    void execute(controlProtocol* pCP)
    {
        if( (pCP->*m_pStartUpATCmd)(m_destId) )
            cout << "\nstart autotune temp control successful" << endl;
        else
            cout << "\nstart autotune temp control failed" << endl;
    }
    
    private:
    menuStartUpATCmd(const menuItemBase&);
    menuStartUpATCmd& operator=(const menuItemBase&);
};


// bool    ShutDownCmd(uint16_t);
class menuShutDownCmd : public menuItemBase
{
    public:
    pShutDownCmd_t m_pShutDownCmd;

    menuShutDownCmd()
        :   menuItemBase("shutdown system", "stop temp control"), // TODO : add back 'chiller not affected'
            m_pShutDownCmd(&controlProtocol::ShutDownCmd) {}

    void execute(controlProtocol* pCP)
    {
        if( (pCP->*m_pShutDownCmd)(m_destId) )
            cout << "\nstop temp control successful" << endl;
        else
            cout << "\nstop temp control failed" << endl;
    }
    
    private:
    menuShutDownCmd(const menuItemBase&);
    menuShutDownCmd& operator=(const menuItemBase&);
};


// bool    GetStatus(uint16_t, uint16_t*, uint16_t*, uint16_t*);
class menuGetStatus : public menuItemBase
{
    public:
    pGetStatus_t m_pGetStatus;

    menuGetStatus()
        :   menuItemBase("get status", "report TCU and RTD states"), // TODO add back 'humidity and chiller state'
            m_pGetStatus(&controlProtocol::GetStatus) {}

    void execute(controlProtocol* pCP)
    {
        if( (pCP->*m_pGetStatus)(m_destId, &RTDErrors, &ACUsRunning, &chiller_humidity) )
        {
          cout << endl;

          // ACUs output first 
          if( (ACUsRunning & (1 << 12)) )  // lame, but works
            cout << "ASIC ACU is OFFLINE";
          else
            cout << "ASIC ACU is ONLINE";

          if( (ACUsRunning & (1 << 8)) )
            cout << " and NOT RUNNING" << endl;
          else
            cout << " and RUNNING" << endl;

          if( (ACUsRunning & (1 << 4)) )  // lame, but works
            cout << "DDR ACU is OFFLINE";
          else
            cout << "DDR ACU is ONLINE";

          if( (ACUsRunning & (1)) )
            cout << " and NOT RUNNING" << endl;
          else
            cout << " and RUNNING" << endl;

          // RTD ouput next
          cout << "ASIC Chiller RTD";
          if( (RTDErrors & (1 << 12)) )
            cout << " has faults" << endl;
          else
            cout << " has no faults" << endl;

          cout << "DDR1 RTD";
          if( (RTDErrors & (1 << 12)) )
            cout << " has faults" << endl;
          else
            cout << " has no faults" << endl;

          cout << "DDR2 RTD";
          if( (RTDErrors & (1 << 8)) )
            cout << " has faults" << endl;
          else
            cout << " has no faults" << endl;

          cout << "DDR Chiller RTD";
          if( (RTDErrors & 1) )
            cout << " has faults" << endl;
          else
            cout << " has no faults" << endl;

          #ifdef __USING_CHILLER__
          if( (chiller_humidity & 0x1000) )
            cout << "chiller offline" << endl;
          else
            cout << "chiller online" << endl;

          if( (chiller_humidity & 0x0100) )
            cout << "chiller running" << endl;
          else
            cout << "chiller not running" << endl;
          #endif

          #ifdef __USING_HUMIDITY__
          if( (chiller_humidity & 0x0010) )
            cout << "humidity sensor offline" << endl;
          else
            cout << "humidity sensor online" << endl;

          if( (chiller_humidity & 0x0001) )
            cout << "humidity high" << endl;
          else
            cout << "humidity ok" << endl;
          #endif

        } else
        {
            cout << "\nunable to get status" << endl;
        }
    }

    uint16_t RTDErrors;
    uint16_t ACUsRunning;
    uint16_t chiller_humidity;
    
    private:
    menuGetStatus(const menuItemBase&);
    menuGetStatus& operator=(const menuItemBase&);
};


// bool    SetACUTemperature(uint16_t, uint16_t, float);
class menuSetACUTemperature : public menuItemBase
{
    public:
    pSetACUTemperature_t m_pSetACUTemperature;
    bool getParameters(void)
    {
        cout << "enter ACU address:  ";
        cin >> ACUAddress;
/*
        if( !(cin >> ACUAddress) )
        {
          cout << "bad input" << endl;
          return(false);
        }
*/
        cout << "enter temperature: ";
        cin >> temperature;
/*
        if( !(cin >> temperature) )
        {
          cout << "bad input" << endl;
          return(false);
        }
*/
        return(true);
    }

    menuSetACUTemperature()
        :   menuItemBase("set SV", "set target temp"),
            m_pSetACUTemperature(&controlProtocol::SetACUTemperature) {}

    void execute(controlProtocol* pCP)
    {
        if( (pCP->*m_pSetACUTemperature)(m_destId, ACUAddress, temperature) )
            cout << "\nset target temp succesful" << endl;
          else
            cout << "\nset target temp failed" << endl;
    }

    uint16_t ACUAddress;
    float temperature; 
    
    private:
    menuSetACUTemperature(const menuItemBase&);
    menuSetACUTemperature& operator=(const menuItemBase&);
};


//    bool    GetACUTemperature(uint16_t, uint16_t, uint16_t*, float*);
class menuGetACUTemperature : public menuItemBase
{
    public:
    pGetACUTemperature_t m_pGetACUTemperature;
    bool getParameters(void)
    {
        cout << "enter ACU address:  ";

        cin >> ACUAddress;
/*
        if( !(cin >> ACUAddress) )
        {
          cout << "bad input" << endl;
          return(false);
        }
*/

        return(true);
    }

    menuGetACUTemperature()
        :   menuItemBase("get SV", "get target temp"),
            m_pGetACUTemperature(&controlProtocol::GetACUTemperature) {}

    void execute(controlProtocol* pCP)
    {
        if( (pCP->*m_pGetACUTemperature)(m_destId, ACUAddress, &result, &temperature) )
        {
            if( (result) )
                cout << "\nSV temp: " << temperature << endl;
            else
                cout << "\nget SV failed to return temp" << endl;
        } else
            cout << "\nget SV temp failed" << endl;
    }

    uint16_t ACUAddress;
    uint16_t result;
    float temperature; 
    
    private:
    menuGetACUTemperature(const menuItemBase&);
    menuGetACUTemperature& operator=(const menuItemBase&);
};


class menuGetACUObjTemperature : public menuItemBase
{
    public:
    pGetACUObjTemperature_t m_pGetACUObjTemperature;
    
    bool getParameters(void)
    {
        cout << "enter ACU address:  ";

        cin >> ACUAddress;
/*
        if( !(cin >> ACUAddress) )
        {
          cout << "bad input" << endl;
          return(false);
        }
*/

        return(true);
    }

    menuGetACUObjTemperature()
        :   menuItemBase("get PV", "get actual temp"),
            m_pGetACUObjTemperature(&controlProtocol::GetACUObjTemperature) {}

    void execute(controlProtocol* pCP)
    {
        if( (pCP->*m_pGetACUObjTemperature)(m_destId, ACUAddress, &result, &temperature) )
        {
            if( (result) )
                cout << "\nactual temp: " << temperature << endl;
            else
                cout << "\nget PV failed to return temp" << endl;
        } else
            cout << "\nget PV failed" << endl;
    }

    uint16_t ACUAddress;
    uint16_t result;
    float temperature; 
    
    private:
    menuGetACUObjTemperature(const menuItemBase&);
    menuGetACUObjTemperature& operator=(const menuItemBase&);
};

//    bool    StartChiller(uint16_t);
class menuStartChiller : public menuItemBase
{
    public:
    pStartChiller_t m_pStartChiller;

    menuStartChiller()
        :   menuItemBase("start chiller", "start the chiller"),
            m_pStartChiller(&controlProtocol::StartChiller) {}

    void execute(controlProtocol* pCP)
    {
        if( (pCP->*m_pStartChiller)(m_destId) )
            cout << "\nstart chiller successful" << endl;
        else
            cout << "\nstart chiller failed" << endl;
    }
    
    private:
    menuStartChiller(const menuItemBase&);
    menuStartChiller& operator=(const menuItemBase&);
};


//    bool    StopChiller(uint16_t);
class menuStopChiller : public menuItemBase
{
    public:
    pStopChiller_t m_pStopChiller  = &controlProtocol::StopChiller;

    menuStopChiller()
        :   menuItemBase("stop chiller", "stop the chiller, also stops ACUs"),
            m_pStopChiller(&controlProtocol::StopChiller) {}

    void execute(controlProtocol* pCP)
    {
        if( (pCP->*m_pStopChiller)(m_destId) )
            cout << "\nstop chiller successful" << endl;
        else
            cout << "\nstop chiller failed" << endl;
    }
    
    private:
    menuStopChiller(const menuItemBase&);
    menuStopChiller& operator=(const menuItemBase&);
};


//    bool    GetChillerInfo(uint16_t, char*, uint8_t);
class menuGetChillerInfo : public menuItemBase
{
    public:
    pGetChillerInfo_t m_pGetChillerInfo;

    menuGetChillerInfo()
        :   menuItemBase("get chiller info", "get the chiller's name"),
            m_pGetChillerInfo(&controlProtocol::GetChillerInfo) {}

    void execute(controlProtocol* pCP)
    {
        if( (pCP->*m_pGetChillerInfo)(m_destId, chillerInfo, 64) )
            cout << "\nget chiller info: " << chillerInfo << endl;
        else
            cout << "\nget chiller info failed" << endl;
    }

    char chillerInfo[64];
    
    private:
    menuGetChillerInfo(const menuItemBase&);
    menuGetChillerInfo& operator=(const menuItemBase&);
};


//    bool    SetChillerTemperature(uint16_t, float);
class menuSetChillerTemperature : public menuItemBase
{
    public:
    pSetChillerTemperature_t m_pSetChillerTemperature  = &controlProtocol::SetChillerTemperature;
    bool getParameters(void)
    {
        cout << "enter temperature: ";

        //
        // check if cin got a float ??
        //
        cin >> temperature;
/*
        if( (cin.bad()) )
        {
          cout << "bad input" << endl;
        }
*/
        return(true);
    }

    menuSetChillerTemperature()
        :   menuItemBase("set chiller temperature", "set the chiller set-point temp"),
            m_pSetChillerTemperature(&controlProtocol::SetChillerTemperature) {}

    void execute(controlProtocol* pCP)
    {
        if( (pCP->*m_pSetChillerTemperature)(m_destId, temperature) )
            cout << "\nset chiller temperature successful" << endl;
        else
            cout << "\nset chiller temperature failed" << endl;
    }

    float   temperature;

    private:
    menuSetChillerTemperature(const menuItemBase&);
    menuSetChillerTemperature& operator=(const menuItemBase&);
};


//    bool    GetChillerTemperature(uint16_t, float*);
class menuGetChillerTemperature : public menuItemBase
{
    public:
    pGetChillerTemperature_t m_pGetChillerTemperature;

    menuGetChillerTemperature()
        :   menuItemBase("get chiller temperature", "get the chiller set-point temp"),
            m_pGetChillerTemperature(&controlProtocol::GetChillerTemperature) {}

    void execute(controlProtocol* pCP)
    {
        if( (pCP->*m_pGetChillerTemperature)(m_destId, &temperature) )
            cout << "\nchiller set-point temperature: " << temperature << endl;
        else
            cout << "\nget chiller temperature failed" << endl;
    }

    float   temperature;

    private:
    menuGetChillerTemperature(const menuItemBase&);
    menuGetChillerTemperature& operator=(const menuItemBase&);
};


class menuGetChillerObjTemperature : public menuItemBase
{
    public:
    pGetChillerObjTemperature_t m_pGetChillerObjTemperature;
    
    menuGetChillerObjTemperature()
        :   menuItemBase("get chiller obj temperature", "get the chiller internal temp"),
            m_pGetChillerObjTemperature(&controlProtocol::GetChillerObjTemperature) {}

    void execute(controlProtocol* pCP)
    {
        if( (pCP->*m_pGetChillerObjTemperature)(m_destId, &temperature) )
            cout << "\nchiller internal temperature: " << temperature << endl;
        else
            cout << "\nget chiller obj temperature failed" << endl;
    }

    float   temperature;

    private:
    menuGetChillerObjTemperature(const menuItemBase&);
    menuGetChillerObjTemperature& operator=(const menuItemBase&);
};


//    bool    EnableACUs(uint16_t);
class menuEnableACUs : public menuItemBase
{
    public:
    pEnableACUs_t m_pEnableACUs;

    menuEnableACUs()
        :   menuItemBase("enable", "start temp control"),
            m_pEnableACUs(&controlProtocol::EnableACUs) {}

    void execute(controlProtocol* pCP)
    {
        if( (pCP->*m_pEnableACUs)(m_destId) )
            cout << "\nstart temp control successful" << endl;
        else
            cout << "\nstart temp control failed" << endl;
    }
    
    private:
    menuEnableACUs(const menuItemBase&);
    menuEnableACUs& operator=(const menuItemBase&);
};


//    bool    DisableACUs(uint16_t);
class menuDisableACUs : public menuItemBase
{
    public:
    pDisableACUs_t m_pDisableACUs;

    menuDisableACUs()
        :   menuItemBase("disable", "stop temp control"),
            m_pDisableACUs(&controlProtocol::DisableACUs) {}

    void execute(controlProtocol* pCP)
    {
        if( (pCP->*m_pDisableACUs)(m_destId) )
            cout << "\nstop temp control successful" << endl;
        else
            cout << "\nstop temp control failed" << endl;
    }
    
    private:
    menuDisableACUs(const menuItemBase&);
    menuDisableACUs& operator=(const menuItemBase&);
};


//    bool    GetACUInfo(uint16_t, uint16_t, uint32_t*, uint32_t*, uint32_t*, uint32_t*);
class menuGetACUInfo : public menuItemBase
{
    public:
    pGetACUInfo_t m_pGetACUInfo;
    bool getParameters(void)
    {
        cout << "enter ACU address:  ";

        cin >> acu_address;
/*
        if( !(cin >> acu_address) )
        {
          cout << "bad input" << endl;
          return(false);
        }
*/

        return(true);
    }

    menuGetACUInfo()
        :   menuItemBase("get ACU info", "get a ACU's h/w & s/w version, model, and serial number"),
            m_pGetACUInfo(&controlProtocol::GetACUInfo) {}

    void execute(controlProtocol* pCP)
    {
        if( (pCP->*m_pGetACUInfo)(m_destId, acu_address, &OutL, &WkErno,
                                &Ver, &SerialNo) )
        {
            cout << "\nOutL: " << OutL <<
                    " WkErno: " << WkErno <<
                    " fwVersion: " << Ver <<
                    " serialNum: " << SerialNo << endl;
        } else
        {
            cout << "\nget ACU info failed" << endl;
        }
    }

    uint16_t acu_address;
    uint32_t OutL;
    uint32_t WkErno;
    uint32_t Ver;
    uint32_t SerialNo;
    
    private:
    menuGetACUInfo(const menuItemBase&);
    menuGetACUInfo& operator=(const menuItemBase&);
};


class menuSetRTCCmd : public menuItemBase
{
    public:
    pSetRTCCmd_t m_pSetRTCCmd;

    menuSetRTCCmd()
        :   menuItemBase("set clock", "set the clock"),
            m_pSetRTCCmd(&controlProtocol::SetRTCCmd) {}

    void execute(controlProtocol* pCP)
    {   
        if( (pCP->*m_pSetRTCCmd)(m_destId) )
            cout << "\nset clock successful" << endl;
        else
            cout << "\nset clock failed" << endl;
    }   
    
    private:
    menuSetRTCCmd(const menuItemBase&);
    menuSetRTCCmd& operator=(const menuItemBase&);
};


// bool    GetRTCCmd(uint16_t);
class menuGetRTCCmd : public menuItemBase
{
    public:
    pGetRTCCmd_t m_pGetRTCCmd;

    menuGetRTCCmd()
        :   menuItemBase("get clock", "get the clock"),
            m_pGetRTCCmd(&controlProtocol::GetRTCCmd) {}

    void execute(controlProtocol* pCP)
    {   
        if( (pCP->*m_pGetRTCCmd)(m_destId, &ltime) )
        {
            ltime.tm_mon   -=1;
            ltime.tm_year  -= 1;
            // output the time 
            cout << "time : " << asctime(&ltime) << endl;
        }
        else
            cout << "\nget clock failed" << endl;
    }   
    
    private:
    struct tm ltime;


    menuGetRTCCmd(const menuItemBase&);
    menuGetRTCCmd& operator=(const menuItemBase&);
};


// bool    CleEventLogCmd(uint16_t);
class menuClrEventLogCmd : public menuItemBase
{
    public:
    pClrEventLogCmd_t m_pClrEventLogCmd  = &controlProtocol::ClrEventLogCmd;

    menuClrEventLogCmd()
        :   menuItemBase("clear eventlog", "clear the eventlog"),
            m_pClrEventLogCmd(&controlProtocol::ClrEventLogCmd) {}

    void execute(controlProtocol* pCP)
    {
        if( (pCP->*m_pClrEventLogCmd)(m_destId) )
            cout << "\nclear event log  successful" << endl;
        else
            cout << "\nclear event log failed" << endl;
    }

    private:
    menuClrEventLogCmd(const menuItemBase&);
    menuClrEventLogCmd& operator=(const menuItemBase&);
};


class menuGetEventLogCmd : public menuItemBase
{
    public:
    pGetEventLogCmd_t m_pGetEventLogCmd  = &controlProtocol::GetEventLogCmd;

    menuGetEventLogCmd()
        :   menuItemBase("get eventlog", "get the eventlog"),
            m_pGetEventLogCmd(&controlProtocol::GetEventLogCmd) {}

    void execute(controlProtocol* pCP)
    {
      memset(&eventlog, '\0', sizeof(eventlog));

      if( (pCP->*m_pGetEventLogCmd)(m_destId, &eventlog[0]) )
      {
        for(int i = 0; i < MAX_ELOG_ENTRY; i++)
        {
          // get the time stamp
          memset(&ltime, '\0', sizeof(ltime));
          ltime.tm_sec  = eventlog[i].ts.sec;
          ltime.tm_min  = eventlog[i].ts.min;
          ltime.tm_hour = eventlog[i].ts.hour + 1;
          ltime.tm_mon  = eventlog[i].ts.mon - 1;
          ltime.tm_year = eventlog[i].ts.year + 101;
          ltime.tm_wday = eventlog[i].ts.wday;
          ltime.tm_mday = eventlog[i].ts.mday;

          // get the id and the instance
          id  = eventlog[i].id & 0x0000ffff;
          inst  = (eventlog[i].id  >> 16) & 0x0000ffff;
          memset(time_buff, '\0', sizeof(time_buff));
          if( (0 == asctime_r(&ltime, time_buff)) )
          {
            snprintf(time_buff, 30, "no event time");
          } else
          {
            time_buff[strlen(time_buff) - 1] = 0; // rid of the \n at the end
          }
          switch(id)
          {
            case ACUNotOnLine:
            {
              printf("%-26s : %-18s ACU %u not on line\n",
                time_buff, "ACUNotOnLine", inst);
                //asctime(&ltime), "TCUNotOnLine", inst);
              break;
            }
            case ACUNotRunning:
            {
              printf("%-26s : %-18s ACU %u not running\n",
                time_buff, "ACUNotRunning", inst);
                //asctime(&ltime), "TCUNotRunning", inst);
              break;
            }
            case ACUIsMismatch:
            {
              printf("%-26s : %-18s ACU %u state mismatch\n",
                time_buff, "ACUIsMismatch", inst);
                //asctime(&ltime), "TCUIsMismatch", inst);

              break;
            }
            case ASIC_RTDFault:
            {
              printf("%-26s : %-18s ASCI RTD fault\n",
                time_buff, "ASIC_RTDFault");
                //asctime(&ltime), "ChillerOffline");

              break;
            }
            case ASIC_Chiller_RTDFault:
            {
              printf("%-26s : %-18s ASCIC chiller RTD fault\n",
                time_buff, "ASIC_Chiller_RTDFault");
                //asctime(&ltime), "ChillerOffline");

              break;
            }
            case DDR_RTDFault: // TODO: add the instance to the output
            {
              printf("%-26s : %-18s DDR RTD fault\n",
                time_buff, "DDR_RTDFault");
                //asctime(&ltime), "ChillerOffline");

              break;
            }
            case DDR_Chiller_RTDFault: // TODO: add the instance to the output
            {
              printf("%-26s : %-18s DDR chiller RTD fault\n",
                time_buff, "DDR_Chiller_RTDFault");
                //asctime(&ltime), "ChillerOffline");

              break;
            }
            case ASIC_Chiller_RTDHot: // TODO: add the temp to the output
            {
              printf("%-26s : %-18s ASIC chiller RTD hot\n",
                time_buff, "ASIC_Chiller_RTDHot");
                //asctime(&ltime), "ChillerOffline");

              break;
            }
            case DDR_Chiller_RTDHot: // TODO: add the temp to the output
            {
              printf("%-26s : %-18s DDR chiller RTD hot\n",
                time_buff, "DDR_Chiller_RTDHot");
                //asctime(&ltime), "ChillerOffline");

              break;
            }
            case ChillerOffline:
            {
              printf("%-26s : %-18s chiller offline\n",
                time_buff, "ChillerOffline");
                //asctime(&ltime), "ChillerOffline");

              break;
            }
            case ChillerNotRunning:
            {
              printf("%-26s : %-18s chiller not running\n",
                time_buff, "ChillerNotRunning");
                //asctime(&ltime), "ChillerNotRunning");

              break;
            }
            case HumidityHigh:
            {
              printf("%-26s : %-18s humidity beyond threshold\n",
                time_buff, "HumidityHigh");
                //asctime(&ltime), "ChillerNotRunning");

              break;
            }
            case HumiditySensorFail:
            {
              printf("%-26s : %-18s humidity sensor fail\n",
                time_buff, "HumiditySensorFail");
                //asctime(&ltime), "ChillerNotRunning");

              break;
            }
            default:
            {
              // show the bytes
              printf("%-26s : %-18s Id: %zu data[0] %zu data[1] %zu data[2] %zu data[3] %zu data[4] %zu\n",
                time_buff, "no event", inst, htons(eventlog[i].data[0]),
                //asctime(&ltime), "no event", inst, htons(eventlog[i].data[0]),
                htons(eventlog[i].data[1]), htons(eventlog[i].data[2]),
                htons(eventlog[i].data[3]), htons(eventlog[i].data[4]));
              break;
            }
          }
        }

      }
      else
        cout << "\nfailed to get event log" << endl;
    }

    private:
    elogentry eventlog[MAX_ELOG_ENTRY];
    timeind*  pTimeStamp;
    struct tm ltime;
    uint16_t  id;       // chiller, TCU, humidity sensor
    uint16_t  inst;     // in case of ACU is the ACU number
    char      time_buff[30];



    menuGetEventLogCmd(const menuItemBase&);
    menuGetEventLogCmd& operator=(const menuItemBase&);
};

//    bool    SetChillerTemperature(uint16_t, float);
class menuSetH20AlarmASIC : public menuItemBase
{
    public:
    pSetH20AlarmASIC_t m_pSetH20AlarmASIC  = &controlProtocol::SetH20AlarmASIC;
    bool getParameters(void)
    {
        cout << "enter temperature: ";

        cin >> temperature;
/*
        if( !(cin >> temperature) )
        {
          cout << "bad input" << endl;
          return(false);
        }
*/

        return(true);
    }

    menuSetH20AlarmASIC()
        :   menuItemBase("set H2O alarm ASIC", "set the shut-off threshold for ASIC H2O temp"),
            m_pSetH20AlarmASIC(&controlProtocol::SetH20AlarmASIC) {}

    void execute(controlProtocol* pCP)
    {
        if( (pCP->*m_pSetH20AlarmASIC)(m_destId, temperature) )
            cout << "\nset H2O alarm ASIC successful" << endl;
        else
            cout << "\nset H2O alarm ASIC failed" << endl;
    }

    float   temperature;

    private:
    menuSetH20AlarmASIC(const menuItemBase&);
    menuSetH20AlarmASIC& operator=(const menuItemBase&);
};


//    bool    GetChillerTemperature(uint16_t, float*);
class menuGetH20AlarmASIC : public menuItemBase
{
    public:
    pGetH20AlarmASIC_t m_pGetH20AlarmASIC;

    menuGetH20AlarmASIC()
        :   menuItemBase("get H2O alarm ASIC", "get the the temp"),
            m_pGetH20AlarmASIC(&controlProtocol::GetH20AlarmASIC) {}

    void execute(controlProtocol* pCP)
    {
        if( (pCP->*m_pGetH20AlarmASIC)(m_destId, &temperature) )
            cout << "\nH2O alarm ASIC: " << temperature << endl;
        else
            cout << "\nget H2O alarm ASIC failed" << endl;
    }

    float   temperature;

    private:
    menuGetH20AlarmASIC(const menuItemBase&);
    menuGetH20AlarmASIC& operator=(const menuItemBase&);
};


//    bool    SetChillerTemperature(uint16_t, float);
class menuSetH20AlarmDDR : public menuItemBase
{
    public:
    pSetH20AlarmDDR_t m_pSetH20AlarmDDR  = &controlProtocol::SetH20AlarmDDR;
    bool getParameters(void)
    {
        cout << "enter temperature: ";

        cin >> temperature;
/*
        if( !(cin >> temperature) )
        {
          cout << "bad input" << endl;
          return(false);
        }
*/

        return(true);
    }

    menuSetH20AlarmDDR()
        :   menuItemBase("set H2O alarm DDR", "set the shut-off threshold for DDR H2O temp"),
            m_pSetH20AlarmDDR(&controlProtocol::SetH20AlarmDDR) {}

    void execute(controlProtocol* pCP)
    {
        if( (pCP->*m_pSetH20AlarmDDR)(m_destId, temperature) )
            cout << "\nset H2O alarm DDR successful" << endl;
        else
            cout << "\nset H2O alarm DDR failed" << endl;
    }

    float   temperature;

    private:
    menuSetH20AlarmDDR(const menuItemBase&);
    menuSetH20AlarmDDR& operator=(const menuItemBase&);
};


//    bool    GetChillerTemperature(uint16_t, float*);
class menuGetH20AlarmDDR : public menuItemBase
{
    public:
    pGetH20AlarmDDR_t m_pGetH20AlarmDDR;

    menuGetH20AlarmDDR()
        :   menuItemBase("get H2O alarm DDR", "get the the temp"),
            m_pGetH20AlarmDDR(&controlProtocol::GetH20AlarmDDR) {}

    void execute(controlProtocol* pCP)
    {
        if( (pCP->*m_pGetH20AlarmDDR)(m_destId, &temperature) )
            cout << "\nH2O alarm DDR: " << temperature << endl;
        else
            cout << "\nget H2O alarm DDR failed" << endl;
    }

    float   temperature;

    private:
    menuGetH20AlarmDDR(const menuItemBase&);
    menuGetH20AlarmDDR& operator=(const menuItemBase&);
};


class menuGetHumidity : public menuItemBase
{
    public:
    pGetHumidity_t m_pGetHumidity;

    menuGetHumidity()
        :   menuItemBase("get humidity", "get current humidity measurement"),
            m_pGetHumidity(&controlProtocol::GetHumidity) {}

    void execute(controlProtocol* pCP)
    {
        if( (pCP->*m_pGetHumidity)(m_destId, &humidity) )
        {
            cout << "\nhumidity: " << humidity << endl;
        } else
        {
            cout << "\nunable to get humidity" << endl;
        }
    }

    float humidity;

    private:
    menuGetHumidity(const menuItemBase&);
    menuGetHumidity& operator=(const menuItemBase&);
};


//bool    SetHumidityThreshold(uint16_t, uint16_t);
class menuSetHumidityThreshold : public menuItemBase
{
    public:
    pSetHumidityThreshold_t m_pSetHumidityThreshold;
    bool getParameters(void)
    {
        cout << "enter humidity threshold: "; cin >> humidityThreshold;
        return(true);
    }

    menuSetHumidityThreshold()
        :   menuItemBase("set humidity threshold", "set running humidity threshold"),
            m_pSetHumidityThreshold(&controlProtocol::SetHumidityThreshold) {}

    void execute(controlProtocol* pCP)
    {
        if( (pCP->*m_pSetHumidityThreshold)(m_destId, humidityThreshold) )
            cout << "\nset humidity threshold successful" << endl;
        else
            cout << "\nset humidity threshold failed" << endl;
    }

    float humidityThreshold;

    private:
    menuSetHumidityThreshold(const menuItemBase&);
    menuSetHumidityThreshold& operator=(const menuItemBase&);
};


//bool (controlProtocol::*pGetHumidityThreshold_t)(uint16_t, uint16_t*);
class menuGetHumidityThreshold : public menuItemBase
{
    public:
    pGetHumidityThreshold_t m_pGetHumidityThreshold;

    menuGetHumidityThreshold()
        :   menuItemBase("get humidity threshold", "get current humidity threshold"),
            m_pGetHumidityThreshold(&controlProtocol::GetHumidityThreshold) {}

    void execute(controlProtocol* pCP)
    {
        if( (pCP->*m_pGetHumidityThreshold)(m_destId, &humidityThreshold) )
        {
            cout << "\nhumidity threshold: " << humidityThreshold << endl;
        } else
        {
            cout << "\nget humidity threshold failed" << endl;
        }
    }

    uint16_t humidityThreshold;

    private:
    menuGetHumidityThreshold(const menuItemBase&);
    menuGetHumidityThreshold& operator=(const menuItemBase&);
};

class menuGetTempCmd : public menuItemBase
{
    public:
    pGetTempCmd_t m_pGetTempCmd;

    menuGetTempCmd()
        :   menuItemBase("get ambient temp", "get the ambient temperature"),
            m_pGetTempCmd(&controlProtocol::GetTempCmd) {}

    void execute(controlProtocol* pCP)
    {   
        if( (pCP->*m_pGetTempCmd)(m_destId, &temp) )
        {
            base      = temp & 0x00ff;
            fraction  = (temp & 0xff00) >> 8;
            ftemp     = base + (((float)(fraction))/100);
            printf("temperature (celcius) : %.2f\n", ftemp);
        }
        else
        {
            printf("get temperature failed\n");
        }
    }   
    
    private:
    uint16_t  temp      = 0;
    uint16_t  base      = 0;
    uint16_t  fraction  = 0;
    float     ftemp     = 0;


    menuGetTempCmd(const menuItemBase&);
    menuGetTempCmd& operator=(const menuItemBase&);
};


#endif

