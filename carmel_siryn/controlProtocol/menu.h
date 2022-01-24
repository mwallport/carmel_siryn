#ifndef __MENU__
#define __MENU__
#include <unistd.h>
#include <iostream>
#include <string>
#include <cstdint>
#include "controlProtocol.h"
#include "events.h"

    

using namespace std;

typedef bool (controlProtocol::*pStartUpCmd_t)(uint16_t);
typedef bool (controlProtocol::*pShutDownCmd_t)(uint16_t);
typedef bool (controlProtocol::*pGetStatus_t)(uint16_t, uint16_t*, uint16_t*, uint16_t*);
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


class menuItemBase
{
    public:
    string  m_name;			            // name for the command to list in a menu
    string  m_description;	            // breif description of the command
    uint16_t m_destId;

    menuItemBase(const string& name, const string& description, const uint16_t destId = 1)
        : m_name(name), m_description(description), m_destId(destId) {};
    virtual ~menuItemBase() {};
    virtual void getParameters(void) {};    // prompt for parameters for cmd - derived as needed
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
        :   menuItemBase("startup system", "start TCUs and chiller"),
            m_pStartUpCmd(&controlProtocol::StartUpCmd) {}

    void execute(controlProtocol* pCP)
    {
        if( (pCP->*m_pStartUpCmd)(m_destId) )
            cout << "\nstartup successful" << endl;
        else
            cout << "\nstartup failed" << endl;
    }
    
    private:
    menuStartUpCmd(const menuItemBase&);
    menuStartUpCmd& operator=(const menuItemBase&);
};


// bool    ShutDownCmd(uint16_t);
class menuShutDownCmd : public menuItemBase
{
    public:
    pShutDownCmd_t m_pShutDownCmd;

    menuShutDownCmd()
        :   menuItemBase("shutdown system", "stop TCUs, chiller not affected"),
            m_pShutDownCmd(&controlProtocol::ShutDownCmd) {}

    void execute(controlProtocol* pCP)
    {
        if( (pCP->*m_pShutDownCmd)(m_destId) )
            cout << "\nshutdown successful" << endl;
        else
            cout << "\nshutdown failed" << endl;
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
        :   menuItemBase("get status", "report humidity alert, ACU states, chiller state"),
            m_pGetStatus(&controlProtocol::GetStatus) {}

    void execute(controlProtocol* pCP)
    {
        if( (pCP->*m_pGetStatus)(m_destId, &RTDsRunning, &ACUsRunning, &chillerRunning) )
        {
            cout << "\nRTDs running:    " << RTDsRunning << endl <<
                    "ACUs running:    " << ACUsRunning << endl <<
                    "chiller running: " << chillerRunning << endl;
        } else
        {
            cout << "\nunable to get status" << endl;
        }
    }

    uint16_t RTDsRunning;
    uint16_t ACUsRunning;
    uint16_t chillerRunning;
    
    private:
    menuGetStatus(const menuItemBase&);
    menuGetStatus& operator=(const menuItemBase&);
};


// bool    SetACUTemperature(uint16_t, uint16_t, float);
class menuSetACUTemperature : public menuItemBase
{
    public:
    pSetACUTemperature_t m_pSetACUTemperature;
    void getParameters(void)
    {
        cout << "enter ACU address (i.e. 1, 2, or 3):  "; cin >> ACUAddress;
        cout << "enter temperature (i.e. 5.0 or -5.0): "; cin >> temperature;
    }

    menuSetACUTemperature()
        :   menuItemBase("set ACU SV", "set an SV temperature"),
            m_pSetACUTemperature(&controlProtocol::SetACUTemperature) {}

    void execute(controlProtocol* pCP)
    {
        if( (pCP->*m_pSetACUTemperature)(m_destId, ACUAddress, temperature) )
            cout << "\nset ACU temperature succesful" << endl;
          else
            cout << "\nset ACU temperature failed" << endl;
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
    void getParameters(void)
    {
        cout << "enter ACU address:  "; cin >> ACUAddress;
    }

    menuGetACUTemperature()
        :   menuItemBase("get ACU SV", "get an SV temperature"),
            m_pGetACUTemperature(&controlProtocol::GetACUTemperature) {}

    void execute(controlProtocol* pCP)
    {
        if( (pCP->*m_pGetACUTemperature)(m_destId, ACUAddress, &result, &temperature) )
        {
            if( (result) )
                cout << "\nACU temperature: " << temperature << endl;
            else
                cout << "\nget ACU temperature failed to return temp" << endl;
        } else
            cout << "\nget ACU temperature failed" << endl;
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
    
    void getParameters(void)
    {
        cout << "enter ACU address:  "; cin >> ACUAddress;
    }

    menuGetACUObjTemperature()
        :   menuItemBase("get ACU PV", "get a PV temperature"),
            m_pGetACUObjTemperature(&controlProtocol::GetACUObjTemperature) {}

    void execute(controlProtocol* pCP)
    {
        if( (pCP->*m_pGetACUObjTemperature)(m_destId, ACUAddress, &result, &temperature) )
        {
            if( (result) )
                cout << "\nACU obj temperature: " << temperature << endl;
            else
                cout << "\nget ACU object temperature failed to return temp" << endl;
        } else
            cout << "\nget ACU object temperature failed" << endl;
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
    void getParameters(void)
    {
        cout << "enter temperature (i.e. 24.0 or -10.5): "; cin >> temperature;
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
        :   menuItemBase("start ACUs", "start all ACUs"),
            m_pEnableACUs(&controlProtocol::EnableACUs) {}

    void execute(controlProtocol* pCP)
    {
        if( (pCP->*m_pEnableACUs)(m_destId) )
            cout << "\nstart ACUs successful" << endl;
        else
            cout << "\nstart ACUs failed" << endl;
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
        :   menuItemBase("stop ACUs", "stop all ACUs"),
            m_pDisableACUs(&controlProtocol::DisableACUs) {}

    void execute(controlProtocol* pCP)
    {
        if( (pCP->*m_pDisableACUs)(m_destId) )
            cout << "\nstop ACUs successful" << endl;
        else
            cout << "\nstop ACUs failed" << endl;
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
    void getParameters(void)
    {
        cout << "enter ACU address:  "; cin >> acu_address;
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
            cout << "\nset RTC successful" << endl;
        else
            cout << "\nset RTC failed" << endl;
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
            // output the time 
            cout << "time : " << asctime(&ltime) << endl;
        }
        else
            cout << "\nget RTC failed" << endl;
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
          ltime.tm_mon  = eventlog[i].ts.mon;
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




#endif

