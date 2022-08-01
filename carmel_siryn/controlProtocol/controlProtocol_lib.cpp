
//#ifdef __USING_WINDOWS_USB__
#include "pch.h"
//#endif

#include <stdio.h>
#include <string.h>
#include <time.h>
#include <stdlib.h>
#include <malloc.h>
#include "controlProtocol_lib.h"
#include "controlProtocol.h"
#include "eventlog.h"

extern "C"
{

    //-----------------------------------------------------------------------------
    // globals 
    //
    // Serial connectivity defaults that are to be changed by an initialization function
    //
#define USB_PORT_LEN  64
    static uint16_t    myAddress = 0;
    static uint16_t    peerAddress = 1;
    static uint32_t    usbSpeed = 19200;  // cannot be changed, the unit is 19200 always
    static char        usbPort[USB_PORT_LEN + 1] = {  // /dev/ttypUSB0 is the default
      '/', 'd', 'e', 'v', '/', 't', 't', 'y', 'U', 'S', 'B', '0', '\0'
    };



    // USB connectivity - this libraries address - needs to be 0
    void set_MyAddress(uint16_t addr)
    {
        myAddress = addr;
    }


    // USB connectivity - the peers (unit) address - needs to be 1
    void set_PeerAddress(uint16_t addr)
    {
        peerAddress = addr;
    }


    // USB connectivity - the USB port on this computer connected to the unit
    void set_UsbPort(const char* port)
    {
        strncpy(usbPort, port, sizeof(usbPort));
    }

    void set_Speed(uint32_t speed)
    {
        usbSpeed = speed;
    }


    // USB connectivity
    void showConnParams()
    {
        fprintf(stdout, "%14s: %u\n", "myAddress", myAddress);
        fprintf(stdout, "%14s: %u\n", "peerAddress", peerAddress);
        fprintf(stdout, "%14s: %s\n", "usbPort", usbPort);
        fprintf(stdout, "%14s: %u\n", "speed", usbSpeed);
    }


    //-----------------------------------------------------------------------------
    // execute the GetStatus command
    //
    bool get_Status(uint16_t* RTDsRunning, uint16_t* ACUsRunning, uint16_t* chillerOnLine)
    {
        bool retVal = true;
        controlProtocol cp(myAddress, peerAddress, usbPort, usbSpeed);


        if ((true == cp.serialConnected()))
        {
            retVal = cp.GetStatus(peerAddress, RTDsRunning, ACUsRunning, chillerOnLine);

            if ((false == retVal))
                fprintf(stdout, "fail\n");
            else
                fprintf(stdout, "success\n");
        }
        else
        {
            retVal = false;
            fprintf(stdout, "fail to open USB port\n");
        }

        return(retVal);
    }


    //-----------------------------------------------------------------------------
    // execute the StartUpCmd command
    //
    bool startUp()
    {
        bool retVal = true;
        controlProtocol cp(myAddress, peerAddress, usbPort, usbSpeed);


        if ((true == cp.serialConnected()))
        {
            retVal = cp.StartUpCmd(peerAddress);

            if ((false == retVal))
                fprintf(stdout, "fail\n");
            else
                fprintf(stdout, "success\n");
        }
        else
        {
            retVal = false;
            fprintf(stdout, "fail to open USB port\n");
        }

        return(retVal);
    }


    //-----------------------------------------------------------------------------
    // execute the StartUpATCmd command
    //
    bool startUpAT()
    {
        bool retVal = true;
        controlProtocol cp(myAddress, peerAddress, usbPort, usbSpeed);


        if ((true == cp.serialConnected()))
        {
            retVal = cp.StartUpATCmd(peerAddress);

            if ((false == retVal))
                fprintf(stdout, "fail\n");
            else
                fprintf(stdout, "success\n");
        }
        else
        {
            retVal = false;
            fprintf(stdout, "fail to open USB port\n");
        }

        return(retVal);
    }


    //-----------------------------------------------------------------------------
    // execute the ShutDownCmd command
    //
    bool shutDown()
    {
        bool retVal = true;
        controlProtocol cp(myAddress, peerAddress, usbPort, usbSpeed);


        if ((true == cp.serialConnected()))
        {
            retVal = cp.ShutDownCmd(peerAddress);

            if ((false == retVal))
                fprintf(stdout, "fail\n");
            else
                fprintf(stdout, "success\n");
        }
        else
        {
            retVal = false;
            fprintf(stdout, "fail to open USB port\n");
        }

        return(retVal);
    }


    //-----------------------------------------------------------------------------
    // execute the SetACUTemperature command for ASIC_RS485_ID
    //
    bool set_SVASIC(float temp)
    {
        bool retVal = true;
        controlProtocol cp(myAddress, peerAddress, usbPort, usbSpeed);


        if ((true == cp.serialConnected()))
        {
            retVal = cp.SetACUTemperature(peerAddress, ASIC_RS485_ID, temp);

            if ((false == retVal))
                fprintf(stdout, "fail\n");
            else
                fprintf(stdout, "success\n");
        }
        else
        {
            retVal = false;
            fprintf(stdout, "fail to open USB port\n");
        }

        return(retVal);
    }

    //-----------------------------------------------------------------------------
    // execute the GetACUTemperature command for ASIC_RS485_ID
    //
    bool get_SVASIC(float* temp)
    {
        bool retVal = false;
        controlProtocol cp(myAddress, peerAddress, usbPort, usbSpeed);
        uint16_t  result = 0;


        if ((true == cp.serialConnected()))
        {
            retVal = cp.GetACUTemperature(peerAddress, ASIC_RS485_ID, &result, temp);

            if ((false == retVal))
            {
                fprintf(stdout, "fail\n");

            }
            else
            {
                if ((result))
                {
                    retVal = true;
                    fprintf(stdout, "success\n");

                }
                else
                {
                    fprintf(stdout, "fail\n");
                }
            }
        }
        else
        {
            fprintf(stdout, "fail to open USB port\n");
        }

        return(retVal);
    }


    //-----------------------------------------------------------------------------
    // execute the SetACUTemperature command for DDR_RS485_ID
    //
    bool set_SVDDR(float temp)
    {
        bool retVal = true;
        controlProtocol cp(myAddress, peerAddress, usbPort, usbSpeed);


        if ((true == cp.serialConnected()))
        {
            retVal = cp.SetACUTemperature(peerAddress, DDR_RS485_ID, temp);

            if ((false == retVal))
                fprintf(stdout, "fail\n");
            else
                fprintf(stdout, "success\n");
        }
        else
        {
            retVal = false;
            fprintf(stdout, "fail to open USB port\n");
        }

        return(retVal);
    }

    //-----------------------------------------------------------------------------
    // execute the GetACUTemperature command for DDR_RS485_ID
    //
    bool get_SVDDR(float* temp)
    {
        bool retVal = false;
        controlProtocol cp(myAddress, peerAddress, usbPort, usbSpeed);
        uint16_t  result = 0;


        if ((true == cp.serialConnected()))
        {
            retVal = cp.GetACUTemperature(peerAddress, DDR_RS485_ID, &result, temp);

            if ((false == retVal))
            {
                fprintf(stdout, "fail\n");
            }
            else
            {
                if ((result))
                {
                    retVal = true;
                    fprintf(stdout, "success\n");
                }
                else
                {
                    fprintf(stdout, "fail\n");
                }
            }
        }
        else
        {
            fprintf(stdout, "fail to open USB port\n");
        }

        return(retVal);
    }


    //-----------------------------------------------------------------------------
    // execute the GetACUObjTemperature command for ASIC_RS485_ID
    //
    bool get_PVASIC(float* temp)
    {
        bool retVal = false;
        controlProtocol cp(myAddress, peerAddress, usbPort, usbSpeed);
        uint16_t  result = 0;


        if ((true == cp.serialConnected()))
        {
            retVal = cp.GetACUObjTemperature(peerAddress, ASIC_RS485_ID, &result, temp);

            if ((false == retVal))
            {
                fprintf(stdout, "fail\n");
            }
            else
            {
                if ((result))
                {
                    retVal = true;
                    fprintf(stdout, "success\n");
                }
                else
                {
                    fprintf(stdout, "fail\n");
                }
            }
        }
        else
        {
            fprintf(stdout, "fail to open USB port\n");
        }

        return(retVal);
    }


    //-----------------------------------------------------------------------------
    // execute the GetACUObjTemperature command for ASIC_RS485_ID
    //
    bool get_PVDDR(float* temp)
    {
        bool retVal = false;
        controlProtocol cp(myAddress, peerAddress, usbPort, usbSpeed);
        uint16_t  result = 0;


        if ((true == cp.serialConnected()))
        {
            retVal = cp.GetACUObjTemperature(peerAddress, DDR_RS485_ID, &result, temp);

            if ((false == retVal))
            {
                fprintf(stdout, "fail\n");
            }
            else
            {
                if ((result))
                {
                    retVal = true;
                    fprintf(stdout, "success\n");
                }
                else
                {
                    fprintf(stdout, "fail\n");
                }
            }
        }
        else
        {
            fprintf(stdout, "fail to open USB port\n");
        }

        return(retVal);
    }


    //-----------------------------------------------------------------------------
    // execute the EnableACUs command
    //
    bool do_enableACUs(void)
    {
        bool retVal = false;
        controlProtocol cp(myAddress, peerAddress, usbPort, usbSpeed);


        if ((true == cp.serialConnected()))
        {
            retVal = cp.EnableACUs(peerAddress);

            if ((false == retVal))
            {
                fprintf(stdout, "fail\n");

            }
            else
            {
                retVal = true;
                fprintf(stdout, "success\n");
            }
        }
        else
        {
            fprintf(stdout, "fail to open USB port\n");
        }

        return(retVal);
    }


    //-----------------------------------------------------------------------------
    // execute the DisableACUs command
    //
    bool do_disableACUs(void)
    {
        bool retVal = false;
        controlProtocol cp(myAddress, peerAddress, usbPort, usbSpeed);


        if ((true == cp.serialConnected()))
        {
            retVal = cp.DisableACUs(peerAddress);

            if ((false == retVal))
            {
                fprintf(stdout, "fail\n");

            }
            else
            {
                retVal = true;
                fprintf(stdout, "success\n");
            }
        }
        else
        {
            fprintf(stdout, "fail to open USB port\n");
        }

        return(retVal);
    }


    //-----------------------------------------------------------------------------
    // execute the SetRTCCmd command
    //
    bool set_RTC(void)
    {
        bool retVal = false;
        controlProtocol cp(myAddress, peerAddress, usbPort, usbSpeed);


        if ((true == cp.serialConnected()))
        {
            retVal = cp.SetRTCCmd(peerAddress);

            if ((false == retVal))
            {
                fprintf(stdout, "fail\n");

            }
            else
            {
                retVal = true;
                fprintf(stdout, "success\n");
            }
        }
        else
        {
            fprintf(stdout, "fail to open USB port\n");
        }

        return(retVal);
    }


    //-----------------------------------------------------------------------------
    // execute the getRTCCmd command
    //
    bool get_RTC(struct tm* ltime)
    {
        bool retVal = false;
        controlProtocol cp(myAddress, peerAddress, usbPort, usbSpeed);


        if ((true == cp.serialConnected()))
        {
            retVal = cp.GetRTCCmd(peerAddress, ltime);

            if ((false == retVal))
            {
                fprintf(stdout, "fail\n");

            }
            else
            {
                retVal = true;
                fprintf(stdout, "success\n");
            }
        }
        else
        {
            fprintf(stdout, "fail to open USB port\n");
        }

        return(retVal);
    }


    //-----------------------------------------------------------------------------
    // execute the ClrEventLogCmd command
    //
    bool clr_EventLog(void)
    {
        bool retVal = false;
        controlProtocol cp(myAddress, peerAddress, usbPort, usbSpeed);


        if ((true == cp.serialConnected()))
        {
            retVal = cp.ClrEventLogCmd(peerAddress);

            if ((false == retVal))
            {
                fprintf(stdout, "fail\n");

            }
            else
            {
                retVal = true;
                fprintf(stdout, "success\n");
            }
        }
        else
        {
            fprintf(stdout, "fail to open USB port\n");
        }

        return(retVal);
    }


    //-----------------------------------------------------------------------------
    // execute the GetEventLogCmd command
    //
    // this function will calloc a buffer to hold the event_log
    //
    // the caller must free that memory
    //
    bool get_EventLog(char** logs)
    {
        bool retVal = false;
        struct tm ltime;
        elogentry eventlogs[MAX_ELOG_ENTRY];
        char* eventlog;
        uint16_t  id;       // chiller, TCU, humidity sensor
        uint16_t  inst;     // in case of ACU is the ACU number
        char      time_buff[30];
        char* p_time = 0;
        int index = 0;
        controlProtocol cp(myAddress, peerAddress, usbPort, usbSpeed);


        if ((true == cp.serialConnected()))
        {
            retVal = cp.GetEventLogCmd(peerAddress, &eventlogs[0]);

            if ((false == retVal))
            {
                fprintf(stdout, "fail GetEventLogCmd\n");
            }
            else
            {
                //
                // get memory for the eventlog using the input logs
                //
                eventlog = 0;
                eventlog = (char*)calloc(1024, sizeof(char));  // need to find out how big this really needs to be

                if ((0 != eventlog))
                {
                    *logs = eventlog;

                    //
                    // copy over the event log
                    //
                    for (int i = 0; i < MAX_ELOG_ENTRY; i++)
                    {
                        if ((0 != eventlogs[i].ts.year))
                        {
                            // get the time stamp
                            memset(&ltime, '\0', sizeof(ltime));
                            ltime.tm_sec = eventlogs[i].ts.sec;
                            ltime.tm_min = eventlogs[i].ts.min;
                            ltime.tm_hour = eventlogs[i].ts.hour + 1;
                            ltime.tm_mon = eventlogs[i].ts.mon - 1;
                            ltime.tm_year = eventlogs[i].ts.year + 101;
                            ltime.tm_wday = eventlogs[i].ts.wday;
                            ltime.tm_mday = eventlogs[i].ts.mday;

                            memset(time_buff, '\0', sizeof(time_buff));
                            if ((0 == (p_time = asctime(&ltime))))
                            {
                                snprintf(time_buff, 30, "no event time");
                            }
                            else
                            {
                                sprintf(time_buff, "%s", p_time);
                                time_buff[strlen(time_buff) - 1] = 0; // rid of the \n at the end
                            }
                        }
                        else
                        {
                            snprintf(time_buff, 30, "no event time");
                        }

                        // get the id and the instance
                        id = eventlogs[i].id & 0x0000ffff;
                        inst = (eventlogs[i].id >> 16) & 0x0000ffff;

                        switch (id)
                        {
                            case ACUNotOnLine:
                            {
                                index += sprintf(&eventlog[index], "%-26s : %-18s ACU %u not on line\n",
                                    time_buff, "ACUNotOnLine", inst);
                                //asctime(&ltime), "TCUNotOnLine", inst);
                                break;
                            }
                            case ACUNotRunning:
                            {
                                index += sprintf(&eventlog[index], "%-26s : %-18s ACU %u not running\n",
                                    time_buff, "ACUNotRunning", inst);
                                //asctime(&ltime), "TCUNotRunning", inst);
                                break;
                            }
                            case ACUIsMismatch:
                            {
                                index += sprintf(&eventlog[index], "%-26s : %-18s ACU %u state mismatch\n",
                                    time_buff, "ACUIsMismatch", inst);
                                //asctime(&ltime), "TCUIsMismatch", inst);

                                break;
                            }
                            case ASIC_RTDFault:
                            {
                                index += sprintf(&eventlog[index], "%-26s : %-18s ASCI RTD fault\n",
                                    time_buff, "ASIC_RTDFault");
                                //asctime(&ltime), "ChillerOffline");

                                break;
                            }
                            case ASIC_Chiller_RTDFault:
                            {
                                index += sprintf(&eventlog[index], "%-26s : %-18s ASIC chiller RTD fault\n",
                                    time_buff, "ASIC_Chiller_RTDFault");
                                //asctime(&ltime), "ChillerOffline");

                                break;
                            }
                            case DDR_RTDFault: // TODO: add the instance to the output
                            {
                                index += sprintf(&eventlog[index], "%-26s : %-18s DDR RTD fault\n",
                                    time_buff, "DDR_RTDFault");
                                //asctime(&ltime), "ChillerOffline");

                                break;
                            }
                            case DDR_Chiller_RTDFault: // TODO: add the instance to the output
                            {
                                index += sprintf(&eventlog[index], "%-26s : %-18s DDR chiller RTD fault\n",
                                    time_buff, "DDR_Chiller_RTDFault");
                                //asctime(&ltime), "ChillerOffline");

                                break;
                            }
                            case ASIC_Chiller_RTDHot: // TODO: add the temp to the output
                            {
                                index += sprintf(&eventlog[index], "%-26s : %-18s ASIC chiller RTD hot\n",
                                    time_buff, "ASIC_Chiller_RTDHot");
                                //asctime(&ltime), "ChillerOffline");

                                break;
                            }
                            case DDR_Chiller_RTDHot: // TODO: add the temp to the output
                            {
                                index += sprintf(&eventlog[index], "%-26s : %-18s DDR chiller RTD hot\n",
                                    time_buff, "DDR_Chiller_RTDHot");
                                //asctime(&ltime), "ChillerOffline");

                                break;
                            }
                            case ChillerOffline:
                            {
                                index += sprintf(&eventlog[index], "%-26s : %-18s chiller offline\n",
                                    time_buff, "ChillerOffline");
                                //asctime(&ltime), "ChillerOffline");

                                break;
                            }
                            case ChillerNotRunning:
                            {
                                index += sprintf(&eventlog[index], "%-26s : %-18s chiller not running\n",
                                    time_buff, "ChillerNotRunning");
                                //asctime(&ltime), "ChillerNotRunning");

                                break;
                            }
                            case HumidityHigh:
                            {
                                index += sprintf(&eventlog[index], "%-26s : %-18s humidity beyond threshold\n",
                                    time_buff, "HumidityHigh");
                                //asctime(&ltime), "ChillerNotRunning");

                                break;
                            }
                            case HumiditySensorFail:
                            {
                                index += sprintf(&eventlog[index], "%-26s : %-18s humidity sensor fail\n",
                                    time_buff, "HumiditySensorFail");
                                //asctime(&ltime), "ChillerNotRunning");

                                break;
                            }
                            default:
                            {
                                // show the bytes
                                index += sprintf(&eventlog[index], "%-26s : %-18s Id: %zu data[0] %zu data[1] %zu data[2] %zu data[3] %zu data[4] %zu\n",
                                    time_buff, "no event", inst, htons(eventlogs[i].data[0]),
                                    //asctime(&ltime), "no event", inst, htons(eventlog[i].data[0]),
                                    htons(eventlogs[i].data[1]), htons(eventlogs[i].data[2]),
                                    htons(eventlogs[i].data[3]), htons(eventlogs[i].data[4]));
                                break;
                            }
                        }  // end switch

                    }  // end for (int i = 0; i < MAX_ELOG_ENTRY; i++)

                    retVal = true;
                    fprintf(stdout, "success\n");

                } else  // if ((0 != *eventlog))
                {
                    retVal = false;
                    fprintf(stdout, "fail\n");
                }
            }
        }
        else
        {
            retVal = false;
            fprintf(stdout, "fail to open USB port\n");
        }

        return(retVal);
    }

    //-----------------------------------------------------------------------------
    // execute the SetH20AlarmASIC command
    //
    bool set_H20AlarmASIC(float temp)
    {
        bool retVal = false;
        controlProtocol cp(myAddress, peerAddress, usbPort, usbSpeed);


        if ((true == cp.serialConnected()))
        {
            retVal = cp.SetH20AlarmASIC(peerAddress, temp);

            if ((false == retVal))
            {
                fprintf(stdout, "fail\n");

            }
            else
            {
                retVal = true;
                fprintf(stdout, "success here\n");
            }
        }
        else
        {
            fprintf(stdout, "fail to open USB port\n");
        }

        return(retVal);
    }


    //-----------------------------------------------------------------------------
    // execute the GetH20AlarmASIC command
    //
    bool get_H20AlarmASIC(float* temp)
    {
        bool retVal = false;
        controlProtocol cp(myAddress, peerAddress, usbPort, usbSpeed);


        if ((true == cp.serialConnected()))
        {
            retVal = cp.GetH20AlarmASIC(peerAddress, temp);

            if ((false == retVal))
            {
                fprintf(stdout, "fail\n");

            }
            else
            {
                retVal = true;
                fprintf(stdout, "success\n");
            }
        }
        else
        {
            fprintf(stdout, "fail to open USB port\n");
        }

        return(retVal);
    }


    //-----------------------------------------------------------------------------
    // execute the SetH20AlarmDDR command
    //
    bool set_H20AlarmDDR(float temp)
    {
        bool retVal = false;
        controlProtocol cp(myAddress, peerAddress, usbPort, usbSpeed);


        if ((true == cp.serialConnected()))
        {
            retVal = cp.SetH20AlarmDDR(peerAddress, temp);

            if ((false == retVal))
            {
                fprintf(stdout, "fail\n");

            }
            else
            {
                retVal = true;
                fprintf(stdout, "success\n");
            }
        }
        else
        {
            fprintf(stdout, "fail to open USB port\n");
        }

        return(retVal);
    }


    //-----------------------------------------------------------------------------
    // execute the GetH20AlarmDDR command
    //
    bool get_H20AlarmDDR(float* temp)
    {
        bool retVal = false;
        controlProtocol cp(myAddress, peerAddress, usbPort, usbSpeed);


        if ((true == cp.serialConnected()))
        {
            retVal = cp.GetH20AlarmDDR(peerAddress, temp);

            if ((false == retVal))
            {
                fprintf(stdout, "fail\n");

            }
            else
            {
                retVal = true;
                fprintf(stdout, "success\n");
            }
        }
        else
        {
            fprintf(stdout, "fail to open USB port\n");
        }

        return(retVal);
    }

}
