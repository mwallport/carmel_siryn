#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "controlProtocol_lib.h"


/*
Simple program to show how to use the library calls.

1st parameter is the com port to connect with
    - note that for COM port 10 and above, you need to open them with the
    command \\.\\COMn, which corresponds to the C string \\\\.\\COMn, so
    you would enter \\\\.\\COM11 for COM11 on the cmd line for 1st argument

2nd parameter is the ASIC H2O alarm temp

3rd parameter is the DDR alarm temp

*/

int main(int argc, char** argv)
{
    uint16_t rtd = 0, acu = 0, chiller = 0;
    float temp;
    float asic_temp = 0;
    float ddr_temp = 0;
    char port[32];
    struct tm ltime;
    char* logs = 0;


    strncpy(port, argv[1], sizeof(port));
    sscanf(argv[2], "%f", &asic_temp);
    sscanf(argv[3], "%f", &ddr_temp);

    printf("found cmd line parameters port [%s] ASIC H2O alarm [%f] DDR H2O alarm [%f]\n\n",
        port, asic_temp, ddr_temp);

    /*
    Note that for COM port 10 and above, you need to open them with the command \\.\\COMn,
    which corresponds to the C string \\\\.\\COMn
    (where n is the 1 or 2 digits specifying the COM port number). See http://support2.microsoft.com/kb/115831.
    */
    set_UsbPort(port);

    //
    // show the USB serial connection parameters
    //
    showConnParams();

    //
    // set the clock on the unit, the time is used to tag event log entries
    // the used to set is the time on 'this PC'
    //
    if( (!set_RTC()) )
        fprintf(stderr, "set_RTC() failed\n");

/*
    //
    // set/get the H2O high temp alarm for the ASIC
    //
    printf("setting H20 alarm ASIC to [%f]\n", asic_temp);
    if ((!set_H20AlarmASIC(asic_temp)))
        fprintf(stderr, "set_H2)AlarmASIC failed\n");

    temp = 0;
    printf("calling get H2O alarm ASIC\n");
    if ((!get_H20AlarmASIC(&temp)))
        fprintf(stderr, "get_H20AlarmsASIC failed\n");
    else
        printf("got back H20 alarm ASIC [%f]\n", temp);


    //
    // set/get the H2O high temp alarm for the DDR
    //
    printf("setting H20 alarm DDR to [%f]\n", ddr_temp);
    if ((!set_H20AlarmDDR(ddr_temp)))
        fprintf(stderr, "set_H20AlarmDDR failed\n");

    temp = 0;
    printf("calling get H20 alarm DDR\n");
    if ((!get_H20AlarmDDR(&temp)))
        fprintf(stderr, "get_H20AlarmDDR faile\n");
    else
        printf("got back H20 alarm DDR [%f]\n", temp);

    //
    // set/get the SV for the ASIC thermal control
    //
    temp = (float)27.80;
    printf("calling set SV for the ASIC thermal control to [%f]\n", temp);
    if ((!set_SVASIC(temp)))
        fprintf(stderr, "set_SVACIS() failed\n");

    printf("calling get SV for the ASIC thermal control\n");
    temp = 0;
    if ((!get_SVASIC(&temp)))
        fprintf(stderr, "get_SVASIC() failed\n");
    else
        printf("got back ASIC SV [%f]\n", temp);


    //
    // set/get the SV for the DDR thermal control
    //
    temp = (float)27.30;
    printf("calling set SV for the DDR thermal control to [%f]\n", temp);
    if ((!set_SVDDR(temp)))
        fprintf(stderr, "set_SVDDR() failed\n");
        
    printf("calling get SV for the DDR thermal control\n");
    temp = 0;
    if ((!get_SVDDR(&temp)))
        fprintf(stderr, "get_SVDDR() failed\n");
    else
        printf("got back ASIC SV [%f]\n", temp);


    //
    // clear the event log
    //
    printf("calling clr_EventLog()\n");
    if ((!clr_EventLog()))
        fprintf(stderr, "clr_EventLog() failed\n");
    
    
    //
    // start thermal control
    // 
    // startUPAT() is also an option here
    //  - it will begin the auto-tune on the thermal control
    //
    printf("calling startUp()\n");
    if ((!startUp()))
        fprintf(stderr, "startUP() failed\n");

 */

 //
// clear the event log
//
    printf("calling clr_EventLog()\n");
    if ((!clr_EventLog()))
        fprintf(stderr, "clr_EventLog() failed\n");


    //
    // execute the remaining commands in a loop
    // 
    //  - generally for some period of time
    //  - or until some failure is reported back by the get_Status() command
    //
    for (int i = 0; i < 3000; i++)
    {
  /*
        //
        // get the the real time clock from the unit
        //
        printf("calling get_RTC()\n");
        memset(&ltime, '\0', sizeof(ltime));
        if ((!get_RTC(&ltime)))
            fprintf(stderr, "get_RTC() failed\n");
        else
        {
            ltime.tm_mon -= 1;
            printf("time on unit [%s]\n", asctime(&ltime));
        }

*/
        //
        // use the get_Status() command to get the health the slave's components
        // 
        // should add code here to interrogate the return paramters
        // ensure it issafe to proceed
        //
        printf("calling getStatus()\n");
        if ((!get_Status(&rtd, &acu, &chiller)))
            fprintf(stderr, "get_Status() failed\n");
        else
        {
            // print the values of the return parameters ..
            printf("rtd [0x%X] acu [0x%X] chiller [0x%x]\n", rtd, acu, chiller);

            //
            // the return parameters are bit flags
            // 
            // dissect, print out, act upon the status reported by the bit flags
            // 
            // ACUs output first 
            if( (acu & ASIC_THERMAL_CTRL_OFFLINE) )
                printf("ASIC ACU is OFFLINE");
            else
                printf("ASIC ACU is ONLINE");

            if( (acu & ASIC_THERMAL_CTRL_NOT_RUNNING) )
                printf(" and NOT RUNNING\n");
            else
                printf(" and RUNNING\n");

            if( (acu & DDR_THERMAL_CTRL_OFFLINE) )
                printf("DDR ACU is OFFLINE");
            else
                printf("DDR ACU is ONLINE");

            if( (acu & DDR_THERMAL_CTRL_NOT_RUNNING) )
                printf(" and NOT RUNNING\n");
            else
                printf(" and RUNNING\n");

            // RTD ouput next
            printf("ASIC Chiller RTD");
            if( (rtd & ASIC_CHILLER_RTD_HAS_FAULTS) )
                printf(" has faults\n");
            else
                printf(" has no faults\n");

            printf("DDR1 RTD");
            if( (rtd & DDR1_RTD_HAS_FAULTS) )
                printf(" has faults\n");
            else
                printf(" has no faults\n");

            printf("DDR2 RTD");
            if( (rtd & DDR2_RTD_HAS_FAULTS) )
                printf(" has faults\n");
            else
                printf(" has no faults\n");

            printf("DDR Chiller RTD");
            if( (rtd & DDR_CHILLER_RTD_HAS_FAULTS) )
                printf(" has faults\n");
            else
                printf(" has no faults\n");
        }

        //
        // get and print the PV for the ASIC thermal control
        //
        temp = 0;
        if ((!get_PVASIC(&temp)))
            fprintf(stderr, "get_PVASIC() failed\n");
        else
            printf("got back ASIC PV [%f]\n", temp);

        //
        // get and print the PV for the DDR thermal controls
        //
        temp = 0;
        if ((!get_PVDDR(&temp)))
            fprintf(stderr, "get_PVDDR() failed\n");
        else
            printf("got back DDR SV [%f]\n", temp);


        //
        // dump out the event log
        //
        // do some string manupilations to dissect this output for now
        //
        // this commnad's agrument is a char* pointer's address
        // - the function will calloc memory using the parameter
        // - it is the calling routine's responsibility to free the memory
        //
        printf("calling get_EventLog\n");
        get_EventLog(&logs);
        if ((0 != logs))
        {
            printf("event log :\n%s\n\n", logs);
            free(logs);
            logs = 0;
        }
        else
            printf("get_EventLog fail, no buffer returned\n");

        fprintf(stderr, "iteration [%d]\n", i);
    }
 
    //
    // do shutDown() when done
    //
    printf("calling shutDown()\n");
    if ((!shutDown()))
        fprintf(stderr, "shutdown() failed\n");

    return(0);
}
