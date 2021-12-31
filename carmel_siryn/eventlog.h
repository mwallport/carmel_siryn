#ifndef __EVENTLOG__
#define __EVENTLOG__

#include <stdint.h>


#define MAX_DATA        5   // number of data elements in each log entry, is 5
                            // to keep overall size of the elogentry struct on
                            // an even 32bit boundry

#define MAX_ELOG_ENTRY  3   // max number of eventlog entrys

//
// data type to hold time stamp - and used w/ Controllino RTC
//
typedef struct _timeind
{
  /* Day, WeekDay, Month, Year, Hour, Minute, Second); 
  Controllino_SetTimeDate(12,4,1,17,15,41,23); */
    uint8_t sec;      // Seconds (0-60)
    uint8_t min;      // Minutes (0-59)
    uint8_t hour;     // Hours (0-23)
    uint8_t mday;     // Day of the month (1-31)
    uint8_t mon;      // Month (0-11)
    uint8_t year;     // Year - 1900
    uint8_t wday;     // Day of the week (0-6, Sunday = 0)
    uint8_t fill;     // fill to keep the buff length 
} timeind;


//
// the eventlog
//

// entry
typedef struct _elogentry
{
  timeind   ts;             // time stamp of this elogentry
  uint32_t  id;             // the id of the subsystem this elogentry is for
  uint32_t  data[MAX_DATA]; // bytes of data for this entry
} elogentry;


//
// APIs to work on the event log
//
uint8_t     addEventLogEntry(elogentry*); // returns 0 if no wrap around, 1 if wrap around
elogentry   getEventLogEntry(uint8_t);    // return a pointer to the elogentry at given index
void        clrEventLog(void);            // clear all events in the eventlog
void        clrEventLogEntry(elogentry*); // clear the contents of the formal parameter
const elogentry* getEventLog(void);

#endif
