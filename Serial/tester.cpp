#include <unistd.h>
#include <sys/select.h>
#include "Serial.h"


int main(int argc, char** argv)
{
  Serial Serial1("/dev/ttyS0", 9600);
  uint8_t   input[64];
  int    idx = 0;
  struct  timeval tv;
  char  helloBytes[] = "hello from Raspberry Pi\r\n";
  int count;


  Serial1.begin();


  while(1)
  {
    tv.tv_sec   = 0;
    tv.tv_usec  = 500;

    // give the hardware a chance .. ?
    select(0, NULL, NULL, NULL, &tv);

/* write loop 

    fprintf(stderr, "writing to serial port..\n");

    for(size_t i = 0; i < strlen(helloBytes); i++)
    {
      Serial1.writeByte((const uint8_t)helloBytes[i]);
    }
*/

/*  read loop */
	  idx = 0;

    sleep(0);

	  memset(input, '\0', sizeof(input));
    count = Serial1.readByte(input, 3);
    fprintf(stderr,"read [%d] bytes [%s]\n", count, input);

    sleep(0);

	  memset(input, '\0', sizeof(input));
    count = Serial1.readByte(input, 4);
    fprintf(stderr,"read [%d] bytes [%s]\n", count, input);

    sleep(0);

	  memset(input, '\0', sizeof(input));
    count = Serial1.readByte(input, 3);
    fprintf(stderr,"read [%d] bytes [%s]\n", count, input);

/*
	  //printf("available bytes: %d\n", Serial1.available());
  	while(Serial1.available())
    {
  		input[idx] = Serial1.readByte();

      if( ('\n' == input[idx++]) )
      {
        fprintf(stderr, "read line: %s\n", input);
        break;
      }
    }
*/
  }
}
