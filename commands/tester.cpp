#include <unistd.h>
#include <stdio.h>
#include <sys/select.h>
#include <inttypes.h>
#include <string.h>
#include "commands.h"
#include "ctrl_commands.h"

uint8_t buff[64];


int main(int argc, char** argv)
{
  uint8_t len = 64;
  uint16_t  dataCnt = 6;
  uint16_t  data  = 0x0226;
  uint8_t id  = 1;
  uint8_t length;



  cmd  r;
  length = r.buildReadCmd(buff, len, dataCnt, id, SV);

  printf("made read command length(%u) : 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",
    length, buff[0], buff[1], buff[2], buff[3], buff[4], buff[5], buff[6], buff[7]);

  cmd s;
  s.buildWriteCmd(buff, len, data, id, PVPVOF);

  printf("made write command length(%u): 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",
    length, buff[0], buff[1], buff[2], buff[3], buff[4], buff[5], buff[6], buff[7]);

  uint8_t rx_buff[64];
  len = 8;
  rx_buff[0] = 0x01; rx_buff[1] = 0x06; rx_buff[2] = 0x00; rx_buff[3] = 0x00; rx_buff[4] = 0x26; rx_buff[5] = 0x02; rx_buff[6] = 0x27; rx_buff[7] = 0x04;

  cmdResp resp = s.buildWriteResp(rx_buff, len, 0x0226, id);

  printf("cmdResp.m_retCode is: %d\n", resp.m_retCode);

  memset(rx_buff, '\0', 64);
  
  rx_buff[0] = 0x01; rx_buff[1] = 0x03; rx_buff[2] = 0x06; rx_buff[3] = 0x00; rx_buff[4] = 0x01; rx_buff[5] = 0x02; rx_buff[6] = 0x03; rx_buff[7] = 0x04; rx_buff[8] = 0x05; rx_buff[9] = 0x06;
  rx_buff[10] = 0xc6; rx_buff[11] = 0x1b;

  len = 12;

  cmdResp resp2 = r.buildReadResp(rx_buff, len, 6, id);
  printf("cmdResp2.m_retCode is: %d\n", resp2.m_retCode);
  printf("cmdResp2.m_bufflen is: %d\n", resp2.m_bufflen);
  printf("cmdResp2 buff contents : ");
  for(int idx = 0; idx < resp2.m_bufflen; idx++)
    printf("0x%02x ", resp2.m_buff[idx]);

  printf("\n");

}
