#ifndef __COMMANDS__
#define __COMMANDS__
#include <inttypes.h>

// temp controller read and write commands
static const uint16_t   CTRLR_READ_FUNC   = 0x03;
static const uint16_t   CTRLR_WRITE_FUNC  = 0x06;


class cmdResp
{
  public:
  cmdResp(bool retCode, uint8_t* buff, uint8_t bufflen)
    : m_retCode(retCode), m_buff(buff), m_bufflen(bufflen) {}

  bool      m_retCode;  // true is success, false is failed
  uint8_t*  m_buff;     // for read cmd, this is the data that came back
                        // or , should this just be the whole packet . . ?
  uint16_t  m_bufflen;  // the lenght of the data in the buff
};


class commandBase
{
  public:
  commandBase();
  virtual ~commandBase();
  virtual uint8_t buildCmd(uint8_t* buff, uint8_t bufflen, uint16_t dataCnt, uint8_t id) = 0;
  virtual cmdResp buildResp(uint8_t*, uint8_t, uint16_t, uint8_t)  = 0;

  commandBase(const commandBase&) = delete;
  commandBase& operator=(const commandBase&) = delete;

  static bool validateCtrlrRxPkt(uint8_t* buff, uint8_t bufflen, uint16_t cmd, uint8_t id, bool=false, uint16_t=0);

  protected:

  private:
};

//
// commands that can be sent
// will be a struct for each command
//
class readPVPVOF : public commandBase
{
  public:
  readPVPVOF();
  virtual ~readPVPVOF();

  uint8_t buildCmd(uint8_t* buff, uint8_t bufflen, uint16_t dataCnt, uint8_t id);
  cmdResp buildResp(uint8_t*, uint8_t, uint16_t, uint8_t);
};

class writeSV : public commandBase
{
  public:
  writeSV();
  virtual ~writeSV();

  uint8_t buildCmd(uint8_t* buff, uint8_t bufflen, uint16_t dataCnt, uint8_t id);
  cmdResp buildResp(uint8_t*, uint8_t, uint16_t, uint8_t);
};


// and many more ..

#endif

