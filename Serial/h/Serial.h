// simple class to mock the Serial object from Arduino land
// only doin N81 for now

#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <inttypes.h>
#include <termios.h>
#include <string>


class Serial
{
  public:
  Serial(const std::string&, const int);
  bool begin();
  virtual ~Serial();
  uint8_t readByte(void);
  uint8_t readByte(uint8_t*, ssize_t);
  const size_t writeByte(const uint8_t);
  const size_t writeByte(const uint8_t*);
  const size_t writeByte(const uint8_t*, ssize_t);
  int flush(const int = TCIOFLUSH);
  int drain();

  const size_t available();        // The number of bytes available to read

  protected:
  int         m_fd;
  FILE*       m_FILE;
  std::string m_port;
  int         m_speed;

  private:
  Serial(const Serial&);
  const Serial& operator=(const Serial&);
  const Serial& operator=(Serial&&);
};

