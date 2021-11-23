// file controlProtocol.cpp
#include <stdio.h>
#include <stdlib.h>

#ifdef __RUNNING_ON_CONTROLLINO__
    #if defined(ARDUINO) && ARDUINO >= 100
        #include "Arduino.h"
    #else
        #include "WProgram.h"
    #endif
#endif

#include "controlProtocol.h"



// crc16.cpp
uint16_t getCRC16(uint16_t CRC, uint8_t byte)
{
    CRC = ( (CRC % 256) << 8 ) ^ ( CRC16_table_C[ (CRC >> 8) ^ byte ] );
    return (CRC);
}


uint16_t calcCRC16(uint8_t* pBuff, uint16_t length)
{
    uint16_t    CRC = 0;


    for(uint16_t i = 0; i < length; i++)
    {
        CRC = getCRC16(CRC, pBuff[i]);
    }

    return(CRC);
}
// end crc16.cpp


bool controlProtocol::openUSBPort(const char* usbPort, uint32_t Speed)
{
    bool    retVal  = false;


#if defined(__USING_LINUX_USB__)

    struct  termios options;


    m_fd = open(usbPort, O_RDWR | O_NOCTTY | O_NDELAY);
    if( (m_fd == -1) )
    {
        fprintf(stderr, "unable to open %s\n", usbPort);
        retVal  = false;
    } else
    {
        // FNDELAY makes the fd non-blocking, doing a blocking calls
        fcntl(m_fd, F_SETFL, 0);
        retVal  = true;
 
        //
        // and set 9600N81
        //
        tcgetattr(m_fd, &options);
    
        switch(Speed)
        {
            case 19200:
                cfsetispeed(&options, B19200);
                cfsetospeed(&options, B19200);
                break;
    
            case 38400:
                cfsetispeed(&options, B38400);
                cfsetospeed(&options, B38400);
                break;
    
            case 57600:
                cfsetispeed(&options, B57600);
                cfsetospeed(&options, B57600);
                break;
    
            default:
            case 9600:
                cfsetispeed(&options, B9600);
                cfsetospeed(&options, B9600);
                break;
        }
    
        // Enable the receiver and set local mode...
        options.c_cflag |= (CLOCAL | CREAD);
    
        // 8 data bits
        //options.c_cflag &= ~CSIZE; /* Mask the character size bits */
        //options.c_cflag |= CS8;    /* Select 8 data bits */
    
        // no parity
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;
    
        // no hardware flow control
        options.c_cflag &= ~CRTSCTS;
    
        // use raw input
        //options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        options.c_lflag &= ~(ICANON | ISIG);
    
        // no software flow control
        options.c_iflag &= ~(IXON | IXOFF | IXANY | INLCR | ICRNL);
    
        // raw output
        options.c_oflag &= ~(OPOST | ONLCR);
    
        // have 5 second timeout
        options.c_cc[VMIN] = 0;
        options.c_cc[VTIME]= 50;
    
        //Set the new options for the port...
        tcsetattr(m_fd, TCSANOW, &options);
    }

#endif

#ifdef __USING_WINDOWS_USB__

    // Initializing DCB structure
    DCB dcbSerialParams = { 0 };
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
    
    m_fd = CreateFile(usbPort,                //port name
                      GENERIC_READ | GENERIC_WRITE, //Read/Write
                      0,                            // No Sharing
                      NULL,                         // No Security
                      OPEN_EXISTING,// Open existing port only
                      0,            // Non Overlapped I/O
                      NULL);        // Null for Comm Devices
                      
    if (m_fd == INVALID_HANDLE_VALUE)
        fprintf(stderr, "unable to open %s\n", usbPort);
    else
    {
        retVal  = true;
        
        if(true == GetCommState(m_fd, &dcbSerialParams) )
        {
            // set the attributes we so desire ..  mm get some !
            // speed
            switch(Speed)
            {
                case 14400:
                    dcbSerialParams.BaudRate = CBR_14400;
                    break;
        
                case 19200:
                    dcbSerialParams.BaudRate = CBR_19200;
                    break;
        
                case 38400:
                    dcbSerialParams.BaudRate = CBR_38400;
                    break;
        
                case 57600:
                    dcbSerialParams.BaudRate = CBR_57600;
                    break;
        
                default:
                case 9600:
                    dcbSerialParams.BaudRate = CBR_9600;
                    break;
            }
            
            // set 8N1 parity
            dcbSerialParams.ByteSize = 8;         // Setting ByteSize = 8
            dcbSerialParams.StopBits = ONESTOPBIT;// Setting StopBits = 1
            dcbSerialParams.Parity   = NOPARITY;  // Setting Parity = None
            
            // binary mode
            dcbSerialParams.fBinary = true;

            // no software and hardware flow control
            dcbSerialParams.fOutxCtsFlow        = false;
            dcbSerialParams.fOutxDsrFlow        = false;
            dcbSerialParams.fDtrControl         = DTR_CONTROL_DISABLE;
            dcbSerialParams.fDsrSensitivity     = false;
            dcbSerialParams.fTXContinueOnXoff   = false;
            dcbSerialParams.fOutX               = false;
            dcbSerialParams.fInX                = false;
            dcbSerialParams.fErrorChar          = false;
            dcbSerialParams.fErrorChar          = false;
            dcbSerialParams.fRtsControl         = RTS_CONTROL_DISABLE;
            dcbSerialParams.fAbortOnError       = false;
            //XonLim                          = 0; // TODO: what is this
            //XoffLim                         = 0; // TODO: what is this
            //XonChar                       = ?; // not changing
            //XoffChar                      = ?; // not changing
            //ErrorChar                     = ?; // not changing
            //EofChar                       = ?; // not changing
            //EvtChar                       = ?; // not changing

            // set the new serial comm settings
            if( (false == (SetCommState(m_fd, &dcbSerialParams))) )
            {
                fprintf(stderr, "unable to SetCommState\n");
                retVal = false;
            }
        } else
        {
            fprintf(stderr, "unable to GetCommState\n");
            retVal = false;
        }
    }

#endif
    return(retVal);
}


//
// TODO: throw if this fails
//
controlProtocol::controlProtocol(uint16_t myAddress, uint16_t peerAddress, const char* usbPort, uint32_t Speed)
    : m_seqNum(0x0000), m_myAddress(myAddress), m_peerAddress(peerAddress)
{
    //
    // only coding this for the master for now .. 
    //
    TxCommand   = &controlProtocol::TxCommandUSB;
    RxResponse  = &controlProtocol::RxResponseUSB;

    //
    // assuming this always passes TODO: don't assume
    //
    openUSBPort(usbPort, Speed);
};


//
// TODO: throw if this fails
//
controlProtocol::controlProtocol(uint16_t myAddress, uint16_t peerAddress, uint32_t Speed)
    : m_seqNum(0x0000), m_myAddress(myAddress), m_peerAddress(peerAddress)
{
    //
    // for now this will always be the Controllino
    //
    RxCommand   = &controlProtocol::RxCommandSerial;
    TxResponse  = &controlProtocol::TxResponseSerial;

    #ifdef __RUNNING_ON_CONTROLLINO__
    Serial1.begin(Speed, SERIAL_8N1);
    #endif
};


controlProtocol::~controlProtocol()
{
#if defined(__USING_LINUX_USB__)
    close(m_fd);
#endif

#ifdef __USING_WINDOWS_USB__
    CloseHandle(m_fd);
#endif
};


bool controlProtocol::TxCommandUSB(uint16_t length)
{
    #ifdef __DEBUG_CTRL_PROTO__
    printf(__PRETTY_FUNCTION__);
    printf("\n");
    printf("writing %u bytes: ", length);
    for(int i = 0; i < length; i++)
    {
        printf("0x%02X ", m_buff[i]);
    }
    printf("\n");
    #endif
    
    #if defined(__USING_LINUX_USB__)
    int n = write(m_fd, m_buff, length);

    if( (n < 0) )
    {
        fprintf(stderr, "%s write() failed\n", __PRETTY_FUNCTION__);
    }
    #endif

    
    #ifdef __USING_WINDOWS_USB__
    DWORD dNoOfBytesWritten = 0;     // No of bytes written to the port
    
    if( false == WriteFile(m_fd, m_buff, length, &dNoOfBytesWritten, NULL) )
    {
        fprintf(stderr, "%s WriteFile() failed\n", __PRETTY_FUNCTION__);
    }
    #endif

    return(true);
}


bool controlProtocol::RxResponseUSB(uint16_t timeout)
{
#if defined(__USING_LINUX_USB__)
    uint32_t        nbytes  = 0;
    uint32_t        length;
    uint8_t*        bufptr;
    msgHeader_t*    pmsgHeader;
    struct  termios options;


    //
    // set the passed in timeout - VTIME wants 10ths of a second
    // TODO: assuming the caller will provide milliseconds !
    //
    tcgetattr(m_fd, &options);          // get the attribures (all)
    options.c_cc[VMIN] = 0;             // update the VMIN and VTIME
    options.c_cc[VTIME]= timeout / 100; 
    tcsetattr(m_fd, TCSANOW, &options); // set the attributes

    //
    // clear da buffa' bra'
    //
    memset(m_buff, '\0', MAX_BUFF_LENGTH_CP + 1);

    //
    // read the message header
    //
    bufptr = m_buff;
    while ((nbytes = read(m_fd, bufptr, m_buff + sizeof(msgHeader_t) - bufptr)) > 0)
    {
        bufptr += nbytes;
    }

    //
    // read the rest of the message using length
    //
    pmsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
    length  = pmsgHeader->length;
    while( (nbytes = read(m_fd, bufptr, length - (bufptr - m_buff))) > 0)
    {
        bufptr += nbytes;
    }

#endif

#ifdef __USING_WINDOWS_USB__
    DWORD           nbytes  = 0;
    DWORD           length;
    uint8_t*        bufptr;
    msgHeader_t*    pmsgHeader;
    COMMTIMEOUTS    timeouts = { 0 };


    timeouts.ReadIntervalTimeout         = timeout; // in milliseconds
    timeouts.ReadTotalTimeoutConstant    = timeout; // in milliseconds
    timeouts.ReadTotalTimeoutMultiplier  = timeout; // in milliseconds
    timeouts.WriteTotalTimeoutConstant   = timeout; // in milliseconds
    timeouts.WriteTotalTimeoutMultiplier = timeout; // in milliseconds

    SetCommTimeouts(m_fd, &timeouts);  // assuming this works 'everytime' !

    //
    // clear da buffa' bra'
    //
    memset(m_buff, '\0', MAX_BUFF_LENGTH_CP + 1);

    //
    // read the message header
    //
    bufptr = m_buff;
    while( (ReadFile(m_fd, static_cast<void*>(bufptr),
        m_buff + sizeof(msgHeader_t) - bufptr, &nbytes, NULL)) )
    {
        bufptr += nbytes;
        nbytes  = 0;
        if( 0 == (m_buff + sizeof(msgHeader_t) - bufptr)) break;
    }

    //
    // read the rest of the message using length
    //
    pmsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
    length  = pmsgHeader->length;
    nbytes  = 0;
    while( (ReadFile(m_fd, bufptr, length - (bufptr - m_buff), &nbytes, NULL)) )
    {
        bufptr += nbytes;
        nbytes  = 0;
        if( (0 == (length - (bufptr - m_buff)))) break;
    }

#endif

    return(true);
}


bool controlProtocol::RxCommandSerial(uint16_t TimeoutMs)
{
    bool retVal             = false;
#ifdef __RUNNING_ON_CONTROLLINO__
    bool done               = false;
    bool gotSTX             = false;     // TODO: for now dont' find a defined start char
    bool gotLength          = false;
    bool timedOut           = false;
    int32_t bytes_read      = 0;
    int32_t length          = MAX_BUFF_LENGTH_CP;   // adjusts when pkt length is read
    const uint8_t STX       = COMMAND;
    unsigned long startTime = millis();


    memset(reinterpret_cast<void*>(m_buff), '\0', MAX_BUFF_LENGTH_CP + 1);

    // try to read a packet for a total of TimeoutMs milliseconds
    while( (!done) && (!timedOut) &&
            ((bytes_read < (length)) && (bytes_read < MAX_BUFF_LENGTH_CP)) )
    {
        if( ((millis() - startTime) > TimeoutMs) )
        {
            timedOut = true;
        } else
        {
            if( (Serial1.available()) )
            {
                m_buff[bytes_read] = Serial1.read();

                //
                // look for start of frame
                //
                if( (!gotSTX) )
                {
                    if( (STX == m_buff[bytes_read]) )
                    {
                        // TODO: restart startTime here, give more time to get the packet?
                        gotSTX = true;
                        bytes_read += 1;
                    } // else don't increment bytes_read effectively discarding this byte

                    continue;
                }

                //
                // length is byte 2 (1 for zero based count)
                //
                if( (!gotLength) )
                {
                    gotLength  = true;
                    length = m_buff[bytes_read++];
                    #ifdef __DEBUG_CONTROL_PKT_RX__
                    Serial.print("RxCommandSerial found length: ");
                    Serial.print(length, HEX);
                    Serial.println("");
                    Serial.flush();
                    #endif
                    continue;
                }


                //
                // read the rest of the packet
                //
                bytes_read += 1;

            } else
            {
                // TODO: too long, too short ?
                // no data available, wait a bit before checking again
                //Serial.println("Serial1 no bytes available");
                delay(100);
            }
        }
    }

    // always null terminate just in case we want to dump out for debug
    m_buff[bytes_read] = 0;


    // debug stuff
    #ifdef __DEBUG_CONTROL_PKT_RX__
    Serial.print(__PRETTY_FUNCTION__);
    Serial.flush();
    Serial.print(" received ");
    Serial.flush();
    Serial.print(bytes_read, DEC);
    Serial.flush();
    Serial.println(" bytes");
    Serial.flush();
    for(int i = 0; i < bytes_read; i++)
    {
        Serial.print(m_buff[i], HEX);
        Serial.print(" ");
    }
    Serial.println("");
    Serial.flush();
    #endif

    if( (length == (bytes_read)) )
        retVal = true;

    #ifdef __DEBUG_CONTROL_ERROR__
    if( !(retVal) )
    {
        Serial.println("RxCommand found bad formatted packet");
        Serial.flush();
    }
    #endif

#endif
    return(retVal);
}


bool controlProtocol::TxResponseSerial(uint16_t length)
{
    bool    retVal  = true;
#ifdef __RUNNING_ON_CONTROLLINO__
    uint8_t lenWritten;

    // class member Buff is filled in by the member functions
    lenWritten = Serial1.write(m_buff, length);
    Serial1.flush();

    if( (lenWritten != length) )
    {
        #ifdef __DEBUG_HUBER_ERROR__
        Serial.flush();
        Serial.println("TxCommand failed");
        #endif
        retVal  = false;
    #ifdef __DEBUG_CONTROL_PKT_TX__
    } else
    {
        Serial.flush();
        Serial.print(__PRETTY_FUNCTION__);
        Serial.print(" sent: ");
        for(int i = 0; i < length; i++)
        {
            Serial.print(reinterpret_cast<uint8_t>(m_buff[i]), HEX);
            Serial.print(" ");
        }
        Serial.println("");
        Serial.flush();
    #endif
    }

#endif

    return(retVal);
}


bool controlProtocol::GetStatus(uint16_t destAddress, uint16_t* humidityAlert,
                            uint16_t* TECsRunning,  uint16_t* chillerOnLine)
{
    bool                retVal  = false;
    uint16_t            seqNum;
    msgHeader_t*        pMsgHeader;
    getStatusResp_t*    pgetStatusResp;


    //
    // increment the sequence number for this transaction
    //
    ++m_seqNum;

    if( (doTxCommand(Make_getStatus(destAddress, m_buff))) )
    {
        //
        // save the seqNum
        //
        pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
        seqNum  = pMsgHeader->seqNum;

        // get the return packet
        if( (doRxResponse(COMM_TIMEOUT)) )
        {
            #ifdef __DEBUG_CTRL_PROTO__
            //
            // dump out what we got
            //
            for(uint16_t i = 0; i < sizeof(getStatusResp_t); i++)
            {
                printf("0x%02X ", m_buff[i]);
            }
            printf("\n");
            #endif

            //
            // check got the expected message number
            //
            pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
            if( (getStatusResp != pMsgHeader->msgNum) )
            {
                fprintf(stderr, "ERROR: %s got unexpected msg %hu\n",
                    __PRETTY_FUNCTION__, pMsgHeader->msgNum);

                //
                // no need to continue processing
                //
                return(false);
            }

            //
            // cast into the buffer, pick up the CRC
            //
            pgetStatusResp = reinterpret_cast<getStatusResp_t*>(m_buff);

            //
            // verify seqNum and CRC
            //
            if( !(verifyMessage(len_getStatusResp_t, ntohs(pgetStatusResp->crc),
                                            seqNum, ntohs(pgetStatusResp->eop))) )
            {
                // TODO: drop the packet
                fprintf(stderr, "ERROR: %s CRC bad, seqNum mismatch, or wrong address\n",
                        __PRETTY_FUNCTION__);

                //
                // no need to continue processing
                //
                return(false);
            }


            //
            // report the health
            //
            Parse_getStatusResp(m_buff, humidityAlert, TECsRunning, chillerOnLine, &seqNum);

            #ifdef __DEBUG_CTRL_PROTO__
            printf("found in packet humidityAlert %u TECsRunning %u, chillerOnLine %u, seqNumer 0x%02x\n",
                *humidityAlert, *TECsRunning, *chillerOnLine, seqNum);
            #endif

            retVal  = true;
        } else
        {
            fprintf(stderr, "ERROR: did not get a m_buffer back\n");
        }

    } else
    {
        fprintf(stderr, "ERROR: unable to Make_getStatus\n");
    }

    return(retVal);
}


bool controlProtocol::GetHumidity(uint16_t destAddress, float* humidity)
{
    bool                retVal  = false;
    uint16_t            seqNum;
    msgHeader_t*        pMsgHeader;
    getHumidityResp_t*    pgetHumidityResp;


    //
    // increment the sequence number for this transaction
    //
    ++m_seqNum;

    if( (doTxCommand(Make_getHumidity(destAddress, m_buff))) )
    {
        //
        // save the seqNum
        //
        pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
        seqNum  = pMsgHeader->seqNum;

        // get the return packet
        if( (doRxResponse(COMM_TIMEOUT)) )
        {
            #ifdef __DEBUG_CTRL_PROTO__
            //
            // dump out what we got
            //
            for(uint16_t i = 0; i < sizeof(getHumidityResp_t); i++)
            {
                printf("0x%02X ", m_buff[i]);
            }
            printf("\n");
            #endif

            //
            // check got the expected message number
            //
            pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
            if( (getHumidityResp != pMsgHeader->msgNum) )
            {
                fprintf(stderr, "ERROR: %s got unexpected msg %hu\n",
                    __PRETTY_FUNCTION__, pMsgHeader->msgNum);

                //
                // no need to continue processing
                //
                return(false);
            }

            //
            // cast into the buffer, pick up the CRC
            //
            pgetHumidityResp = reinterpret_cast<getHumidityResp_t*>(m_buff);

            //
            // verify seqNum and CRC
            //
            if( !(verifyMessage(len_getHumidityResp_t, ntohs(pgetHumidityResp->crc),
                                    seqNum, ntohs(pgetHumidityResp->eop))) )
            {
                // TODO: drop the packet
                fprintf(stderr, "ERROR: %s CRC bad, seqNum mismatch, or wrong address\n",
                        __PRETTY_FUNCTION__);

                //
                // no need to continue processing
                //
                return(false);
            }


            Parse_getHumidityResp(m_buff, humidity, &seqNum);

            #ifdef __DEBUG_CTRL_PROTO__
            printf("found in packet humidity %lf, seqNumer 0x%02x\n", *humidity, seqNum);
            #endif
            
            retVal  = true;
        } else
        {
            fprintf(stderr, "ERROR: did not get a m_buffer back\n");
        }

    } else
    {
        fprintf(stderr, "ERROR: unable to Make_getStatus\n");
    }

    return(retVal);
}


bool controlProtocol::SetHumidityThreshold(uint16_t destAddress, uint16_t threshold)
{
    bool                retVal  = false;
    uint16_t            seqNum;
    uint16_t            result;
    msgHeader_t*        pMsgHeader;
    setHumidityThresholdResp_t*    psetHumidityThresholdResp;


    //
    // increment the sequence number for this transaction
    //
    ++m_seqNum;

    if( (doTxCommand(Make_setHumidityThreshold(destAddress, m_buff, threshold))) )
    {
        //
        // save the seqNum
        //
        pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
        seqNum  = pMsgHeader->seqNum;

        // get the return packet
        if( (doRxResponse(COMM_TIMEOUT)) )
        {
            #ifdef __DEBUG_CTRL_PROTO__
            //
            // dump out what we got
            //
            for(uint16_t i = 0; i < sizeof(setHumidityThresholdResp_t); i++)
            {
                printf("0x%02X ", m_buff[i]);
            }
            printf("\n");
            #endif

            //
            // check got the expected message number
            //
            pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
            if( (setHumidityThresholdResp != pMsgHeader->msgNum) )
            {
                fprintf(stderr, "ERROR: %s got unexpected msg %hu\n",
                    __PRETTY_FUNCTION__, pMsgHeader->msgNum);

                //
                // no need to continue processing
                //
                return(false);
            }

            //
            // cast into the buffer, pick up the CRC
            //
            psetHumidityThresholdResp = reinterpret_cast<setHumidityThresholdResp_t*>(m_buff);

            //
            // verify seqNum and CRC
            //
            if( !(verifyMessage(len_setHumidityThresholdResp_t, ntohs(psetHumidityThresholdResp->crc),
                                            seqNum, ntohs(psetHumidityThresholdResp->eop))) )
            {
                // TODO: drop the packet
                fprintf(stderr, "ERROR: %s CRC bad, seqNum mismatch, or wrong address\n",
                        __PRETTY_FUNCTION__);

                //
                // no need to continue processing
                //
                return(false);
            }


            //
            // report the health
            //
            Parse_setHumidityThresholdResp(m_buff, &result, &seqNum);

            #ifdef __DEBUG_CTRL_PROTO__
            printf("found in packet result %d seqNumer 0x%02x\n", result, seqNum);
            #endif

            if( (result) )
                retVal  = true;
            else
                retVal  = false;
        } else
        {
            fprintf(stderr, "ERROR: did not get a m_buffer back\n");
        }

    } else
    {
        fprintf(stderr, "ERROR: unable to Make_getStatus\n");
    }

    return(retVal);
}


bool controlProtocol::GetHumidityThreshold(uint16_t destAddress, uint16_t* threshold)
{
    bool                retVal  = false;
    uint16_t            seqNum;
    msgHeader_t*        pMsgHeader;
    getHumidityThresholdResp_t*    pgetHumidityThresholdResp;


    //
    // increment the sequence number for this transaction
    //
    ++m_seqNum;

    if( (doTxCommand(Make_getHumidityThreshold(destAddress, m_buff))) )
    {
        //
        // save the seqNum
        //
        pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
        seqNum  = pMsgHeader->seqNum;

        // get the return packet
        if( (doRxResponse(COMM_TIMEOUT)) )
        {
            #ifdef __DEBUG_CTRL_PROTO__
            //
            // dump out what we got
            //
            for(uint16_t i = 0; i < sizeof(getHumidityThresholdResp_t); i++)
            {
                printf("0x%02X ", m_buff[i]);
            }
            printf("\n");
            #endif

            //
            // check got the expected message number
            //
            pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
            if( (getHumidityThresholdResp != pMsgHeader->msgNum) )
            {
                fprintf(stderr, "ERROR: %s got unexpected msg %hu\n",
                    __PRETTY_FUNCTION__, pMsgHeader->msgNum);

                //
                // no need to continue processing
                //
                return(false);
            }

            //
            // cast into the buffer, pick up the CRC
            //
            pgetHumidityThresholdResp = reinterpret_cast<getHumidityThresholdResp_t*>(m_buff);

            //
            // verify seqNum and CRC
            //
            if( !(verifyMessage(len_getHumidityThresholdResp_t, ntohs(pgetHumidityThresholdResp->crc),
                                            seqNum, ntohs(pgetHumidityThresholdResp->eop))) )
            {
                // TODO: drop the packet
                fprintf(stderr, "ERROR: %s CRC bad, seqNum mismatch, or wrong address\n",
                        __PRETTY_FUNCTION__);

                //
                // no need to continue processing
                //
                return(false);
            }


            //
            // report the health
            //
            Parse_getHumidityThresholdResp(m_buff, threshold, &seqNum);

            #ifdef __DEBUG_CTRL_PROTO__
            printf("found in packet threshold %d seqNumer 0x%02x\n", *threshold, seqNum);
            #endif

            retVal  = true;
        } else
        {
            fprintf(stderr, "ERROR: did not get a m_buffer back\n");
        }

    } else
    {
        fprintf(stderr, "ERROR: unable to Make_getStatus\n");
    }

    return(retVal);
}


bool controlProtocol::SetTECTemperature(uint16_t destAddress, uint16_t tec_address, float temperature)
{
    bool                retVal  = false;
    uint16_t            seqNum;
    uint16_t            result;
    msgHeader_t*        pMsgHeader;
    setTECTemperatureResp_t*    psetTECTemperatureResp;


    //
    // increment the sequence number for this transaction
    //
    ++m_seqNum;

    if( (doTxCommand(Make_setTECTemperature(destAddress, m_buff, tec_address, temperature))) )
    {
        //
        // save the seqNum
        //
        pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
        seqNum  = pMsgHeader->seqNum;

        // get the return packet
        if( (doRxResponse(COMM_TIMEOUT)) )
        {
            #ifdef __DEBUG_CTRL_PROTO__
            //
            // dump out what we got
            //
            for(uint16_t i = 0; i < sizeof(setTECTemperatureResp_t); i++)
            {
                printf("0x%02X ", m_buff[i]);
            }
            printf("\n");
            #endif

            //
            // check got the expected message number
            //
            pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
            if( (setTECTemperatureResp != pMsgHeader->msgNum) )
            {
                fprintf(stderr, "ERROR: %s got unexpected msg %hu\n",
                    __PRETTY_FUNCTION__, pMsgHeader->msgNum);

                //
                // no need to continue processing
                //
                return(false);
            }

            //
            // cast into the buffer, pick up the CRC
            //
            psetTECTemperatureResp = reinterpret_cast<setTECTemperatureResp_t*>(m_buff);

            //
            // verify seqNum and CRC
            //
            if( !(verifyMessage(len_setTECTemperatureResp_t, ntohs(psetTECTemperatureResp->crc),
                                        seqNum, ntohs(psetTECTemperatureResp->eop))) )
            {
                // TODO: drop the packet
                fprintf(stderr, "ERROR: %s CRC bad, seqNum mismatch, or wrong address\n",
                        __PRETTY_FUNCTION__);

                //
                // no need to continue processing
                //
                return(false);
            }


            //
            // report the health
            //
            Parse_setTECTemperatureResp(m_buff, &result, &seqNum);

            #ifdef __DEBUG_CTRL_PROTO__
            printf("found in packet result %d seqNumer 0x%02x\n", result, seqNum);
            #endif

            if( (result) )
                retVal  = true;
            else
                retVal  = false;
        } else
        {
            fprintf(stderr, "ERROR: did not get a m_buffer back\n");
        }

    } else
    {
        fprintf(stderr, "ERROR: unable to Make_getStatus\n");
    }

    return(retVal);
}


bool controlProtocol::GetTECTemperature(uint16_t destAddress, uint16_t tec_address, uint16_t* result, float* temperature)
{
    bool                retVal  = false;
    uint16_t            seqNum;
    msgHeader_t*        pMsgHeader;
    getTECTemperatureResp_t*    pgetTECTemperatureResp;


    //
    // increment the sequence number for this transaction
    //
    ++m_seqNum;

    if( (doTxCommand(Make_getTECTemperature(destAddress, m_buff, tec_address))) )
    {
        //
        // save the seqNum
        //
        pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
        seqNum  = pMsgHeader->seqNum;

        // get the return packet
        if( (doRxResponse(COMM_TIMEOUT)) )
        {
            #ifdef __DEBUG_CTRL_PROTO__
            //
            // dump out what we got
            //
            for(uint16_t i = 0; i < sizeof(getTECTemperatureResp_t); i++)
            {
                printf("0x%02X ", m_buff[i]);
            }
            printf("\n");
            #endif

            //
            // check got the expected message number
            //
            pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
            if( (getTECTemperatureResp != pMsgHeader->msgNum) )
            {
                fprintf(stderr, "ERROR: %s got unexpected msg %hu\n",
                    __PRETTY_FUNCTION__, pMsgHeader->msgNum);

                //
                // no need to continue processing
                //
                return(false);
            }

            //
            // cast into the buffer, pick up the CRC
            //
            pgetTECTemperatureResp = reinterpret_cast<getTECTemperatureResp_t*>(m_buff);

            //
            // verify seqNum and CRC
            //
            if( !(verifyMessage(len_getTECTemperatureResp_t, ntohs(pgetTECTemperatureResp->crc),
                                            seqNum, ntohs(pgetTECTemperatureResp->eop))) )
            {
                // TODO: drop the packet
                fprintf(stderr, "ERROR: %s CRC bad, seqNum mismatch, or wrong address\n",
                        __PRETTY_FUNCTION__);

                //
                // no need to continue processing
                //
                return(false);
            }


            //
            // report the health
            //
            Parse_getTECTemperatureResp(m_buff, result, temperature, &seqNum);

            #ifdef __DEBUG_CTRL_PROTO__
            printf("found in packet temperature %lf seqNumer 0x%02x\n", *temperature, seqNum);
            #endif

            if( (result) )
                retVal  = true;
            else
                retVal  = false;
        } else
        {
            fprintf(stderr, "%s ERROR: did not get a m_buffer back\n", __PRETTY_FUNCTION__);
        }

    } else
    {
        fprintf(stderr, "%s ERROR: unable to Make_getStatus\n", __PRETTY_FUNCTION__);
    }

    return(retVal);
}

bool controlProtocol::GetTECObjTemperature(uint16_t destAddress, uint16_t tec_address, uint16_t* result, float* temperature)
{
    bool                retVal  = false;
    uint16_t            seqNum;
    msgHeader_t*        pMsgHeader;
    getTECObjTemperatureResp_t*    pgetTECObjTemperatureResp;


    //
    // increment the sequence number for this transaction
    //
    ++m_seqNum;

    if( (doTxCommand(Make_getTECObjTemperature(destAddress, m_buff, tec_address))) )
    {
        //
        // save the seqNum
        //
        pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
        seqNum  = pMsgHeader->seqNum;

        // get the return packet
        if( (doRxResponse(COMM_TIMEOUT)) )
        {
            #ifdef __DEBUG_CTRL_PROTO__
            //
            // dump out what we got
            //
            for(uint16_t i = 0; i < sizeof(getTECObjTemperatureResp_t); i++)
            {
                printf("0x%02X ", m_buff[i]);
            }
            printf("\n");
            #endif

            //
            // check got the expected message number
            //
            pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
            if( (getTECObjTemperatureResp != pMsgHeader->msgNum) )
            {
                fprintf(stderr, "ERROR: %s got unexpected msg %hu\n",
                    __PRETTY_FUNCTION__, pMsgHeader->msgNum);

                //
                // no need to continue processing
                //
                return(false);
            }

            //
            // cast into the buffer, pick up the CRC
            //
            pgetTECObjTemperatureResp = reinterpret_cast<getTECObjTemperatureResp_t*>(m_buff);

            //
            // verify seqNum and CRC
            //
            if( !(verifyMessage(len_getTECObjTemperatureResp_t, ntohs(pgetTECObjTemperatureResp->crc),
                                            seqNum, ntohs(pgetTECObjTemperatureResp->eop))) )
            {
                // TODO: drop the packet
                fprintf(stderr, "ERROR: %s CRC bad, seqNum mismatch, or wrong address\n",
                        __PRETTY_FUNCTION__);

                //
                // no need to continue processing
                //
                return(false);
            }


            //
            // report the health
            //
            Parse_getTECObjTemperatureResp(m_buff, result, temperature, &seqNum);

            #ifdef __DEBUG_CTRL_PROTO__
            printf("found in packet temperature %lf seqNumer 0x%02x\n", *temperature, seqNum);
            #endif 

            if( (result) )
                retVal  = true;
            else
                retVal  = false;
        } else
        {
            fprintf(stderr, "%s ERROR: did not get a m_buffer back\n", __PRETTY_FUNCTION__);
        }

    } else
    {
        fprintf(stderr, "%s ERROR: unable to Make_getStatus\n", __PRETTY_FUNCTION__);
    }

    return(retVal);
}


bool controlProtocol::StartChiller(uint16_t destAddress)
{
    bool                retVal  = false;
    uint16_t            seqNum;
    uint16_t            result;
    msgHeader_t*        pMsgHeader;
    startChillerMsgResp_t* pstartChillerMsgResp;


    //
    // increment the sequence number for this transaction
    //
    ++m_seqNum;

    if( (doTxCommand(Make_startChillerMsg(destAddress, m_buff))) )
    {
        //
        // save the seqNum
        //
        pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
        seqNum  = pMsgHeader->seqNum;

        // get the return packet
        if( (doRxResponse(COMM_TIMEOUT)) )
        {
            #ifdef __DEBUG_CTRL_PROTO__
            //
            // dump out what we got
            //
            for(uint16_t i = 0; i < sizeof(startChillerMsgResp_t); i++)
            {
                printf("0x%02X ", m_buff[i]);
            }
            printf("\n");
            #endif

            //
            // check got the expected message number
            //
            pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
            if( (startChillerMsgResp != pMsgHeader->msgNum) )
            {
                fprintf(stderr, "ERROR: %s got unexpected msg %hu\n",
                    __PRETTY_FUNCTION__, pMsgHeader->msgNum);

                //
                // no need to continue processing
                //
                return(false);
            }

            //
            // cast into the buffer, pick up the CRC
            //
            pstartChillerMsgResp = reinterpret_cast<startChillerMsgResp_t*>(m_buff);

            //
            // verify seqNum and CRC
            //
            if( !(verifyMessage(len_startChillerMsgResp_t, ntohs(pstartChillerMsgResp->crc),
                                            seqNum, ntohs(pstartChillerMsgResp->eop))) )
            {
                // TODO: drop the packet
                fprintf(stderr, "ERROR: %s CRC bad, seqNum mismatch, or wrong address\n",
                        __PRETTY_FUNCTION__);

                //
                // no need to continue processing
                //
                return(false);
            }


            //
            // report the health
            //
            Parse_startChillerMsgResp(m_buff, &result, &seqNum);

            #ifdef __DEBUG_CTRL_PROTO__
            printf("found in packet result %d seqNumer 0x%02x\n", result, seqNum);
            #endif

            if( (result) )
                retVal  = true;
            else
                retVal  = false;
        } else
        {
            fprintf(stderr, "%s ERROR: did not get a m_buffer back\n", __PRETTY_FUNCTION__);
        }

    } else
    {
        fprintf(stderr, "%s ERROR: unable to Make_getStatus\n", __PRETTY_FUNCTION__);
    }

    return(retVal);
}


bool controlProtocol::StopChiller(uint16_t destAddress)
{
    bool                retVal  = false;
    uint16_t            seqNum;
    uint16_t            result;
    msgHeader_t*        pMsgHeader;
    stopChillerResp_t*  pstopChillerResp;


    //
    // increment the sequence number for this transaction
    //
    ++m_seqNum;

    if( (doTxCommand(Make_stopChiller(destAddress, m_buff))) )
    {
        //
        // save the seqNum
        //
        pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
        seqNum  = pMsgHeader->seqNum;

        // get the return packet
        if( (doRxResponse(COMM_TIMEOUT)) )
        {
            #ifdef __DEBUG_CTRL_PROTO__
            //
            // dump out what we got
            //
            for(uint16_t i = 0; i < sizeof(stopChillerResp_t); i++)
            {
                printf("0x%02X ", m_buff[i]);
            }
            printf("\n");
            #endif

            //
            // check got the expected message number
            //
            pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
            if( (stopChillerResp != pMsgHeader->msgNum) )
            {
                fprintf(stderr, "ERROR: %s got unexpected msg %hu\n",
                    __PRETTY_FUNCTION__, pMsgHeader->msgNum);

                //
                // no need to continue processing
                //
                return(false);
            }

            //
            // cast into the buffer, pick up the CRC
            //
            pstopChillerResp = reinterpret_cast<stopChillerResp_t*>(m_buff);

            //
            // verify seqNum and CRC
            //
            if( !(verifyMessage(len_stopChillerResp_t, ntohs(pstopChillerResp->crc),
                                            seqNum, ntohs(pstopChillerResp->eop))) )
            {
                // TODO: drop the packet
                fprintf(stderr, "ERROR: %s CRC bad, seqNum mismatch, or wrong address\n",
                        __PRETTY_FUNCTION__);

                //
                // no need to continue processing
                //
                return(false);
            }


            //
            // report the health
            //
            Parse_stopChillerResp(m_buff, &result, &seqNum);

            #ifdef __DEBUG_CTRL_PROTO__
            printf("found in packet result %d seqNumer 0x%02x\n", result, seqNum);
            #endif

            if( (result) )
                retVal  = true;
            else
                retVal  = false;
        } else
        {
            fprintf(stderr, "%s ERROR: did not get a m_buffer back\n", __PRETTY_FUNCTION__);
        }

    } else
    {
        fprintf(stderr, "%s ERROR: unable to Make_getStatus\n", __PRETTY_FUNCTION__);
    }

    return(retVal);
}


bool controlProtocol::SetChillerTemperature(uint16_t destAddress, float temperature)
{
    bool                retVal  = false;
    uint16_t            seqNum;
    uint16_t            result;
    msgHeader_t*        pMsgHeader;
    setChillerTemperatureResp_t*    psetChillerTemperatureResp;


    //
    // increment the sequence number for this transaction
    //
    ++m_seqNum;

    if( (doTxCommand(Make_setChillerTemperature(destAddress, m_buff, temperature))) )
    {
        //
        // save the seqNum
        //
        pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
        seqNum  = pMsgHeader->seqNum;

        // get the return packet
        if( (doRxResponse(COMM_TIMEOUT)) )
        {
            #ifdef __DEBUG_CTRL_PROTO__
            //
            // dump out what we got
            //
            for(uint16_t i = 0; i < sizeof(setChillerTemperatureResp_t); i++)
            {
                printf("0x%02X ", m_buff[i]);
            }
            printf("\n");
            #endif

            //
            // check got the expected message number
            //
            pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
            if( (setChillerTemperatureResp != pMsgHeader->msgNum) )
            {
                fprintf(stderr, "ERROR: %s got unexpected msg %hu\n",
                    __PRETTY_FUNCTION__, pMsgHeader->msgNum);

                //
                // no need to continue processing
                //
                return(false);
            }

            //
            // cast into the buffer, pick up the CRC
            //
            psetChillerTemperatureResp = reinterpret_cast<setChillerTemperatureResp_t*>(m_buff);

            //
            // verify seqNum and CRC
            //
            if( !(verifyMessage(len_setChillerTemperatureResp_t, ntohs(psetChillerTemperatureResp->crc),
                                            seqNum, ntohs(psetChillerTemperatureResp->eop))) )
            {
                // TODO: drop the packet
                fprintf(stderr, "ERROR: %s CRC bad, seqNum mismatch, or wrong address\n",
                        __PRETTY_FUNCTION__);

                //
                // no need to continue processing
                //
                return(false);
            }


            //
            // report the health
            //
            Parse_setChillerTemperatureResp(m_buff, &result, &seqNum);

            #ifdef __DEBUG_CTRL_PROTO__
            printf("found in packet result %d seqNumer 0x%02x\n", result, seqNum);
            #endif

            if( (result) )
                retVal  = true;
            else
                retVal  = false;
        } else
        {
            fprintf(stderr, "%s ERROR: did not get a m_buffer back\n", __PRETTY_FUNCTION__);
        }

    } else
    {
        fprintf(stderr, "%s ERROR: unable to Make_getStatus\n", __PRETTY_FUNCTION__);
    }

    return(retVal);
}


bool controlProtocol::GetChillerTemperature(uint16_t destAddress, float* temperature)
{
    bool                retVal  = false;
    uint16_t            seqNum;
    msgHeader_t*        pMsgHeader;
    getChillerTemperatureResp_t*    pgetChillerTemperatureResp;


    //
    // increment the sequence number for this transaction
    //
    ++m_seqNum;

    if( (doTxCommand(Make_getChillerTemperature(destAddress, m_buff))) )
    {
        //
        // save the seqNum
        //
        pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
        seqNum  = pMsgHeader->seqNum;

        // get the return packet
        if( (doRxResponse(COMM_TIMEOUT)) )
        {
            #ifdef __DEBUG_CTRL_PROTO__
            //
            // dump out what we got
            //
            for(uint16_t i = 0; i < sizeof(getChillerTemperatureResp_t); i++)
            {
                printf("0x%02X ", m_buff[i]);
            }
            printf("\n");
            #endif

            //
            // check got the expected message number
            //
            pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
            if( (getChillerTemperatureResp != pMsgHeader->msgNum) )
            {
                fprintf(stderr, "ERROR: %s got unexpected msg %hu\n",
                    __PRETTY_FUNCTION__, pMsgHeader->msgNum);

                //
                // no need to continue processing
                //
                return(false);
            }

            //
            // cast into the buffer, pick up the CRC
            //
            pgetChillerTemperatureResp = reinterpret_cast<getChillerTemperatureResp_t*>(m_buff);

            //
            // verify seqNum and CRC
            //
            if( !(verifyMessage(len_getChillerTemperatureResp_t, ntohs(pgetChillerTemperatureResp->crc),
                                            seqNum, ntohs(pgetChillerTemperatureResp->eop))) )
            {
                // TODO: drop the packet
                fprintf(stderr, "ERROR: %s CRC bad, seqNum mismatch, or wrong address\n",
                        __PRETTY_FUNCTION__);

                //
                // no need to continue processing
                //
                return(false);
            }


            //
            // report the health
            //
            Parse_getChillerTemperatureResp(m_buff, temperature, &seqNum);

            #ifdef __DEBUG_CTRL_PROTO__
            printf("found in packet temperature %lf seqNumer 0x%02x\n", *temperature, seqNum);
            #endif

            retVal  = true;
        } else
        {
            fprintf(stderr, "%s ERROR: did not get a m_buffer back\n", __PRETTY_FUNCTION__);
        }

    } else
    {
        fprintf(stderr, "%s ERROR: unable to Make_getStatus\n", __PRETTY_FUNCTION__);
    }

    return(retVal);
}


bool controlProtocol::GetChillerObjTemperature(uint16_t destAddress, float* temperature)
{
    bool                retVal  = false;
    uint16_t            seqNum;
    msgHeader_t*        pMsgHeader;
    getChillerObjTemperatureResp_t*    pgetChillerObjTemperatureResp;


    //
    // increment the sequence number for this transaction
    //
    ++m_seqNum;

    if( (doTxCommand(Make_getChillerObjTemperature(destAddress, m_buff))) )
    {
        //
        // save the seqNum
        //
        pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
        seqNum  = pMsgHeader->seqNum;

        // get the return packet
        if( (doRxResponse(COMM_TIMEOUT)) )
        {
            #ifdef __DEBUG_CTRL_PROTO__
            //
            // dump out what we got
            //
            for(uint16_t i = 0; i < sizeof(getChillerObjTemperatureResp_t); i++)
            {
                printf("0x%02X ", m_buff[i]);
            }
            printf("\n");
            #endif

            //
            // check got the expected message number
            //
            pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
            if( (getChillerObjTemperatureResp != pMsgHeader->msgNum) )
            {
                fprintf(stderr, "ERROR: %s got unexpected msg %hu\n",
                    __PRETTY_FUNCTION__, pMsgHeader->msgNum);

                //
                // no need to continue processing
                //
                return(false);
            }

            //
            // cast into the buffer, pick up the CRC
            //
            pgetChillerObjTemperatureResp = reinterpret_cast<getChillerObjTemperatureResp_t*>(m_buff);

            //
            // verify seqNum and CRC
            //
            if( !(verifyMessage(len_getChillerObjTemperatureResp_t, ntohs(pgetChillerObjTemperatureResp->crc),
                                            seqNum, ntohs(pgetChillerObjTemperatureResp->eop))) )
            {
                // TODO: drop the packet
                fprintf(stderr, "ERROR: %s CRC bad, seqNum mismatch, or wrong address\n",
                        __PRETTY_FUNCTION__);

                //
                // no need to continue processing
                //
                return(false);
            }


            //
            // report the health
            //
            Parse_getChillerObjTemperatureResp(m_buff, temperature, &seqNum);

            #ifdef __DEBUG_CTRL_PROTO__
            printf("found in packet temperature %lf seqNumer 0x%02x\n", *temperature, seqNum);
            #endif

            retVal  = true;
        } else
        {
            fprintf(stderr, "%s ERROR: did not get a m_buffer back\n", __PRETTY_FUNCTION__);
        }

    } else
    {
        fprintf(stderr, "%s ERROR: unable to Make_getStatus\n", __PRETTY_FUNCTION__);
    }

    return(retVal);
}


bool controlProtocol::GetChillerInfo(uint16_t destAddress, char* info, uint8_t length)
{
    bool                retVal  = false;
    uint16_t            result;
    uint16_t            seqNum;
    msgHeader_t*        pMsgHeader;
    getChillerInfoResp_t*    pgetChillerInfoResp;


    //
    // increment the sequence number for this transaction
    //
    ++m_seqNum;

    if( (doTxCommand(Make_getChillerInfo(destAddress, m_buff))) )
    {
        //
        // save the seqNum
        //
        pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
        seqNum  = pMsgHeader->seqNum;

        // get the return packet
        if( (doRxResponse(COMM_TIMEOUT)) )
        {
            #ifdef __DEBUG_CTRL_PROTO__
            //
            // dump out what we got
            //
            for(uint16_t i = 0; i < sizeof(getChillerInfoResp_t); i++)
            {
                printf("0x%02X ", m_buff[i]);
            }
            printf("\n");
            #endif

            //
            // check got the expected message number
            //
            pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
            if( (getChillerInfoResp != pMsgHeader->msgNum) )
            {
                fprintf(stderr, "ERROR: %s got unexpected msg %hu\n",
                    __PRETTY_FUNCTION__, pMsgHeader->msgNum);

                //
                // no need to continue processing
                //
                return(false);
            }

            //
            // cast into the buffer, pick up the CRC
            //
            pgetChillerInfoResp = reinterpret_cast<getChillerInfoResp_t*>(m_buff);

            //
            // verify seqNum and CRC
            //
            if( !(verifyMessage(len_getChillerInfoResp_t, ntohs(pgetChillerInfoResp->crc),
                                            seqNum, ntohs(pgetChillerInfoResp->eop))) )
            {
                // TODO: drop the packet
                fprintf(stderr, "ERROR: %s CRC bad, seqNum mismatch, or wrong address\n",
                        __PRETTY_FUNCTION__);

                //
                // no need to continue processing
                //
                return(false);
            }


            //
            // report the health
            //
            Parse_getChillerInfoResp(m_buff, &result, info, length, &seqNum);

            #ifdef __DEBUG_CTRL_PROTO__
            printf("found in packet info \'%s\' seqNumer 0x%02x\n", reinterpret_cast<char*>(info), seqNum);
            #endif

            if( (result) )
                retVal  = true;
            else
                retVal  = false;
        } else
        {
            fprintf(stderr, "%s ERROR: did not get a m_buffer back\n", __PRETTY_FUNCTION__);
        }

    } else
    {
        fprintf(stderr, "%s ERROR: unable to Make_getStatus\n", __PRETTY_FUNCTION__);
    }

    return(retVal);
}


bool controlProtocol::EnableTECs(uint16_t destAddress)
{
    bool                retVal  = false;
    uint16_t            seqNum;
    uint16_t            result;
    msgHeader_t*        pMsgHeader;
    enableTECsResp_t*    penableTECsResp;


    //
    // increment the sequence number for this transaction
    //
    ++m_seqNum;

    if( (doTxCommand(Make_enableTECs(destAddress, m_buff))) )
    {
        //
        // save the seqNum
        //
        pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
        seqNum  = pMsgHeader->seqNum;

        // get the return packet
        if( (doRxResponse(COMM_TIMEOUT)) )
        {
            #ifdef __DEBUG_CTRL_PROTO__
            //
            // dump out what we got
            //
            for(uint16_t i = 0; i < sizeof(enableTECsResp_t); i++)
            {
                printf("0x%02X ", m_buff[i]);
            }
            printf("\n");
            #endif

            //
            // check got the expected message number
            //
            pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
            if( (enableTECsResp != pMsgHeader->msgNum) )
            {
                fprintf(stderr, "ERROR: %s got unexpected msg %hu\n",
                    __PRETTY_FUNCTION__, pMsgHeader->msgNum);

                //
                // no need to continue processing
                //
                return(false);
            }

            //
            // cast into the buffer, pick up the CRC
            //
            penableTECsResp = reinterpret_cast<enableTECsResp_t*>(m_buff);

            //
            // verify seqNum and CRC
            //
            if( !(verifyMessage(len_enableTECsResp_t, ntohs(penableTECsResp->crc),
                                        seqNum, ntohs(penableTECsResp->eop))) )
            {
                // TODO: drop the packet
                fprintf(stderr, "ERROR: %s CRC bad, seqNum mismatch, or wrong address\n",
                        __PRETTY_FUNCTION__);

                //
                // no need to continue processing
                //
                return(false);
            }


            //
            // report the health
            //
            Parse_enableTECsResp(m_buff, &result, &seqNum);

            #ifdef __DEBUG_CTRL_PROTO__
            printf("found in packet result %d seqNumer 0x%02x\n", result, seqNum);
            #endif

            if( (result) )
                retVal  = true;
            else
                retVal  = false;
        } else
        {
            fprintf(stderr, "%s ERROR: did not get a m_buffer back\n", __PRETTY_FUNCTION__);
        }

    } else
    {
        fprintf(stderr, "%s ERROR: unable to Make_getStatus\n", __PRETTY_FUNCTION__);
    }

    return(retVal);
}


bool controlProtocol::DisableTECs(uint16_t destAddress)
{
    bool                retVal  = false;
    uint16_t            seqNum;
    uint16_t            result;
    msgHeader_t*        pMsgHeader;
    disableTECsResp_t*  pdisableTECsResp;


    //
    // increment the sequence number for this transaction
    //
    ++m_seqNum;

    if( (doTxCommand(Make_disableTECs(destAddress, m_buff))) )
    {
        //
        // save the seqNum
        //
        pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
        seqNum  = pMsgHeader->seqNum;

        // get the return packet
        if( (doRxResponse(COMM_TIMEOUT)) )
        {
            #ifdef __DEBUG_CTRL_PROTO__
            //
            // dump out what we got
            //
            for(uint16_t i = 0; i < sizeof(disableTECsResp_t); i++)
            {
                printf("0x%02X ", m_buff[i]);
            }
            printf("\n");
            #endif

            //
            // check got the expected message number
            //
            pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
            if( (disableTECsResp != pMsgHeader->msgNum) )
            {
                fprintf(stderr, "ERROR: %s got unexpected msg %hu\n",
                    __PRETTY_FUNCTION__, pMsgHeader->msgNum);

                //
                // no need to continue processing
                //
                return(false);
            }

            //
            // cast into the buffer, pick up the CRC
            //
            pdisableTECsResp = reinterpret_cast<disableTECsResp_t*>(m_buff);

            //
            // verify seqNum and CRC
            //
            if( !(verifyMessage(len_disableTECsResp_t, ntohs(pdisableTECsResp->crc),
                                        seqNum, ntohs(pdisableTECsResp->eop))) )
            {
                // TODO: drop the packet
                fprintf(stderr, "ERROR: %s CRC bad, seqNum mismatch, or wrong address\n",
                        __PRETTY_FUNCTION__);

                //
                // no need to continue processing
                //
                return(false);
            }


            //
            // report the health
            //
            Parse_disableTECsResp(m_buff, &result, &seqNum);

            #ifdef __DEBUG_CTRL_PROTO__
            printf("found in packet result %d seqNumer 0x%02x\n", result, seqNum);
            #endif

            if( (result) )
                retVal  = true;
            else
                retVal  = false;
        } else
        {
            fprintf(stderr, "%s ERROR: did not get a m_buffer back\n", __PRETTY_FUNCTION__);
        }

    } else
    {
        fprintf(stderr, "%s ERROR: unable to Make_getStatus\n", __PRETTY_FUNCTION__);
    }

    return(retVal);
}


bool controlProtocol::GetTECInfo(uint16_t destAddress, uint16_t tec_address,
                    uint32_t* deviceType, uint32_t* hwVersion, uint32_t* fwVersion, uint32_t* serialNum)
{
    bool                retVal  = false;
    uint16_t            seqNum;
    uint16_t            result;
    msgHeader_t*        pMsgHeader;
    getTECInfoMsgResp_t*    pgetTECInfoMsgResp;


    //
    // increment the sequence number for this transaction
    //
    ++m_seqNum;

    if( (doTxCommand(Make_getTECInfoMsg(destAddress, m_buff, tec_address))) )
    {
        //
        // save the seqNum
        //
        pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
        seqNum  = pMsgHeader->seqNum;

        // get the return packet
        if( (doRxResponse(COMM_TIMEOUT)) )
        {
            #ifdef __DEBUG_CTRL_PROTO__
            //
            // dump out what we got
            //
            for(uint16_t i = 0; i < sizeof(getTECInfoMsgResp_t); i++)
            {
                printf("0x%02X ", m_buff[i]);
            }
            printf("\n");
            #endif

            //
            // check got the expected message number
            //
            pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
            if( (getTECInfoMsgResp != pMsgHeader->msgNum) )
            {
                fprintf(stderr, "ERROR: %s got unexpected msg %hu\n",
                    __PRETTY_FUNCTION__, pMsgHeader->msgNum);

                //
                // no need to continue processing
                //
                return(false);
            }

            //
            // cast into the buffer, pick up the CRC
            //
            pgetTECInfoMsgResp = reinterpret_cast<getTECInfoMsgResp_t*>(m_buff);

            //
            // verify seqNum and CRC
            //
            if( !(verifyMessage(len_getTECInfoMsgResp_t, ntohs(pgetTECInfoMsgResp->crc),
                                            seqNum, ntohs(pgetTECInfoMsgResp->eop))) )
            {
                // TODO: drop the packet
                fprintf(stderr, "ERROR: %s CRC bad, seqNum mismatch, or wrong address\n",
                        __PRETTY_FUNCTION__);

                //
                // no need to continue processing
                //
                return(false);
            }


            //
            // report the health
            //
            Parse_getTECInfoMsgResp(m_buff, &result, deviceType, hwVersion, fwVersion, serialNum, &seqNum);

            #ifdef __DEBUG_CTRL_PROTO__
            printf("found in packet result %d, deviceType 0x%02X, hwVersion 0x%02X, fwVersion 0x%02X, \
                serialNum 0x%02X seqNumer 0x%02x\n", result, *deviceType, *hwVersion, *fwVersion, *serialNum, seqNum);
            #endif

            if( (result) )
                retVal  = true;
            else
                retVal  = false;

        } else
        {
            fprintf(stderr, "%s ERROR: did not get a m_buffer back\n", __PRETTY_FUNCTION__);
        }

    } else
    {
        fprintf(stderr, "%s ERROR: unable to Make_getStatus\n", __PRETTY_FUNCTION__);
    }

    return(retVal);
}


bool controlProtocol::StartUpCmd(uint16_t destAddress)
{
    bool                retVal  = false;
    uint16_t            seqNum;
    uint16_t            result;
    msgHeader_t*        pMsgHeader;
    startUpCmdResp_t*   pstartUpCmdResp;


    //
    // increment the sequence number for this transaction
    //
    ++m_seqNum;

    if( (doTxCommand(Make_startUpCmd(destAddress, m_buff))) )
    {
        //
        // save the seqNum
        //
        pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
        seqNum  = pMsgHeader->seqNum;

        // get the return packet
        if( (doRxResponse(COMM_TIMEOUT)) )
        {
            #ifdef __DEBUG_CTRL_PROTO__
            //
            // dump out what we got
            //
            for(uint16_t i = 0; i < sizeof(startUpCmdResp_t); i++)
            {
                printf("0x%02X ", m_buff[i]);
            }
            printf("\n");
            #endif

            //
            // check got the expected message number
            //
            pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
            if( (startUpCmdResp != pMsgHeader->msgNum) )
            {
                fprintf(stderr, "ERROR: %s got unexpected msg %hu\n",
                    __PRETTY_FUNCTION__, pMsgHeader->msgNum);

                //
                // no need to continue processing
                //
                return(false);
            }

            //
            // cast into the buffer, pick up the CRC
            //
            pstartUpCmdResp = reinterpret_cast<startUpCmdResp_t*>(m_buff);

            //
            // verify seqNum and CRC
            //
            if( !(verifyMessage(len_startUpCmdResp_t, ntohs(pstartUpCmdResp->crc),
                                        seqNum, ntohs(pstartUpCmdResp->eop))) )
            {
                // TODO: drop the packet
                fprintf(stderr, "ERROR: %s CRC bad, seqNum mismatch, or wrong address\n",
                        __PRETTY_FUNCTION__);

                //
                // no need to continue processing
                //
                return(false);
            }


            //
            // report the health
            //
            Parse_startUpCmdResp(m_buff, &result, &seqNum);

            #ifdef __DEBUG_CTRL_PROTO__
            printf("found in packet result %d seqNumer 0x%02x\n", result, seqNum);
            #endif

            if( (result) )
                retVal  = true;
            else
                retVal  = false;
        } else
        {
            fprintf(stderr, "%s ERROR: did not get a m_buffer back\n", __PRETTY_FUNCTION__);
        }

    } else
    {
        fprintf(stderr, "%s ERROR: unable to Make_getStatus\n", __PRETTY_FUNCTION__);
    }

    return(retVal);
}


bool controlProtocol::ShutDownCmd(uint16_t destAddress)
{
    bool                retVal  = false;
    uint16_t            seqNum;
    uint16_t            result;
    msgHeader_t*        pMsgHeader;
    shutDownCmdResp_t*  pshutDownCmdResp;


    //
    // increment the sequence number for this transaction
    //
    ++m_seqNum;

    if( (doTxCommand(Make_shutDownCmd(destAddress, m_buff))) )
    {
        //
        // save the seqNum
        //
        pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
        seqNum  = pMsgHeader->seqNum;

        // get the return packet
        if( (doRxResponse(COMM_TIMEOUT)) )
        {
            #ifdef __DEBUG_CTRL_PROTO__
            //
            // dump out what we got
            //
            for(uint16_t i = 0; i < sizeof(shutDownCmdResp_t); i++)
            {
                printf("0x%02X ", m_buff[i]);
            }
            printf("\n");
            #endif

            //
            // check got the expected message number
            //
            pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
            if( (shutDownCmdResp != pMsgHeader->msgNum) )
            {
                fprintf(stderr, "ERROR: %s got unexpected msg %hu\n",
                    __PRETTY_FUNCTION__, pMsgHeader->msgNum);

                //
                // no need to continue processing
                //
                return(false);
            }

            //
            // cast into the buffer, pick up the CRC
            //
            pshutDownCmdResp = reinterpret_cast<shutDownCmdResp_t*>(m_buff);

            //
            // verify seqNum and CRC
            //
            if( !(verifyMessage(len_shutDownCmdResp_t, ntohs(pshutDownCmdResp->crc),
                                    seqNum, ntohs(pshutDownCmdResp->eop))) )
            {
                // TODO: drop the packet
                fprintf(stderr, "ERROR: %s CRC bad, seqNum mismatch, or wrong address\n",
                        __PRETTY_FUNCTION__);

                //
                // no need to continue processing
                //
                return(false);
            }


            //
            // report the health
            //
            Parse_shutDownCmdResp(m_buff, &result, &seqNum);

            #ifdef __DEBUG_CTRL_PROTO__
            printf("found in packet result %d seqNumer 0x%02x\n", result, seqNum);
            #endif

            if( (result) )
                retVal  = true;
            else
                retVal  = false;
        } else
        {
            fprintf(stderr, "%s ERROR: did not get a m_buffer back\n", __PRETTY_FUNCTION__);
        }

    } else
    {
        fprintf(stderr, "%s ERROR: unable to Make_getStatus\n", __PRETTY_FUNCTION__);
    }

    return(retVal);
}


uint16_t controlProtocol::Make_startUpCmd(uint16_t Address, uint8_t* pBuff)
{
    startUpCmd_t*    msg  = reinterpret_cast<startUpCmd_t*>(pBuff);
    uint16_t      CRC  = 0;


    memset(m_buff, '\0', MAX_BUFF_LENGTH_CP + 1);
    // create the startUpCmd message in pBuff and CRC16 checksum it
    msg->header.control         = COMMAND;
    msg->header.length          = sizeof(startUpCmd_t);
    msg->header.address.address = htons(Address);
    msg->header.seqNum          = m_seqNum;
    msg->header.msgNum          = startUpCmd;

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_startUpCmd_t);

    // put the CRC
    msg->crc                = htons(CRC);   // TODO: need htons ?

    // put the end of transmission byte
    msg->eop                = htons(EOP_VAL);

    return(sizeof(startUpCmd_t));
}


uint16_t controlProtocol::Make_startUpCmdResp(uint16_t Address, uint8_t* pBuff, uint16_t result, uint16_t SeqNum)
{
    startUpCmdResp_t*  msg  = reinterpret_cast<startUpCmdResp_t*>(pBuff);
    uint16_t        CRC  = 0;


    // create the startUpCmdResp message in pBuff and CRC16 checksum it
    msg->header.control         = RESPONSE;
    msg->header.length          = sizeof(startUpCmdResp_t);
    msg->header.address.address = htons(Address);
    msg->header.seqNum          = SeqNum;
    msg->header.msgNum          = startUpCmdResp;

    // set the result
    msg->result             = htons(result);

    // calculate the CRC
    CRC = calcCRC16(pBuff, len_startUpCmdResp_t);

    // put the CRC
    msg->crc                = htons(CRC);   // TODO: need htons ?

    // put the end of transmission byte
    msg->eop                = htons(EOP_VAL);

    return(sizeof(startUpCmdResp_t));
}


void controlProtocol::Parse_startUpCmdResp(uint8_t* m_buff, uint16_t* result, uint16_t* pSeqNum)
{
    startUpCmdResp_t* pResponse = reinterpret_cast<startUpCmdResp_t*>(m_buff);


    *result     = ntohs(pResponse->result);
    *pSeqNum    = pResponse->header.seqNum;
}


uint16_t controlProtocol::Make_shutDownCmd(uint16_t Address, uint8_t* pBuff)
{
    shutDownCmd_t* msg  = reinterpret_cast<shutDownCmd_t*>(pBuff);
    uint16_t    CRC  = 0;


    // create the shutDownCmdResp message in pBuff and CRC16 checksum it
    msg->header.control         = COMMAND;
    msg->header.length          = sizeof(shutDownCmd_t);
    msg->header.address.address = htons(Address);
    msg->header.seqNum          = m_seqNum;
    msg->header.msgNum          = shutDownCmd;

    // calculate the CRC
    CRC = calcCRC16(pBuff, len_shutDownCmd_t);

    // put the CRC
    msg->crc                = htons(CRC);   // TODO: need htons ?

    // put the end of transmission byte
    msg->eop                = htons(EOP_VAL);

    return(sizeof(shutDownCmd_t));
}


uint16_t controlProtocol::Make_shutDownCmdResp(uint16_t Address, uint8_t* pBuff, uint16_t result, uint16_t SeqNum)
{
    shutDownCmdResp_t* msg  = reinterpret_cast<shutDownCmdResp_t*>(pBuff);
    uint16_t        CRC  = 0;


    // create the shutDownCmdResp message in pBuff and CRC16 checksum it
    msg->header.control         = RESPONSE;
    msg->header.length          = sizeof(shutDownCmdResp_t);
    msg->header.address.address = htons(Address);
    msg->header.seqNum          = SeqNum;
    msg->header.msgNum          = shutDownCmdResp;

    // set the result
    msg->result             = htons(result);

    // calculate the CRC
    CRC = calcCRC16(pBuff, len_shutDownCmdResp_t);

    // put the CRC
    msg->crc                = htons(CRC);   // TODO: need htons ?

    // put the end of transmission byte
    msg->eop                = htons(EOP_VAL);

    return(sizeof(shutDownCmdResp_t));
}


void controlProtocol::Parse_shutDownCmdResp(uint8_t* m_buff, uint16_t* result, uint16_t* pSeqNum)
{
    shutDownCmdResp_t* pResponse = reinterpret_cast<shutDownCmdResp_t*>(m_buff);


    *result     = ntohs(pResponse->result);
    *pSeqNum    = pResponse->header.seqNum;
}


//
// send a getStatus message to controllino at Address - future expandability
// TODO: protect against short pBuff by having caller include length ?
//
uint16_t controlProtocol::Make_getStatus(uint16_t Address, uint8_t* pBuff)
{
    getStatus_t*    msg  = reinterpret_cast<getStatus_t*>(pBuff);
    uint16_t        CRC  = 0;


    // create the getStatus message in pBuff and CRC16 checksum it
    msg->header.control         = COMMAND;
    msg->header.length          = sizeof(getStatus_t);
    msg->header.address.address = htons(Address);
    msg->header.seqNum          = m_seqNum;
    msg->header.msgNum          = getStatusCmd;

    // calculate the CRC
    CRC = calcCRC16(pBuff, len_getStatus_t);

    // put the CRC
    msg->crc                = htons(CRC);   // TODO: need htons ?

    // put the end of transmission byte
    msg->eop                = htons(EOP_VAL);

    return(sizeof(getStatus_t));
}


uint16_t controlProtocol::Make_getStatusResp(uint16_t Address, uint8_t* pBuff, uint16_t humidityAlert,
                    uint16_t TECsRunning, uint16_t chillerOnLine, uint16_t SeqNum)
{
    getStatusResp_t*    msg  = reinterpret_cast<getStatusResp_t*>(pBuff);
    uint16_t            CRC  = 0;


    // create the getStatusResp message in pBuff and CRC16 checksum it
    msg->header.control         = RESPONSE;
    msg->header.length          = sizeof(getStatusResp_t);
    msg->header.address.address = htons(Address);
    msg->header.seqNum          = SeqNum;
    msg->header.msgNum          = getStatusResp;
    msg->status.humidityAlert   = htons(humidityAlert);
    msg->status.TECsRunning     = htons(TECsRunning);
    msg->status.chillerOnLine   = htons(chillerOnLine);

    // calculate the CRC
    CRC = calcCRC16(pBuff, len_getStatusResp_t);

    // put the CRC
    msg->crc                = htons(CRC);   // TODO: need htons ?

    // put the end of transmission byte
    msg->eop                = htons(EOP_VAL);

    return(sizeof(getStatusResp_t));
}


void controlProtocol::Parse_getStatusResp(uint8_t* m_buff, uint16_t* humidityAlert,
        uint16_t* TECsRunning, uint16_t* chillerOnLine, uint16_t* pSeqNum)
{
    getStatusResp_t* pResponse = reinterpret_cast<getStatusResp_t*>(m_buff);

    //
    // fill in the pReport
    //
    *humidityAlert  = ntohs(pResponse->status.humidityAlert);
    *TECsRunning    = ntohs(pResponse->status.TECsRunning);
    *chillerOnLine  = ntohs(pResponse->status.chillerOnLine);
    *pSeqNum        = pResponse->header.seqNum;
}


uint16_t controlProtocol::Make_getHumidityThreshold(uint16_t Address, uint8_t* pBuff)
{
    getHumidityThreshold_t*    msg  = reinterpret_cast<getHumidityThreshold_t*>(pBuff);
    uint16_t        CRC  = 0;


    // create the getHumidityThreshold message in pBuff and CRC16 checksum it
    msg->header.control         = COMMAND;
    msg->header.length          = sizeof(getHumidityThreshold_t);
    msg->header.address.address = htons(Address);   // need htons here ?
    msg->header.seqNum          = m_seqNum;    // htons ?
    msg->header.msgNum          = getHumidityThreshold;

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_getHumidityThreshold_t);

    // put the CRC
    msg->crc                = htons(CRC);   // TODO: need htons ?

    // put the end of transmission byte
    msg->eop                = htons(EOP_VAL);

    return(sizeof(getHumidityThreshold_t));
}


uint16_t controlProtocol::Make_getHumidityThresholdResp(uint16_t Address, uint8_t* pBuff, uint16_t threshold, uint16_t SeqNum)
{
    getHumidityThresholdResp_t* msg  = reinterpret_cast<getHumidityThresholdResp_t*>(pBuff);
    uint16_t                    CRC  = 0;


    // create the getHumidityThresholdResp message in pBuff and CRC16 checksum it
    msg->header.control         = RESPONSE;
    msg->header.length          = sizeof(getHumidityThresholdResp_t);
    msg->header.address.address = htons(Address);   // need htons here ?
    msg->header.seqNum          = SeqNum;    // htons ?
    msg->header.msgNum          = getHumidityThresholdResp;
    msg->threshold              = htons(threshold);

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_getHumidityThresholdResp_t);

    // put the CRC
    msg->crc                = htons(CRC);   // TODO: need htons ?

    // put the end of transmission byte
    msg->eop                = htons(EOP_VAL);

    return(sizeof(getHumidityThresholdResp_t));
}


void controlProtocol::Parse_getHumidityThresholdResp(uint8_t* m_buff, uint16_t* threshold, uint16_t* pSeqNum)
{
    getHumidityThresholdResp_t* pResponse = reinterpret_cast<getHumidityThresholdResp_t*>(m_buff);


    *threshold  = ntohs(pResponse->threshold);
    *pSeqNum    = pResponse->header.seqNum;
}


uint16_t controlProtocol::Make_setHumidityThreshold(uint16_t Address, uint8_t* pBuff, uint16_t threshold)
{
    setHumidityThreshold_t* msg = reinterpret_cast<setHumidityThreshold_t*>(pBuff);
    uint16_t CRC = 0;


    // create the getStatus message in pBuff and CRC16 checksum it
    msg->header.control         = COMMAND;
    msg->header.length          = sizeof(setHumidityThreshold_t);
    msg->header.address.address = htons(Address);
    msg->header.seqNum          = m_seqNum;
    msg->header.msgNum          = setHumidityThreshold;

    // pass in the humidity threshold to set
    msg->threshold          = htons(threshold);

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_setHumidityThreshold_t);

    // put the CRC
    msg->crc    = htons(CRC);   // TODO: need htons ?

    // put the end of transmission byte
    msg->eop                = htons(EOP_VAL);

    return(sizeof(setHumidityThreshold_t));
}


uint16_t controlProtocol::Make_setHumidityThresholdResp(uint16_t Address, uint8_t* pBuff, uint16_t result, uint16_t SeqNum)
{
    setHumidityThresholdResp_t* msg = reinterpret_cast<setHumidityThresholdResp_t*>(pBuff);
    uint16_t CRC = 0;


    // create the getStatus message in pBuff and CRC16 checksum it
    msg->header.control         = RESPONSE;
    msg->header.length          = sizeof(setHumidityThresholdResp_t);
    msg->header.address.address = htons(Address);
    msg->header.seqNum          = SeqNum;
    msg->header.msgNum          = setHumidityThresholdResp;

    // set the result
    msg->result                 = htons(result);

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_setHumidityThresholdResp_t);

    // put the CRC
    msg->crc    = htons(CRC);   // TODO: need htons ?

    // put the end of transmission byte
    msg->eop                = htons(EOP_VAL);

    return(sizeof(setHumidityThresholdResp_t));
}


void controlProtocol::Parse_setHumidityThresholdResp(uint8_t* m_buff, uint16_t* result, uint16_t* pSeqNum)
{
    setHumidityThresholdResp_t* pResponse = reinterpret_cast<setHumidityThresholdResp_t*>(m_buff);


    *result     = ntohs(pResponse->result);
    *pSeqNum    = pResponse->header.seqNum;
}


uint16_t controlProtocol::Make_getHumidity(uint16_t Address, uint8_t* pBuff)
{
    getHumidity_t* msg = reinterpret_cast<getHumidity_t*>(pBuff);
    uint16_t CRC = 0;


    // create the getStatus message in pBuff and CRC16 checksum it
    msg->header.control         = COMMAND;
    msg->header.length          = sizeof(getHumidity_t);
    msg->header.address.address = htons(Address);
    msg->header.seqNum          = m_seqNum;
    msg->header.msgNum          = getHumidity;

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_getHumidity_t);

    // put the CRC
    msg->crc    = htons(CRC);   // TODO: need htons ?

    // put the end of transmission byte
    msg->eop                = htons(EOP_VAL);

    return(sizeof(getHumidity_t));
}


uint16_t controlProtocol::Make_getHumidityResp(uint16_t Address, uint8_t* pBuff, float humidity, uint16_t SeqNum)
{
    getHumidityResp_t* msg = reinterpret_cast<getHumidityResp_t*>(pBuff);
    uint16_t CRC = 0;


    // create the getStatus message in pBuff and CRC16 checksum it
    msg->header.control         = RESPONSE;
    msg->header.length          = sizeof(getHumidityResp_t);
    msg->header.address.address = htons(Address);
    msg->header.seqNum          = SeqNum;
    msg->header.msgNum          = getHumidityResp;

    #ifdef __RUNNING_ON_CONTROLLINO__
    //
    // use the dostrf function, left justified, max 7 characters total, max 2 decimal places
    //
    dtostrf(humidity, -(MAX_HUMIDITY_LENGTH), 2, reinterpret_cast<char*>(msg->humidity));
    #else
    //
    // use the snprintf function, 
    //
    snprintf(reinterpret_cast<char*>(msg->humidity), MAX_HUMIDITY_LENGTH, "%-+3.2f", humidity);
    #endif

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_getHumidityResp_t);

    // put the CRC
    msg->crc    = htons(CRC);   // TODO: need htons ?

    // put the end of transmission byte
    msg->eop                = htons(EOP_VAL);

    return(sizeof(getHumidityResp_t));
}


void controlProtocol::Parse_getHumidityResp(uint8_t* m_buff, float* humidity, uint16_t* pSeqNum)
{
    getHumidityResp_t* pResponse = reinterpret_cast<getHumidityResp_t*>(m_buff);


    #ifdef __RUNNING_ON_CONTROLLINO__
    //
    // on uC use atof, sscanf support is dodgy
    //
    *humidity = atof(reinterpret_cast<char*>(pResponse->humidity));
    #else
    sscanf(reinterpret_cast<char*>(pResponse->humidity), "%6f", humidity);
    #endif

    *pSeqNum    = pResponse->header.seqNum;
}


uint16_t controlProtocol::Make_setTECTemperature(uint16_t Address, uint8_t* pBuff, uint16_t tec_address, float temperature)
{
    setTECTemperature_t* msg = reinterpret_cast<setTECTemperature_t*>(pBuff);
    uint16_t CRC = 0;


    // create the getStatus message in pBuff and CRC16 checksum it
    msg->header.control         = COMMAND;
    msg->header.length          = sizeof(setTECTemperature_t);
    msg->header.address.address = htons(Address);
    msg->header.seqNum          = m_seqNum;
    msg->header.msgNum          = setTECTemperature;
    msg->tec_address            = htons(tec_address);
    #ifdef __RUNNING_ON_CONTROLLINO__
    //
    // use the dostrf function, left justified, max 7 characters total, max 2 decimal places
    //
    dtostrf(temperature, -(MAX_TEC_TEMP_LENGH), 2, reinterpret_cast<char*>(msg->temperature));
    #else
    //
    // use the snprintf function, 
    //
    snprintf(reinterpret_cast<char*>(msg->temperature), MAX_TEC_TEMP_LENGH, "%-+3.2f", temperature);
    #endif

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_setTECTemperature_t);

    // put the CRC
    msg->crc    = htons(CRC);   // TODO: need htons ?

    // put the end of transmission byte
    msg->eop                = htons(EOP_VAL);

    return(sizeof(setTECTemperature_t));
}


uint16_t controlProtocol::Make_setTECTemperatureResp(uint16_t Address, uint8_t* pBuff, uint16_t tec_address, uint16_t result, uint16_t SeqNum)
{
    setTECTemperatureResp_t* msg = reinterpret_cast<setTECTemperatureResp_t*>(pBuff);
    uint16_t CRC = 0;


    // create the getStatus message in pBuff and CRC16 checksum it
    msg->header.control         = RESPONSE;
    msg->header.length          = sizeof(setTECTemperatureResp_t);
    msg->header.address.address = htons(Address);
    msg->header.seqNum          = SeqNum;
    msg->header.msgNum          = setTECTemperatureResp;
    msg->tec_address            = htons(tec_address);
    msg->result                 = htons(result);

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_setTECTemperatureResp_t);

    // put the CRC
    msg->crc    = htons(CRC);   // TODO: need htons ?

    // put the end of transmission byte
    msg->eop                = htons(EOP_VAL);

    return(sizeof(setTECTemperatureResp_t));
}


void controlProtocol::Parse_setTECTemperatureResp(uint8_t* m_buff, uint16_t* result, uint16_t* pSeqNum)
{
    setTECTemperatureResp_t* pResponse = reinterpret_cast<setTECTemperatureResp_t*>(m_buff);


    *result     = ntohs(pResponse->result);
    *pSeqNum    = pResponse->header.seqNum;
}


uint16_t controlProtocol::Make_getTECTemperature(uint16_t Address, uint8_t* pBuff, uint16_t tec_address)
{
    getTECTemperature_t* msg = reinterpret_cast<getTECTemperature_t*>(pBuff);
    uint16_t CRC = 0;


    // create the getStatus message in pBuff and CRC16 checksum it
    msg->header.control         = COMMAND;
    msg->header.length          = sizeof(getTECTemperature_t);
    msg->header.address.address = htons(Address);
    msg->header.seqNum          = m_seqNum;
    msg->header.msgNum          = getTECTemperature;
    msg->tec_address            = htons(tec_address);

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_getTECTemperature_t);

    // put the CRC
    msg->crc    = htons(CRC);   // TODO: need htons ?


    // put the end of transmission byte
    msg->eop                = htons(EOP_VAL);

    return(sizeof(getTECTemperature_t));
}


uint16_t controlProtocol::Make_getTECTemperatureResp(uint16_t Address, uint8_t* pBuff, uint16_t tec_address, uint16_t result, float temperature, uint16_t SeqNum)
{
    getTECTemperatureResp_t* msg = reinterpret_cast<getTECTemperatureResp_t*>(pBuff);
    uint16_t CRC = 0;


    // create the getStatus message in pBuff and CRC16 checksum it
    msg->header.control         = RESPONSE;
    msg->header.length          = sizeof(getTECTemperatureResp_t);
    msg->header.address.address = htons(Address);
    msg->header.seqNum          = SeqNum;
    msg->header.msgNum          = getTECTemperatureResp;
    msg->tec_address            = htons(tec_address);
    msg->result                 = htons(result);
    #ifdef __RUNNING_ON_CONTROLLINO__
    //
    // use the dostrf function, left justified, max 7 characters total, max 2 decimal places
    //
    dtostrf(temperature, -(MAX_TEC_TEMP_LENGH), 2, reinterpret_cast<char*>(msg->temperature));
    #else
    //
    // use the snprintf function, 
    //
    snprintf(reinterpret_cast<char*>(msg->temperature), MAX_TEC_TEMP_LENGH, "%-+3.2f", temperature);
    #endif

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_getTECTemperatureResp_t);

    // put the CRC
    msg->crc    = htons(CRC);   // TODO: need htons ?


    // put the end of transmission byte
    msg->eop                = htons(EOP_VAL);

    return(sizeof(getTECTemperatureResp_t));
}

void controlProtocol::Parse_getTECTemperatureResp(uint8_t* m_buff, uint16_t* result, float* temperature, uint16_t* pSeqNum)
{
    getTECTemperatureResp_t* pResponse = reinterpret_cast<getTECTemperatureResp_t*>(m_buff);


    *result = ntohs(pResponse->result);

    #ifdef __RUNNING_ON_CONTROLLINO__
    //
    // on uC use atof, sscanf support is dodgy
    //
    *temperature = atof(reinterpret_cast<char*>(pResponse->temperature));
    #else
    sscanf(reinterpret_cast<char*>(pResponse->temperature), "%f", temperature);
    #endif

    *pSeqNum    = pResponse->header.seqNum;
}


uint16_t controlProtocol::Make_getTECObjTemperature(uint16_t Address, uint8_t* pBuff, uint16_t tec_address)
{
    getTECObjTemperature_t* msg = reinterpret_cast<getTECObjTemperature_t*>(pBuff);
    uint16_t CRC = 0;


    // create the getStatus message in pBuff and CRC16 checksum it
    msg->header.control         = COMMAND;
    msg->header.length          = sizeof(getTECObjTemperature_t);
    msg->header.address.address = htons(Address);
    msg->header.seqNum          = m_seqNum;
    msg->header.msgNum          = getTECObjTemperature;
    msg->tec_address            = htons(tec_address);

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_getTECObjTemperature_t);

    // put the CRC
    msg->crc    = htons(CRC);   // TODO: need htons ?


    // put the end of transmission byte
    msg->eop                = htons(EOP_VAL);

    return(sizeof(getTECObjTemperature_t));
}


uint16_t controlProtocol::Make_getTECObjTemperatureResp(uint16_t Address, uint8_t* pBuff, uint16_t tec_address, uint16_t result, float temperature, uint16_t SeqNum)
{
    getTECObjTemperatureResp_t* msg = reinterpret_cast<getTECObjTemperatureResp_t*>(pBuff);
    uint16_t CRC = 0;


    // create the getStatus message in pBuff and CRC16 checksum it
    msg->header.control         = RESPONSE;
    msg->header.length          = sizeof(getTECObjTemperatureResp_t);
    msg->header.address.address = htons(Address);
    msg->header.seqNum          = SeqNum;
    msg->header.msgNum          = getTECObjTemperatureResp;
    msg->tec_address            = htons(tec_address);
    msg->result                 = htons(result);
    #ifdef __RUNNING_ON_CONTROLLINO__
    //
    // use the dostrf function, left justified, max 7 characters total, max 2 decimal places
    //
    dtostrf(temperature, -(MAX_TEC_TEMP_LENGH), 2, reinterpret_cast<char*>(msg->temperature));
    #else
    //
    // use the snprintf function, 
    //
    snprintf(reinterpret_cast<char*>(msg->temperature), MAX_TEC_TEMP_LENGH, "%-+3.2f", temperature);
    #endif

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_getTECObjTemperatureResp_t);

    // put the CRC
    msg->crc    = htons(CRC);   // TODO: need htons ?


    // put the end of transmission byte
    msg->eop                = htons(EOP_VAL);

    return(sizeof(getTECObjTemperatureResp_t));
}

void controlProtocol::Parse_getTECObjTemperatureResp(uint8_t* m_buff, uint16_t* result, float* temperature, uint16_t* pSeqNum)
{
    getTECObjTemperatureResp_t* pResponse = reinterpret_cast<getTECObjTemperatureResp_t*>(m_buff);


    *result = ntohs(pResponse->result);

    #ifdef __RUNNING_ON_CONTROLLINO__
    //
    // on uC use atof, sscanf support is dodgy
    //
    *temperature = atof(reinterpret_cast<char*>(pResponse->temperature));
    #else
    sscanf(reinterpret_cast<char*>(pResponse->temperature), "%f", temperature);
    #endif

    *pSeqNum    = pResponse->header.seqNum;
}


uint16_t controlProtocol::Make_getTECInfoMsg(uint16_t Address, uint8_t* pBuff, uint16_t tec_address)
{
    getTECInfoMsg_t* msg = reinterpret_cast<getTECInfoMsg_t*>(pBuff);
    uint16_t CRC = 0;


    // create the getStatus message in pBuff and CRC16 checksum it
    msg->header.control         = COMMAND;
    msg->header.length          = sizeof(getTECInfoMsg_t);
    msg->header.address.address = htons(Address);
    msg->header.seqNum          = m_seqNum;
    msg->header.msgNum          = getTECInfoMsg;
    msg->tec_address            = htons(tec_address);

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_getTECInfoMsg_t);

    // put the CRC
    msg->crc    = htons(CRC);   // TODO: need htons ?


    // put the end of transmission byte
    msg->eop                = htons(EOP_VAL);

    return(sizeof(getTECInfoMsg_t));
}


uint16_t controlProtocol::Make_getTECInfoMsgResp(uint16_t Address, uint8_t* pBuff, uint16_t tec_address,
        uint16_t result, uint32_t deviceType, uint32_t hwVersion, uint32_t fwVersion,
        uint32_t serialNumber, uint16_t SeqNum)
{
    getTECInfoMsgResp_t* msg = reinterpret_cast<getTECInfoMsgResp_t*>(pBuff);
    uint16_t CRC = 0;

    // create the getStatus message in pBuff and CRC16 checksum it
    msg->header.control         = RESPONSE;
    msg->header.length          = sizeof(getTECInfoMsgResp_t);
    msg->header.address.address = htons(Address);
    msg->header.seqNum          = SeqNum;
    msg->header.msgNum          = getTECInfoMsgResp;
    msg->result                 = htons(result);
    msg->tec_address            = htons(tec_address);
    msg->deviceType             = htonl(deviceType);
    msg->hwVersion              = htonl(hwVersion);
    msg->fwVersion              = htonl(fwVersion);
    msg->serialNumber           = htonl(serialNumber);

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_getTECInfoMsgResp_t);

    // put the CRC
    msg->crc    = htons(CRC);   // TODO: need htons ?


    // put the end of transmission byte
    msg->eop                = htons(EOP_VAL);

    return(sizeof(getTECInfoMsgResp_t));
}


void controlProtocol::Parse_getTECInfoMsgResp(uint8_t* m_buff, uint16_t* result, uint32_t* deviceType,
        uint32_t* hwVersion, uint32_t* fwVersion, uint32_t* serialNumber, uint16_t* pSeqNum)
{
    getTECInfoMsgResp_t* pResponse = reinterpret_cast<getTECInfoMsgResp_t*>(m_buff);


    *result         = ntohs(pResponse->result);
    *deviceType     = pResponse->deviceType;
    *hwVersion      = pResponse->hwVersion;
    *fwVersion      = pResponse->fwVersion;
    *serialNumber   = pResponse->serialNumber;
    *pSeqNum        = pResponse->header.seqNum;
}



uint16_t controlProtocol::Make_enableTECs(uint16_t Address, uint8_t* pBuff)
{
    enableTECs_t* msg = reinterpret_cast<enableTECs_t*>(pBuff);
    uint16_t CRC = 0;


    // create the getStatus message in pBuff and CRC16 checksum it
    msg->header.control         = COMMAND;
    msg->header.length          = sizeof(enableTECs_t);
    msg->header.address.address = htons(Address);
    msg->header.seqNum          = m_seqNum;
    msg->header.msgNum          = enableTECs;

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_enableTECs_t);

    // put the CRC
    msg->crc    = htons(CRC);   // TODO: need htons ?

    // put the end of transmission byte
    msg->eop                = htons(EOP_VAL);

    return(sizeof(enableTECs_t));
}


uint16_t controlProtocol::Make_enableTECsResp(uint16_t Address, uint8_t* pBuff, uint16_t result, uint16_t SeqNum)
{
    enableTECsResp_t* msg = reinterpret_cast<enableTECsResp_t*>(pBuff);
    uint16_t CRC = 0;


    // create the getStatus message in pBuff and CRC16 checksum it
    msg->header.control         = RESPONSE;
    msg->header.length          = sizeof(enableTECsResp_t);
    msg->header.address.address = htons(Address);
    msg->header.seqNum          = SeqNum;
    msg->header.msgNum          = enableTECsResp;
    msg->result                 = htons(result);

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_enableTECsResp_t);

    // put the CRC
    msg->crc    = htons(CRC);   // TODO: need htons ?

    // put the end of transmission byte
    msg->eop                = htons(EOP_VAL);

    return(sizeof(enableTECsResp_t));
}


void controlProtocol::Parse_enableTECsResp(uint8_t* m_buff, uint16_t* result, uint16_t* pSeqNum)
{
    enableTECsResp_t* pResponse = reinterpret_cast<enableTECsResp_t*>(m_buff);


    *result     = ntohs(pResponse->result);
    *pSeqNum    = pResponse->header.seqNum;
}


uint16_t controlProtocol::Make_disableTECs(uint16_t Address, uint8_t* pBuff)
{
    disableTECs_t* msg = reinterpret_cast<disableTECs_t*>(pBuff);
    uint16_t CRC = 0;


    // create the getStatus message in pBuff and CRC16 checksum it
    msg->header.control         = COMMAND;
    msg->header.length          = sizeof(disableTECs_t);
    msg->header.address.address = htons(Address);
    msg->header.seqNum          = m_seqNum;
    msg->header.msgNum          = disableTECs;

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_disableTECs_t);

    // put the CRC
    msg->crc    = htons(CRC);   // TODO: need htons ?

    // put the end of transmission byte
    msg->eop                = htons(EOP_VAL);

    return(sizeof(disableTECs_t));
}


uint16_t controlProtocol::Make_disableTECsResp(uint16_t Address, uint8_t* pBuff, uint16_t result, uint16_t SeqNum)
{
    disableTECsResp_t* msg = reinterpret_cast<disableTECsResp_t*>(pBuff);
    uint16_t CRC = 0;


    // create the getStatus message in pBuff and CRC16 checksum it
    msg->header.control         = RESPONSE;
    msg->header.length          = sizeof(disableTECsResp_t);
    msg->header.address.address = htons(Address);
    msg->header.seqNum          = SeqNum;
    msg->header.msgNum          = disableTECsResp;
    msg->result                 = htons(result);

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_disableTECsResp_t);

    // put the CRC
    msg->crc    = htons(CRC);   // TODO: need htons ?

    // put the end of transmission byte
    msg->eop                = htons(EOP_VAL);

    return(sizeof(disableTECsResp_t));
}


void controlProtocol::Parse_disableTECsResp(uint8_t* m_buff, uint16_t* result, uint16_t* pSeqNum)
{
    disableTECsResp_t* pResponse = reinterpret_cast<disableTECsResp_t*>(m_buff);


    *result     = ntohs(pResponse->result);
    *pSeqNum    = pResponse->header.seqNum;
}


uint16_t controlProtocol::Make_startChillerMsg(uint16_t Address, uint8_t* pBuff)
{
    startChillerMsg_t* msg  = reinterpret_cast<startChillerMsg_t*>(pBuff);
    uint16_t      CRC  = 0;


    memset(m_buff, '\0', MAX_BUFF_LENGTH_CP + 1);
    // create the startChillerMsg message in pBuff and CRC16 checksum it
    msg->header.control         = COMMAND;
    msg->header.length          = sizeof(startChillerMsg_t);
    msg->header.address.address = htons(Address);
    msg->header.seqNum          = m_seqNum;
    msg->header.msgNum          = startChillerMsg;

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_startChillerMsg_t);

    // put the CRC
    msg->crc                = htons(CRC);   // TODO: need htons ?

    // put the end of transmission byte
    msg->eop                = htons(EOP_VAL);

    return(sizeof(startChillerMsg_t));
}


uint16_t controlProtocol::Make_startChillerMsgResp(uint16_t Address, uint8_t* pBuff, uint16_t result, uint16_t SeqNum)
{
    startChillerMsgResp_t* msg  = reinterpret_cast<startChillerMsgResp_t*>(pBuff);
    uint16_t      CRC  = 0;


    memset(m_buff, '\0', MAX_BUFF_LENGTH_CP + 1);
    // create the startChillerMsgResp message in pBuff and CRC16 checksum it
    msg->header.control         = RESPONSE;
    msg->header.length          = sizeof(startChillerMsgResp_t);
    msg->header.address.address = htons(Address);
    msg->result                 = htons(result);
    msg->header.seqNum          = SeqNum;
    msg->header.msgNum          = startChillerMsgResp;

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_startChillerMsgResp_t);

    // put the CRC
    msg->crc                = htons(CRC);   // TODO: need htons ?

    // put the end of transmission byte
    msg->eop                = htons(EOP_VAL);

    return(sizeof(startChillerMsgResp_t));
}


void controlProtocol::Parse_startChillerMsgResp(uint8_t* m_buff, uint16_t* result, uint16_t* pSeqNum)
{
    startChillerMsgResp_t* pResponse = reinterpret_cast<startChillerMsgResp_t*>(m_buff);


    *result     = ntohs(pResponse->result);
    *pSeqNum    = pResponse->header.seqNum;
}


uint16_t controlProtocol::Make_stopChiller(uint16_t Address, uint8_t* pBuff)
{
    stopChiller_t* msg  = reinterpret_cast<stopChiller_t*>(pBuff);
    uint16_t      CRC  = 0;


    memset(m_buff, '\0', MAX_BUFF_LENGTH_CP + 1);
    // create the stopChiller message in pBuff and CRC16 checksum it
    msg->header.control         = COMMAND;
    msg->header.length          = sizeof(stopChiller_t);
    msg->header.address.address = htons(Address);
    msg->header.seqNum          = m_seqNum;
    msg->header.msgNum          = stopChiller;

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_stopChiller_t);

    // put the CRC
    msg->crc                = htons(CRC);   // TODO: need htons ?

    // put the end of transmission byte
    msg->eop                = htons(EOP_VAL);

    return(sizeof(stopChiller_t));
}


uint16_t controlProtocol::Make_stopChillerResp(uint16_t Address, uint8_t* pBuff, uint16_t result, uint16_t SeqNum)
{
    stopChillerResp_t* msg = reinterpret_cast<stopChillerResp_t*>(pBuff);
    uint16_t CRC = 0;


    // create the getStatus message in pBuff and CRC16 checksum it
    msg->header.control         = RESPONSE;
    msg->header.length          = sizeof(stopChillerResp_t);
    msg->header.address.address = htons(Address);
    msg->header.seqNum          = SeqNum;
    msg->header.msgNum          = stopChillerResp;
    msg->result                 = htons(result);
    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_stopChillerResp_t);

    // put the CRC
    msg->crc    = htons(CRC);   // TODO: need htons ?

    // put the end of transmission byte
    msg->eop                = htons(EOP_VAL);

    return(sizeof(stopChillerResp_t));
}


void controlProtocol::Parse_stopChillerResp(uint8_t* m_buff, uint16_t* result, uint16_t* pSeqNum)
{
    stopChillerResp_t* pResponse = reinterpret_cast<stopChillerResp_t*>(m_buff);


    *result     = ntohs(pResponse->result);
    *pSeqNum    = pResponse->header.seqNum;
}


uint16_t controlProtocol::Make_setChillerTemperature(uint16_t Address, uint8_t* pBuff, float temperature)
{
    setChillerTemperature_t* msg = reinterpret_cast<setChillerTemperature_t*>(pBuff);
    uint16_t CRC = 0;


    // create the getStatus message in pBuff and CRC16 checksum it
    msg->header.control         = COMMAND;
    msg->header.length          = sizeof(setChillerTemperature_t);
    msg->header.address.address = htons(Address);
    msg->header.seqNum          = m_seqNum;
    msg->header.msgNum          = setChillerTemperature;
    #ifdef __RUNNING_ON_CONTROLLINO__
    //
    // use the dostrf function, left justified, max 7 characters total, max 2 decimal places
    //
    dtostrf(temperature, -(MAX_CHILLER_TEMP_LENGH), 2, reinterpret_cast<char*>(msg->temperature));
    #else
    //
    // use the snprintf function, 
    //
    snprintf(reinterpret_cast<char*>(msg->temperature), MAX_CHILLER_TEMP_LENGH, "%-+3.2f", temperature);
    #endif

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_setChillerTemperature_t);

    // put the CRC
    msg->crc    = htons(CRC);   // TODO: need htons ?

    // put the end of transmission byte
    msg->eop                = htons(EOP_VAL);

    return(sizeof(setChillerTemperature_t));
}


uint16_t controlProtocol::Make_setChillerTemperatureResp(uint16_t Address, uint8_t* pBuff, uint16_t result, uint16_t SeqNum)
{
    setChillerTemperatureResp_t* msg = reinterpret_cast<setChillerTemperatureResp_t*>(pBuff);
    uint16_t CRC = 0;


    // create the getStatus message in pBuff and CRC16 checksum it
    msg->header.control         = RESPONSE;
    msg->header.length          = sizeof(setChillerTemperatureResp_t);
    msg->header.address.address = htons(Address);
    msg->header.seqNum          = SeqNum;
    msg->header.msgNum          = setChillerTemperatureResp;
    msg->result                 = htons(result);
    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_setChillerTemperatureResp_t);

    // put the CRC
    msg->crc    = htons(CRC);   // TODO: need htons ?

    // put the end of transmission byte
    msg->eop                = htons(EOP_VAL);

    return(sizeof(setChillerTemperatureResp_t));
}


void controlProtocol::Parse_setChillerTemperatureResp(uint8_t* m_buff, uint16_t* result, uint16_t* pSeqNum)
{
    setChillerTemperatureResp_t* pResponse = reinterpret_cast<setChillerTemperatureResp_t*>(m_buff);


    *result     = ntohs(pResponse->result);
    *pSeqNum    = pResponse->header.seqNum;
}


uint16_t controlProtocol::Make_getChillerTemperature(uint16_t Address, uint8_t* pBuff)
{
    getChillerTemperature_t* msg = reinterpret_cast<getChillerTemperature_t*>(pBuff);
    uint16_t CRC = 0;


    // create the getStatus message in pBuff and CRC16 checksum it
    msg->header.control         = COMMAND;
    msg->header.length          = sizeof(getChillerTemperature_t);
    msg->header.address.address = htons(Address);
    msg->header.seqNum          = m_seqNum;
    msg->header.msgNum          = getChillerTemperature;

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_getChillerTemperature_t);

    // put the CRC
    msg->crc    = htons(CRC);   // TODO: need htons ?


    // put the end of transmission byte
    msg->eop                = htons(EOP_VAL);

    return(sizeof(getChillerTemperature_t));
}

uint16_t controlProtocol::Make_getChillerTemperatureResp(uint16_t Address, uint8_t* pBuff, float temperature, uint16_t SeqNum)
{
    getChillerTemperatureResp_t* msg = reinterpret_cast<getChillerTemperatureResp_t*>(pBuff);
    uint16_t CRC = 0;


    // create the getStatus message in pBuff and CRC16 checksum it
    msg->header.control         = RESPONSE;
    msg->header.length          = sizeof(getChillerTemperatureResp_t);
    msg->header.address.address = htons(Address);
    msg->header.seqNum          = SeqNum;
    msg->header.msgNum          = getChillerTemperatureResp;
    #ifdef __RUNNING_ON_CONTROLLINO__
    //
    // use the dostrf function, left justified, max 7 characters total, max 2 decimal places
    //
    dtostrf(temperature, -(MAX_CHILLER_TEMP_LENGH), 2, reinterpret_cast<char*>(msg->temperature));
    #else
    //
    // use the snprintf function, 
    //
    snprintf(reinterpret_cast<char*>(msg->temperature), MAX_CHILLER_TEMP_LENGH, "%-+3.2f", temperature);
    #endif

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_getChillerTemperatureResp_t);

    // put the CRC
    msg->crc    = htons(CRC);   // TODO: need htons ?


    // put the end of transmission byte
    msg->eop                = htons(EOP_VAL);

    return(sizeof(getChillerTemperatureResp_t));
}

void controlProtocol::Parse_getChillerTemperatureResp(uint8_t* m_buff, float* temperature, uint16_t* pSeqNum)
{
    getChillerTemperatureResp_t* pResponse = reinterpret_cast<getChillerTemperatureResp_t*>(m_buff);


    #ifdef __RUNNING_ON_CONTROLLINO__
    //
    // on uC use atof, sscanf support is dodgy
    //
    *temperature = atof(reinterpret_cast<char*>(pResponse->temperature));
    #else
    sscanf(reinterpret_cast<char*>(pResponse->temperature), "%f", temperature);
    #endif

    *pSeqNum    = pResponse->header.seqNum;
}


uint16_t controlProtocol::Make_getChillerObjTemperature(uint16_t Address, uint8_t* pBuff)
{
    getChillerObjTemperature_t* msg = reinterpret_cast<getChillerObjTemperature_t*>(pBuff);
    uint16_t CRC = 0;


    // create the getStatus message in pBuff and CRC16 checksum it
    msg->header.control         = COMMAND;
    msg->header.length          = sizeof(getChillerObjTemperature_t);
    msg->header.address.address = htons(Address);
    msg->header.seqNum          = m_seqNum;
    msg->header.msgNum          = getChillerObjTemperature;

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_getChillerObjTemperature_t);

    // put the CRC
    msg->crc    = htons(CRC);   // TODO: need htons ?


    // put the end of transmission byte
    msg->eop                = htons(EOP_VAL);

    return(sizeof(getChillerObjTemperature_t));
}

uint16_t controlProtocol::Make_getChillerObjTemperatureResp(uint16_t Address, uint8_t* pBuff, float temperature, uint16_t SeqNum)
{
    getChillerObjTemperatureResp_t* msg = reinterpret_cast<getChillerObjTemperatureResp_t*>(pBuff);
    uint16_t CRC = 0;


    // create the getStatus message in pBuff and CRC16 checksum it
    msg->header.control         = RESPONSE;
    msg->header.length          = sizeof(getChillerObjTemperatureResp_t);
    msg->header.address.address = htons(Address);
    msg->header.seqNum          = SeqNum;
    msg->header.msgNum          = getChillerObjTemperatureResp;
    #ifdef __RUNNING_ON_CONTROLLINO__
    //
    // use the dostrf function, left justified, max 7 characters total, max 2 decimal places
    //
    dtostrf(temperature, -(MAX_CHILLER_TEMP_LENGH), 2, reinterpret_cast<char*>(msg->temperature));
    #else
    //
    // use the snprintf function, 
    //
    snprintf(reinterpret_cast<char*>(msg->temperature), MAX_CHILLER_TEMP_LENGH, "%-+3.2f", temperature);
    #endif

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_getChillerObjTemperatureResp_t);

    // put the CRC
    msg->crc    = htons(CRC);   // TODO: need htons ?


    // put the end of transmission byte
    msg->eop                = htons(EOP_VAL);

    return(sizeof(getChillerObjTemperatureResp_t));
}

void controlProtocol::Parse_getChillerObjTemperatureResp(uint8_t* m_buff, float* temperature, uint16_t* pSeqNum)
{
    getChillerObjTemperatureResp_t* pResponse = reinterpret_cast<getChillerObjTemperatureResp_t*>(m_buff);


    #ifdef __RUNNING_ON_CONTROLLINO__
    //
    // on uC use atof, sscanf support is dodgy
    //
    *temperature = atof(reinterpret_cast<char*>(pResponse->temperature));
    #else
    sscanf(reinterpret_cast<char*>(pResponse->temperature), "%f", temperature);
    #endif

    *pSeqNum    = pResponse->header.seqNum;
}


uint16_t controlProtocol::Make_getChillerInfo(uint16_t Address, uint8_t* pBuff)
{
    getChillerInfo_t* msg = reinterpret_cast<getChillerInfo_t*>(pBuff);
    uint16_t CRC = 0;


    // create the getStatus message in pBuff and CRC16 checksum it
    msg->header.control         = COMMAND;
    msg->header.length          = sizeof(getChillerInfo_t);
    msg->header.address.address = htons(Address);
    msg->header.seqNum          = m_seqNum;
    msg->header.msgNum          = getChillerInfo;

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_getChillerInfo_t);

    // put the CRC
    msg->crc    = htons(CRC);   // TODO: need htons ?


    // put the end of transmission byte
    msg->eop                = htons(EOP_VAL);

    return(sizeof(getChillerInfo_t));
}


uint16_t controlProtocol::Make_getChillerInfoResp(uint16_t Address, uint8_t* pBuff, uint16_t result, uint8_t* info, uint8_t length, uint16_t SeqNum)
{
    getChillerInfoResp_t* msg = reinterpret_cast<getChillerInfoResp_t*>(pBuff);
    uint16_t CRC = 0;


    // create the getStatus message in pBuff and CRC16 checksum it
    msg->header.control         = RESPONSE;
    msg->header.length          = sizeof(getChillerInfoResp_t);
    msg->header.address.address = htons(Address);
    msg->result                 = htons(result);
    msg->header.seqNum          = SeqNum;
    msg->header.msgNum          = getChillerInfoResp;

    strncpy(reinterpret_cast<char*>(msg->info), reinterpret_cast<char*>(info),
        length < MAX_CHILLER_INFO_LENGTH ? length : MAX_CHILLER_INFO_LENGTH); 

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_getChillerInfoResp_t);

    // put the CRC
    msg->crc    = htons(CRC);   // TODO: need htons ?


    // put the end of transmission byte
    msg->eop                = htons(EOP_VAL);

    return(sizeof(getChillerInfoResp_t));
}


void controlProtocol::Parse_getChillerInfoResp(uint8_t* m_buff, uint16_t* result, char* info, uint8_t length, uint16_t* pSeqNum)
{
    getChillerInfoResp_t* pResponse = reinterpret_cast<getChillerInfoResp_t*>(m_buff);


    *result = htons(pResponse->result);

    strncpy(info, reinterpret_cast<char*>(pResponse->info),
        (length < MAX_CHILLER_INFO_LENGTH ? length : MAX_CHILLER_INFO_LENGTH)); 

    *pSeqNum    = pResponse->header.seqNum;
}


uint16_t controlProtocol::Make_NACK(uint16_t Address, uint8_t* pBuff, uint16_t SeqNum)
{
    NACK_t* msg = reinterpret_cast<NACK_t*>(pBuff);
    uint16_t CRC = 0;


    // create the getStatus message in pBuff and CRC16 checksum it
    msg->header.control         = RESPONSE;
    msg->header.length          = sizeof(getChillerTemperatureResp_t);
    msg->header.address.address = htons(Address);
    msg->header.seqNum          = SeqNum;
    msg->header.msgNum          = NACK;

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_NACK_t);

    // put the CRC
    msg->crc    = htons(CRC);   // TODO: need htons ?

    // put the end of transmission byte
    msg->eop                = htons(EOP_VAL);

    return(sizeof(NACK_t));
}


//
// uses class members:
// 1. m_buff    - has the received message in it
// 2. m_seqNum  - is the current sequence number expected in this reply
//
// the calling function supplies the length of the message in m_buff
//
bool controlProtocol::verifyMessage(uint16_t buffLength, uint16_t pktCRC, uint16_t expSeqNum, EOP eot)
{
    if( (verifyMessageSeqNum(buffLength, expSeqNum))    // seqNum
        && (verifyMessageCRC(buffLength, pktCRC))       // CRC
        && (verifyMessageLength(eot)) )                 // length
    {
        return(true);
    } else
    {
        return(false);
    }
}


bool controlProtocol::verifyMessage(uint16_t buffLength, uint16_t pktCRC, EOP eot)
{
        if( (verifyMessageCRC(buffLength, pktCRC))       // CRC
        && (verifyMessageLength(eot)) )                 // length
    {
        return(true);
    } else
    {
        return(false);
    }
}


bool controlProtocol::verifyMessageSeqNum(uint16_t buffLength, uint16_t expSeqNum)
{
    bool            retVal      = true;
    msgHeader_t*    pMsgHeader  = reinterpret_cast<msgHeader_t*>(m_buff);


    //
    // check seqNum match
    //
    if( (expSeqNum != pMsgHeader->seqNum) )
    {
        fprintf(stderr, "ERROR: %s seqNum mismatch\n", __PRETTY_FUNCTION__);
        retVal  = false;
    }

    return(retVal);
}


bool controlProtocol::verifyMessageCRC(uint16_t buffLength, uint16_t pktCRC)
{
    bool            retVal      = true;
    uint16_t        CRC         = calcCRC16(m_buff, buffLength);


    if( (CRC != pktCRC) ) 
    {
        #ifndef __RUNNING_ON_CONTROLLINO__
        fprintf(stderr, "ERROR: %s CRC mismatch 0x%x:0x%x\n", __PRETTY_FUNCTION__, CRC, pktCRC);
        #else
        Serial.print(__PRETTY_FUNCTION__);
        Serial.print(" ERROR: CRC mismatch: 0x");
        Serial.print(CRC, HEX);
        Serial.print(":");
        Serial.println(pktCRC, HEX);
        Serial.flush();
        #endif
        retVal  = false;
    }

    return(retVal);
}


bool controlProtocol::verifyMessageLength(EOP eot)
{
    //
    // should be an EOP_VAL at the end of the buffer
    //
    if( (EOP_VAL == eot) )
        return(true);
    else
        return(false);
}

uint16_t controlProtocol::getMsgId()
{
    msgHeader_t* pmsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);

    return(pmsgHeader->msgNum);
}

