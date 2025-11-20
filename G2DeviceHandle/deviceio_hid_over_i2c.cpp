#include "deviceio_hid_over_i2c.h"

/* Linux */
#include <linux/types.h>
#include <linux/input.h>
#include <linux/hidraw.h>
/*
 * Ugly hack to work around failing compilation on systems that don't
 * yet populate new version of hidraw.h to userspace.
 *
 * If you need this, please have your distro update the kernel headers.
 */
#ifndef HIDIOCSFEATURE
#define HIDIOCSFEATURE(len)    _IOC(_IOC_WRITE|_IOC_READ, 'H', 0x06, len)
#define HIDIOCGFEATURE(len)    _IOC(_IOC_WRITE|_IOC_READ, 'H', 0x07, len)
#endif

/* Unix */
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

/* C */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>

#include "logmanager.h"
#include "deviceexception.h"
#include "deviceio_hid_over_i2c.h"
#include "packet.h"
#include <list>
#include <errno.h>

#define CMD_F1_RETRY_MAX    5
#define OUTPUT_IGNORE       4


/* cmd info */
// HID_I2C_OUTPUT_REGISTER, HID_I2C_OUTPUT_WO_LEN, report id & seq # & STX, ETX, CMD, Length
// for Read, HID_I2C_OUTPUT_REGISTER was excluded.

#define CMD_HW_RESET                    (0xD8)
#define CMD_MAIN_CMD                    (0x13)
#define CMD_CU_ERASE                    (0xB3)
#define CMD_CU_WRITE                    (0xB4)

#define CMD_FW_VER                      (0xF1)

#define G2_SUB_0x13_FLASH_START         0x04
#define G2_SUB_0x13_FLASH_ERASE         0x05
#define G2_SUB_0x13_FLASH_WRITE         0x06
#define G2_SUB_0x13_0x09_FLASH_CHECKSUM 0x09
#define G2_SUB_0x13_FLASH_FINISH        0x07


#define G2_SUB_0x13_GOTOBOOT            0x80
#define G2_SUB_0x13_FWDOWNREADY         0x81
#define G2_SUB_0x13_FWERASE             0x82
#define G2_SUB_0x13_FWWRITE             0x83
#define G2_SUB_0x13_FLASHCHKSUM         0x84
#define G2_SUB_0x13_FLASHFINISH         0x85
#define G2_SUB_0x13_SYSTEMRESET         0x90
#define G2_SUB_0x13_FLASH_ERASE         0x05
#define G2_SUB_0x13_FLASH_WRITE         0x06
#define G2_SUB_0x13_FLASH_BOOT          0x00
#define G2_SUB_0x13_FLASH_CU            0x01
#define G2_SUB_0x13_FLASH_TSMINFO       0x02
#define G2_SUB_0x13_FLASH_DEADCELL      0x03
#define G2_SUB_0x13_FLASH_MAINAPP       0x10
#define G2_SUB_0x13_FLASH_READ          0x08
#define G2_SUB_0x13_FW_DOWNLOAD_READY   0x10
#define G2_SUB_0x13_GOTO_FW_MODE        0x11
#define G2_SUB_0x13_DOWNLOAD_FINISH     0x14

#define HID_I2C_OUTPUT_REGISTER     (0x00E7)

#define TOKEN_STX1 0x02
#define TOKEN_STX2 0xA3

#define TOKEN_ETX1 0x03
#define TOKEN_ETX2 0xb3

#define FW_START_POS 0x8000
#define CU_START_POS 0x4000
#define BOOT_VER_POS 0x400
#define BOOT_FILE_SIZE 0x4000

#define CU_FILE_SIZE 0xb00
#define CU9_FILE_SIZE 0xbC0
#define CU12_FILE_SIZE 0x3300


#define CU_PAGE_CNT 0x2c

#define DUMP_SIZE 0x100

#define PARTITION_ADDR 0x450
#define PARTITION_ADDR_MCHIP 0x350

//partition table index
#define BOOT_REGION         0x00
#define CU_REGION           0x01
#define BASEBIN_REGION      0x06
#define APP_REGION          0x10
#define MT_REGION           0x20
#define FT_REGION           0x21
#define BASEBIN_INTEGRITY   0x30
#define TIC_INTEGRITY       0x31
#define LOAD_BALANCE_REGION 0x40
#define UR_REGION           0x50
#define URD_REGION          0x51

//MCU VID
#define STM_VID      0x0483
#define GD_VID       0x28E9
#define MCHIP_VID    0x04D8
#define INFINION_VID 0x04B4

#define STM_L432_PID      0x01B0
#define STM_U535_PID      0x0217
#define STM_U375_PID      0x0177 

#define G2_FLASH_REGION_ALL 0xAA
#define G2_SUB_0x13_PARTITION_REQUEST_INFO 0x00

#define ADDRESS_META_DATA_IN_BOOT     0x400

#define ADDRESS_META_DATA_IN_APP     0x200
#define ADDRESS_META_DATA_IN_MCHIP_U535     0x300

#define OFFSET_MCU_TYPE_IN_META_DATA 0x40

#define GD_STML432_FWSTRADDRR_POS 0x8200
#define GD_STML432_FWVIRGIN_CODE_POS 0x1fff0
#define STML432_VIRGIN_CODE_POS 0x57F0
#define GD_VIRGIN_CODE_POS 0x5FF0


using namespace G2;
using namespace G2::DeviceIO;

#define TAG "deviceio_hid_over_i2c"

DeviceIO_hid_over_i2c::DeviceIO_hid_over_i2c(CArgHandler *argHandler):
    m_fd(0), out_buffer(0), in_buffer(0), rxdbgbuf(0), tmpRxUnit(0),
    index(0), packet_length(0), dbgidx_push(0), dbgidx_pop(0), 
	m_bOpened(false),GetFWStartFullAddress(0),GetFWStartFullErasesize(0), GetFWStartAddress(0), GetFWEraseSize(0),GetBootStartAddress(0),GetBootEraseSize(0),
	GetCUStartAddress(0), GetCUEraseSize(0), GetLOADBALANCEStartAddress(0), GetLOADBALANCEEraseSize(0),
	GetURStartAddress(0), GetUREraseSize(0), GetURDStartAddress(0), GetURDEraseSize(0), GetMTStartAddress(0), GetMTEraseSize(0),
    GetFTStartAddress(0), GetFTEraseSize(0), GetBASEBINStartAddress(0), GetBASEBINEraseSize(0), MCUPID(0), MCUVID(0),
    BaseStartaddr(0), Protocol_Ver(0), Bootloaderpos(0), FwFeaturepos(0), Partitionpos(0), CU_size_from_FileVirgincode(0), cu_ver(0), Interface_Info(-1),m_Partition_NotMatched(false)
{
    /* malloc out_buffer, in_buffer */
    out_buffer = new unsigned char[HID_OUTPUT_MAX_LEN];
    in_buffer = new unsigned char[HID_INPUT_MAX_LEN];
    rxdbgbuf = new unsigned char[RXDBGBUFSIZE];
    initBuffer();
    index =0;
    packet_length=0;
    tmpRxUnit = new rxUnit();
    dbgidx_push = 0;
    dbgidx_pop = 0;
}


DeviceIO_hid_over_i2c::~DeviceIO_hid_over_i2c()
{
    closeDevice();
    delete [] out_buffer;
    delete [] in_buffer;
    delete [] rxdbgbuf;
    delete tmpRxUnit;
}

void
DeviceIO_hid_over_i2c::initBuffer()
{
    memset(out_buffer, 0x0, sizeof(unsigned char) * HID_OUTPUT_MAX_LEN);
}

#ifdef USE_EXCEPTION
void
DeviceIO_hid_over_i2c::throwNotSupportException(std::string functionName)
{
	std::string msg = "";
	char errorMsg[1024] = "";
	sprintf(errorMsg, "G2DeviceIO_hid_over_i2c, not support : %s", functionName.c_str());
	msg.append(errorMsg);
	throw G2DeviceException(msg, -1);
}
#endif

int
DeviceIO_hid_over_i2c::openDevice(string hidRawName)
{
    char* cptr;
    cptr = new char[hidRawName.length() + 1];
    strcpy(cptr, hidRawName.c_str());
    const char* devName = cptr;

    LOG_G2_D(CLog::getLogOwner(), TAG, "devName=%s", devName);

    if(m_fd >= 0)
    {
        close(m_fd);
        m_fd = -1;

        LOG_G2_I(CLog::getLogOwner(), TAG, "opendevice-close\n" );
    }

    if(isUsingIOCTL())
    {
        LOG_G2_I(CLog::getLogOwner(), TAG, "isUsingIOCTL : true");
        m_fd = open(devName, O_RDWR | O_NONBLOCK); // ioctl : O_NONBLOCK flag has no effect
    }
    else
    {
        LOG_G2_I(CLog::getLogOwner(), TAG, "isUsingIOCTL : false");
        m_fd = open(devName, O_RDWR | O_NONBLOCK);
    }

    delete [] cptr;

    /* handle error */
    if (m_fd < 0)
    {
        std::string msg = "";
        char errorMsg[1024] = "";
        sprintf(errorMsg, "openDevice, unable to open device, errno=%d (%s)", errno, strerror(errno));
        msg.append(errorMsg);

#ifdef USE_EXCEPTION
        throw G2DeviceException(msg, -1);
#else
        LOG_G2_E(CLog::getLogOwner(), TAG, "%s", msg.c_str());
#endif

        return m_fd;
    }
    else
    {
    	m_bOpened = true;
    }

    return m_fd;
}

int
DeviceIO_hid_over_i2c::closeDevice()
{
    LOG_G2_I(CLog::getLogOwner(), TAG, "closeDevice, m_fd : %d",  m_fd);

    if( m_fd < 0 )
    {
        return 0;
    }

    int ret = 0;
    if(m_fd >= 0)
    {
        ret = close(m_fd);
        m_fd = -1;
    }

    if (ret < 0)
    {
        std::string msg = "";
        char errorMsg[1024] = "";
        sprintf(errorMsg, "closeDevice, unable to close device, errno=%d (%s)", errno, strerror(errno));
        msg.append(errorMsg);
#ifdef USE_EXCEPTION
        throw G2DeviceException(msg, -1);
#else
        LOG_G2_E(CLog::getLogOwner(), TAG, "%s", msg.c_str());
#endif

        return m_fd;
    }

    return ret;
}

int
DeviceIO_hid_over_i2c::writeData(unsigned char * buf, int size, int timeoutMsec = 0)
{
    int ret = 0;

    if(isUsingIOCTL())
    {
        ret = ioctl(m_fd, HIDIOCSFEATURE(size), buf);
    }
    else
    {
        ret = write(m_fd, buf, size);
        //ret = write(m_fd, buf + OUTPUT_IGNORE, size - OUTPUT_IGNORE);
    }

    return ret;
}

int
DeviceIO_hid_over_i2c::ReadDataPush( unsigned char * buf, int size)
{
    int i, head;
    int ret = 0;

    head = dbgidx_push;

    // SKIP 2bytes ... Report ID[1byte], Index[1byte]
    for(i = 2; i < size; i++)
    {
        rxdbgbuf[head&(RXDBGBUFSIZE-1)] = buf[i];
        ++head;
        head = head & (RXDBGBUFSIZE-1);
    }
    dbgidx_push = head;
    return ret;
}

int
DeviceIO_hid_over_i2c::ParsePushedData( unsigned char * buf, int size, int Maincommand, int Subcommand, int chk_ack = 0)
{
    unsigned int i=0, len=0, head=0, tail=0, cnt=0;
    int ret = 0;
    unsigned char tmpbuf[RXDBGBUFSIZE];

    head = dbgidx_push;
    tail = dbgidx_pop;

    if(head == tail)
    {
        ret = 0;
        goto end;
    }
    else if(head > tail)
    {
		memcpy(tmpbuf, &rxdbgbuf[tail], head-tail);
		cnt = head - tail;
    }
    else
    {
		memcpy(tmpbuf, &rxdbgbuf[tail], RXDBGBUFSIZE-tail);
		memcpy(&tmpbuf[RXDBGBUFSIZE-tail],rxdbgbuf,head);
		cnt = (RXDBGBUFSIZE-tail) + head;
    }

    if(cnt < 7)
    {
        ret = 0;
        goto end;
    }

    len = 0;
    for(i=0; i<cnt; i++)
    {
        if((tmpbuf[i] == 0x02) && (tmpbuf[i+1] == 0xA3 ))
        {
            len = (tmpbuf[i+3]<<8) + tmpbuf[i+4];

            if(((cnt-i) < 7) || ((cnt-i-7) < len)){ 
                ret = 0;
                goto end;
            }

            if((tmpbuf[i+len+5] != 0x03) || (tmpbuf[i+len+6] != 0xB3)) { 
                LOG_G2_D(CLog::getLogOwner(), TAG, "Packet Corrupted, i=%d, len=%d, etx=[%02X, %02X]", i, len, tmpbuf[i+len+5], tmpbuf[i+len+6]);
                i += 7;
            }
            else
            {
                LOG_G2_D(CLog::getLogOwner(), TAG, "MainCmd=%02X/SubCmd=%02X - idx[Push=%d/Pop=%d]", Maincommand, Subcommand, dbgidx_push, dbgidx_pop);
                LOG_G2_D(CLog::getLogOwner(), TAG, "Packet Matched. cmd[%02X]/len[%02X]/body[%02X/%02X/%02X/%02X/%02X]",
                    tmpbuf[i+2], len, tmpbuf[i+5], tmpbuf[i+6], tmpbuf[i+7], tmpbuf[i+8], tmpbuf[i+9]);

                ret = len+1;
                if(chk_ack && Maincommand == tmpbuf[i+2])
                {
                    LOG_G2_D(CLog::getLogOwner(), TAG, "ACK.. MainCmd=%02X/SubCmd=%02X - idx[Push=%d/Pop=%d]", Maincommand, Subcommand, dbgidx_push, dbgidx_pop);
                    LOG_G2_D(CLog::getLogOwner(), TAG, "ACK.. Packet Matched. cmd[%02X]/len[%02X]/body[%02X/%02X/%02X/%02X/%02X]",
                        tmpbuf[i+2], len, tmpbuf[i+5], tmpbuf[i+6], tmpbuf[i+7], tmpbuf[i+8], tmpbuf[i+9]);
                    LOG_G2_D(CLog::getLogOwner(), TAG, "buf-body-2[%02X/%02X/%02X/%02X/%02X/%02X/%02X/%02X]",
                        buf[2], buf[3], buf[4], buf[5], buf[6], buf[7], buf[8], buf[9]);
                }
                break;
            }
        }
    }

    if(ret)
    {
        dbgidx_pop = (tail + i + len+7) & (RXDBGBUFSIZE-1);
        if(chk_ack && Maincommand != tmpbuf[i+2]) ret = 0;
    }
    else if(!len)
        dbgidx_pop = (tail + i) & (RXDBGBUFSIZE-1);

end:
	return ret;
}

int
DeviceIO_hid_over_i2c::readData( unsigned char * buf, int size, int Maincommand, int Subcommand)
{
    int ret = 0;
    int stx_etx_ret=0;

    if(isUsingIOCTL())
    {
        ret = ioctl(m_fd, HIDIOCGFEATURE(size), buf);
    }
    else
    {
        ret = read(m_fd, buf, size);

        if( ret > 0 )
        {
            if( (buf[2] == TOKEN_STX1) && (buf[3] == TOKEN_STX2) )
            {
                packet_length = (((unsigned int)(buf[5] << 8) + (unsigned int)buf[6]))& 0xffff;

                if((Maincommand == 0xF1) && (buf[4] == 0x51) && (Maincommand != 0))
                {
                    if(strstr((char *)&buf[7],"Not Supprot Command(0xF1") != NULL )
                    {
                        LOG_G2_E(CLog::getLogOwner(), TAG, "Stayed in Bootloader.. ");
                    }
                    else if(strstr((char *)&buf[7],"USB bootloader - Not Supported Command(F1") != NULL )
                    {
                        LOG_G2_E(CLog::getLogOwner(), TAG, "Stayed in USB Bootloader.. ");
                    }
                    else
                    {
                        LOG_G2_D(CLog::getLogOwner(), TAG, "version get.. strange response %02x %02x ", buf[4], Maincommand);
                        return 0; //error
                    }

                }
                else if((buf[4] != Maincommand) && (Maincommand != 0))
                {
                    LOG_G2_I(CLog::getLogOwner(), TAG, "MainCommend different : %02x %02x ", buf[4], Maincommand);
                    return 2; //error
                }

                if((buf[7] != Subcommand) && (Subcommand != 0))
                {
                    LOG_G2_I(CLog::getLogOwner(), TAG, "SubCommand different : %02x %02x ", buf[7], Subcommand);
                    return 2; //error
                }

                if(( buf[7+packet_length] == TOKEN_ETX1) && (buf[8+packet_length] == TOKEN_ETX2))
                {
                    stx_etx_ret = 1;
                }
                else if(packet_length > 45)
                {
                    stx_etx_ret = 1;
                }
                else
                {
                    LOG_G2_D(CLog::getLogOwner(), TAG, "end packet different1 : %02x %02x ", buf[7+packet_length] , buf[8+packet_length] );
                    packet_length=0;
                    return 0; //errors
                }

            }
            else if(packet_length > 58)
            {
                stx_etx_ret = 1;
            }
            else
            {

                if(( buf[packet_length] == TOKEN_ETX1) && (buf[packet_length+1] == TOKEN_ETX2))
                {
                    stx_etx_ret = 1;
                }
                else
                {
                    LOG_G2_D(CLog::getLogOwner(), TAG, "end packet different2 : %02x %02x ", buf[packet_length] , buf[packet_length] );
                    stx_etx_ret = 0;
                }
            }

            if(stx_etx_ret == 0)
                ret = 0;
            /* --- */
        }
        else if(ret <= 0)
        {
            return 0;
        }

    }

    return ret;
}

bool
DeviceIO_hid_over_i2c::isUsingIOCTL()
{
	return false;
}

bool DeviceIO_hid_over_i2c::isDeviceOpened()
{
	return m_bOpened;
}

//TODO: HID_I2C_OUTPUT_REGISTER, HID_I2C_OUTPUT_WO_LEN should be determined based on HID descriptor
//TODO: Report ID should be get from HID descriptor
//TODO: sequence number should be increased
int DeviceIO_hid_over_i2c::CreateCmdBuffer(unsigned char cmd, unsigned char* data, int data_len)
{
    int idx = 0;

    memset(out_buffer, 0, HID_OUTPUT_MAX_LEN);
    out_buffer[idx++] = HID_OUT_REPORT_ID;
    out_buffer[idx++] = index++;

    out_buffer[idx++] = TOKEN_STX1;
    out_buffer[idx++] = TOKEN_STX2;

    out_buffer[idx++] = cmd;
    out_buffer[idx++] = (data_len >> 8) & 0xff;
    out_buffer[idx++] = data_len & 0xff;

    for(int i=0; i<data_len; i++)
    {
        out_buffer[idx++] = data[i];
    }

    out_buffer[idx++] = TOKEN_ETX1;
    out_buffer[idx++] = TOKEN_ETX2;

#if defined(USE_HID_USB)
    if(idx != HID_OUTPUT_MAX_LEN)
    {
        for(; idx < (HID_OUTPUT_MAX_LEN);idx++){
            out_buffer[idx] = 0;
        }
    }
#endif
    return idx;
}

int DeviceIO_hid_over_i2c::waitRxData(int &fd, int uSec)
{
	fd_set rfds;
	struct timeval tv;
	int retval;

	FD_ZERO(&rfds); /* clear the set */
	FD_SET(fd, &rfds); /* add our file descriptor to the set */

	LOG_G2_I(CLog::getLogOwner(), TAG, "waitIOReady : %d(uSec)", uSec);

	tv.tv_sec = 0;
	tv.tv_usec = uSec;

	retval = select(fd + 1, &rfds, NULL, NULL, &tv);

    //recover return value
	if (retval == -1)
	{
		LOG_G2_D(CLog::getLogOwner(), TAG, "! select, errno=%d (%s)", errno, strerror(errno)); /* an error accured */
		return errno;
	}
	else if (retval == 0)
	{
		LOG_G2_D(CLog::getLogOwner(), TAG, "! IO timeout, uSecWait=%d", uSec); /* a timeout occured */
		return -1;
	}
	else
	{
		LOG_G2_I(CLog::getLogOwner(), TAG, "IO data ready"); /* there was data to read */
		return 0;
	}
}


// TODO: read size should be changed based on descriptor
// TODO: architecture change
int DeviceIO_hid_over_i2c::TxSingleCmdWaitAck(unsigned char cmd, unsigned char* data, int data_len, int uSecWait)
{
	int nReadSize = -1;
	unsigned char *tmp_read = new unsigned char[HID_INPUT_MAX_LEN];

	waitRxData(m_fd, uSecWait);
	nReadSize = readData(tmp_read, HID_INPUT_MAX_LEN, cmd, 0);

	LOG_G2_I(CLog::getLogOwner(), TAG, "nReadSize : %d", nReadSize);
	////////////////////////////////////////////////////////////////////////
	// Inside of readData, ather commands were ignored in this architecture.
    if((nReadSize > 2) && ((tmp_read[4] == cmd) || (cmd == CMD_FW_VER)))
    {
        bool bRet = Check_Nak(tmp_read);
        if(bRet == true)
        {
            tmpRxUnit->setBuf(tmp_read, nReadSize);
            tmpRxUnit->setSize(nReadSize);
        }
        else
        {
            delete [] tmp_read;
            return -1;
        }
        //rxListUnits.push_back(tmpRxUnit);	// will be handled later.
    }

    else
    {
        tmpRxUnit->clearBuf();
        tmpRxUnit->setSize(0);
    }

	delete [] tmp_read;

	return nReadSize;
}

// return uSec
int DeviceIO_hid_over_i2c::GetCmdWaitAckTime(unsigned char cmd, int mSec)
{
	return mSec * 1000;
}

int DeviceIO_hid_over_i2c::TryWriteData(unsigned char cmd, unsigned char* data, int data_len, int trial = 1, int mSecWait = 1000)
{
    int uSecWait = 0;
    int nTry = 0;
    int nTryResult = 2;
    int size = -1;
    int ret =0;

    while(size != 0 && nTry++ < 10)
    {
        waitRxData(m_fd, uSecWait);
        size = readData(in_buffer, HID_INPUT_MAX_LEN, 0, 0);
    }

    size = CreateCmdBuffer(cmd, data, data_len);	// m_buffer allocated

    uSecWait = GetCmdWaitAckTime(cmd, mSecWait);  // 30 msec for Normal command, but some command needs more time

    // Tx and Check Ack
    nTry = 0;
    while (++nTry <= trial)
    {
        int uSecWaitWr = uSecWait < 200000 ? uSecWait : 200000;
        ret = writeData(out_buffer, size);
        if (ret > 2)
        {	// Write success
            break;
        }
        usleep(uSecWaitWr);
    }

    if(ret <= 0)
    {
        LOG_G2_E(CLog::getLogOwner(), TAG, "writeData fail");
        return ret;
    }

    LOG_G2_I(CLog::getLogOwner(), TAG, "uSecWait : %d", uSecWait);

    ret =0;
    nTry = 0;
    while (++nTry <= trial)
    {
        nTryResult = TxSingleCmdWaitAck(cmd, data, data_len, uSecWait);
        if (nTryResult > 2)
        {	// ACK received
            ret = 1;
            break;
        }
    }

    return ret;
}

// Try 5 times
string DeviceIO_hid_over_i2c::TxRequestFwVer(int mSec, int format=0)
{
    int nRequestResult = TryWriteData(CMD_FW_VER, NULL, 0, CMD_F1_RETRY_MAX, mSec);
    unsigned char* buf = tmpRxUnit->getBuf();
    int size=tmpRxUnit->getSize();

    if (nRequestResult > 0)
    {
    	// parsing FW_VER PACKET
    	if (size < 5)
    	{
    		LOG_G2_E(CLog::getLogOwner(), TAG, "Try check FW Ver, but rxUnit has wrong data : count of ack packet %d", size);
    		return "";
    	}
    	else if (buf[4] != CMD_FW_VER && buf[4] != 0x51)
    	{
    		LOG_G2_E(CLog::getLogOwner(), TAG, "Try check FW Ver, but rxUnit has different command : 0x%x", buf[4]);
    		return "";
    	}
    	else
    	{
    		// set FW Version & print out to scrn
            if(strstr((char *)&buf[7],"Not Supprot Command(0xF1") != NULL )
            {
                LOG_G2_I(CLog::getLogOwner(), TAG, "Force Set FW version 0.0.0");
                buf[10] = buf[11] = buf[12] = buf[13] = 0;
            }
            else if(strstr((char *)&buf[7],"USB bootloader - Not Supported Command(F1") != NULL )
            {
                LOG_G2_D(CLog::getLogOwner(), TAG, "Stayed in USB bootloader");
                LOG_G2_I(CLog::getLogOwner(), TAG, "Force Set FW version 0.0.0");
                buf[10] = buf[11] = buf[12] = buf[13] = 0;
            }
    		LOG_G2_I(CLog::getLogOwner(), TAG, "FW VERSION %d.%d.%d", buf[10], buf[11], (buf[12] * 256 + buf[13]));

            char temp[17];
            string strVersion = "";
            int cnt = 0;
            if(format == 0)
            {
                cnt = snprintf(temp, 16, "%02d.%02d.%05d", buf[10], buf[11], (buf[12] * 256 + buf[13]));
            }
            else
            {
                cnt = snprintf(temp, 16, "%02X.%02X.%04X", buf[10], buf[11], (buf[12] * 256 + buf[13]));
            }
            
    		if (cnt >= 0 && cnt < 100)
    		{
    			strVersion = string(temp, cnt);
    		}
    		else
    		{
    			LOG_G2_E(CLog::getLogOwner(), TAG, "Cannot change FW Version packet to string");
    		}

    		return strVersion;
    	}

    }

    return "";
}

/*
int DeviceIO_hid_over_i2c::TxRequestBASEBINUpdate(unsigned char* file_buf)
{
    LOG_G2_I(CLog::getLogOwner(), TAG, "TxRequest BaseBins Update START");

    unsigned char buf[64];

    int buf_size = 0;
    int nRequestResult = 0;
    unsigned char send_buffer[256]={0,};
    int pos = 0;
    int end_pos = 0;
    int send_length = 0x100;
    unsigned int checksum = 0;

    pos = GetBASEBINStartAddress;
    end_pos = GetBASEBINEraseSize;
    

    //BASEBIN Start
    buf_size = BaseBin_start_data(buf, BASEBIN_REGION);
    nRequestResult = TryWriteData(CMD_MAIN_CMD, buf, buf_size, CMD_F1_RETRY_MAX, 1000);
    if (nRequestResult <= 0)
    {
        LOG_G2_E(CLog::getLogOwner(), TAG, "BaseBin_erase_data error");
        return nRequestResult;
    }

    //BASEBIN erase
    buf_size = BaseBin_erase_data(buf, BASEBIN_REGION);
    nRequestResult = TryWriteData(CMD_MAIN_CMD, buf, buf_size, CMD_F1_RETRY_MAX, 1000);
    if (nRequestResult <= 0)
    {
        LOG_G2_E(CLog::getLogOwner(), TAG, "BaseBin_erase_data error");
        return nRequestResult;
    }

    //BASEBIN WRITE
    while(pos < end_pos)
    {
        LOG_G2_D( CLog::getLogOwner(), TAG, "BaseBin send_pos : %X, end_pos : %X", pos, end_pos);
        memcpy(send_buffer, file_buf+pos, sizeof(send_buffer));
        nRequestResult = File_Write_CMD(send_buffer, send_length, CMD_MAIN_CMD, BASEBIN_REGION);

        if (nRequestResult <= 0)
        {
            LOG_G2_E(CLog::getLogOwner(), TAG, "BaseBin Write error");
            return nRequestResult;
        }

        pos+=send_length;
    }

    //checksum calculate
    for(int i=pos; i<end_pos; ++i)
        checksum += file_buf[i];


    //BASEBIN CHECKSUM
    buf_size = Flash0x09CheckSum_data(buf,end_pos,checksum);
    nRequestResult = TryWriteData(CMD_MAIN_CMD, buf, buf_size, CMD_F1_RETRY_MAX, 1000);

    if (nRequestResult <= 0)
    {
        LOG_G2_E(CLog::getLogOwner(), TAG, "CMD_FlashCheckSum send error");
        return nRequestResult;
    }

    if(nRequestResult == 1)
    {
        LOG_G2(CLog::getLogOwner(), TAG, "BaseBin Update Success");
    }
    else
    {
        LOG_G2_E(CLog::getLogOwner(), TAG, "BaseBin Update Fail");
        LOG_G2_E(CLog::getLogOwner(), TAG, "Do not shut down !! Please retry again.");
    }

    return nRequestResult;

}

// Try 5 times
int DeviceIO_hid_over_i2c::TxRequestLOADBALANCEUpdate(unsigned char* file_buf)
{
    LOG_G2_I(CLog::getLogOwner(), TAG, "TxRequest BaseBins Update START");

    unsigned char buf[64];

    int buf_size = 0;
    int nRequestResult = 0;
    unsigned char send_buffer[256]={0,};
    int pos = 0;
    int end_pos = 0;
    int send_length = 0x100;
    unsigned int checksum = 0;


    pos = GetLOADBALANCEStartAddress;
    end_pos = GetLOADBALANCEEraseSize;
    

    //BASEBIN Start
    buf_size = BaseBin_start_data(buf, LOAD_BALANCE_REGION);
    nRequestResult = TryWriteData(CMD_MAIN_CMD, buf, buf_size, CMD_F1_RETRY_MAX, 1000);
    if (nRequestResult <= 0)
    {
        LOG_G2_E(CLog::getLogOwner(), TAG, "Load Balance erase_data error");
        return nRequestResult;
    }

    //BASEBIN erase
    buf_size = BaseBin_erase_data(buf, LOAD_BALANCE_REGION);
    nRequestResult = TryWriteData(CMD_MAIN_CMD, buf, buf_size, CMD_F1_RETRY_MAX, 1000);
    if (nRequestResult <= 0)
    {
        LOG_G2_E(CLog::getLogOwner(), TAG, "Load Balance erase_data error");
        return nRequestResult;
    }

    //BASEBIN WRITE
    while(pos < end_pos)
    {
        LOG_G2_D( CLog::getLogOwner(), TAG, "Load Balance send_pos : %X, end_pos : %X", pos, end_pos);
        memcpy(send_buffer, file_buf+pos, sizeof(send_buffer));
        nRequestResult = File_Write_CMD(send_buffer, send_length, CMD_MAIN_CMD, LOAD_BALANCE_REGION);

        if (nRequestResult <= 0)
        {
            LOG_G2_E(CLog::getLogOwner(), TAG, "Load Balance Write error");
            return nRequestResult;
        }

        pos+=send_length;
    }

    //checksum calculate
    for(int i=pos; i<end_pos; ++i)
        checksum += file_buf[i];


    //BASEBIN CHECKSUM
    buf_size = Flash0x09CheckSum_data(buf,end_pos,checksum);
    nRequestResult = TryWriteData(CMD_MAIN_CMD, buf, buf_size, CMD_F1_RETRY_MAX, 1000);

    if (nRequestResult <= 0)
    {
        LOG_G2_E(CLog::getLogOwner(), TAG, "CMD_FlashCheckSum send error");
        return nRequestResult;
    }

    if(nRequestResult == 1)
    {
        LOG_G2(CLog::getLogOwner(), TAG, "Load Balance Update Success");
    }
    else
    {
        LOG_G2_E(CLog::getLogOwner(), TAG, "Load Balance Update Fail");
        LOG_G2_E(CLog::getLogOwner(), TAG, "Do not shut down !! Please retry again.");
    }

    return nRequestResult;

}
*/



int DeviceIO_hid_over_i2c::HWReset()
{
    int nRequestResult = TxRequestHW_Reset();
    if (nRequestResult <= 0)
    {
        LOG_G2_E(CLog::getLogOwner(), TAG, "TxRequestHW_Reset error");
        return nRequestResult;
    }
    usleep(150000);    

    return nRequestResult;

}

int DeviceIO_hid_over_i2c::GoToBoot()
{
    int nRequestResult = 1;
    int buf_size = 0;
    unsigned char buf[64] = {0,};    

    buf_size = GotoBoot_data(buf);
    nRequestResult = TryWriteData(CMD_MAIN_CMD, buf, buf_size, CMD_F1_RETRY_MAX, 1000);

    if(nRequestResult <= 0)
    {
        LOG_G2_E(CLog::getLogOwner(), TAG, "GotoBoot_data error");
        return nRequestResult;
    }
    usleep(300000);

    return nRequestResult;

}



// Try 3 times
int DeviceIO_hid_over_i2c::TxRequestCuUpdate(unsigned char* file_buf, bool bPartition)
{
    unsigned char send_buffer[256]={0,};
    unsigned char dump_buffer[0x1000]={0,};
    unsigned short send_length = 0x40;
    int pos = CU_START_POS;
    int custrpos_File = CU_START_POS;
    int cuend_pos = (CU_START_POS + CU_FILE_SIZE);
    unsigned char buf[64] = {0,};    
    int buf_size = 0;
    int cu_page = 0x320;
    int cu_page_count = 0;
    int nRequestResult = 0x00;
    int retry = 0;
    unsigned int FWStartAddress = 0;
    unsigned int CUStartAddress = 0x08004000;    
    
    if(bPartition == true)
    {    
        FWStartAddress = BaseStartaddr + GetFWStartAddress;
        CUStartAddress = BaseStartaddr + GetCUStartAddress;
        pos = GetCUStartAddress;
        cuend_pos = GetCUStartAddress + GetCUEraseSize;
        custrpos_File = GetCUStartAddress;

        if(CU_size_from_FileVirgincode <= 0xFFFF)
        {
            cuend_pos = GetCUStartAddress + CU_size_from_FileVirgincode;
        }
        else
        {
            if (file_buf[pos] == 0x07)
            {
                cuend_pos = GetCUStartAddress+ 0x630;
            }
            else if (file_buf[pos] == 0x09) 
            {
                cuend_pos = GetCUStartAddress + CU9_FILE_SIZE;
            }
            else if (file_buf[pos] == 0x0A) 
            {
                cuend_pos = GetCUStartAddress+ CU9_FILE_SIZE;
            }
            else if (file_buf[pos] == 0x0B) 
            {
                cuend_pos = GetCUStartAddress + 0x930;
            }
            else if (file_buf[pos] == 0x0C) 
            {
                cuend_pos = GetCUStartAddress + CU12_FILE_SIZE;
            }
            else
            {
                cuend_pos = GetCUStartAddress + CU_FILE_SIZE;
            }
        }
    }
    else
    {
        FWStartAddress = 0x08008000;

        if(CU_size_from_FileVirgincode <= 0xFFFF)
        {
            cuend_pos = CU_START_POS + CU_size_from_FileVirgincode;
        }
        else
        {        
            if (file_buf[pos] == 0x07)
            {
                cuend_pos = CU_START_POS+ 0x630;
            }
            else if (file_buf[pos] == 0x09) 
            {
                cuend_pos = CU_START_POS + CU9_FILE_SIZE;
            }
            else if (file_buf[pos] == 0x0A) 
            {
                cuend_pos = CU_START_POS+ CU9_FILE_SIZE;
            }
            else if (file_buf[pos] == 0x0B) 
            {
                cuend_pos = CU_START_POS + 0x930;
            }
            else if (file_buf[pos] == 0x0C) 
            {
                cuend_pos = CU_START_POS + CU12_FILE_SIZE;
            }
            else //cu2
            {
                cuend_pos = CU_START_POS + CU_FILE_SIZE;
            }
        }
    }

    LOG_G2_D(CLog::getLogOwner(), TAG, "CUStartAddress 0x%x, GetCUStartAddress 0x%x",CUStartAddress, GetCUStartAddress);    
    LOG_G2_D(CLog::getLogOwner(), TAG, "cuend_pos 0x%x, pos 0x%x",cuend_pos, pos);

    cu_page_count = ((cuend_pos - pos) % send_length == 0) ? ((cuend_pos - pos) / send_length) : (((cuend_pos - pos) / send_length) + 1);
    LOG_G2_D(CLog::getLogOwner(), TAG, "cu_page_count 0x%x",cu_page_count);

    if(bPartition == true)
    {
        if((GetFWStartFullAddress == GetFWStartAddress) && (GetFWStartFullErasesize == GetFWEraseSize))
        {
            LOG_G2_E(CLog::getLogOwner(), TAG, "FW info not matched 0x%x, 0x%x, 0x%x, 0x%x", GetFWStartFullAddress, GetFWStartAddress, GetFWStartFullErasesize , GetFWEraseSize);
            return -1;
        }
    }
    LOG_G2_D(CLog::getLogOwner(), TAG, "StartAddress 0x%x",FWStartAddress);

    buf_size = FWDownReady_data(buf, FWStartAddress);
    nRequestResult = TryWriteData(CMD_MAIN_CMD, buf, buf_size, CMD_F1_RETRY_MAX, 4000);
    if (nRequestResult <= 0)
    {
        LOG_G2_E(CLog::getLogOwner(), TAG, "CMD_FWDownReady");
        return nRequestResult;
    }

    while(retry < CMD_F1_RETRY_MAX)
    {
        //CUErase

        buf_size = Cu_Erase_data(buf, cu_page_count);
        cuend_pos = CU_START_POS + (cu_page_count * send_length);  // update with page count
        nRequestResult = TryWriteData(CMD_CU_ERASE, buf, buf_size, CMD_F1_RETRY_MAX, 5000);

        if(nRequestResult <= 0)
        {
            LOG_G2_E(CLog::getLogOwner(), TAG, "Cu_Erase_data error");
            if(retry == CMD_F1_RETRY_MAX-1)
                return nRequestResult;
        }
        else
        {
            break;
        }
        retry++;
        cu_page_count = CU_PAGE_CNT;    // back to CU_PAGE_CNT for next loop
    }

    //CU WRITE
    while(pos < cuend_pos)
    {
        LOG_G2_I( CLog::getLogOwner(), TAG, "CU send_pos : %X, size : %X", pos, cuend_pos);
        memcpy(send_buffer, file_buf+pos, sizeof(send_buffer));
        nRequestResult = CU_Write_CMD(send_buffer, send_length, CMD_CU_WRITE, cu_page);

        if(nRequestResult <= 0)
        {
            LOG_G2_E(CLog::getLogOwner(), TAG, "CU Write error");
            return nRequestResult;
        }

        pos+=send_length;
        cu_page++;
    }

    //Dump & compare
    pos = 0;
    while(pos < (cu_page_count * send_length))
    {
        LOG_G2_I( CLog::getLogOwner(), TAG, "CU dump send_pos : %X, end_pos : %X", pos, (cu_page_count * send_length));    
        nRequestResult = Dump(dump_buffer+pos, CUStartAddress+pos, DUMP_SIZE);

        if(nRequestResult <= 0)
        {
            LOG_G2_E(CLog::getLogOwner(), TAG, "CU Dump error");
            return nRequestResult;
        }

        pos+=DUMP_SIZE;
    }    

    LOG_G2_D( CLog::getLogOwner(), TAG, "CU dump addr : %X, size : %X", CUStartAddress, (cu_page_count * send_length));
    nRequestResult = dumpTofile_compare(dump_buffer, file_buf+custrpos_File, (cu_page_count * send_length));
    if(nRequestResult == 1)
    {
        LOG_G2_D(CLog::getLogOwner(), TAG, "CU Write Success");
    }
    else
    {
        LOG_G2_E(CLog::getLogOwner(), TAG, "CU Write Fail");
    }

    return nRequestResult;
}


// Try 5 times
int DeviceIO_hid_over_i2c::TxRequestFwUpdate(unsigned char* file_buf, bool bPartition)
{
    unsigned char buf[64];
    int buf_size = 0;
    int nRequestResult = 0;
    int pos = FW_START_POS;
    int strpos_infile = FW_START_POS;
    int fw_size = 0x18000; 
    int fw_end_pos = (FW_START_POS + fw_size);
    int send_length = 0x100;
    unsigned char send_buffer[256]={0,};
    unsigned int fw_checksum = 0;
    unsigned char* read_buf;
    unsigned int base_fwstraddress = 0x08008000; //fw area
    unsigned int fw_erase_size = 0x00018000;
    unsigned int fw_metadata_pos = ADDRESS_META_DATA_IN_APP;
    unsigned short addr_idx = 0;

    if(bPartition == true)
    {
        pos = GetFWStartAddress;
        strpos_infile = GetFWStartAddress;
        base_fwstraddress = GetFWStartAddress + BaseStartaddr;
        fw_size = Fw_write_size(file_buf, (GetFWStartAddress + GetFWEraseSize - 0x10));
        fw_end_pos = (GetFWStartAddress + fw_size);
        addr_idx = FwFeaturepos;
    }
    else
    {
        GetFWEraseSize = fw_erase_size;	
        Fw_write_size(file_buf, GetFWEraseSize);
        fw_size = Fw_write_size(file_buf, (FW_START_POS + fw_erase_size - 0x10));
        fw_end_pos = (FW_START_POS + fw_size);
        addr_idx = FW_START_POS + fw_metadata_pos;
    }

    if (fw_size == 0)
    {
        LOG_G2_E(CLog::getLogOwner(), TAG, "fw_write_size ERR");
        return nRequestResult;
    }    
    
    nRequestResult = Get_AppStartAddr_fromBinFile(file_buf, addr_idx, &base_fwstraddress, bPartition);
    if(nRequestResult <= 0)
    {
        LOG_G2_E(CLog::getLogOwner(), TAG, "ERROR : FW File hasn't Address!!!");
        return nRequestResult;
    }
    
    LOG_G2_D( CLog::getLogOwner(), TAG, "base_fwstraddress : %X, GetFWEraseSize : %X", base_fwstraddress, GetFWEraseSize);

    //FwDownReady
    buf_size = FWDownReady_data(buf, base_fwstraddress);
    nRequestResult = TryWriteData(CMD_MAIN_CMD, buf, buf_size, CMD_F1_RETRY_MAX, 4000);

    if (nRequestResult <= 0)
    {
        LOG_G2_E(CLog::getLogOwner(), TAG, "CMD_FWDownReady");
        return nRequestResult;
    }

    //FLASHERASE
    buf_size = FlashErase_data(buf, base_fwstraddress, GetFWEraseSize);
    nRequestResult = TryWriteData(CMD_MAIN_CMD, buf, buf_size, CMD_F1_RETRY_MAX, 5000);

    if (nRequestResult <= 0)
    {
        LOG_G2_E(CLog::getLogOwner(), TAG, "CMD_FlashErase");
        return nRequestResult;
    }

    //FLASHWRITE
    while(pos < fw_end_pos)
    {
        LOG_G2_I( CLog::getLogOwner(), TAG, "FW send_pos : %X, size : %X", pos, fw_end_pos);
        memcpy(send_buffer, file_buf+pos, sizeof(send_buffer));
        nRequestResult = FW_Write_CMD(send_buffer, send_length, CMD_MAIN_CMD);

        if (nRequestResult <= 0)
        {
            LOG_G2_E(CLog::getLogOwner(), TAG, "FW Write error");
            return nRequestResult;
        }

        pos+=send_length;
    }

    //checksum calculate
    for(int i=strpos_infile; i<fw_end_pos; ++i)
        fw_checksum += file_buf[i];

    //FLASHCHECKSUM
    buf_size = FlashCheckSum_data(buf, base_fwstraddress, fw_size,fw_checksum);
    nRequestResult = TryWriteData(CMD_MAIN_CMD, buf, buf_size, CMD_F1_RETRY_MAX, 1000);

    if (nRequestResult <= 0)
    {
        LOG_G2_E(CLog::getLogOwner(), TAG, "CMD_FlashCheckSum send error");
        return nRequestResult;
    }        


    read_buf = tmpRxUnit->getBuf();
    nRequestResult = FlashCheckSum_Check(read_buf, fw_checksum);

    if (nRequestResult <= 0)
    {
        LOG_G2_E(CLog::getLogOwner(), TAG, "CheckSum Error");
        return nRequestResult;
    }

    //FLASHFINISH
    buf_size = FlashFinish_data(buf);
    nRequestResult = TryWriteData(CMD_MAIN_CMD, buf, buf_size, CMD_F1_RETRY_MAX, 1000);

    if (nRequestResult <= 0)
    {
        LOG_G2_E(CLog::getLogOwner(), TAG, "CMD_FlashFinish send error");
        return nRequestResult;
    }

    read_buf = tmpRxUnit->getBuf();
    nRequestResult = FlashFinish_Check(read_buf);

    if (nRequestResult <= 0)
    {
        LOG_G2_E(CLog::getLogOwner(), TAG, "CMD_FlashFinish error");
        return nRequestResult;
    }
    else
    {
        LOG_G2(CLog::getLogOwner(), TAG, "FW update Success");
    }

    return nRequestResult;
}

int DeviceIO_hid_over_i2c::Fw_write_size(unsigned char* file_buf, unsigned int erasesize)
{
    int i=0;
    int write_size=0;

    for(i=0; i<4; i++)
    {
        if(file_buf[(erasesize)+i] != 0xa5)
        {
            return write_size;
        }
    }

    write_size = (file_buf[erasesize + 11] <<24);
    write_size+= (file_buf[erasesize + 10] <<16);
    write_size+= (file_buf[erasesize + 9] <<8);
    write_size+= file_buf[erasesize + 8];

    if(write_size%0x100 != 0)
    {
        int blank_data = (256-(write_size%256));
        write_size += blank_data;
    }

    return write_size;
}

int DeviceIO_hid_over_i2c::TxRequestHW_Reset()
{
    int nRequestResult = TryWriteData(CMD_HW_RESET, NULL, 0, CMD_F1_RETRY_MAX, 2000);

    LOG_G2_D(CLog::getLogOwner(), TAG, "nRequestResult : %d", nRequestResult);

    if (nRequestResult <= 0)
    {
        LOG_G2_E(CLog::getLogOwner(), TAG, "TxRequestHW_Reset Error");
    }

    return nRequestResult;
}

int DeviceIO_hid_over_i2c::TxRequestSystem_Reset()
{
    unsigned char buf[64]={0,};
    int buf_size = 0;

    buf_size = system_reset_data(buf);
    int nRequestResult = TryWriteData(CMD_MAIN_CMD, buf, buf_size, CMD_F1_RETRY_MAX, 1000);

    if (nRequestResult <= 0)
    {
        LOG_G2_E(CLog::getLogOwner(), TAG, "TxRequestHW_Reset Error");
    }

    return nRequestResult;
}

unsigned short DeviceIO_hid_over_i2c::MCUType_Verify(unsigned char* file_buf, int filebuf_idx, int filebuf_bootidx, bool getPartition)
{

    unsigned char temp[4]={0,};
    unsigned char dump_buffer[0x1000]={0,};
    int a = 0;
    int bootver_straddress = 0x08000000;
    int iRet = 0;

    //check mcu type in file
    temp[0] = file_buf[filebuf_idx];
    temp[1] = file_buf[filebuf_idx + 1];
    temp[2] = file_buf[filebuf_idx + 2];
    temp[3] = file_buf[filebuf_idx + 3];

    if(Interface_Info == -1) //MCUVID PID info is FileInfo
    {
        MCUVID = (unsigned short)((file_buf[filebuf_bootidx + 1] << 8) + file_buf[filebuf_bootidx]);
        MCUPID = (unsigned short)((file_buf[filebuf_bootidx + 3] << 8) + file_buf[filebuf_bootidx + 2]);    
    }

    //check repeat
    for (a = 1; a < 4; a++)
    {
        if (temp[0] != file_buf[filebuf_idx + (a * 4)] || temp[1] != file_buf[filebuf_idx + (a * 4 + 1)] || temp[2] != file_buf[filebuf_idx + (a * 4 + 2)] || temp[3] != file_buf[filebuf_idx + (a * 4 + 3)])
        {
            if(Interface_Info == -1)
            {
                MCUVID = STM_VID;
                MCUPID = STM_L432_PID; // No MCU Type repeat pattern found. regard as old STM FW
            }
            else
            {
          
                return 0xFF;// error
            }
        }
    }

    if(Interface_Info == -1) //MCUVID PID info is FileInfo
    {    
        //refer file

        if(BaseStartaddr != 0x00)
        {
            bootver_straddress = BaseStartaddr;
        }
        
        LOG_G2_D(CLog::getLogOwner(), TAG, "MCU Type Dump address 0x%x",(bootver_straddress + Bootloaderpos + OFFSET_MCU_TYPE_IN_META_DATA));
        iRet= Dump(dump_buffer, (bootver_straddress + Bootloaderpos + OFFSET_MCU_TYPE_IN_META_DATA), 0x10);    
        if(iRet <= 0)
        {
            LOG_G2_E(CLog::getLogOwner(), TAG, "MCU Type Dump Fail");
            return 0xFF; 
        }

        //check dump mcu type in file
        temp[0] = dump_buffer[0];
        temp[1] = dump_buffer[1];
        temp[2] = dump_buffer[2];
        temp[3] = dump_buffer[3];

        //check repeat
        for (a = 1; a < 4; a++)
        {
            if (temp[0] != dump_buffer[(a * 4)] || temp[1] != dump_buffer[(a * 4 + 1)] || temp[2] != dump_buffer[(a * 4 + 2)] || temp[3] != dump_buffer[(a * 4 + 3)])
            {
                if((MCUVID == STM_VID) && (MCUPID == STM_L432_PID))
                {
                    return 0;
                }
                else
                {
                    LOG_G2_E(CLog::getLogOwner(), TAG, "VID PID Error");                
                    return 0xFF;// error
                }
            }
        }
        
        iRet = dumpTofile_compare(dump_buffer, file_buf+filebuf_idx, 0x10);


        
        if(iRet == 0) //non matched
        {
            LOG_G2_D(CLog::getLogOwner(), TAG, "Not matched FILE MCUVID 0x%x, FILE MCUPID 0x%x",MCUVID, MCUPID);
            LOG_G2_D(CLog::getLogOwner(), TAG, "Not matched Target MCUVID 0x%x, Target MCUPID 0x%x",(short)((dump_buffer[1] << 8) + dump_buffer[0]), (short)((dump_buffer[3] << 8) + dump_buffer[2]));

            if((MCUVID == GD_VID) && (MCUPID == 0x00 || (short)((dump_buffer[3] << 8) + dump_buffer[2]) == 0x00))
            {
                LOG_G2_D(CLog::getLogOwner(), TAG, "GD MCU OLD FW");
            }
            else
            {
                return 0xFF;
            }
           
        }        
    }
    else //MCUVID PID info is TargetInfo
    {
        if((MCUVID  != ((unsigned short)((file_buf[filebuf_bootidx + 1] << 8) + file_buf[filebuf_bootidx]))) ||
          (MCUPID  != ((unsigned short)((file_buf[filebuf_bootidx + 3] << 8) + file_buf[filebuf_bootidx + 2]))))
        {
            return 0xFF; //error
        }
    }

    if(getPartition == false)
    {    
        SET_basestraddr(MCUVID); //set basestartaddr again
    }
    LOG_G2_D(CLog::getLogOwner(), TAG, "MCUVID 0x%x, MCUPID 0x%x",MCUVID, MCUPID);

    return MCUPID;
}


void DeviceIO_hid_over_i2c::SET_basestraddr(unsigned short vid_temp)
{
    LOG_G2_D(CLog::getLogOwner(), TAG, "SET_basestraddr 0x%x", vid_temp);
    if(vid_temp == STM_VID) //STM
    {
        BaseStartaddr = 0x08000000;
    }
    else if(vid_temp == GD_VID) //GD
    {
        BaseStartaddr = 0x08000000;
    }
    else if(vid_temp == MCHIP_VID) //MicroChip
    {
        BaseStartaddr = 0x00000000;
    }    
    else if(vid_temp == INFINION_VID) //Infinion
    {
        BaseStartaddr = 0x10000000;                
    }
}

int DeviceIO_hid_over_i2c::checkFW_CUVirginCode(unsigned char* file_buf, int fwstraddr_pos ,int fwvirginaddr, int custraddr, int cuvirginaddr)
{
    unsigned int cu_checksum=0, cu_checksum_cmp = 0;
    int i=0;
    unsigned int cu_end_size = 0;
    unsigned int cu_end_addr = 0;
    unsigned int fwstraddr = 0x08008000;

    if(GetFWStartAddress != 0x00)
    {
        fwstraddr = BaseStartaddr + GetFWStartAddress;
    }

    LOG_G2_D(CLog::getLogOwner(), TAG, "fwstraddr 0x%x, fwstraddr_pos 0x%x", fwstraddr, fwstraddr_pos);
    

    //fw address
    for(i=0; i<13; i+=4)
    {
        if((file_buf[fwstraddr_pos+i+3] == (unsigned char)((fwstraddr >> 24)&0xff)) && (file_buf[fwstraddr_pos+i+2] ==(unsigned char)((fwstraddr >> 16)&0xff)) && (file_buf[fwstraddr_pos+i+1] == (unsigned char)((fwstraddr >> 8)&0xff)) && (file_buf[fwstraddr_pos+i] == (unsigned char)((fwstraddr)&0xff)))
        {            
            LOG_G2_I(CLog::getLogOwner(), TAG, "binfile fw address match !!!!!!!!");
        }
        else
        {
            LOG_G2_E(CLog::getLogOwner(), TAG, "binfile fw address not match !!!!!!!!");
            return -1;
        }
    }
    
    //fw write size magic code
    for(i=0; i<4; i++)
    {
        if(file_buf[fwvirginaddr+i] != 0xa5)
        {
            LOG_G2_E(CLog::getLogOwner(), TAG, "binfile fw magic error !!!!!!!!");
            return -1;
        }
    }


    //cu magic code check
    for(i=0; i<8; i++)
    {
        if(file_buf[cuvirginaddr+i] == 0xff)
        {
            LOG_G2_E(CLog::getLogOwner(), TAG, "Cannot Find CU Virgin code !!!!!!!!");
            return -1;
        }
    }

    //cu checksum check
    cu_checksum = (file_buf[cuvirginaddr + 7] <<24);
    cu_checksum+= (file_buf[cuvirginaddr + 6] <<16);
    cu_checksum+= (file_buf[cuvirginaddr + 5] <<8);
    cu_checksum+= file_buf[cuvirginaddr + 4];

    cu_end_size = (file_buf[cuvirginaddr + 15] <<24);
    cu_end_size+= (file_buf[cuvirginaddr + 14] <<16);
    cu_end_size+= (file_buf[cuvirginaddr + 13] <<8);
    cu_end_size+= file_buf[cuvirginaddr + 12];

    if(cu_end_size > 0xFFFF) //don't have size
    {
        LOG_G2_D(CLog::getLogOwner(), TAG, "cu end size not include in file!!");
    
        if ((cu_ver == 0x02) || (cu_ver == 0x03))
        {            
            cu_end_addr = custraddr + CU_FILE_SIZE;
        }
        if(cu_ver == 0x09)
        {            
            cu_end_addr = custraddr + CU9_FILE_SIZE;
        }
        if(cu_ver == 0x0A)
        {     
            LOG_G2_D(CLog::getLogOwner(), TAG, "cu_end_addr set");
            cu_end_addr = custraddr + CU9_FILE_SIZE;
        }
        if(cu_ver == 0x0C)
        {            
            cu_end_addr = custraddr + CU12_FILE_SIZE;
        }        
    }
    else
    {
        cu_end_addr = custraddr + cu_end_size;
    }

    LOG_G2_D(CLog::getLogOwner(), TAG, "cu_addr !!!!!!!! 0x%x, 0x%x 0x%x, 0x%x 0x%x", custraddr, cu_end_addr, cuvirginaddr, cu_end_size, cu_ver);


    //checksum calculate
    for(int i=custraddr; i < (int)cu_end_addr; i++)
        cu_checksum_cmp += file_buf[i];

    if(cu_checksum != cu_checksum_cmp)
    {
        LOG_G2_E(CLog::getLogOwner(), TAG, "binfile cu checksum error !!!!!!!! 0x%x, 0x%x", cu_checksum, cu_checksum_cmp);
        return -1;
    }
    else
    {
        CU_size_from_FileVirgincode = (cu_end_addr - custraddr);
    }


    return 0;

}


int DeviceIO_hid_over_i2c::Precheckforupdate(unsigned char* file_buf, bool bBoot_force_update, bool getPartition)
{
    LOG_G2_D(CLog::getLogOwner(), TAG, "Precheckforupdate START");
    int nRequestResult = 0;
    unsigned int partition_address = 0;
    int fwstraddr_pos = FwFeaturepos;

    if(getPartition == 1)
    {
        partition_address = (Bootloaderpos + Partitionpos);
    }
    else
    {
        partition_address = PARTITION_ADDR;
    }
    LOG_G2_D(CLog::getLogOwner(), TAG, "getPartition %d",getPartition);    

    nRequestResult = DumpFileInfo(file_buf, partition_address);
    if(nRequestResult == -1)
    {
        LOG_G2_D(CLog::getLogOwner(), TAG, "Partition Table Not matched... Update using File information ");
        m_Partition_NotMatched = true;
    }

    //check VID PID
    nRequestResult = check_VID_PID(file_buf, getPartition);
    if(nRequestResult < 0)
    {
        LOG_G2_E(CLog::getLogOwner(), TAG, "VID & PID Not matched");
        return nRequestResult;
    }

    LOG_G2_D(CLog::getLogOwner(), TAG, "VID & PID Matched");

    if(getPartition == true)
    {
        //check cu version
        cu_ver = file_buf[GetCUStartAddress];

        LOG_G2_D(CLog::getLogOwner(), TAG, "GetFWStartAddress 0x%x, GetFWEraseSize 0x%x", GetFWStartAddress, GetFWEraseSize);
        
        nRequestResult = checkFW_CUVirginCode(file_buf, fwstraddr_pos, (GetFWStartAddress + GetFWEraseSize - 0x10), GetCUStartAddress ,(GetCUStartAddress + GetCUEraseSize - 0x10));
        if(nRequestResult < 0)
        {
            LOG_G2_E(CLog::getLogOwner(), TAG, "CU VirginCode Not matched");
            return nRequestResult;
        }
    }
    else
    {
        if(MCUVID == MCHIP_VID)
            partition_address = PARTITION_ADDR_MCHIP;

        if((MCUVID == MCHIP_VID) || ((MCUVID == STM_VID) && (MCUPID == STM_U535_PID)) || ((MCUVID == STM_VID) && (MCUPID == STM_U375_PID)))
        {
            fwstraddr_pos = 0x300;
        }

        cu_ver = file_buf[CU_START_POS];
        
        nRequestResult = checkFW_CUVirginCode(file_buf,GD_STML432_FWSTRADDRR_POS, GD_STML432_FWVIRGIN_CODE_POS, CU_START_POS, STML432_VIRGIN_CODE_POS);
        if(nRequestResult < 0)
        {
            nRequestResult = checkFW_CUVirginCode(file_buf,GD_STML432_FWSTRADDRR_POS, GD_STML432_FWVIRGIN_CODE_POS, CU_START_POS, GD_VIRGIN_CODE_POS);
            if(nRequestResult < 0)
            {                
                LOG_G2_E(CLog::getLogOwner(), TAG, "CU VirginCode Not matched");
                return nRequestResult;
            }
        }        
    }
    
    return nRequestResult; //

}

int DeviceIO_hid_over_i2c::check_VID_PID(unsigned char* file_buf, bool getPartition)
{

    int filebuf_idx = (FwFeaturepos + OFFSET_MCU_TYPE_IN_META_DATA);
    int filebuf_bootidx = (Bootloaderpos + OFFSET_MCU_TYPE_IN_META_DATA);
    unsigned short PID = 0;

    LOG_G2_D(CLog::getLogOwner(), TAG, "Check_VID_PID START");

    PID = MCUType_Verify(file_buf, filebuf_idx, filebuf_bootidx, getPartition);    
    if(PID == 0xff)
    {
        return -1;
    }    
    else if(PID == 0)
    {
        MCUPID = STM_L432_PID;
        MCUVID = STM_VID;
        BaseStartaddr = 0x08000000;
    }

    return PID;
}


bool DeviceIO_hid_over_i2c::RequestPartitionInfo(bool bFileInfo)
{
    int read_retry = 3;
    bool bRet = false;
    int uSecWait = GetCmdWaitAckTime(0x13, 1000); 
    unsigned char m_pcReadBuf[HID_INPUT_MAX_LEN];
    unsigned char data_buffer[0x200];
    int size = HID_INPUT_MAX_LEN;    
    int pos = 0;
    int iRet = 0;
    int a = 0;
    int id0 = 0; 
    int id1 = 0; 
    int id2 = 0;
    unsigned char str_ver = 0;
    unsigned char cnt_record = 0;
    unsigned char checksum = 0;
    int partition_offset = 0;
    int nTry = 0;    

    LOG_G2_D(CLog::getLogOwner(), TAG, "RequestPartitionInfo Start");

    //read
    while(size != 0 && nTry++ < 20)
    {
        waitRxData(m_fd, uSecWait);
        size = readData(in_buffer, HID_INPUT_MAX_LEN, 0, 0);
    }
    size = HID_INPUT_MAX_LEN;

    //send partition request
    iRet =  Patition_Request(G2_FLASH_REGION_ALL);
    
    if(iRet < 4)
    {
        LOG_G2_E(CLog::getLogOwner(), TAG, "Patition request Send fail");
        return bRet;
    }
    
    while(pos < size)
    {        
        waitRxData(m_fd, uSecWait);
        iRet = readData(m_pcReadBuf, HID_INPUT_MAX_LEN, CMD_MAIN_CMD, G2_SUB_0x13_PARTITION_REQUEST_INFO); //dump

        LOG_G2_D(CLog::getLogOwner(), TAG, "iRet %d",iRet);
        LOG_G2_D(CLog::getLogOwner(), TAG, "m_pcReadBuf 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x,",m_pcReadBuf[4], m_pcReadBuf[5], m_pcReadBuf[6], m_pcReadBuf[7], m_pcReadBuf[8], m_pcReadBuf[9]);
        if( read_retry == 0)
        {
            return bRet ;
        }        

        read_retry--;
        if(iRet < 36) //partition header + one partition info
        {
            continue;
        }

        if(pos == 0)
        {
            if(m_pcReadBuf[4] !=0x13) continue;

            memcpy(data_buffer, m_pcReadBuf+8, iRet-8);
            
            if((int)packet_length >= (iRet-9))
            {
                packet_length-= (iRet-9); //start packet 7 + next packet header 2
            }
            else
            {
                packet_length =0;
            }

            pos+=(iRet-8);
            size = ((unsigned int)m_pcReadBuf[5] << 8) + (unsigned int)m_pcReadBuf[6];

            //LOG_G2_D(CLog::getLogOwner(), TAG, "packet_length: %x", packet_length);
        }
        else
        {
            memcpy(data_buffer+pos, m_pcReadBuf+2, iRet-2);
            if((int)packet_length >= iRet)
            {
                packet_length-=iRet; //+report id
            }
            else
            {
                packet_length=0;
            }
            pos+=(iRet-2);
            
            //LOG_G2_D(CLog::getLogOwner(), TAG, "pos: %x, size: %x", pos, size);
        }

        LOG_G2_I(CLog::getLogOwner(), TAG, "remain packet_length : %x",packet_length);

    }

    //InBuffer partition
    if (data_buffer[1] == 0x00)
    {
        if (data_buffer[0] == 0xaa)
        {
        
            id0 = data_buffer[2];
            id1 = data_buffer[10];
            id2 = data_buffer[18];

            if((id0 == 0x00) && (data_buffer[4] == 0xEA))
            {
                for (a = 2; a < 9; a++)
                {
                    checksum += data_buffer[a];
                }
                if (checksum != data_buffer[9])
                {
                    LOG_G2_D(CLog::getLogOwner(), TAG, "Partition data error0\n");
                    return false;
                }
                
                str_ver = data_buffer[3];
                BaseStartaddr = (unsigned int)((data_buffer[8] << 24) + (data_buffer[7] << 16) + (data_buffer[6] << 8) + (data_buffer[5] << 0)); //little endian

                checksum = 0;
            }

            if((id1 == 0x01) && (data_buffer[12] == 0xEA))
            {
                for (a = 10; a < 17; a++)
                {
                    checksum += data_buffer[a];
                }
                if (checksum != data_buffer[17])            
                {
                    LOG_G2_D(CLog::getLogOwner(), TAG, "Partition data error1\n");
                    return false;
                }

                checksum = 0;
                cnt_record = data_buffer[11];            
                Protocol_Ver = (short)((data_buffer[14] * 256) + data_buffer[13]); //little endian
            }


            LOG_G2_D(CLog::getLogOwner(), TAG, "BaseStartaddr : 0x%x, Protocol Ver 0x%x",BaseStartaddr,Protocol_Ver);
            if (data_buffer[3] < 0x10)
            {
                LOG_G2_E(CLog::getLogOwner(), TAG, "Patition didn't use");
                return false;
            }
            else
            {

                if((id2 == 0x02) && (data_buffer[20] == 0xEA) && (str_ver >  0x10))
                {
                    for (a = 18; a < 25; a++)
                    {
                        checksum += data_buffer[a];
                    }
                    if (checksum == data_buffer[25])
                    {

                        LOG_G2_D(CLog::getLogOwner(), TAG, "Partition table ID2 exist\n");

                        partition_offset = 8;
                        Interface_Info = data_buffer[19];
                        MCUVID = (unsigned short)((data_buffer[22] << 8) + data_buffer[21]);
                        MCUPID = (unsigned short)((data_buffer[24] << 8) + data_buffer[23]);
                    }

                    checksum = 0;
                }

                for (int a = 0; a < cnt_record; a++)
                {
                    if(data_buffer[partition_offset + 19 + (a * 8)] == 0x01)
                    {
                        bRet = Get_Partition_info(partition_offset + 20 + (a * 8), data_buffer);
                    }
                }
            }
        }
        else
        {
            bRet = Get_Partition_info(partition_offset + 4, data_buffer); //real data only
        }
    }
    else
    {
        LOG_G2_E(CLog::getLogOwner(), TAG, "Patition request NAK");
        bRet = false;
    }    

    return bRet;
}

bool DeviceIO_hid_over_i2c::Get_Partition_info(int idx, unsigned char* m_abytContent)
{
    unsigned int str_addr = (unsigned int)((m_abytContent[idx + 1]) + (m_abytContent[idx + 2] << 8)) * 512;
    unsigned int size = (unsigned int)((m_abytContent[idx + 3]) + (m_abytContent[idx + 4] << 8)) * 512;

    //LOG_G2_D(CLog::getLogOwner(), TAG, "Rx->idx:%d",idx);

    if (m_abytContent[idx] == APP_REGION)
    {
        GetFWStartAddress = str_addr;
        GetFWEraseSize = size;
        LOG_G2_D(CLog::getLogOwner(), TAG, "Rx->Partition Region MAINAPP str_addr: 0x%x  size: 0x%x",GetFWStartAddress, GetFWEraseSize);
    }
    else if (m_abytContent[idx] == BOOT_REGION)
    {
        GetBootStartAddress = str_addr;
        GetBootEraseSize = size;
        LOG_G2_D(CLog::getLogOwner(), TAG, "Rx->Partition Region BOOT str_addr: 0x%x  size: 0x%x",GetBootStartAddress, GetBootEraseSize);
    }
    else if (m_abytContent[idx] == CU_REGION)
    {
        GetCUStartAddress = str_addr;
        GetCUEraseSize = size;
        LOG_G2_D(CLog::getLogOwner(), TAG, "Rx->Partition Region CU str_addr: 0x%x  size: 0x%x",GetCUStartAddress, GetCUEraseSize);
    }
    else if (m_abytContent[idx] == BASEBIN_REGION)
    {
        GetBASEBINStartAddress = str_addr;
        GetBASEBINEraseSize = size;
        LOG_G2_D(CLog::getLogOwner(), TAG, "Rx->Partition Region BASEBIN str_addr: 0x%x  size: 0x%x",str_addr, size);
    }
    else if (m_abytContent[idx] == UR_REGION)
    {
        GetURStartAddress = str_addr;
        GetUREraseSize = size;
        LOG_G2_D(CLog::getLogOwner(), TAG, "Rx->Partition Region UR str_addr: 0x%x  size: 0x%x",str_addr, size);

    }
    else if (m_abytContent[idx] == URD_REGION)
    {
        GetURDStartAddress = str_addr;
        GetURDEraseSize = size;
        LOG_G2_D(CLog::getLogOwner(), TAG, "Rx->Partition Region URD str_addr: 0x%x  size: 0x%x",str_addr, size);
    }
    else if (m_abytContent[idx] == LOAD_BALANCE_REGION)
    {
        GetLOADBALANCEStartAddress = str_addr;
        GetLOADBALANCEEraseSize = size;
        LOG_G2_D(CLog::getLogOwner(), TAG, "Rx->Partition Region LBP str_addr: 0x%x  size: 0x%x",str_addr, size);
    }    

    return true;
    
}


int DeviceIO_hid_over_i2c::DumpFileInfo(unsigned char* file_buf, unsigned int partition_address)
{
    int tablecount = file_buf[partition_address + 0x09];
    //int Partition_structure_Ver = file_buf[partition_address + 0x11];
    int iRet = 1;
    int i=0, idx = 0;
    int tableactive = 0;
    unsigned int file_straddr = 0;
    unsigned int file_erasesize = 0;
    unsigned char checksumboot = 0;
    int offset = 0;

    int id0 = file_buf[partition_address];
    int id1 = file_buf[partition_address + 0x08];
    int id2 = file_buf[partition_address + 0x10];

    if((id0 == 0x00) && (file_buf[partition_address + 0x02] == 0xEA))
    {
        for (i = (int)(partition_address); i < (int)(partition_address + 0x07); i++)
        {
            checksumboot += file_buf[i];
        }
        if (checksumboot == file_buf[(partition_address + 0x07)])
        {
            BaseStartaddr = (unsigned int)((file_buf[partition_address + 0x06] << 24) + (file_buf[partition_address + 0x05] << 16) + (file_buf[partition_address + 0x04] << 8) + (file_buf[partition_address + 0x03] << 0)); //little endian
            LOG_G2_D(CLog::getLogOwner(), TAG, "using file Baseaddress 0x%x",BaseStartaddr);            
        }
    }
   
    if((id2 == 0x02) && (file_buf[partition_address + 0x12] == 0xEA))
    {
        for (i = (int)(partition_address + 0x10); i < (int)(partition_address + 0x17); i++)
        {
            checksumboot += file_buf[i];
        }
        if (checksumboot == file_buf[(partition_address + 0x17)])
        {
            LOG_G2_D(CLog::getLogOwner(), TAG, "ID2 exist");
            offset = 8;

            if(Interface_Info == -1)
            {
                LOG_G2_D(CLog::getLogOwner(), TAG, "file includes ID2, but the Target does not include ID2, So apply File Info");
                Interface_Info = file_buf[partition_address + 0x11];
                MCUVID = (unsigned short)(((file_buf[partition_address + 0x14]) << 8) + (file_buf[partition_address + 0x13]));
                MCUPID = (unsigned short)(((file_buf[partition_address + 0x16]) << 8) + (file_buf[partition_address + 0x15]));                
            }
        }

        checksumboot = 0;
    }

    if((id0 == 0x00) && (file_buf[partition_address + 0x02] == 0xEA) && (id1 == 0x01) && (file_buf[partition_address + 0x0A] == 0xEA) && (tablecount != 0x00))
    {
        for( i =0; i< tablecount; i++)
        {
            idx = file_buf[partition_address + offset + 0x12 + (i*8)];
            tableactive = file_buf[partition_address + offset + 0x11 + (i*8)];

            if(tableactive == 1)
            {
                switch(idx)
                {
                    case BOOT_REGION:
                        file_straddr = (unsigned int)(file_buf[partition_address + offset + 0x13 + (i*8)] + (file_buf[partition_address + offset  + 0x14 + (i*8)] << 8) )* 0x200;
                        file_erasesize = (unsigned int)(file_buf[partition_address + offset + 0x15 + (i*8)] + (file_buf[partition_address + offset + 0x16 + (i*8)] << 8))* 0x200;
                        
                        if(GetBootStartAddress != file_straddr)
                        {
                            LOG_G2_D(CLog::getLogOwner(), TAG, "Partition different Boot straddr: 0x%x, Fileaddr: 0x%x",GetBootStartAddress, file_straddr);
                            GetBootStartAddress = file_straddr;
                            iRet = -1;
                        }
                    
                        if(GetBootEraseSize != file_erasesize)
                        {
                            LOG_G2_D(CLog::getLogOwner(), TAG, "Boot Erase straddr: 0x%x, Fileaddr: 0x%x",GetBootEraseSize, file_erasesize);
                            GetBootEraseSize = file_erasesize;
                            iRet = -1;
                        }                        
                        break;
                    case CU_REGION:
                        file_straddr = (unsigned int)(file_buf[partition_address + offset + 0x13 + (i*8)] + (file_buf[partition_address + offset + 0x14 + (i*8)] << 8)) * 0x200;
                        file_erasesize = (unsigned int)(file_buf[partition_address + offset + 0x15 + (i*8)] + (file_buf[partition_address + offset + 0x16 + (i*8)] << 8))* 0x200;
                    
                        if(GetCUStartAddress != file_straddr)
                        {
                            LOG_G2_D(CLog::getLogOwner(), TAG, "CU straddr: 0x%x, Fileaddr: 0x%x",GetCUStartAddress, file_straddr);
                            GetCUStartAddress = file_straddr;
                            
                            iRet = -1;
                        }

                        if(GetCUEraseSize != file_erasesize)
                        {
                            LOG_G2_D(CLog::getLogOwner(), TAG, "CU Erase straddr: 0x%x, Fileaddr: 0x%x",GetFWEraseSize, file_erasesize);
                            GetCUEraseSize = file_erasesize;                            
                            iRet = -1;
                        }

                        break;
                    case APP_REGION:
                        file_straddr = (unsigned int)(file_buf[partition_address + offset + 0x13 + (i*8)] + (file_buf[partition_address + offset + 0x14 + (i*8)] << 8)) * 0x200;
                        file_erasesize = (unsigned int)(file_buf[partition_address + offset + 0x15 + (i*8)] + (file_buf[partition_address + offset + 0x16 + (i*8)] << 8)) * 0x200;
                    
                        if(GetFWStartAddress != file_straddr)
                        {
                            LOG_G2_D(CLog::getLogOwner(), TAG, "FW straddr: 0x%x, Fileaddr: 0x%x",GetFWStartAddress, file_straddr);
                            GetFWStartAddress = file_straddr;                            
                            iRet = -1;
                        }

                        if(GetFWEraseSize != file_erasesize)
                        {
                            LOG_G2_D(CLog::getLogOwner(), TAG, "FW Erase straddr: 0x%x, Fileaddr: 0x%x",GetFWEraseSize, file_erasesize);
                            GetFWEraseSize = file_erasesize;                            
                            iRet = -1;
                        }          
                        break;
                }
            }
        }
    }

    return iRet;    
}



int DeviceIO_hid_over_i2c::TxRequestBootUpdate(unsigned char* file_buf, bool bBoot_force_update, bool bPartition)
{
    LOG_G2_I(CLog::getLogOwner(), TAG, "TxRequestBootUpdate START");

    unsigned char buf[64];
    int bootver_address = 0x08000400;
    int bootstart_address = 0x08000000;
    int buf_size = 0;
    int nRequestResult = 0;
    unsigned char dump_buffer[BOOT_FILE_SIZE+10]={0,};
    unsigned char send_buffer[256]={0,};
    int pos = 0;
    int boot_end_pos = BOOT_FILE_SIZE;
    int send_length = 0x100;
    string fw_ver = "";

    fw_ver = TxRequestFwVer(1000);
    if(strstr(fw_ver.c_str(), "00.00.0000") != NULL)
    {
        //LOG_G2_E(CLog::getLogOwner(), TAG, "Can't update bootloader.. Try it again After flashing !!!");
        return BOOTRUNNING;
    }

    if(bPartition == true)
    {
        bootver_address = BaseStartaddr + GetBootStartAddress + 0x400;
        bootstart_address = BaseStartaddr + GetBootStartAddress;
        boot_end_pos = GetBootEraseSize;
        pos = GetBootStartAddress;
    }
    LOG_G2_D(CLog::getLogOwner(), TAG, "bootver_address 0x%x, bootstart_address 0x%x",bootver_address,bootstart_address);    
    LOG_G2_D(CLog::getLogOwner(), TAG, "Boot strpos : 0x%x, endpos : 0x%x",pos, boot_end_pos);

    //boot ver
    if(bBoot_force_update == false) //boot force update option
    {
        char curr_boot[9]={0,}, targ_boot[9]={0,};
        nRequestResult = Dump(dump_buffer, bootver_address, 0x20);
        LOG_G2_D(CLog::getLogOwner(), TAG, "Try Dump Boot Version!!!!");        
        if(nRequestResult <= 0)
        {
            LOG_G2_E(CLog::getLogOwner(), TAG, "Version Dump Fail");
            return nRequestResult;
        }

        nRequestResult = dumpTofile_compare(dump_buffer, file_buf+BOOT_VER_POS, 0x20);

        memcpy((char*)curr_boot, (char*)(dump_buffer+8), 8);
        memcpy((char*)targ_boot, (char*)(file_buf+BOOT_VER_POS+8), 8);
        //boot_ver check`   `
        if(nRequestResult == 1)
        {
            LOG_G2(CLog::getLogOwner(), TAG, "Same Bootloader");
            LOG_G2_D(CLog::getLogOwner(), TAG, "Target Boot Ver : %s",curr_boot);
            LOG_G2_D(CLog::getLogOwner(), TAG, "Binary Boot Ver : %s",targ_boot);
            return BOOTSAMEVERSION; //boot_ver same
        }
        else
        {
            LOG_G2(CLog::getLogOwner(), TAG, "Different Bootloader, Bootloader Update Needed");
            LOG_G2_D(CLog::getLogOwner(), TAG, "Target Boot Ver : %s",curr_boot);
            LOG_G2_D(CLog::getLogOwner(), TAG, "Binary Boot Ver : %s",targ_boot);
        }
    }

    //boot erase
    buf_size = boot_erase_data(buf, boot_end_pos);
    nRequestResult = TryWriteData(CMD_MAIN_CMD, buf, buf_size, CMD_F1_RETRY_MAX, 1000);
    if (nRequestResult <= 0)
    {
        LOG_G2_E(CLog::getLogOwner(), TAG, "boot_erase_data error");
        return nRequestResult;
    }

    //Boot WRITE
    while(pos < boot_end_pos)
    {
        LOG_G2_I( CLog::getLogOwner(), TAG, "Boot dump send_pos : %X, end_pos : %X", pos, boot_end_pos);
        memcpy(send_buffer, file_buf+pos, sizeof(send_buffer));
        nRequestResult = Boot_Write_CMD(send_buffer, send_length, CMD_MAIN_CMD);

        if(nRequestResult <= 0)
        {
            LOG_G2_E(CLog::getLogOwner(), TAG, "Boot Write error");
            return nRequestResult;
        }

        pos+=send_length;
    }

    //Dump & compare
    pos = 0;
    while(pos < BOOT_FILE_SIZE)
    {
        LOG_G2_I( CLog::getLogOwner(), TAG, "Boot send_pos : %X, end_pos : %X", pos, boot_end_pos);
        nRequestResult = Dump(dump_buffer+pos, bootstart_address+pos, DUMP_SIZE);

        if(nRequestResult <= 0)
        {
            LOG_G2_E(CLog::getLogOwner(), TAG, "Boot Dump error");
            return nRequestResult;
        }

        pos+=DUMP_SIZE;
    }

    nRequestResult = dumpTofile_compare(dump_buffer, file_buf, BOOT_FILE_SIZE);

    if(nRequestResult == 1)
    {
        LOG_G2(CLog::getLogOwner(), TAG, "Bootloader Update Success");
        nRequestResult = BOOTWRITEFINISH; //write boot finish
    }
    else
    {
        LOG_G2_E(CLog::getLogOwner(), TAG, "Bootloader Update Fail");
        LOG_G2_E(CLog::getLogOwner(), TAG, "Do not shut down !! Please retry again.");
    }

    return nRequestResult;

}

int DeviceIO_hid_over_i2c::boot_erase_data(unsigned char* buf, int erase_size)
{
    int size = 0;
    unsigned int eraseSize = erase_size ; // 16KB

    buf[size++] = G2_SUB_0x13_FLASH_ERASE;
    buf[size++] = G2_SUB_0x13_FLASH_BOOT;  //boot
    buf[size++]=(eraseSize>>24)&0xFF;
    buf[size++]=(eraseSize>>16)&0xFF;
    buf[size++]=(eraseSize>>8)&0xFF;
    buf[size++]=(eraseSize)&0xFF;

    return size;
}

int DeviceIO_hid_over_i2c::system_reset_data(unsigned char* buf)
{
    int size = 0;

    buf[size++] = G2_SUB_0x13_SYSTEMRESET;

    return size;
}

int DeviceIO_hid_over_i2c::dumpTofile_compare(unsigned char* dump_buffer, unsigned char* file_buf, int compare_size)
{
    int i=0;

    for(i=0; i<compare_size; i++)
    {
        if(dump_buffer[i] != file_buf[i])
        {
            LOG_G2_I(CLog::getLogOwner(), TAG, "dump[%X]=%02X, buf[%X]=%02X", i, dump_buffer[i], i, file_buf[i]);
            return 0;
        }
    }

    return 1;
}

int DeviceIO_hid_over_i2c::GotoBoot_data(unsigned char* buf)
{
    int size = 0;

    buf[size++] = G2_SUB_0x13_GOTOBOOT;
    return size;
}

int DeviceIO_hid_over_i2c::Cu_Erase_data(unsigned char* buf, int page_cnt)
{
    int size = 0;

    buf[size++]=0x03;
    buf[size++]=0x20;
    buf[size++] = (page_cnt >> 8) & 0xFF;
    buf[size++] = page_cnt & 0xFF;

    return size;
}

int DeviceIO_hid_over_i2c::FWDownReady_data(unsigned char* buf, unsigned int base_address)
{
    int size = 0;
    buf[size++] = G2_SUB_0x13_FWDOWNREADY;
    buf[size++]=(base_address>>24)&0xFF;
    buf[size++]=(base_address>>16)&0xFF;
    buf[size++]=(base_address>>8)&0xFF;
    buf[size++]=(base_address)&0xFF;

    return size;
}

int DeviceIO_hid_over_i2c::FlashErase_data(unsigned char* buf, unsigned int base_address, unsigned int fw_erase_size)
{
    int size = 0;

    buf[size++] = G2_SUB_0x13_FWERASE;
    buf[size++]=(base_address>>24)&0xFF;
    buf[size++]=(base_address>>16)&0xFF;
    buf[size++]=(base_address>>8)&0xFF;
    buf[size++]=(base_address)&0xFF;
    buf[size++]=(fw_erase_size>>24)&0xFF;
    buf[size++]=(fw_erase_size>>16)&0xFF;
    buf[size++]=(fw_erase_size>>8)&0xFF;
    buf[size++]=(fw_erase_size)&0xFF;

    return size;
}


int DeviceIO_hid_over_i2c::FlashCheckSum_data(unsigned char* buf, unsigned int base_fwstraddress, int file_size, unsigned int fw_checksum)
{
    int size = 0;

    buf[size++] = G2_SUB_0x13_FLASHCHKSUM;
    buf[size++]=(base_fwstraddress>>24)&0xFF;
    buf[size++]=(base_fwstraddress>>16)&0xFF;
    buf[size++]=(base_fwstraddress>>8)&0xFF;
    buf[size++]=(base_fwstraddress)&0xFF;
    buf[size++]=(file_size>>24)&0xFF;
    buf[size++]=(file_size>>16)&0xFF;
    buf[size++]=(file_size>>8)&0xFF;
    buf[size++]=(file_size)&0xFF;
    buf[size++]=(fw_checksum>>24)&0xFF;
    buf[size++]=(fw_checksum>>16)&0xFF;
    buf[size++]=(fw_checksum>>8)&0xFF;
    buf[size++]=(fw_checksum)&0xFF;

    return size;
}

int DeviceIO_hid_over_i2c::FlashCheckSum_Check(unsigned char* buf, unsigned int fw_checksum)
{
    int ret = 0;

    if(buf[7] == G2_SUB_0x13_FLASHCHKSUM)
    {
        if((buf[9] == ((fw_checksum>>24)&0xFF)) &&  (buf[10] == ((fw_checksum>>16)&0xFF)) &&\
            (buf[11] == ((fw_checksum>>8)&0xFF) ) &&  (buf[12] == ((fw_checksum)&0xFF))  )
        {
            LOG_G2_I(CLog::getLogOwner(), TAG, "CHECKSUM  OK");
            ret = 1;
        }
        else
        {
            LOG_G2_I(CLog::getLogOwner(), TAG, "CHECKSUM  ERROR");
        }
    }

    return ret;
}

int DeviceIO_hid_over_i2c::FlashFinish_data(unsigned char* buf)
{
    int size = 0;

    buf[size++] = G2_SUB_0x13_FLASHFINISH;
    buf[size++] = 0x00;

    return size;
}

int DeviceIO_hid_over_i2c::FlashFinish_Check(unsigned char* read_buf)
{
    if((read_buf[7] == G2_SUB_0x13_FLASHFINISH) && (read_buf[8] == 0x00) )
    {
        return 1;
    }

    return 0;
}

int DeviceIO_hid_over_i2c::CU_Write_CMD(unsigned char* send_buffer, unsigned short send_length, unsigned char send_cmd, int cu_page)
{
    int uSecWait = GetCmdWaitAckTime(0xb4, 1000);  // 30 msec for Normal command, but some command needs more time
    unsigned char buf[512+9];
    unsigned char i2c_buf[HID_OUTPUT_MAX_LEN+10];
    int ret = 1;
    unsigned long i=0,pos=0,sended_len, send_len, len;
    int retry_ack = 5;

    memset(i2c_buf, 0x00, sizeof(i2c_buf));

    len = send_length+2;
    buf[0] = TOKEN_STX1;
    buf[1] = TOKEN_STX2;
    buf[2] = send_cmd;
    buf[3] = (len>>8)  & 0xFF;
    buf[4] = len & 0xFF;

    buf[5] = (cu_page>>8)  & 0xFF;
    buf[6] = cu_page & 0xFF;

    for(i=0; i<len-2; i++) {
        buf[7+i] = send_buffer[i];
    }
    buf[5+len] = TOKEN_ETX1;
    buf[6+len] = TOKEN_ETX2;

    len += 7;

    sended_len = 0;
    while(sended_len < len)
    {
        if(len < HID_OUTPUT_MAX_LEN - 2) send_len = len;
        else send_len = ((len - sended_len) > (HID_OUTPUT_MAX_LEN - 2))? (HID_OUTPUT_MAX_LEN - 2) : (len - sended_len);

        i2c_buf[0] = HID_OUT_REPORT_ID;
        i2c_buf[1] = index++;
        if(index == 0xAA) index++;

        for(i= 0; i < send_len; i++){
            i2c_buf[i+2] = buf[pos++];
        }
        for(i=send_len; i < (HID_OUTPUT_MAX_LEN);i++){
            i2c_buf[i+2] = 0;
        }
        sended_len += send_len;

        ret = writeData( i2c_buf, HID_OUTPUT_MAX_LEN);

        if(ret == -1)
        {
            LOG_G2_I(CLog::getLogOwner(), TAG, "FW Write error2");
            return ret;
        }
    }

    while(retry_ack >= 0)
    {
        retry_ack--;
        waitRxData(m_fd, uSecWait); //30ms

        ret = readData(buf, HID_INPUT_MAX_LEN, 0xb4, 0 );

        if(ret > 2)
        {
            break;
        }
        usleep(1000 * 500);
    }

    return ret;
}


int DeviceIO_hid_over_i2c::FW_Write_CMD(unsigned char* send_buffer, unsigned short send_length, unsigned char send_cmd)
{
    int uSecWait = GetCmdWaitAckTime(0xb4, 1000);  // 30 msec for Normal command, but some command needs more time
    unsigned char buf[512+9];
    unsigned char i2c_buf[HID_OUTPUT_MAX_LEN+10];
    int ret = 1;
    unsigned long i=0,pos=0,sended_len, send_len, len;
    int retry_ack = 5;

    memset(i2c_buf, 0x00, sizeof(i2c_buf));

    len = send_length+1;

    buf[0] = TOKEN_STX1;
    buf[1] = TOKEN_STX2;
    buf[2] = send_cmd;
    buf[3] = (len>>8)  & 0xFF;
    buf[4] = len & 0xFF;
    buf[5] = G2_SUB_0x13_FWWRITE;

    for(i=0; i<len-1; i++) {
        buf[6+i] = send_buffer[i];
    }
    buf[5+len] = TOKEN_ETX1;
    buf[6+len] = TOKEN_ETX2;

    len += 7;

    sended_len = 0;
    while(sended_len < len)
    {
        if(len < HID_OUTPUT_MAX_LEN - 2) send_len = len;
        else send_len = ((len - sended_len) > (HID_OUTPUT_MAX_LEN - 2))? (HID_OUTPUT_MAX_LEN - 2) : (len - sended_len);

        i2c_buf[0] = HID_OUT_REPORT_ID;
        i2c_buf[1] = index++;

        if(index == 0xAA) index++;

        for(i= 0; i < send_len; i++){
            i2c_buf[i+2] = buf[pos++];
        }
        for(i=send_len; i < (HID_OUTPUT_MAX_LEN);i++){
            i2c_buf[i+2] = 0;
        }
        sended_len += send_len;

        ret = writeData( i2c_buf, HID_OUTPUT_MAX_LEN);

        if(ret == -1)
        {
            LOG_G2_I(CLog::getLogOwner(), TAG, "FW Write error2");
            return ret;
        }
    }

    while(retry_ack >= 0)
    {
        retry_ack--;
        waitRxData(m_fd, uSecWait); //30ms
        ret = readData(buf, HID_INPUT_MAX_LEN, 0x13, 0x83 );

        if(ret > 2)
        {
            break;
        }
        usleep(1000 * 500);
    }

    return ret;
}

int DeviceIO_hid_over_i2c::File_Write_CMD(unsigned char* send_buffer, unsigned short send_length, unsigned char send_cmd, unsigned char  region)
{
    int uSecWait = GetCmdWaitAckTime(0xb4, 1000);  // 30 msec for Normal command, but some command needs more time
    unsigned char buf[512+9];
    unsigned char i2c_buf[HID_OUTPUT_MAX_LEN+10];
    int ret = 1;
    unsigned long i=0,pos=0,sended_len, send_len, len;
    int retry_ack = 5;

    memset(i2c_buf, 0x00, sizeof(i2c_buf));

    len = send_length+2;

    buf[0] = TOKEN_STX1;
    buf[1] = TOKEN_STX2;
    buf[2] = send_cmd;
    buf[3] = (len>>8)  & 0xFF;
    buf[4] = len & 0xFF;
    buf[5] = G2_SUB_0x13_FLASH_WRITE;
    buf[6] = region;

    for(i=0; i<len-2; i++) {
        buf[7+i] = send_buffer[i];
    }
    buf[5+len] = TOKEN_ETX1;
    buf[6+len] = TOKEN_ETX2;

    len += 7;

    sended_len = 0;
    while(sended_len < len)
    {
        if(len < HID_OUTPUT_MAX_LEN - 2) send_len = len;
        else send_len = ((len - sended_len) > (HID_OUTPUT_MAX_LEN - 2))? (HID_OUTPUT_MAX_LEN - 2) : (len - sended_len);

        i2c_buf[0] = HID_OUT_REPORT_ID;
        i2c_buf[1] = 0x00;

        if(index == 0xAA) index++;

        for(i= 0; i < send_len; i++){
            i2c_buf[i+2] = buf[pos++];
        }
        for(i=send_len; i < (HID_OUTPUT_MAX_LEN);i++){
            i2c_buf[i+2] = 0;
        }
        sended_len += send_len;

        ret = writeData( i2c_buf, HID_OUTPUT_MAX_LEN);

        if(ret == -1)
        {
            LOG_G2_I(CLog::getLogOwner(), TAG, "Boot Write error2");
            return ret;
        }
    }

    while(retry_ack >= 0)
    {
        retry_ack--;
        waitRxData(m_fd, uSecWait); //30ms
        ret = readData(buf, HID_INPUT_MAX_LEN, 0x13, 0x06 );

        if(ret > 2)
        {
            break;
        }
        usleep(1000 * 500);
    }

    return ret;
}


int DeviceIO_hid_over_i2c::Boot_Write_CMD(unsigned char* send_buffer, unsigned short send_length, unsigned char send_cmd)
{
    int uSecWait = GetCmdWaitAckTime(0xb4, 1000);  // 30 msec for Normal command, but some command needs more time
    unsigned char buf[512+9];
    unsigned char i2c_buf[HID_OUTPUT_MAX_LEN+10];
    int ret = 1;
    unsigned long i=0,pos=0,sended_len, send_len, len;
    int retry_ack = 5;

    memset(i2c_buf, 0x00, sizeof(i2c_buf));

    len = send_length+2;

    buf[0] = TOKEN_STX1;
    buf[1] = TOKEN_STX2;
    buf[2] = send_cmd;
    buf[3] = (len>>8)  & 0xFF;
    buf[4] = len & 0xFF;
    buf[5] = G2_SUB_0x13_FLASH_WRITE;
    buf[6] = 0x00;

    for(i=0; i<len-2; i++) {
        buf[7+i] = send_buffer[i];
    }
    buf[5+len] = TOKEN_ETX1;
    buf[6+len] = TOKEN_ETX2;

    len += 7;

    sended_len = 0;
    while(sended_len < len)
    {
        if(len < HID_OUTPUT_MAX_LEN - 2) send_len = len;
        else send_len = ((len - sended_len) > (HID_OUTPUT_MAX_LEN - 2))? (HID_OUTPUT_MAX_LEN - 2) : (len - sended_len);

        i2c_buf[0] = HID_OUT_REPORT_ID;
        i2c_buf[1] = 0x00;

        if(index == 0xAA) index++;

        for(i= 0; i < send_len; i++){
            i2c_buf[i+2] = buf[pos++];
        }
        for(i=send_len; i < (HID_OUTPUT_MAX_LEN);i++){
            i2c_buf[i+2] = 0;
        }
        sended_len += send_len;

        ret = writeData( i2c_buf, HID_OUTPUT_MAX_LEN);

        if(ret == -1)
        {
            LOG_G2_I(CLog::getLogOwner(), TAG, "Boot Write error2");
            return ret;
        }
    }

    while(retry_ack >= 0)
    {
        retry_ack--;
        waitRxData(m_fd, uSecWait); //30ms
        ret = readData(buf, HID_INPUT_MAX_LEN, 0x13, 0x06 );

        if(ret > 2)
        {
            break;
        }
        usleep(1000 * 500);
    }

    return ret;
}


int DeviceIO_hid_over_i2c::Dump(unsigned char* dump_buffer, int address, int size)
{
    LOG_G2_I(CLog::getLogOwner(), TAG, "Dump : Verifying Start " ) ;
    int uSecWait = GetCmdWaitAckTime(0x13, 1000);  // 30 msec for Normal command, but some command needs more time
    int read_retry = 5;
    int m_nReadBufCnt=0;
    int m_nReadAddr = address;
    unsigned char *m_pcReadBuf;
    unsigned char index=0x42;
    int pos=0;
    int iRet = 0;

    m_pcReadBuf = new unsigned char[size + HID_INPUT_MAX_LEN];

    iRet = FlashDump(address, size, m_nReadBufCnt, index) ;


    while(pos < size)
    {
        waitRxData(m_fd, uSecWait); //30ms
        iRet = readData(m_pcReadBuf, HID_INPUT_MAX_LEN, 0, 0); //dump

        if( read_retry == 0)
        {
            delete m_pcReadBuf;
            return false ;
        }

        if(iRet == 0)
        {
            read_retry--;
            continue;
        }

        if(pos == 0)
        {
            if(m_pcReadBuf[4] !=0x13) continue;

            int rx_addr = ((unsigned int)m_pcReadBuf[8] << 24 | (unsigned int)m_pcReadBuf[9] << 16 | (unsigned int)m_pcReadBuf[10] << 8 | (unsigned int)m_pcReadBuf[11]);

            if(m_nReadAddr != rx_addr)
            {
                delete m_pcReadBuf;
                return false;
            }
            memcpy(dump_buffer+pos, m_pcReadBuf+16, iRet-16);

            if((int)packet_length >= iRet-15)
            {
                packet_length-= (iRet-15);
            }
            else
            {
                packet_length =0;
            }

            pos+=(iRet-16);
        }
        else
        {
            memcpy(dump_buffer+pos, m_pcReadBuf+2, iRet-2);

            if((int)packet_length >= iRet)
            {
                packet_length-=iRet; //+report id
            }
            else
            {
                packet_length=0;
            }

            pos+=(iRet-2);
        }

        LOG_G2_I(CLog::getLogOwner(), TAG, "remain packet_length : %x",packet_length);

    }

    delete m_pcReadBuf;

    return true ;

}

int DeviceIO_hid_over_i2c::FlashDump(int address, int size, int m_nReadBufCnt, unsigned char index)
{
    int ret = 0;
    int idx = 0;

    if( m_nReadBufCnt >= size )
    {
        return -1;
    }

    unsigned char Buf[22] ;

    Buf[idx++] = HID_OUT_REPORT_ID;
    Buf[idx++] = index;

    Buf[idx++] =  TOKEN_STX1 ;
    Buf[idx++] =  TOKEN_STX2 ;
    Buf[idx++] =  (unsigned char)0x13 ;
    Buf[idx++] =  (unsigned char)0x00 ;
    Buf[idx++] =  (unsigned char)0x09 ;
    Buf[idx++] =  (unsigned char)0x08 ;

    Buf[idx++]=(address>>24)&0xFF;
    Buf[idx++]=(address>>16)&0xFF;
    Buf[idx++]=(address>>8)&0xFF;
    Buf[idx++]=(address)&0xFF;

    Buf[idx++]=(size>>24)&0xFF;
    Buf[idx++]=(size>>16)&0xFF;
    Buf[idx++]=(size>>8)&0xFF;
    Buf[idx++]=(size)&0xFF;

    Buf[idx++] =  (unsigned char)0x03 ;
    Buf[idx++] =  (unsigned char)0xb3 ;

    ret = writeData(Buf , 22) ;

    return ret;

}

int DeviceIO_hid_over_i2c::Patition_Request(unsigned char region)
{
    int ret = true;
    int idx = 0;

    unsigned char Buf[HID_OUTPUT_MAX_LEN] = {0,};

    Buf[idx++] = HID_OUT_REPORT_ID;
    Buf[idx++] = index;

    Buf[idx++] = TOKEN_STX1 ;
    Buf[idx++] = TOKEN_STX2 ;
    Buf[idx++] = (unsigned char)0x13 ;
    Buf[idx++] = (unsigned char)0x00 ;
    Buf[idx++] = (unsigned char)0x02 ;
    Buf[idx++] = (unsigned char)G2_SUB_0x13_PARTITION_REQUEST_INFO ;

    Buf[idx++]= region;

    Buf[idx++] = TOKEN_ETX1;
    Buf[idx++] = TOKEN_ETX2;

    ret = writeData(Buf , HID_OUTPUT_MAX_LEN) ;

    return ret;

}


bool DeviceIO_hid_over_i2c::Check_Nak(unsigned char *Rx_buf)
{
    int bRet = true;
    switch(Rx_buf[4])
    {
        case CMD_MAIN_CMD:
            switch(Rx_buf[7])
            {
                case G2_SUB_0x13_FLASH_ERASE:
                    if(Rx_buf[9] == 0x00)
                    {
                        LOG_G2_I(CLog::getLogOwner(), TAG, "G2_SUB_0x13_FLASH_ERASE: ACK");
                        return bRet;
                    }
                    else
                    {
                        LOG_G2_D(CLog::getLogOwner(), TAG, "G2_SUB_0x13_FLASH_ERASE: NAK");
                        bRet = false;
                        return bRet;
                    }
                case G2_SUB_0x13_FLASH_WRITE:
                    if(Rx_buf[9] == 0x00)
                    {
                        LOG_G2_I(CLog::getLogOwner(), TAG, "G2_SUB_0x13_FLASH_WRITE: ACK");
                        return bRet;
                    }
                    else
                    {
                        LOG_G2_D(CLog::getLogOwner(), TAG, "G2_SUB_0x13_FLASH_WRITE: NAK");
                        bRet = false;
                        return bRet;

                    }
                case G2_SUB_0x13_GOTOBOOT:
                    if(Rx_buf[8] == 0x00)
                    {
                        if((Rx_buf[17] == 0x03) && (Rx_buf[18] == 0xb3))
                        {
                            GetFWStartFullAddress = (unsigned int)(Rx_buf[9] << 24) + (unsigned int)(Rx_buf[10] << 16) + (unsigned int)(Rx_buf[11] << 8) + (unsigned int)(Rx_buf[12]);
                            GetFWStartFullErasesize = (unsigned int)(Rx_buf[13] << 24) + (unsigned int)(Rx_buf[14] << 16) + (unsigned int)(Rx_buf[15] << 8) + (unsigned int)(Rx_buf[16]);
                        }
                        LOG_G2_I(CLog::getLogOwner(), TAG, "G2_SUB_0x13_GOTOBOOT: ACK");
                        return bRet;
                    }
                    else
                    {
                        LOG_G2_D(CLog::getLogOwner(), TAG, "G2_SUB_0x13_GOTOBOOT: NAK");
                        bRet = false;
                        return bRet;
                    }
                case G2_SUB_0x13_FWDOWNREADY:
                    if(Rx_buf[8] == 0x00)
                    {
                        if((Rx_buf[17] == 0x03) && (Rx_buf[18] == 0xb3))
                        {
                            GetFWStartFullAddress = (unsigned int)(Rx_buf[9] << 24) + (unsigned int)(Rx_buf[10] << 16) + (unsigned int)(Rx_buf[11] << 8) + (unsigned int)(Rx_buf[12]);
                            GetFWStartFullErasesize = (unsigned int)(Rx_buf[13] << 24) + (unsigned int)(Rx_buf[14] << 16) + (unsigned int)(Rx_buf[15] << 8) + (unsigned int)(Rx_buf[16]);
                        }
                        LOG_G2_I(CLog::getLogOwner(), TAG, "G2_SUB_0x13_FWDOWNREADY: ACK");
                        return bRet;
                    }
                    else
                    {
                        LOG_G2_D(CLog::getLogOwner(), TAG, "G2_SUB_0x13_FWDOWNREADY: NAK");
                        bRet = false;
                        return bRet;
                    }
                    break;
                case G2_SUB_0x13_FWERASE:
                    if(Rx_buf[8] == 0x00)
                    {
                        LOG_G2_I(CLog::getLogOwner(), TAG, "G2_SUB_0x13_FWERASE: ACK");
                        return bRet;
                    }
                    else
                    {
                        LOG_G2_D(CLog::getLogOwner(), TAG, "G2_SUB_0x13_FWERASE: NAK");
                        bRet = false;
                        return bRet;
                    }
                case G2_SUB_0x13_FWWRITE:
                    if(Rx_buf[8] == 0x00)
                    {
                        LOG_G2_I(CLog::getLogOwner(), TAG, "G2_SUB_0x13_FWWRITE: ACK");
                        return bRet;
                    }
                    else
                    {
                        LOG_G2_D(CLog::getLogOwner(), TAG, "G2_SUB_0x13_FWWRITE: NAK");
                        bRet = false;
                        return bRet;
                    }
                case G2_SUB_0x13_FLASHCHKSUM:
                    if(Rx_buf[8] == 0x00)
                    {
                        LOG_G2_I(CLog::getLogOwner(), TAG, "G2_SUB_0x13_FLASHCHKSUM: ACK");
                        return bRet;
                    }
                    else
                    {
                        LOG_G2_D(CLog::getLogOwner(), TAG, "G2_SUB_0x13_FLASHCHKSUM: NAK");
                        bRet = false;
                        return bRet;
                    }
                case G2_SUB_0x13_FLASHFINISH:
                    if(Rx_buf[8] == 0x00)
                    {
                        LOG_G2_I(CLog::getLogOwner(), TAG, "G2_SUB_0x13_FLASHFINISH: ACK");
                        return bRet;
                    }
                    else
                    {
                        LOG_G2_D(CLog::getLogOwner(), TAG, "G2_SUB_0x13_FLASHFINISH: NAK");
                        bRet = false;
                        return bRet;
                    }
                 default:
                     return bRet;
            }
        case CMD_CU_ERASE:
            if(Rx_buf[7] == 0x00)
            {
                LOG_G2_I(CLog::getLogOwner(), TAG, "CMD_CU_ERASE: ACK");
                return bRet;
            }
            else
            {
                LOG_G2_D(CLog::getLogOwner(), TAG, "CMD_CU_ERASE: NAK \n");
                //LOG_G2_D(CLog::getLogOwner(), TAG, "%X %X %X %X %X %X %X %X",Rx_buf[0],Rx_buf[1],Rx_buf[2],Rx_buf[3],Rx_buf[4],Rx_buf[5],Rx_buf[6],Rx_buf[7]);
                bRet = false;
                return bRet;
            }
        case CMD_CU_WRITE:
            if(Rx_buf[7] == 0x00)
            {
                LOG_G2_I(CLog::getLogOwner(), TAG, "CMD_CU_Write: ACK");
                return bRet;
            }
            else
            {
                LOG_G2_D(CLog::getLogOwner(), TAG, "CMD_CU_Write: NAK");
                bRet = false;
                return bRet;
            }
        default:
            return bRet;
    }

    return bRet;
}

int DeviceIO_hid_over_i2c::Get_AppStartAddr_fromBinFile(unsigned char* file_buf, unsigned short idx, unsigned int* FW_Startaddr, bool GetPartition)
{
    int a = 0;
    unsigned char temp[4]={0,};    
    unsigned int addr = (unsigned int)(file_buf[3 + idx] << 24 | file_buf[2 + idx] << 16 | file_buf[1 + idx] << 8 | file_buf[0 + idx]);

        //check mcu type in file
    temp[0] = file_buf[idx];
    temp[1] = file_buf[idx + 1];
    temp[2] = file_buf[idx + 2];
    temp[3] = file_buf[idx + 3];

    LOG_G2_D(CLog::getLogOwner(), TAG, "addr 0x%x idx 0x%x\n",addr, idx);
    //LOG_G2_D(CLog::getLogOwner(), TAG, "file_buf 0x%X %x %x %x \n",file_buf[3 + idx], file_buf[2 + idx], file_buf[1 + idx], file_buf[idx]);

    if(GetPartition)
    {
        *FW_Startaddr = BaseStartaddr + GetFWStartAddress;
    }
    else
    {
        //check repeat
        for (a = 1; a < 4; a++)
        {
            if (temp[0] != file_buf[idx + (a * 4)] || temp[1] != file_buf[1 + idx + (a * 4)] || temp[2] != file_buf[2 + idx + (a * 4)] || temp[3] != file_buf[3 + idx +  (a * 4)])
            {
                LOG_G2_D(CLog::getLogOwner(), TAG, "fw address is not repeat\n");
                return -1;
            }
        }        
        *FW_Startaddr = addr;
    }

    return 1;
}


int DeviceIO_hid_over_i2c::BaseBin_start_data(unsigned char* buf, unsigned char region)
{
    int size = 0;

    buf[size++] = G2_SUB_0x13_FLASH_START;
    buf[size++] = region;
    return size;
}


int DeviceIO_hid_over_i2c::BaseBin_erase_data(unsigned char* buf, unsigned char region)
{
    int size = 0;

    buf[size++] = G2_SUB_0x13_FLASH_ERASE;
    buf[size++] = region;
    return size;
}

int DeviceIO_hid_over_i2c::Flash0x09CheckSum_data(unsigned char* buf, int file_size, unsigned int fw_checksum)
{
    int size = 0;

    buf[size++] = G2_SUB_0x13_0x09_FLASH_CHECKSUM;    
    buf[size++] = BASEBIN_REGION;
    buf[size++]=(file_size>>24)&0xFF;
    buf[size++]=(file_size>>16)&0xFF;
    buf[size++]=(file_size>>8)&0xFF;
    buf[size++]=(file_size)&0xFF;
    buf[size++]=(fw_checksum>>24)&0xFF;
    buf[size++]=(fw_checksum>>16)&0xFF;
    buf[size++]=(fw_checksum>>8)&0xFF;
    buf[size++]=(fw_checksum)&0xFF;

    return size;
}

int DeviceIO_hid_over_i2c::BaseBin_finish(unsigned char* buf, unsigned char region)
{
    int size = 0;

    buf[size++] = G2_SUB_0x13_FLASH_FINISH;
    buf[size++] = region;
    return size;
}


unsigned int DeviceIO_hid_over_i2c::FindFWFeature(unsigned char *file_buf, unsigned int buf_size)
{
    bool found = 0;
    unsigned char G2TouchVID[16] = {0x94, 0x2A, 0x00, 0x00, 0x94, 0x2A, 0x00, 0x00, 0x94, 0x2A, 0x00, 0x00, 0x94, 0x2A, 0x00, 0x00};

    // 0x10
    for (FwFeaturepos = 0; FwFeaturepos < buf_size; FwFeaturepos += 0x10) {
        if (memcmp(&file_buf[FwFeaturepos], G2TouchVID, 16) == 0) {
            found = 1;
            break;  // find"BootVer-"
        }
    }
    
    FwFeaturepos-=0x30;
    LOG_G2_D(CLog::getLogOwner(), TAG, "FwFeaturepos 0x%x \n",FwFeaturepos);

    if (found == 0) {
        FwFeaturepos = 0x200;
        return -1;   // not find
        
    }

    return FwFeaturepos;
}



int DeviceIO_hid_over_i2c::FindBootVerandPartitionTable (unsigned char *file_buf, unsigned int buf_size)
{
    unsigned int  checksum = 0;
    bool found = 0;
    unsigned char nRet = 0;
    unsigned int  a = 0;


    Partitionpos = 0x60;
    // 0x10
    for (Bootloaderpos = 0; (Bootloaderpos + 0x08) < buf_size; Bootloaderpos += 0x10) {
        if (memcmp(&file_buf[Bootloaderpos], "BootVer-", 8) == 0) {
            found = 1;
            break;  // find"BootVer-"
        }
    }
    LOG_G2_D(CLog::getLogOwner(), TAG, "Bootloaderpos 0x%x \n",Bootloaderpos);    

    if (found == 0) {
        return 0xff;   // error
    }

    if ((file_buf[Bootloaderpos + Partitionpos] == 0x00) && (file_buf[Bootloaderpos + (Partitionpos + 0x02)] == 0xEA) 
        && (file_buf[Bootloaderpos + (Partitionpos + 0x08)] == 0x01) && (file_buf[Bootloaderpos + (Partitionpos + 0x0A)] == 0xEA)) {
        // check partition head checksum
        for (a = Partitionpos; a < (Partitionpos + 0x07); a++) {
            checksum += file_buf[Bootloaderpos + a];
        }
        if (file_buf[Bootloaderpos + (Partitionpos + 0x07)] != (checksum & 0xFF)) {
            nRet = 1;
        }

        checksum = 0; // initialize
        for (a = (Partitionpos + 0x08); a < (Partitionpos + 0x0F); a++) {
            checksum += file_buf[Bootloaderpos + a];
        }
        if (file_buf[Bootloaderpos + (Partitionpos + 0x0F)] != (checksum & 0xFF)) {
            nRet = 1;
        }
    }
    else
    {
        nRet = 1;
    }

    //re-try partition position 0x50
    if(nRet == 1)
    {
        Partitionpos = 0x50;
        if ((file_buf[Bootloaderpos + Partitionpos] == 0x00) && (file_buf[Bootloaderpos + (Partitionpos + 0x02)] == 0xEA) 
            && (file_buf[Bootloaderpos + (Partitionpos + 0x08)] == 0x01) && (file_buf[Bootloaderpos + (Partitionpos + 0x0A)] == 0xEA)) {
                // check partition head checksum
                for (a = Partitionpos; a < (Partitionpos + 0x07); a++) {
                    checksum += file_buf[Bootloaderpos + a];
                }
                if (file_buf[Bootloaderpos + (Partitionpos + 0x07)] != (checksum & 0xFF)) {
                    return 0x00;
                }

                checksum = 0; // initialize
                for (a = (Partitionpos + 0x08); a < (Partitionpos + 0x0F); a++) {
                    checksum += file_buf[Bootloaderpos + a];
                }
                if (file_buf[Bootloaderpos + (Partitionpos + 0x0F)] != (checksum & 0xFF)) {
                    return 0x00;
            }
        }
        else
        {
            return 0x00;
        }        
    }

    return Bootloaderpos;
}

