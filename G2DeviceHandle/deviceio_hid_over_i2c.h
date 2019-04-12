#ifndef DEVICEIO_HID_OVER_I2C_H
#define DEVICEIO_HID_OVER_I2C_H

#include "stdio.h"
#include "string"
#include "arghandler.h"
#include "packet.h"
#include <list>

#define RXDBGBUFSIZE (4*1024)

using namespace std;
using namespace ARG;

namespace G2
{
    namespace DeviceIO
    {
        class DeviceIO_hid_over_i2c
        {
            public:
                DeviceIO_hid_over_i2c(CArgHandler *argHandle);
                ~DeviceIO_hid_over_i2c();

                int openDevice(string hidRawName);
                int closeDevice();

                int writeData(unsigned char * buf, int size, int timeoutMsec);
                int readData( unsigned char * buf, int size, int Maincommend, int Subcommend);
                int ReadDataPush( unsigned char * s_buf, int size);
                int ParsePushedData( unsigned char * s_buf, int size, int Maincommend, int Subcommend, int chk_ack);

                bool isUsingIOCTL();

                bool isDeviceOpened();
                string TxRequestFwVer(int mSec, int format);
                int TxRequestHW_Reset(bool chkack, int trial);
                int TxRequestSystem_Reset();
                int TxRequestBootUpdate(unsigned char* file_buf, bool bBoot_force_update);
                int TxRequestFwUpdate(unsigned char* file_buf);
                int TxRequestCuUpdate(unsigned char* file_buf);
                int ReadDataAll(int duration);
                int hid_Type;
                unsigned char HID_OUTPUT_MAX_LEN;
                unsigned char HID_OUT_REPORT_ID;
                int TxReqOnlyCuUpdate(unsigned char* file_buf);
                int TxReqOnlyFwUpdate(unsigned char* file_buf, int size);

            protected:
                void initBuffer();

            private:
#ifdef USE_EXCEPTION
                void throwNotSupportException(std::string functionName);
#endif
                DeviceIO_hid_over_i2c(const G2::DeviceIO::DeviceIO_hid_over_i2c&); // copy constructor
                DeviceIO_hid_over_i2c & operator = (const G2::DeviceIO::DeviceIO_hid_over_i2c&); // override
                int waitRxData(int &fd, int uSec);
                int GetCmdWaitAckTime(unsigned char cmd, int mSec);
                int CreateCmdBuffer(unsigned char cmd, unsigned char* data, int data_len);
                int TxSingleCmdWaitAck(unsigned char cmd, unsigned char* data, int data_len, int uSecWait);
                int TryWriteData(unsigned char cmd, unsigned char* data, int data_len, int trial, int mSecWait);
                int GotoBoot_data(unsigned char* buf);
                int boot_erase_data(unsigned char* buf);
                int FWDownReady_data(unsigned char* buf);
                int FlashErase_data(unsigned char* buf);
                int FlashCheckSum_data(unsigned char* buf, int file_size, unsigned int fw_checksum);
                int FlashFinish_data(unsigned char* buf);
                int system_reset_data(unsigned char* buf);
                int FW_Write_CMD(unsigned char* send_buffer, unsigned short send_length, unsigned char send_cmd);
                int CU_Write_CMD(unsigned char* send_buffer, unsigned short send_length, unsigned char send_cmd, int cu_page);
                int Boot_Write_CMD(unsigned char* send_buffer, unsigned short send_length, unsigned char send_cmd);
                int Dump(unsigned char* out_buf, int address, unsigned int size, int ntime);
                int FlashDump(int address, int size);
                int FlashCheckSum_Check(unsigned char* buf, unsigned int fw_checksum);
                int FlashFinish_Check(unsigned char* read_buf);
                int Fw_write_size(unsigned char* file_buf);
                int GetFW_size(unsigned char* file_buf, int size);
                int Cu_Erase_data(unsigned char* file_buf);
                int dumpTofile_compare(unsigned char* dump_buf, unsigned char* file_buf, int compare_size);
                void CUToBin(unsigned char* src, unsigned char* dest);
                void Pos_init(bool init_idx);

            private:
                int Targetbootversion(unsigned char* dump_buffer);
                void CheckTargetType(unsigned char* dump_buffer);
                void CheckBootFileType(unsigned char* file_buf);
                void CheckFWFileType(unsigned char* file_buf);
                int CompareTargetToFile();

                int m_fd;
                unsigned char* out_buffer;
                unsigned char* in_buffer;
                unsigned char* rxdbgbuf;
                rxUnit *tmpRxUnit;
                unsigned int index;
                unsigned int m_packetlength;
                unsigned int dbgidx_push;
                unsigned int dbgidx_pop;
                bool m_bOpened;
                unsigned int read_pos;
                unsigned int buf_pos;
                string target_type;
                string file_type;
        };

    }
}

#endif // DEVICEIO_HID_OVER_I2C_H
