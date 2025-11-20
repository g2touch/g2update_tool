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
                int TxRequestHW_Reset();
                int TxRequestSystem_Reset();
                int TxRequestBootUpdate(unsigned char* file_buf, bool bBoot_force_update, bool bPartition);
                int TxRequestFwUpdate(unsigned char* file_buf, bool bPartition);
                int TxRequestCuUpdate(unsigned char* file_buf, bool bPartition);
                //int TxRequestBASEBINUpdate(unsigned char* file_buf);
                //int TxRequestLOADBALANCEUpdate(unsigned char* file_buf);
                int DumpFileInfo(unsigned char* file_buf,unsigned     int partition_address);
                bool RequestPartitionInfo(bool bFileInfo);
                int Precheckforupdate(unsigned char* file_buf, bool bBoot_force_update, bool getPartition);
                int checkFW_CUVirginCode(unsigned char* file_buf, int fwstraddr_pos, int fwvirginaddr, int custraddr, int cuvirginaddr);
                int BaseBin_start_data(unsigned char* buf, unsigned char region);
                int BaseBin_erase_data(unsigned char* buf, unsigned char region);
                int Flash0x09CheckSum_data(unsigned char* buf, int file_size, unsigned int fw_checksum);
                int BaseBin_finish(unsigned char* buf, unsigned char region);
                unsigned short MCUType_Verify(unsigned char* file_buf, int filebuf_idx, int filebuf_bootidx, bool getPartition);
                int File_Write_CMD(unsigned char* send_buffer, unsigned short send_length, unsigned char send_cmd, unsigned char  region);
                int Patition_Request(unsigned char region);
                int check_VID_PID(unsigned char* file_buf, bool getPartition);
                void SET_basestraddr(unsigned short vid_temp);
                int GoToBoot();
                int HWReset();
                unsigned int FindFWFeature(unsigned char *file_buf, unsigned int buf_size);
                int FindBootVerandPartitionTable (unsigned char *file_buf, unsigned int buf_size);                
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
                int boot_erase_data(unsigned char* buf, int erase_size);
				int FWDownReady_data(unsigned char* buf, unsigned int base_address);
				int FlashErase_data(unsigned char* buf, unsigned int base_address, unsigned int fw_erase_size);
                int FlashCheckSum_data(unsigned char* buf, unsigned int base_fwstraddress, int file_size, unsigned int fw_checksum);
                int FlashFinish_data(unsigned char* buf);
                int system_reset_data(unsigned char* buf);
                int FW_Write_CMD(unsigned char* send_buffer, unsigned short send_length, unsigned char send_cmd);
                int CU_Write_CMD(unsigned char* send_buffer, unsigned short send_length, unsigned char send_cmd, int cu_page);
                int Boot_Write_CMD(unsigned char* send_buffer, unsigned short send_length, unsigned char send_cmd);
                int Dump(unsigned char* dump_buffer, int address, int size);
                int FlashDump(int address, int size, int m_nReadBufCnt, unsigned char index);
                int FlashCheckSum_Check(unsigned char* buf, unsigned int fw_checksum);
                int FlashFinish_Check(unsigned char* read_buf);
                int Fw_write_size(unsigned char* file_buf, unsigned int erasesize);
                int Cu_Erase_data(unsigned char* file_buf, int page_cnt);
                int dumpTofile_compare(unsigned char* dump_buf, unsigned char* file_buf, int compare_size);
				bool Check_Nak(unsigned char *Rx_buf);
				int Get_AppStartAddr_fromBinFile(unsigned char* file_buf, unsigned short idx, unsigned int* FW_Startaddr, bool GetPartition);
				bool Get_Partition_info(int idx, unsigned char* m_abytContent);


            private:
                int m_fd;
                unsigned char* out_buffer;
                unsigned char* in_buffer;
                unsigned char* rxdbgbuf;
                rxUnit *tmpRxUnit;
                unsigned int index;
                unsigned int packet_length;
                unsigned int dbgidx_push;
                unsigned int dbgidx_pop;
                bool m_bOpened;
            public:                
                unsigned int GetFWStartFullAddress; //Get goto boot
                unsigned int GetFWStartFullErasesize; //Get goto boot
                unsigned int GetFWStartAddress;
                unsigned int GetFWEraseSize;
                unsigned int GetBootStartAddress;
                unsigned int GetBootEraseSize;
                unsigned int GetCUStartAddress;
                unsigned int GetCUEraseSize;
                unsigned int GetLOADBALANCEStartAddress;
                unsigned int GetLOADBALANCEEraseSize;
                unsigned int GetURStartAddress;
                unsigned int GetUREraseSize;
                unsigned int GetURDStartAddress;
                unsigned int GetURDEraseSize;
                unsigned int GetMTStartAddress; 
                unsigned int GetMTEraseSize;
                unsigned int GetFTStartAddress;
                unsigned int GetFTEraseSize;
                unsigned int GetBASEBINStartAddress;
                unsigned int GetBASEBINEraseSize;
                unsigned short MCUPID;                
                unsigned short MCUVID;
                unsigned int BaseStartaddr;
                unsigned int Protocol_Ver;
                unsigned int Bootloaderpos;
                unsigned int FwFeaturepos;
                unsigned int Partitionpos;
                unsigned int CU_size_from_FileVirgincode;
                unsigned char cu_ver;
                char Interface_Info;
                bool m_Partition_NotMatched;
        };

    }
}

#endif // DEVICEIO_HID_OVER_I2C_H
