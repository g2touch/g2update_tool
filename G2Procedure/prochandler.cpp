#include "arghandler.h"
#include "logmanager.h"
#include "prochandler.h"
#ifdef USE_EXCEPTION
#include "procexception.h"
#endif
#include "gendevicehandle.h"

#include <string.h>

using namespace std;
using namespace ARG;
using namespace G2;
using namespace G2::PROC;
using namespace G2::DeviceIO;

#define EXCEPTION_TITLE "BinLoader Exception : "
#define TAG_BINLOADER "proc_binloader"
#define TAG "prochandler"

#define CU_START_POS 0x4000

CProcHandler::CProcHandler() :
	m_fnameLoaded(""),
	m_bufSize(0),
	m_cnfirmFile(0)
{
	memset(m_bufBinary, 0x00, sizeof(m_bufBinary));
}

CProcHandler::~CProcHandler()
{

}

bool CProcHandler::LoadBinary(CArgHandler *devHandler)
{
    string binfile = devHandler->GetBinFilePath();
    string keywordCU = ".cu2";

    LOG_G2_I(CLog::getLogOwner(), TAG_BINLOADER, "opening file: \"%s\"", binfile.c_str());
    m_fnameLoaded = "";		// reset

    FILE* file = fopen(binfile.c_str(), "r");

    if (!file)
    {
        LOG_G2_E(CLog::getLogOwner(), TAG_BINLOADER, "file not found %s", binfile.c_str());
        return 0;
    }

    fseek(file, 0, SEEK_END);
    int fileSize = ftell(file);
    m_bufSize = fileSize;
    fseek(file, 0, SEEK_SET);

    memset(m_bufBinary, 0, m_bufSize * sizeof(unsigned char));

    int nCnt = fread(m_bufBinary, sizeof(unsigned char), m_bufSize, file);

    if(binfile.find(keywordCU) != string::npos)
    {
        m_fnameLoaded = binfile;
        m_cnfirmFile = UPDATE_CU;
        LOG_G2_I(CLog::getLogOwner(), TAG_BINLOADER, "Loaded successfully (%s)", m_fnameLoaded.c_str());
    }
    else if(CheckFWBinary(m_bufBinary))
    {
        m_fnameLoaded = binfile;
        m_cnfirmFile = UPDATE_FW;
        LOG_G2_I(CLog::getLogOwner(), TAG_BINLOADER, "Loaded successfully (%s)", m_fnameLoaded.c_str());
    }
    else
    {

        if (nCnt != m_bufSize)
        {
        	LOG_G2_E(CLog::getLogOwner(), TAG_BINLOADER, "bytes loaded from %s : %d", binfile.c_str(), nCnt);
            return 0;
        }
        else if(m_bufSize != _128K)
        {
        	LOG_G2_E(CLog::getLogOwner(), TAG_BINLOADER, "binfile size should be 128K(0x20000) but, : 0x%X", m_bufSize);
            return 0;
        }
        else if(!CheckBinary(m_bufBinary))
        {
        	LOG_G2_E(CLog::getLogOwner(), TAG_BINLOADER, "binfile corrupted : %s", binfile.c_str());
            return 0;
        }
        else
        {
            m_fnameLoaded = binfile;
            LOG_G2_I(CLog::getLogOwner(), TAG_BINLOADER, "Loaded successfully (%s)", m_fnameLoaded.c_str());
        }
    }

    return nCnt == m_bufSize;
}

int CProcHandler::DoUpdate(CDeviceHandler *devHandler)
{
    int nRet = 0;

    LOG_G2_D(CLog::getLogOwner(), TAG, "Start Update FW : %s, UpdateSel=%d", m_fnameLoaded.c_str(), devHandler->m_selUpdate);

    switch(devHandler->m_selUpdate)
    {
        case UPDATE_ALL:
        case UPDATE_CUFW:
            if(m_cnfirmFile == UPDATE_CU || m_cnfirmFile == UPDATE_FW)
                return -1;
            nRet = devHandler->G2Update(m_bufBinary);
            break;
        case UPDATE_CU:
            if(m_cnfirmFile != UPDATE_CU)
                return -1;
            nRet = devHandler->G2UpdateCU(m_bufBinary);
            break;
        case UPDATE_FW:
            if(m_cnfirmFile != UPDATE_FW)
                return -1;
            nRet = devHandler->G2UpdateFW(m_bufBinary, m_bufSize);
            break;
        default:
            nRet = -1;
            break;
    }

    return nRet;
}

bool CProcHandler::ChkFwVer(CDeviceHandler *devHandler)
{
    // 0: decimal, 1: Hex
	int nRet = devHandler->CheckFirmwareVersion(devHandler->m_bVerHex);
	LOG_G2_D(CLog::getLogOwner(), TAG, "Check FW Version Result : %d, %d", nRet, devHandler->m_bVerHex);

	return nRet;
}

bool CProcHandler::CheckFWBinary(unsigned char* m_bufBinary)
{
    int i=0;

    for(i=0; i<13; i+=4)
    {
        if((m_bufBinary[0x200+i] == 0x00) && (m_bufBinary[0x200+i+1] == 0x80) && (m_bufBinary[0x200+i+2] == 0x00) && (m_bufBinary[0x200+i+3] == 0x08))
        {

        }
        else
        {
            LOG_G2_E(CLog::getLogOwner(), TAG, "binfile fw address not match !!!!!!!!");
            return false;
        }
    }

    return true;
}

bool CProcHandler::CheckBinary(unsigned char* m_bufBinary)
{
    int cu_checksum=0, cu_checksum_cmp = 0;
    int i=0;

    for(i=0; i<13; i+=4)
    {
        if((m_bufBinary[0x8200+i] == 0x00) && (m_bufBinary[0x8200+i+1] == 0x80) && (m_bufBinary[0x8200+i+2] == 0x00) && (m_bufBinary[0x8200+i+3] == 0x08))
        {

        }
        else
        {
            LOG_G2_E(CLog::getLogOwner(), TAG, "binfile fw address not match !!!!!!!!");
            return false;
        }
    }

    for(i=0; i<4; i++)
    {
        if(m_bufBinary[0x1fff0+i] != 0xa5)
        {
            LOG_G2_E(CLog::getLogOwner(), TAG, "binfile fw magic error !!!!!!!!");
            return false;
        }
    }

    for(i=0; i<8; i++)
    {
        if(m_bufBinary[0x57f0+i] == 0xff)
        {
            LOG_G2_E(CLog::getLogOwner(), TAG, "binfile cu magic error !!!!!!!!");
            return false;
        }
    }

    cu_checksum = (m_bufBinary[0x57f0 + 7] <<24);
    cu_checksum+= (m_bufBinary[0x57f0 + 6] <<16);
    cu_checksum+= (m_bufBinary[0x57f0 + 5] <<8);
    cu_checksum+= m_bufBinary[0x57f0 + 4];

    for(int i=CU_START_POS; i<0x4b00; i++)
        cu_checksum_cmp += m_bufBinary[i];

    if(cu_checksum != cu_checksum_cmp)
    {
        LOG_G2_E(CLog::getLogOwner(), TAG, "binfile cu checksum error !!!!!!!!");
        return false;
    }

    return true;

}

