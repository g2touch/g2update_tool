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

#define CU2_VIRGIN_CODE_POS 0x57f0
#define CU_START_POS 0x4000

CProcHandler::CProcHandler() :
	m_fnameLoaded(""),
	m_bufSize(0)
{
	memset(m_bufBinary, 0x00, sizeof(m_bufBinary));
}

CProcHandler::~CProcHandler()
{

}

bool CProcHandler::LoadBinary(CArgHandler *devHandler)
{
    string binfile = devHandler->GetBinFilePath();
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

    if (!((m_bufSize == _128K) || (m_bufSize == _256K) || (m_bufSize == _512K)))
    {
    	LOG_G2_E(CLog::getLogOwner(), TAG_BINLOADER, "binfile size should be 128K or 256K or 512K but, : 0x%X", m_bufSize);
        return 0;
    }    

    fseek(file, 0, SEEK_SET);
    memset(m_bufBinary, 0, m_bufSize * sizeof(unsigned char));

    int nCnt = fread(m_bufBinary, sizeof(unsigned char), m_bufSize, file);

    if (nCnt != m_bufSize)	// did not read whole file
    {
    	LOG_G2_E(CLog::getLogOwner(), TAG_BINLOADER, "bytes loaded from %s : %d", binfile.c_str(), nCnt);
        return 0;
    }    
    else
    {
    	m_fnameLoaded = binfile;
    	LOG_G2_I(CLog::getLogOwner(), TAG_BINLOADER, "Loaded successfully (%s)", m_fnameLoaded.c_str());
    }

    return nCnt == m_bufSize;
}

int CProcHandler::DoUpdate(CDeviceHandler *devHandler)
{
    LOG_G2_I(CLog::getLogOwner(), TAG, "Start Update FW : %s", m_fnameLoaded.c_str());

    int nRet = devHandler->G2Update(m_bufBinary, m_bufSize);

    return nRet;
}

bool CProcHandler::ChkFwVer(CDeviceHandler *devHandler)
{
    // 0: decimal, 1: Hex
	int nRet = devHandler->CheckFirmwareVersion(devHandler->m_bVerHex);
	LOG_G2_D(CLog::getLogOwner(), TAG, "Check FW Version Result : %d, %d", nRet, devHandler->m_bVerHex);

	return nRet;
}

