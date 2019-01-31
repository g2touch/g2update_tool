#include <iostream>

#include "basedefine.h"
#include "exitcode.h"
#include "logmanager.h"
#include "exception.h"
#include "procexception.h"
#include "gendevicehandle.h"
#include "prochandler.h"

using namespace std;
using namespace ARG;
using namespace G2;
using namespace G2::DeviceIO;
using namespace PROC;

#define KEY_DEF_INTERFACE		"-n"

// prototype
int handleParsedArgument(CArgHandler *arg);

int main(int argc, char *argv[])
{
    int nArgParseResult = -1;
    int nResult = -1;
    USING_EXIT_CODE;
    CArgHandler* argHandle = 0x0;

    ////////////////////////////////////////////////////////////////////
    // Input Argument Handling
    argHandle = new CArgHandler();

    nArgParseResult = argHandle->ParseArg(argc - 1, argv + 1);

    if (handleParsedArgument(argHandle) < 0)
    {
    	EXIT_CODE = EXIT_PASS;
    	delete argHandle;
    	RETURN_EXIT_CODE;    // exit if help required
    }
    switch (nArgParseResult)
    {
        case 0:     // no general err
            if (argHandle->ResolveInterface() == false)
            {
                	LOG_G2(CLog::getLogOwner(), "PARSE_ARGS", "Interface Resolve Fail: %s", argHandle->GetInterfaceResolved().c_str());
                	EXIT_CODE = EXIT_FLOW_ERROR;
                	delete argHandle;
                	RETURN_EXIT_CODE; // exit if help required
            }
            break;
        default:    // general parameter err
            nArgParseResult = -10;
            LOG_G2(CLog::getLogOwner(), "PARSE_ARGS", "PraseErrorInformation: %s", argHandle->GetWholeParam().c_str());
            break;
    }

    printf("g2fwcheck-%s\n", APP_VERSION);


    ///////////////////////////////////////////////////////////////////////////////////////////
    CDeviceHandler *devHandler = new CDeviceHandler(argHandle);

    // Prepare Update. Open & Get Hid Info (VID will be Checked at here)
    devHandler->openDevice();

    if (devHandler->IsDeviceOpened())
    {
    	CProcHandler *ProcHandler = new CProcHandler();
        devHandler->m_bVerHex = argHandle->HasOptionVerHex();

    	nResult = ProcHandler->ChkFwVer(devHandler);
        if (nResult == 0)
        {
            LOG_G2(CLog::getLogOwner(), "PROC_CHKFWVER", "Failed with code: %d", nResult);
            EXIT_CODE = EXIT_FAIL;
        }
        else
        {
            EXIT_CODE = EXIT_PASS;
        }

    	delete ProcHandler;
    }

    delete devHandler;
    delete argHandle;

    RETURN_EXIT_CODE;
}

int handleParsedArgument(CArgHandler *arg)
{
    int nRet = 0;

    if (arg->HasOptionDebug())
    {
        CLog::getLogOwner()->setDebugVisible(true);
        CLog::getLogOwner()->setLogFormat(CLog::FORMATE_DETAIL);
    }
    if (arg->HasOptionHelp())
    {
        LOG_G2(CLog::getLogOwner(), "H", "open a G2touch device with indicated device ID : %s=<OPTION>", KEY_DEF_INTERFACE);
        nRet = -1;
    }

    return nRet;
}
