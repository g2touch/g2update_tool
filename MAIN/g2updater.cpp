#include <iostream>

#include "basedefine.h"
#include "exitcode.h"
#include "logmanager.h"
#include "exception.h"
#ifdef USE_EXCEPTION
#include "procexception.h"
#endif
#include "gendevicehandle.h"
#include "prochandler.h"

using namespace std;
using namespace ARG;
using namespace G2;
using namespace G2::DeviceIO;
using namespace PROC;

int handleParsedArgument(CArgHandler *arg);

int main(int argc, char *argv[])
{
    int nArgParseResult = -1;
    int nUpdateResult = -1;
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
    	RETURN_EXIT_CODE;   // exit if help required
    }
    switch (nArgParseResult)
    {
        case 0:     // download parameter checks (items MUST be specified)
            if (argHandle->IsDownloadable() == false)
            {
            	nArgParseResult = -11;
            	LOG_G2_E(CLog::getLogOwner(), "PARSE_ARGS", "Not enough Argument: %s", argHandle->GetWholeParam().c_str());
            }
            break;
        default:    // general parameter err
            nArgParseResult = -10;
            LOG_G2_E(CLog::getLogOwner(), "PARSE_ARGS", "PraseErrorInformation: %s", argHandle->GetWholeParam().c_str());
            EXIT_CODE = EXIT_FLOW_ERROR;
            return EXIT_CODE;
    }

    printf("g2updater-%s\n", APP_VERSION);

    // Interface parameter handling
    if (argHandle->IsDownloadable())
    {
        if (argHandle->ResolveInterface() == false)
        {
            LOG_G2(CLog::getLogOwner(), "PARSE_ARGS", "Interface Resolve Fail: %s", argHandle->GetInterfaceResolved().c_str());
            EXIT_CODE = EXIT_IC_COMMUNICATION_ERROR;
            delete argHandle;
            RETURN_EXIT_CODE;    // exit if help required
        }
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    CDeviceHandler *devHandler = new CDeviceHandler(argHandle);

    // Prepare Update. Open & Get Hid Info (VID will be Checked at here)
    devHandler->openDevice();

    if (devHandler->IsDeviceOpened() && argHandle->GetBinFilePath().npos > 0)
    {
        CProcHandler *ProcHandler = new CProcHandler();

        if (ProcHandler->LoadBinary(argHandle))
        {
            devHandler->m_bBootUpdateforce = argHandle->HasOptionBootForce();
            devHandler->m_bVerHex = argHandle->HasOptionVerHex();

            nUpdateResult = ProcHandler->DoUpdate(devHandler);

            if (nUpdateResult <= 0)
            {
            	EXIT_CODE = EXIT_FAIL;
            }
            else
            {
            	EXIT_CODE = EXIT_PASS;
            }
        }
        else
        {
            LOG_G2(CLog::getLogOwner(), "PROC_UPDATE", "Load Binary Failed : %s", argHandle->GetBinFilePath().c_str());
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
        arg->showHelp();
        nRet = -1;
    }

    return nRet;
}

