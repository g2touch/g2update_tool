#include <iostream>
#include <fstream>
#include <sys/types.h>
#include <sys/stat.h>

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

bool CheckAndCreate(string folder);
void Savedebug(string msg, string log_path);

int handleParsedArgument(CArgHandler *arg);

int main(int argc, char *argv[])
{
    string currfolder = string(argv[0], 0, (strrchr(argv[0], '/') - argv[0]));
    string logfolder = currfolder + "/log";

    int nArgParseResult = -1;
    int nUpdateResult = -1;
    USING_EXIT_CODE;
    CArgHandler* argHandle = 0x0;

    CheckAndCreate(logfolder);
    Savedebug("G2UPDATER START", logfolder);

    ////////////////////////////////////////////////////////////////////
    // Input Argument Handling
    argHandle = new CArgHandler();

    Savedebug("ARGHANDLE INIT", logfolder);

    nArgParseResult = argHandle->ParseArg(argc - 1, argv + 1);
    if (handleParsedArgument(argHandle) < 0)
    {
         Savedebug("FAIL PARSER ARGUMENT", logfolder);
    	EXIT_CODE = EXIT_PASS;
    	delete argHandle;
    	RETURN_EXIT_CODE;   // exit if help required
    }

    Savedebug("SUCCESS PARSER ARGUMENT", logfolder);
    switch (nArgParseResult)
    {
        case 0:     // download parameter checks (items MUST be specified)
            if (argHandle->IsDownloadable() == false)
            {
                Savedebug("Not enough Argument", logfolder);
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

    Savedebug("SUCCESS nArgParseResult", logfolder);

    // Interface parameter handling
    if (argHandle->IsDownloadable())
    {
        if (argHandle->ResolveInterface() == false)
        {
            Savedebug("Interface Resolve Fail", logfolder);
            LOG_G2(CLog::getLogOwner(), "PARSE_ARGS", "Interface Resolve Fail: %s", argHandle->GetInterfaceResolved().c_str());
            EXIT_CODE = EXIT_IC_COMMUNICATION_ERROR;
            delete argHandle;
            RETURN_EXIT_CODE;    // exit if help required
        }
        Savedebug("argHandle->IsDownloadable()", logfolder);
    }

    Savedebug("SUCCESS Interface Resolve", logfolder);

    ///////////////////////////////////////////////////////////////////////////////////////////
    CDeviceHandler *devHandler = new CDeviceHandler(argHandle);
    devHandler->log_path = logfolder;

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
            	EXIT_CODE = EXIT_TEST_ERROR;//EXIT_FAIL;
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

void Savedebug(string msg, string log_path)
{
    string historyfname = "/var/log/debugmsg.txt";
    fstream fs;

    fs.open(historyfname.c_str(), std::fstream::out | std::fstream::app | std::fstream::ate);

    fs << msg << endl;

    fs.close();
}


bool CheckAndCreate(string folder)
{
    struct stat sb;
    bool bFound = false;
    bool isdir = stat(folder.c_str(), &sb) == 0 && S_ISDIR(sb.st_mode);
    size_t loc = 0;
    string sub;

    if (folder.length() <= 0) return false;
    while (isdir == false)
    {
        if (loc >= folder.length()) break;

        loc = folder.find("/", loc);
        if (loc == string::npos)
        {
            loc = folder.length();
        }
        if (loc == 0)
        {
            ++loc;
            continue;
        }

        sub = folder.substr(0, loc++);
        bFound = stat(sub.c_str(), &sb) == 0 && S_ISDIR(sb.st_mode);
        if (bFound)
        {
            continue;
        }
        else
        {
            mkdir(sub.c_str(), S_IRWXU|S_IRGRP|S_IXGRP|S_IROTH|S_IXOTH);
            bFound = stat(sub.c_str(), &sb) == 0 && S_ISDIR(sb.st_mode);
        }
        //if (folder.compare(sub) == 0) isdir = bFound;
        if (loc >= folder.length()) isdir = bFound;
    }

    return isdir;
}

