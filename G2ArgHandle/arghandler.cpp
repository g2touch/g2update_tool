#include <string.h>
#include "arghandler.h"
#include "logmanager.h"
#include "shellcommand.h"

using namespace std;
using namespace ARG;
using namespace G2;

#define TAG "ArgHandler"

#define CHAR_DELIMETER			"="
#define CHAR_KEY_PRECEDENCE		'-'

#define KEY_DEF_BL_FORCE		"-bf"
#define KEY_DEF_DEBUG			"-debug"
#define KEY_DEF_LOG				"-log"
#define KEY_DEF_INTERFACE		"-n"
#define KEY_DEF_HELP			"-h"
#define KEY_DEF_VERHEX          "-vhex"
#define KEY_DEF_FILE            "-file"

CArgHandler::CArgHandler() :
	m_bOptionBootForce(false),
	m_sInterface(""),
	m_sInterfaceResolved(""),
	m_sWholeParam(""),
	m_bOptionDebug(false),
	m_bOptionHelp(false),
	m_bOptionVerHex(false)
{

}

CArgHandler::~CArgHandler()
{

}

int CArgHandler::ParseArg(int argc, char *argv[])
{
    int nRet = 0;
    string FullArg;
    char *token;

    if (argc < 1)   // no params
    {
        nRet = -2;
        return nRet;
    }

    FullArg = string(argv[0]);  // set first item
    for (int i = 0; i < argc; ++i)
    {
        // TODO: duplicate argument check (how to ?)
        if (i != 0) FullArg = (FullArg.append(" ")).append(string(argv[i]));    // skip first item

        token = strtok(argv[i], CHAR_DELIMETER);    // check 1st token
        if (token[0] == CHAR_KEY_PRECEDENCE)        // strats with '-'
        {
            if (strcmp(token, KEY_DEF_INTERFACE) == 0)
            {
                m_sInterface = strtok(NULL, CHAR_DELIMETER);    // set 2nd token as value
            }
            else if (strcmp(token, KEY_DEF_BL_FORCE) == 0)
            {
                m_bOptionBootForce = true;
            }
            else if (strcmp(token, KEY_DEF_DEBUG) == 0)
            {
                m_bOptionDebug = true;
            }
            else if (strcmp(token, KEY_DEF_LOG) == 0)
            {
                m_sLogFile = (strtok(NULL, CHAR_DELIMETER));
            }
            else if (strcmp(token, KEY_DEF_HELP) == 0)
            {
                m_bOptionHelp = true;
            }
            else if (strcmp(token, KEY_DEF_FILE) == 0)
            {
                m_sBinFilePath = strtok(NULL, CHAR_DELIMETER);  // set 2nd token as value
            }
            else if (strcmp(token, KEY_DEF_VERHEX) == 0)
            {
                m_bOptionVerHex = true;
            }
            else
            {
                nRet = -3;  // unknown param
            }
        }
        else
        {
            LOG_G2_I(CLog::getLogOwner(), TAG, "token : %s",  token);
            m_sBinFilePath = string(token);
            LOG_G2_I(CLog::getLogOwner(), TAG, "m_sBinFilePath : %s",  m_sBinFilePath.c_str());
        }
    }

    if (nRet == 0) m_sWholeParam = FullArg;    // trim last SPACE character

    return nRet;
}

bool CArgHandler::IsDownloadable()
{	// does not check file existance. just check argument existance for interface & filepath
    return ((m_sInterface.npos > 0) && (m_sBinFilePath.npos > 0));
}

void CArgHandler::showHelp()
{
    LOG_G2(CLog::getLogOwner(), "H", "force update Bootloader : %s", KEY_DEF_BL_FORCE);
    LOG_G2(CLog::getLogOwner(), "H", "enable debug log : %s", KEY_DEF_DEBUG);
    LOG_G2(CLog::getLogOwner(), "H", "Show Hex format Version string : %s", KEY_DEF_VERHEX);
    LOG_G2(CLog::getLogOwner(), "H", "open a G2touch device with indicated device ID : %s=<OPTION>", KEY_DEF_INTERFACE);
}

bool CArgHandler::ResolveInterface()
{
    string deviceID;

    /* convert hidraw* to device path */
    string hidrawStr = "hidraw";
    string hidrawNum = m_sInterface;
    if (hidrawNum.substr(0, hidrawStr.size()) == hidrawStr) // if startswith "hidraw"
    {
    	/* find ls_sys_class_hidraw_hidrawN */
    	string cmd_ls_sys_class_hidraw_hidrawN = "ls -l /sys/class/hidraw/"; // " ls -l /sys/class/hidraw/hidraw* "
    	cmd_ls_sys_class_hidraw_hidrawN.append(hidrawNum);
    	LOG_G2_D(CLog::getLogOwner(), TAG, "cmd : %s", cmd_ls_sys_class_hidraw_hidrawN.c_str());
    	std::string ls_sys_class_hidraw_hidrawN = ShellCommand::exec(cmd_ls_sys_class_hidraw_hidrawN.c_str());
    	LOG_G2_D(CLog::getLogOwner(), TAG, "output : %s", ls_sys_class_hidraw_hidrawN.c_str());

    	/* parse to deviceID */
    	string keywordStr1 = "-> ../../";
    	string keywordStr2 = "/hidraw/hidraw";
    	if (ls_sys_class_hidraw_hidrawN.find(keywordStr1) != string::npos &&
    		ls_sys_class_hidraw_hidrawN.find(keywordStr2) != string::npos)
    	{
    		string tmpDeviceID = ls_sys_class_hidraw_hidrawN;
    		tmpDeviceID = tmpDeviceID.substr(tmpDeviceID.find(keywordStr1) + keywordStr1.size(), tmpDeviceID.size());
    		tmpDeviceID = tmpDeviceID.substr(0, tmpDeviceID.find(keywordStr2));
    		tmpDeviceID = tmpDeviceID.substr(0, tmpDeviceID.find_last_of("/")); // remove device (0000:0000:0000.0000)

    		deviceID = "/sys/";
    		deviceID.append(tmpDeviceID);
    	}
    }
    else
    {
         deviceID = m_sInterface;
    }

    m_sInterfaceResolved = deviceID;
    LOG_G2_I(CLog::getLogOwner(), TAG, "set deviceID : \"%s\"", m_sInterfaceResolved.c_str());

    return true;
}

string CArgHandler::GetWholeParam()
{
	return m_sWholeParam;
}

string CArgHandler::GetInterfaceResolved()
{
	return m_sInterfaceResolved;
}

string CArgHandler::GetBinFilePath()
{
    return m_sBinFilePath;
}

bool CArgHandler::HasOptionDebug()
{
	return m_bOptionDebug;
}

bool CArgHandler::HasOptionHelp()
{
	return m_bOptionHelp;
}

bool CArgHandler::HasOptionBootForce()
{
	return m_bOptionBootForce;
}

bool CArgHandler::HasOptionVerHex()
{
	return m_bOptionVerHex;
}

