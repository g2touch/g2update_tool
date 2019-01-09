#ifndef ARGHANDLER_H
#define ARGHANDLER_H

#include <string>

using namespace std;

namespace ARG
{

    class CArgHandler
    {
    public:
    	CArgHandler();
    	~CArgHandler();

    	int ParseArg(int argc, char *argv[]);
    	bool IsDownloadable();
    	void showHelp();
    	bool ResolveInterface();

    	string GetWholeParam();
    	string GetInterfaceResolved();
    	string GetBinFilePath();
    	bool HasOptionDebug();
    	bool HasOptionHelp();
    	bool HasOptionBootForce();
    	bool HasOptionVerHex();

    private:
    	string m_sWholeParam;
    	string m_sInterface;        // -n=[]
    	string m_sInterfaceResolved;
    	string m_sBinFilePath;
    	bool m_bOptionHelp;         // -h
    	bool m_bOptionDebug;        // -debug
    	string m_sLogFile;          // -log
    	bool m_bOptionBootForce;    // -bf
        bool m_bOptionVerHex;       // -vhex
    };

}
#endif
