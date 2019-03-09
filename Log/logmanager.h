#ifndef LOGMANAGER_H
#define LOGMANAGER_H

#include <stdio.h>
#include <string>
#include <map>
#include "G2logdllexportdefine.h"

/* Log Type */
#define LOG_TYPE_NONE     0x1
#define LOG_TYPE_LOGCAT   0x2
#define LOG_TYPE_FILE     0x4

/* Level */
#define LEVEL_DEBUG     "D"
#define LEVEL_INFO      "I"
#define LEVEL_WARNING   "W"
#define LEVEL_ERROR     "E"

/* log instance owner */
#define LOG_OWNER_NAME  "GTCH"

/* symbol*/
#define MESSAGE_BEGIN ":~$ "

namespace G2
{
    class LOG_EXPORT CLog
    {
        public:
            enum LogLabel
            {
                LABEL_TIMESTAMP = 0,
                LABEL_THREAD_ID,
                LABEL_LEVEL,
                LABEL_TAG,
                LABEL_FUNCTION,
                LABEL_FILE,
                LABEL_LINE,
                LABEL_OWNER,
                LABEL_COUNT, // label count, also express none
            };

            enum LogFormat
            {
                FORMATE_SIMPLE = 0,
                FORMATE_DETAIL,
            };

            void releaseLogFile();

            static CLog* getLogOwner();

            static CLog* getInstance(std::string owner);
            static void freeInstance(CLog* G2Log);

            void setLogFormat(LogFormat logFormat);

            void setLogFormat(LogLabel o0 = LABEL_COUNT,
                              LogLabel o1 = LABEL_COUNT,
                              LogLabel o2 = LABEL_COUNT,
                              LogLabel o3 = LABEL_COUNT,
                              LogLabel o4 = LABEL_COUNT,
                              LogLabel o5 = LABEL_COUNT,
                              LogLabel o6 = LABEL_COUNT,
                              LogLabel o7 = LABEL_COUNT);

            void setLogType(int logType);
            void setLogFile(const char* logFile);

            void setDebugVisible(bool debugVisible);
            void setInfoVisible(bool infoVisible);
            void setWarningVisible(bool warningVisible);
            void setErrorVisible(bool errorVisible);

            bool isLOG_D();
            bool isLOG_I();
            bool isLOG_W();
            bool isLOG_E();
            bool isLOG_G2();

            void printLogFormat();
            void printLog(const char* level, const char* tag, const char* function, const char* file, const int line, const char* message, ...);
            void printG2Log(const char* tag, const char* function, const char* file, const int line, const char* message, ...);

        private:
            CLog(std::string owner);
            ~CLog();

            CLog(const CLog&);
            CLog& operator=(const CLog&);
            static std::map<std::string, CLog*> & createInstanceMap();
            void init();
            const std::string getLogLabelStr(LogLabel logLabel);
            const std::string getCurrentTimeString();
            const std::string getCurrentThreadIDString();

            void appendLabelTimeStamp(char* fmt);
            void appendLabelThreadID(char* fmt);
            void appendLabelLevel(char* fmt, const char* level);
            void appendLabelTag(char* fmt, const char* tag);
            void appendLabelFunction(char* fmt, const char* function);
            void appendLabelFile(char* fmt, const char* file);
            void appendLabelLine(char* fmt, const int line);
            void appendLabelOwner(char* fmt);

        private:
            static std::map<std::string, CLog*> *m_pInstances; /* <owner, G2Log> */
            static G2::CLog* m_LogOwner;
            std::string m_owner;

            int m_logType;
            FILE* m_logFile;

            bool m_debugVisible;
            bool m_infoVisible;
            bool m_warningVisible;
            bool m_errorVisible;
            bool m_G2Visible;

            LogLabel m_logLabelOrder[LABEL_COUNT];
    };
} // G2

#define LOG_G2(LogOwner, tag, message, ...) if(LogOwner != 0 && LogOwner->isLOG_G2()) \
    LogOwner->printG2Log( tag, __FUNCTION__, __FILE__, __LINE__, message, ##__VA_ARGS__)

#define LOG_G2_D(LogOwner, tag, message, ...) if(LogOwner != 0 && LogOwner->isLOG_D()) \
    LogOwner->printLog(LEVEL_DEBUG,   tag, __FUNCTION__, __FILE__, __LINE__, message, ##__VA_ARGS__)

#define LOG_G2_I(LogOwner, tag, message, ...) if(LogOwner != 0 && LogOwner->isLOG_I()) \
    LogOwner->printLog(LEVEL_INFO,    tag, __FUNCTION__, __FILE__, __LINE__, message, ##__VA_ARGS__)

#define LOG_G2_W(LogOwner, tag, message, ...) if(LogOwner != 0 && LogOwner->isLOG_W()) \
    LogOwner->printLog(LEVEL_WARNING, tag, __FUNCTION__, __FILE__, __LINE__, message, ##__VA_ARGS__)

#define LOG_G2_E(LogOwner, tag, message, ...) if(LogOwner != 0 && LogOwner->isLOG_E()) \
    LogOwner->printLog(LEVEL_ERROR,   tag, __FUNCTION__, __FILE__, __LINE__, message, ##__VA_ARGS__)

#endif // LOGMANAGER_H
