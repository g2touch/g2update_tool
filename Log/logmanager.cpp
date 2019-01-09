#include "logmanager.h"

#include <stdarg.h>
#include <string.h>
#include <time.h>

#ifdef WIN32
#include "Windows.h"
#else
#include <sys/time.h>
#endif

//#define __DEBUG

using namespace G2;

#define TAG "CLog"

std::map<std::string, CLog*> * G2::CLog::m_pInstances = 0;
G2::CLog* G2::CLog::m_LogOwner = 0;

#define FORMAT_MAX_LENGTH 1024

CLog::CLog(std::string owner) :
    m_owner(owner),
    m_logType(LOG_TYPE_LOGCAT),
    m_logFile(0),
#if defined(__DEBUG)
    m_debugVisible(false),
    m_infoVisible(false),
    m_warningVisible(false),
    m_errorVisible(true),
    m_G2Visible(true)
#else
	m_debugVisible(false),
	m_infoVisible(false),
	m_warningVisible(false),
	m_errorVisible(true),
	m_G2Visible(true)
#endif
{
    init();
}

CLog::~CLog()
{
    // don't delete somethings is non-static
}

void
CLog::releaseLogFile()
{
    if( m_logFile != 0 )
    {
        LOG_G2_D( this, TAG, "release log file : (%d)", m_logFile );

        fclose(m_logFile);
        m_logFile = 0;
    }
}

std::map<std::string, CLog*> &
CLog::createInstanceMap()
{
    static std::map<std::string, CLog*> instanceMap;
    return instanceMap;
}

G2::CLog*
CLog::getLogOwner()
{
    return G2::CLog::m_LogOwner == 0 ? G2::CLog::getInstance(LOG_OWNER_NAME) : G2::CLog::m_LogOwner;
}

CLog*
CLog::getInstance(std::string owner)
{
#ifdef WIN32
    // to do
#else
    //std::lock_guard<std::mutex> guard(g_mutex_log);
#endif

    if( m_pInstances == 0 )
    {
        m_pInstances = &createInstanceMap();
    }

    CLog* pInstance = 0;

    /* find if has instanced */
    std::map<std::string, CLog*>::iterator it = m_pInstances->find( owner );
    if ( it != m_pInstances->end() )
    {
        pInstance = it->second; // has instanced
    }
    else
    {
        pInstance = new CLog(owner); // has not instanced, new
        m_pInstances->insert( std::pair<std::string, CLog*>(owner, pInstance) );

        /* cache m_LogOwner */
        if( owner.compare(LOG_OWNER_NAME) == 0 )
        {
			G2::CLog::m_LogOwner = pInstance;
        }
    }

    return pInstance;
}

void
CLog::freeInstance(CLog* Log)
{
    std::string owner;

    /* find owner if exist */
    for(std::map<std::string, CLog*>::iterator it = m_pInstances->begin(); it != m_pInstances->end(); ++it)
    {
        if( Log == it->second )
        {
            owner = it->first;
            break;
        }
    }

    if( !owner.empty() )
    {
        m_pInstances->erase(owner);
        delete Log;
    }
}

void
CLog::init()
{
    /* default Log Format */
    setLogFormat(FORMATE_SIMPLE);
}

const std::string
CLog::getLogLabelStr(LogLabel logLabel)
{
    switch (logLabel)
    {
        case LABEL_TIMESTAMP:
            return "Timestamp";
            break;
        case LABEL_THREAD_ID:
            return "ThreadID";
            break;
        case LABEL_LEVEL:
            return "Level";
            break;
        case LABEL_TAG:
            return "Tag";
            break;
        case LABEL_FUNCTION:
            return "Function";
            break;
        case LABEL_FILE:
            return "File";
            break;
        case LABEL_LINE:
            return "Line";
            break;
        case LABEL_OWNER:
            return "Owner";
            break;
        case LABEL_COUNT:
        default:
            return "Unknow";
            break;
    }
}

const std::string
CLog::getCurrentTimeString()
{
#ifdef WIN32
    SYSTEMTIME lt;
    GetLocalTime(&lt);

    char currentTime[128] = {0};
    sprintf(currentTime, "%04d-%02d-%02d %02d:%02d:%02d.%03d",
            lt.wYear, lt.wMonth, lt.wDay,
            lt.wHour, lt.wMinute, lt.wSecond,
            lt.wMilliseconds);

    return std::string(currentTime);
#else
    char            fmt[64], buf[64];
    struct timeval  tv;
    struct tm       *tm;

    gettimeofday(&tv, NULL);
    tm = localtime(&tv.tv_sec);
    strftime(fmt, sizeof fmt, "%Y-%m-%d %H:%M:%S.%%06u", tm);
    snprintf(buf, sizeof buf, fmt, tv.tv_usec);
    return std::string(buf);
#endif
}

const std::string
CLog::getCurrentThreadIDString()
{
	return "";
}

void
CLog::setLogFormat(LogFormat logFormat)
{
    switch (logFormat) {
    case FORMATE_SIMPLE:
        setLogFormat(
                    CLog::LABEL_LEVEL
                    );
        break;
    case FORMATE_DETAIL:
        setLogFormat(
                    LABEL_TIMESTAMP,
                    LABEL_LEVEL,
                    LABEL_TAG,
                    LABEL_FILE,
                    LABEL_LINE
                    );
        break;
    default:
        break;
    }
}

void
CLog::setLogFormat(LogLabel o0, LogLabel o1, LogLabel o2, LogLabel o3,
                     LogLabel o4, LogLabel o5, LogLabel o6, LogLabel o7)
{
    /* Note : index < LABEL_COUNT */
    m_logLabelOrder[0] = o0;
    m_logLabelOrder[1] = o1;
    m_logLabelOrder[2] = o2;
    m_logLabelOrder[3] = o3;
    m_logLabelOrder[4] = o4;
    m_logLabelOrder[5] = o5;
    m_logLabelOrder[6] = o6;
    m_logLabelOrder[7] = o7;
}

void
CLog::setLogType(int logType)
{
    m_logType = logType;
}

void
CLog::setLogFile(const char* logFile)
{
    releaseLogFile();

    m_logFile = fopen( logFile, "w+" );

    if ( !m_logFile )
    {
        LOG_G2_E( this, TAG, "Set/Open log file fail: \"%s\"", logFile );
    }
    else
    {
        LOG_G2_D( this, TAG, "Set/Open log file succeed: \"%s\" (%d)", logFile, m_logFile );
    }
}

void
CLog::setDebugVisible(bool debugVisible)
{
    m_debugVisible = debugVisible;
}

void
CLog::setInfoVisible(bool infoVisible)
{
    m_infoVisible = infoVisible;
}

void
CLog::setWarningVisible(bool warningVisible)
{
    m_warningVisible = warningVisible;
}

void
CLog::setErrorVisible(bool errorVisible)
{
    m_errorVisible = errorVisible;
}

bool
CLog::isLOG_D()
{
    return m_debugVisible;
}

bool
CLog::isLOG_I()
{
    return m_infoVisible;
}

bool
CLog::isLOG_W()
{
    return m_warningVisible;
}

bool
CLog::isLOG_E()
{
    return m_errorVisible;
}

bool
CLog::isLOG_G2()
{
    return m_G2Visible;
}

void
CLog::printLogFormat()
{
    char fmt[FORMAT_MAX_LENGTH] = {0};

    /* label */
    for( int order = LABEL_TIMESTAMP; order < LABEL_COUNT; order++ )
    {
        switch (m_logLabelOrder[order])
        {
            case LABEL_TIMESTAMP:
            case LABEL_THREAD_ID:
            case LABEL_LEVEL:
            case LABEL_TAG:
            case LABEL_FUNCTION:
            case LABEL_FILE:
            case LABEL_LINE:
            case LABEL_OWNER:
                strcat(fmt, "[");
                strcat(fmt, getLogLabelStr(m_logLabelOrder[order]).c_str());
                strcat(fmt, "]");
                break;
            case LABEL_COUNT:
            default:
                break;
        }
    }

    /* message */
    strcat(fmt, MESSAGE_BEGIN);
    strcat(fmt, "Message\n");

    /* print */
    if( m_logType & LOG_TYPE_LOGCAT )
    {
        printf(fmt);
        fflush(stdout);
    }

    if( m_logFile && (m_logType & LOG_TYPE_FILE) )
    {
        fprintf(m_logFile, fmt);
        fflush(m_logFile);
    }
}

void
CLog::printLog(const char* level, const char* tag, const char* function, const char* file, const int line, const char* message, ...)
{
    if( m_logType & LOG_TYPE_NONE )
    {
        return;
    }

    /* prepare format */
    const int len = strlen(message) + FORMAT_MAX_LENGTH;
    char* fmt = new char[len];
    memset(fmt, 0, len * sizeof(char));

    /* append label */
    for( int order = LABEL_TIMESTAMP; order < LABEL_COUNT; order++ )
    {
        switch (m_logLabelOrder[order])
        {
            case LABEL_TIMESTAMP:
                appendLabelTimeStamp(fmt);
                break;
            case LABEL_THREAD_ID:
                appendLabelThreadID(fmt);
                break;
            case LABEL_LEVEL:
                appendLabelLevel(fmt, level);
                break;
            case LABEL_TAG:
                appendLabelTag(fmt, tag);
                break;
            case LABEL_FUNCTION:
                appendLabelFunction(fmt, function);
                break;
            case LABEL_FILE:
                appendLabelFile(fmt, file);
                break;
            case LABEL_LINE:
                appendLabelLine(fmt, line);
                break;
            case LABEL_OWNER:
                appendLabelOwner(fmt);
                break;
            case LABEL_COUNT:
            default:
                break;
        }
    }
    strcat(fmt, MESSAGE_BEGIN);
    strcat(fmt, message);
    strcat(fmt, "\n");
    if( m_logType & LOG_TYPE_LOGCAT )
    {
        va_list arglist;
        va_start( arglist, message );
        vprintf(fmt, arglist);
        va_end( arglist );
        fflush(stdout);
    }
    if( m_logFile && (m_logType & LOG_TYPE_FILE) )
    {
        va_list arglist;
        va_start( arglist, message );
        vfprintf(m_logFile, fmt, arglist); // vfprintf may cause crash : arg_ptr somethings error !
        va_end( arglist );
        fflush(m_logFile);
    }
    delete [] fmt;
}

void
CLog::printG2Log(const char* tag, const char* function, const char* file, const int line, const char* message, ...)
{

    /* prepare format */
    const int len = strlen(message) + FORMAT_MAX_LENGTH;
    char* fmt = new char[len];
    memset(fmt, 0, len * sizeof(char));

    /* append message */
    strcat(fmt, message);
    strcat(fmt, "\n");

    /* print */
    if( m_logType & LOG_TYPE_LOGCAT )
    {
        va_list arglist;
        va_start( arglist, message );
        vprintf(fmt, arglist);
        va_end( arglist );

        fflush(stdout);
    }

    if( m_logFile && (m_logType & LOG_TYPE_FILE) )
    {
        va_list arglist;
        va_start( arglist, message );
        vfprintf(m_logFile, fmt, arglist); // vfprintf may cause crash : arg_ptr somethings error !
        va_end( arglist );

        fflush(m_logFile);
    }

    delete [] fmt;
}

void
CLog::appendLabelTimeStamp(char* fmt)
{
    std::string timestamp = getCurrentTimeString();

    strcat(fmt, "[");
    strcat(fmt, timestamp.c_str());
    strcat(fmt, "]");
}

void
CLog::appendLabelThreadID(char* fmt)
{
    std::string threadID = getCurrentThreadIDString();

    strcat(fmt, "[");
    strcat(fmt, threadID.c_str());
    strcat(fmt, "]");
}

void
CLog::appendLabelLevel(char* fmt, const char* level)
{
    strcat(fmt, "[");
    strcat(fmt, level);
    strcat(fmt, "]");
}

void
CLog::appendLabelTag(char* fmt, const char* tag)
{
    strcat(fmt, "[");
    strcat(fmt, tag);
    strcat(fmt, "]");
}

void
CLog::appendLabelFunction(char* fmt, const char* function)
{
    strcat(fmt, "[");
    strcat(fmt, function);
    strcat(fmt, "]");
}

void
CLog::appendLabelFile(char* fmt, const char* file)
{
    strcat(fmt, "[");
    strcat(fmt, file);
    strcat(fmt, "]");
}

void
CLog::appendLabelLine(char* fmt, const int line)
{
    char linestr[64];
    sprintf(linestr, "%d", line);

    strcat(fmt, "[");
    strcat( fmt, linestr );
    strcat(fmt, "]");
}

void
CLog::appendLabelOwner(char* fmt)
{
    strcat(fmt, "[");
    strcat( fmt, m_owner.c_str() );
    strcat(fmt, "]");
}

