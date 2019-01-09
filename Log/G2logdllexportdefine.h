#ifndef G2LOGDLLEXPORTDEFINE
#define G2LOGDLLEXPORTDEFINE

#if defined(WIN32) | defined(_WIN32)
    #ifdef DYNAMIC_G2LOG_LIBRARY
        #define LOG_EXPORT //__declspec(dllexport)
    #else
        #ifdef STATIC_G2LOG_LIBRARY
            #define LOG_EXPORT
        #else
            #define LOG_EXPORT //__declspec(dllimport)
        #endif
    #endif
#else
    #define LOG_EXPORT
#endif

#endif // G2LOGDLLEXPORTDEFINE

