#ifndef BASEDLLEXPORTDEFINE
#define BASEDLLEXPORTDEFINE

#if defined(WIN32) | defined(_WIN32)
    #ifdef DYNAMIC_Base_LIBRARY
        #define Base_EXPORT //__declspec(dllexport)
    #else
        #ifdef STATIC_Base_LIBRARY
            #define Base_EXPORT
        #else
            #define Base_EXPORT //__declspec(dllimport)
        #endif
    #endif
#else
    #define Base_EXPORT
#endif

#endif // BASEDLLEXPORTDEFINE

