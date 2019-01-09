#ifndef PROCEDUREDLLEXPORTDEFINE
#define PROCEDUREDLLEXPORTDEFINE

#if defined(WIN32) | defined(_WIN32)
    #ifdef DYNAMIC_G2PROCEDURE_LIBRARY
        #define G2PROCEDURE_EXPORT //__declspec(dllexport)
    #else
        #ifdef STATIC_G2PROCEDURE_LIBRARY
            #define G2PROCEDURE_EXPORT
        #else
            #define G2PROCEDURE_EXPORT //__declspec(dllimport)
        #endif
    #endif
#else
    #define G2PROCEDURE_EXPORT
#endif

#endif // PROCEDUREDLLEXPORTDEFINE

