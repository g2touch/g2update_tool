#ifndef BASEDEFINE
#define BASEDEFINE

#define USING_EXIT_CODE ExitCode exitCode = EXIT_FAIL;
#define EXIT_CODE exitCode

#define SHOW_RESULT \
switch (EXIT_CODE)\
{\
case EXIT_PASS:\
    LOG_G2(CLog::getLogOwner(), "SHOW_RESULT", "result : PASS");\
    break;\
case EXIT_FAIL:\
    LOG_G2(CLog::getLogOwner(), "SHOW_RESULT", "result : FAIL");\
    break;\
case EXIT_IC_COMMUNICATION_ERROR:\
    LOG_G2(CLog::getLogOwner(), "SHOW_RESULT", "$result : COMMUNICATION_ERROR");\
    break;\
case EXIT_FLOW_ERROR:\
    LOG_G2(CLog::getLogOwner(), "SHOW_RESULT", "$result : FLOW_ERROR");\
    break;\
default:\
    LOG_G2(CLog::getLogOwner(), "SHOW_RESULT", "$result : UNDO");\
    break;\
}\

#define RETURN_EXIT_CODE \
    LOG_G2_I(CLog::getLogOwner(), "RETURN_EXIT_CODE", "");\
    LOG_G2_I(CLog::getLogOwner(), "RETURN_EXIT_CODE", "RETURN_EXIT_CODE: %d", EXIT_CODE);\
    SHOW_RESULT\
    CLog::getLogOwner()->releaseLogFile();\
    return EXIT_CODE;\

#endif // BASEDEFINE

