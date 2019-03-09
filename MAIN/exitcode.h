#ifndef EXITCODE
#define EXITCODE

namespace G2
{
    enum ExitCode
    {
        EXIT_PASS = 0,                      // execute done. And result pass.
        EXIT_IC_COMMUNICATION_ERROR = 2,    // G2touch-IC communication error. It will need to self-retry.
        EXIT_FLOW_ERROR = 3,                 // check flow/somethings error. User may set some things error.
        EXIT_FAIL = 53,                      // execute done. But result fail. Such as compare different
        EXIT_TEST_ERROR = 57                 // check flow/somethings error. User may set some things error.
    };
}

#endif // EXITCODE

