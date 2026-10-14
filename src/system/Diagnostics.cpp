#include "Diagnostics.h"

namespace {
    Diagnostics::ErrorCode currentError = Diagnostics::ErrorCode::NONE;
    bool tofOk = false;
    bool imuOk = false;
}

namespace Diagnostics {
    void begin() { currentError = ErrorCode::NONE; }
    void setError(ErrorCode code) { currentError = code; }
    ErrorCode getError() { return currentError; }
    bool hasError() { return currentError != ErrorCode::NONE; }
    void clearError() { currentError = ErrorCode::NONE; }

    void setSelfCheck(bool tof, bool imu) {
        tofOk = tof;
        imuOk = imu;
    }
    bool tofSelfCheckOk() { return tofOk; }
    bool imuSelfCheckOk() { return imuOk; }
}