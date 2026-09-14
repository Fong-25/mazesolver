#include "Diagnostics.h"

namespace {
    Diagnostics::ErrorCode currentError = Diagnostics::ErrorCode::NONE;
}

namespace Diagnostics {
    void begin() { currentError = ErrorCode::NONE; }
    void setError(ErrorCode code) { currentError = code; }
    ErrorCode getError() { return currentError; }
    bool hasError() { return currentError != ErrorCode::NONE; }
    void clearError() { currentError = ErrorCode::NONE; }
}