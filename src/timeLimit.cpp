#include "timeLimit.h"

std::chrono::time_point<std::chrono::high_resolution_clock> solveStartTime;
double solveTimeLimit = 3600.0;

void checkTimeLimit() {
    double elapsed =
        std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - solveStartTime).count();
    if (elapsed > solveTimeLimit)
        throw TimeLimitError();
}
