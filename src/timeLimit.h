#pragma once
#include <chrono>
#include <stdexcept>

class TimeLimitError : public std::runtime_error {
public:
    TimeLimitError() : std::runtime_error("Time limit exceeded") {}
};

extern std::chrono::time_point<std::chrono::high_resolution_clock> solveStartTime;
extern double solveTimeLimit;
void checkTimeLimit();
