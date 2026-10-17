#pragma once
#include <string>

// Run one instance and save its log, route table and summary.
int runModel(const std::string& instancePath, double timeLimit, const std::string& outputDir,
    int chargersPerStation = 1);
