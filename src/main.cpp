#include "runner.h"
#include <filesystem>
#include <iostream>
#include <string>
#include <windows.h>

int main(int argc, char* argv[]) {
    SetConsoleOutputCP(CP_UTF8);

    // Run directly in CLion: change these two settings as needed.
    std::string instancePath = std::string(PROJECT_DIR) + "/data/c103_21_25.txt";
    double timeLimit = 3600;

    // Batch tests supply: instancePath [timeLimit] [outputDir].
    try {
        if (argc > 4)
            throw std::runtime_error("Usage: EVRP-CRBSS instancePath [timeLimit] [outputDir]");
        if (argc > 1)
            instancePath = argv[1];
        if (argc > 2)
            timeLimit = std::stod(argv[2]);
        std::filesystem::path outputDir =
            std::filesystem::path(PROJECT_DIR) / "results" / std::filesystem::path(instancePath).stem();
        if (argc > 3)
            outputDir = argv[3];
        return runModel(instancePath, timeLimit, outputDir.string());
    } catch (const std::exception& error) {
        std::cerr << error.what() << "\n";
        return 1;
    }
}
