#include "runner.h"
#include <filesystem>
#include <iostream>
#include <string>
#include <windows.h>

int main(int argc, char* argv[]) {
    SetConsoleOutputCP(CP_UTF8);

    // Run directly in CLion: change these settings as needed.
    std::string instancePath = std::string(PROJECT_DIR) + "/data/c103_21_25.txt";
    double timeLimit = 3600;
    int chargersPerStation = 1;

    // Batch tests supply: instancePath [timeLimit] [outputDir] [chargersPerStation].
    try {
        if (argc > 5)
            throw std::runtime_error("Usage: EVRP-CRBSS instancePath [timeLimit] [outputDir] [chargersPerStation]");
        if (argc > 1)
            instancePath = argv[1];
        if (argc > 2)
            timeLimit = std::stod(argv[2]);
        std::filesystem::path outputDir =
            std::filesystem::path(PROJECT_DIR) / "results" / std::filesystem::path(instancePath).stem();
        if (argc > 3)
            outputDir = argv[3];
        if (argc > 4) {
            const std::string value = argv[4];
            size_t parsed = 0;
            chargersPerStation = std::stoi(value, &parsed);
            if (parsed != value.size())
                throw std::invalid_argument("chargersPerStation must be a positive integer");
        }
        return runModel(instancePath, timeLimit, outputDir.string(), chargersPerStation);
    } catch (const std::exception& error) {
        std::cerr << error.what() << "\n";
        return 1;
    }
}
