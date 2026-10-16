#include "runner.h"
#include "solver.h"
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>

// The original two-way log stream: print to the console and a file together.
class LogBuffer : public std::streambuf {
public:
    LogBuffer(std::streambuf* console, std::streambuf* file) : console(console), file(file) {}

protected:
    int overflow(int value) override {
        if (value == EOF)
            return !EOF;
        int consoleResult = console->sputc(value);
        int fileResult = file->sputc(value);
        return consoleResult == EOF || fileResult == EOF ? EOF : value;
    }

    int sync() override {
        int consoleResult = console->pubsync();
        int fileResult = file->pubsync();
        return consoleResult == 0 && fileResult == 0 ? 0 : -1;
    }

private:
    std::streambuf* console;
    std::streambuf* file;
};

int runModel(const std::string& instancePath, double timeLimit, const std::string& outputDir) {
    if (!std::filesystem::is_regular_file(instancePath))
        throw std::runtime_error("Instance file does not exist: " + instancePath);
    if (!std::isfinite(timeLimit) || timeLimit <= 0)
        throw std::runtime_error("timeLimit must be positive and finite");

    std::filesystem::create_directories(outputDir);
    // Clear only this program's previous solution; the log is opened in overwrite mode.
    std::filesystem::remove(std::filesystem::path(outputDir) / "routes.csv");
    std::filesystem::remove(std::filesystem::path(outputDir) / "result.csv");
    std::ofstream logFile(std::filesystem::path(outputDir) / "solver.log");
    if (!logFile)
        throw std::runtime_error("Cannot create solver.log in " + outputDir);

    std::streambuf* console = std::cout.rdbuf();
    LogBuffer logBuffer(console, logFile.rdbuf());
    std::cout.rdbuf(&logBuffer);
    int exitCode = 1;
    try {
        solveTimeLimit = timeLimit;
        std::cout << "Loading Instance: " << instancePath << "\n";
        Solver solver(instancePath, outputDir);
        solver.solve();
        exitCode = solver.hasStopped() ? 1 : (solver.hasSolution() ? 0 : 2);
    } catch (const GRBException& error) {
        std::cout << "Gurobi error " << error.getErrorCode() << ": " << error.getMessage() << "\n";
    } catch (const std::exception& error) {
        std::cout << "Exception: " << error.what() << "\n";
    } catch (...) {
        std::cout.flush();
        std::cout.rdbuf(console);
        throw;
    }
    std::cout.flush();
    std::cout.rdbuf(console);
    return exitCode;
}
