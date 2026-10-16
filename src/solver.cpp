#include "solver.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <numeric>

Solver::Solver(const std::string& dataFile, const std::string& resultDirectory)
    : instance(dataFile), rmp(instance) {
    outputDirectory = resultDirectory;
    startTime = std::chrono::high_resolution_clock::now();
    solveStartTime = startTime;
    upperBound = INFINITY;

    size_t pos = dataFile.find_last_of("/\\");
    instanceName = (pos == std::string::npos) ? dataFile : dataFile.substr(pos + 1);
    size_t dotPos = instanceName.find_last_of('.');
    if (dotPos != std::string::npos)
        instanceName = instanceName.substr(0, dotPos);

    std::vector<Route> initRoutes = createInitialRoutes();
    rmp.addColumns(initRoutes);
}

void Solver::solve() {
    std::cout << "Starting Branch-and-Price with Bidirectional Labeling & Exact Backend Functions...\n\n";
    std::set<std::pair<int, int>> forbidden;
    std::vector<std::string> history;

    try {
        branchAndBound(forbidden, 0, history);
    } catch (const TimeLimitError& e) {
        timedOut = true;
        std::cout << "\n[TERMINATED] " << e.what() << "\n";
        std::cout << "Keeping only incumbents with a verified global schedule.\n";
    } catch (const std::runtime_error& e) {
        stopped = true;
        stopReason = e.what();

        std::cout << "\n[STOPPED WITHOUT CERTIFICATION] " << e.what() << "\n";
    }

    auto endTime = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = endTime - startTime;

    runtime = elapsed.count();
    printSummary();
    saveResult();
}

void Solver::updateIncumbent(double objective, const std::vector<std::pair<Route, double>>& newRoutes,
    const ScheduleResult& schedule) {
    if (schedule.status != ScheduleStatus::Feasible || schedule.visits.size() != newRoutes.size())
        throw std::runtime_error("An incumbent requires a verified global schedule");
    std::map<int, int> coverage;
    int chargeVehicles = 0, swapVehicles = 0;
    double actualCost = 0;
    for (size_t r = 0; r < newRoutes.size(); ++r) {
        const Route& route = newRoutes[r].first;
        if (std::abs(newRoutes[r].second - 1.0) > 1e-5 || schedule.visits[r].size() != route.path.size())
            throw std::runtime_error("Invalid route multiplicity or schedule");
        if (route.vehicleType == 'S')
            ++swapVehicles;
        else
            ++chargeVehicles;
        actualCost += route.vehicleType == 'S' ? instance.swapVehicleCost : instance.chargeVehicleCost;
        for (size_t p = 0; p < route.path.size(); ++p) {
            int node = route.path[p];
            if (schedule.visits[r][p].node != node)
                throw std::runtime_error("Schedule does not match the candidate routes");
            if (std::find(instance.customers.begin(), instance.customers.end(), node) !=
                instance.customers.end())
                ++coverage[node];
            if (std::find(instance.swapStations.begin(), instance.swapStations.end(), node) !=
                instance.swapStations.end())
                actualCost += instance.swapCost;
            if (p > 0)
                actualCost += instance.getDistance(route.path[p - 1], node);
        }
    }
    for (int customer : instance.customers)
        if (coverage[customer] != 1)
            throw std::runtime_error(
                "Candidate does not cover every customer exactly once; check RMP artificials");
    if (chargeVehicles > instance.chargeVehicleCount || swapVehicles > instance.swapVehicleCount ||
        std::abs(actualCost - objective) > 1e-5 * std::max(1.0, std::abs(actualCost)))
        throw std::runtime_error("Candidate fleet count or objective is inconsistent");
    bestSchedule = schedule;
    upperBound = objective;
    bestRoutes = newRoutes;

    std::cout << "    [Update Bound] New Best Obj: " << std::fixed << std::setprecision(2) << objective
              << "\n";

    int rIdx = 1;
    for (const auto& pair : bestRoutes) {
        if (pair.second >= 1e-5) {
            std::cout << "      R" << rIdx++ << " (" << pair.first.vehicleType << "): ";
            for (size_t i = 0; i < pair.first.path.size(); ++i) {
                std::cout << pair.first.path[i] << (i == pair.first.path.size() - 1 ? "" : "->");
            }
            std::cout << "\n";
        }
    }

    std::string outFilename = (std::filesystem::path(outputDirectory) / "routes.csv").string();
    std::ofstream outFile(outFilename);
    if (outFile.is_open()) {
        outFile << "RouteID,Type,Cost,Path\n";
        for (int i = 0; i < (int)bestRoutes.size(); ++i) {
            if (bestRoutes[i].second < 1e-5)
                continue;
            outFile << i + 1 << "," << bestRoutes[i].first.vehicleType << "," << bestRoutes[i].first.cost
                    << ",";
            for (int n : bestRoutes[i].first.path)
                outFile << n << "-";
            outFile << "\n";
        }
        outFile.close();
    }
}

std::vector<std::vector<int>> Solver::findConflicts(
    const std::vector<std::pair<int, double>>& activeVars, const std::vector<Route>& routes) {
    std::vector<int> routeIds;
    std::vector<Route> selectedRoutes;
    for (const auto& item : activeVars) {
        if (item.first < 0 || item.first >= static_cast<int>(routes.size()) ||
            std::abs(item.second - 1.0) > 1e-5)
            throw std::runtime_error("Global scheduling requires valid integer route selections");
        routeIds.push_back(item.first);
        selectedRoutes.push_back(routes[item.first]);
    }
    double elapsed =
        std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - solveStartTime).count();
    lastSchedule = checkGlobalSchedule(instance, selectedRoutes, solveTimeLimit - elapsed);
    if (lastSchedule.status == ScheduleStatus::Unknown) {
        checkTimeLimit();
        throw std::runtime_error(lastSchedule.message);
    }
    if (lastSchedule.status == ScheduleStatus::Feasible)
        return {};
    if (routeIds.empty())
        throw std::runtime_error("An empty route set cannot define a conflict cut");

    return {routeIds};
}
