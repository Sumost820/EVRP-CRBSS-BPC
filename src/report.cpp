// 最终路线表保持原格式；另存一行 CSV 摘要供 pytest 和后续汇总读取。
#include "solver.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <numeric>

void Solver::printSummary() {
    std::cout << "\n========================================\n";
    std::cout << "Optimization Finished / Stopped!\n";

    std::cout << "Final Best Obj (Upper Bound): ";
    if (upperBound < INFINITY) {
        std::cout << std::fixed << std::setprecision(2) << upperBound << "\n";
    } else {
        std::cout << "N/A (No feasible solution found)\n";
    }

    std::cout << "Best Lower Bound: ";
    if (lowerBound > 0) {
        std::cout << std::fixed << std::setprecision(2) << lowerBound << "\n";
    } else {
        std::cout << "N/A (Root node LP not converged)\n";
    }

    if (upperBound < INFINITY && lowerBound > 0) {
        double gap = (upperBound - lowerBound) / upperBound * 100.0;
        if (gap < 0.0 && gap > -1e-5)
            gap = 0.0;
        std::cout << "GAP: " << std::fixed << std::setprecision(2) << gap << " %\n";
    } else {
        std::cout << "GAP: N/A\n";
    }

    std::cout << "Total Time: " << std::fixed << std::setprecision(2) << runtime << " s\n";
    std::cout << "========================================\n\n";

    if (!bestRoutes.empty()) {
        std::cout
            << "=======================================================================================\n";
        std::cout << "                             [Final Feasible Routes Detail Report]\n";
        std::cout
            << "=======================================================================================\n\n";
        int routeIdx = 1;
        for (const auto& pair : bestRoutes) {
            if (pair.second >= 1e-5) {
                std::cout << ">> Route #" << routeIdx++ << "\n";
                std::string vStr = (pair.first.vehicleType == 'S') ? "Battery Swapping" : "Standard";
                std::cout << ">>> Joint Schedule (Vehicle Type: " << vStr << ")\n";
                printRouteDetails(pair.first, bestSchedule.visits.at(routeIdx - 2));
                std::cout << "\n";
            }
        }
    } else {
        std::cout << "\nNo globally verified feasible routes found.\n";
    }
}

void Solver::printRouteDetails(const Route& route, const std::vector<ScheduledVisit>& visits) {
    const std::string separator(96, '-');
    std::cout << separator << "\n";
    std::cout << std::left << std::setw(19) << "node" << std::right << std::setw(9) << "arr_time"
              << std::setw(12) << "wait_time" << std::setw(12) << "ser_start" << std::setw(12) << "ser_time"
              << std::setw(12) << "dep_time" << std::setw(10) << "arr_SOC" << std::setw(10) << "dep_SOC"
              << "\n";
    std::cout << separator << "\n";

    for (size_t i = 0; i < visits.size(); ++i) {
        const ScheduledVisit& visit = visits[i];
        int node = visit.node;
        std::string nodeName;
        if (node == instance.sourceDepot || node == instance.sinkDepot) {
            nodeName = "Depot_" + std::to_string(node);
        } else if (std::find(instance.customers.begin(), instance.customers.end(), node) !=
                   instance.customers.end()) {
            nodeName = "C" + std::to_string(node);
        } else if (std::find(instance.swapStations.begin(), instance.swapStations.end(), node) !=
                   instance.swapStations.end()) {
            nodeName = "S" + std::to_string(node) + "(Swap)";
        } else {
            nodeName = "S" + std::to_string(node) + "(Charge)";
        }

        double arrivalTime = visit.arrivalTime;
        if (i > 0) {
            arrivalTime = visits[i - 1].departureTime + instance.getTravelTime(route.path[i - 1], node);
        }
        double waitTime = std::max(0.0, visit.startTime - arrivalTime);
        double serviceTime = std::max(0.0, visit.departureTime - visit.startTime);

        std::cout << std::left << std::setw(19) << nodeName << std::right << std::fixed
                  << std::setprecision(2) << std::setw(9) << std::max(0.0, arrivalTime) << std::setw(12)
                  << waitTime << std::setw(12) << std::max(0.0, visit.startTime) << std::setw(12)
                  << serviceTime << std::setw(12) << std::max(0.0, visit.departureTime) << std::setw(10)
                  << std::max(0.0, visit.arrivalEnergy) << std::setw(10)
                  << std::max(0.0, visit.departureEnergy) << "\n";
    }
    std::cout << separator << "\n";
}

void Solver::printRouteDiagnostic(const std::vector<int>& path, char vehicleType) {
    std::cout << "\n=================================================================\n";
    std::cout << "[Diagnostic] Exact Route Validation\n";
    std::cout << "Path: ";
    for (int n : path)
        std::cout << n << " ";
    std::cout << "\nVehicle Type: " << vehicleType << "\n";
    std::cout << "-----------------------------------------------------------------\n";

    if (path.empty()) {
        std::cout << "[ERROR] Path is empty.\n";
        return;
    }

    double load = 0;
    for (int node : path) {
        if (std::find(instance.customers.begin(), instance.customers.end(), node) !=
            instance.customers.end()) {
            load += instance.demand[node];
        }
    }
    std::cout << "Route Load: " << load << " / " << instance.loadCapacity;
    if (load > instance.loadCapacity + 1e-6) {
        std::cout << " -> [FAILED] Capacity violated!\n";
        return;
    } else {
        std::cout << " -> [OK]\n";
    }

    EnergyFunction currF({{0.0, 0.0}, {instance.batteryCapacity, 0.0}}, instance.batteryCapacity);

    for (size_t i = 0; i < path.size(); ++i) {
        int u = path[i];

        std::cout << "\n> Node " << u << " (TW: [" << instance.readyTime[u] << ", " << instance.dueTime[u]
                  << "])\n";

        if (currF.points.empty()) {
            std::cout << "  [FAILED] Unreachable! Time window missed or battery depleted before this node.\n";
            return;
        } else {
            std::cout << "  Valid Pareto States (SoC Consumed, Arr Time):\n";
            for (const auto& p : currF.points) {
                std::cout << "    - Used SoC: " << std::fixed << std::setprecision(2) << p.first
                          << " (Remains: " << instance.batteryCapacity - p.first << "), Time: " << p.second
                          << "\n";
            }
        }

        if (i == path.size() - 1)
            break;

        int v = path[i + 1];
        double dq = instance.distance.at({u, v}) * instance.consumptionRate;
        double dt = instance.time.at({u, v}) + instance.serviceTime[u];

        std::cout << "  -> Travel to " << v << " | Dist: " << instance.distance.at({u, v})
                  << " | SoC Cost: " << dq << " | Travel+Service Time: " << dt << "\n";

        EnergyFunction fNext = shiftAndClipForward(
            currF, dq, dt, instance.readyTime[v], instance.dueTime[v], instance.batteryCapacity);

        if (std::find(instance.chargeStations.begin(), instance.chargeStations.end(), v) !=
            instance.chargeStations.end()) {
            currF = chargeAlgorithmUpdate(fNext, instance.chargingFunctions.at(v), instance.batteryCapacity);
            std::cout << "  -> Operation at " << v << ": Charging\n";
        } else if (std::find(instance.swapStations.begin(), instance.swapStations.end(), v) !=
                   instance.swapStations.end()) {
            if (vehicleType == 'C') {
                std::cout << "  [FAILED] Standard EV (Type C) cannot use Swap Station!\n";
                return;
            }
            double tDep = fNext.points.empty() ? INFINITY : fNext.tMin + instance.swapTimes[v];
            currF = EnergyFunction({{0, tDep}, {instance.batteryCapacity, tDep}}, instance.batteryCapacity);
            std::cout << "  -> Operation at " << v << ": Battery Swapping (Takes " << instance.swapTimes[v]
                      << ")\n";
        } else {
            currF = fNext;
        }
    }

    std::cout << "\n=================================================================\n";
    std::cout << "[SUCCESS] Route is strictly FEASIBLE according to Pricing rules!\n";
    std::cout << "=================================================================\n";
}

void Solver::saveResult() {
    std::ofstream file(std::filesystem::path(outputDirectory) / "result.csv");
    file << "instance,status,hasSolution,objective,lowerBound,runtime,routeCount\n";
    std::string status = stopped ? "STOPPED" : (timedOut ? "TIME_LIMIT" : "FINISHED");
    file << instanceName << "," << status << "," << (!bestRoutes.empty()) << ",";
    file << std::setprecision(15);
    if (std::isfinite(upperBound))
        file << upperBound;
    file << ",";
    if (std::isfinite(lowerBound) && lowerBound > 0)
        file << lowerBound;
    file << "," << runtime << "," << bestRoutes.size() << "\n";
}
