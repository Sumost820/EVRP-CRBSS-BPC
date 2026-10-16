// 用节约算法生成初始列，同时保留每个客户的单独访问路线。
#include "solver.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <numeric>

bool Solver::checkInitialRoute(const std::vector<int>& path) {
    double load = 0;
    for (int i : path) {
        if (std::find(instance.customers.begin(), instance.customers.end(), i) != instance.customers.end()) {
            load += instance.demand[i];
        }
    }
    if (load > instance.loadCapacity)
        return false;

    double t = 0.0, soc = instance.batteryCapacity;
    for (size_t i = 0; i < path.size() - 1; ++i) {
        int u = path[i], v = path[i + 1];
        soc -= instance.getDistance(u, v) * instance.consumptionRate;
        if (soc < 0)
            return false;
        t = std::max(t + instance.time.at({u, v}) + instance.serviceTime[u], instance.readyTime[v]);
        if (t > instance.dueTime[v])
            return false;
    }
    return true;
}

std::vector<Route> Solver::createInitialRoutes() {
    std::vector<std::vector<int>> currentPaths;
    for (int c : instance.customers) {
        currentPaths.push_back({instance.sourceDepot, c, instance.sinkDepot});
    }

    struct Saving {
        double val;
        int i;
        int j;
    };
    std::vector<Saving> savings;
    for (int i : instance.customers) {
        for (int j : instance.customers) {
            if (i == j)
                continue;
            double sVal = instance.getDistance(i, instance.sourceDepot) +
                          instance.getDistance(instance.sourceDepot, j) - instance.getDistance(i, j);
            if (sVal > 0)
                savings.push_back({sVal, i, j});
        }
    }

    std::sort(savings.begin(), savings.end(), [](const Saving& a, const Saving& b) { return a.val > b.val; });

    for (const auto& s : savings) {
        auto pathIIt = std::find_if(currentPaths.begin(), currentPaths.end(),
            [&](const std::vector<int>& p) { return p[p.size() - 2] == s.i; });
        auto pathJIt = std::find_if(
            currentPaths.begin(), currentPaths.end(), [&](const std::vector<int>& p) { return p[1] == s.j; });

        if (pathIIt != currentPaths.end() && pathJIt != currentPaths.end() && pathIIt != pathJIt) {
            std::vector<int> newPath = *pathIIt;
            newPath.pop_back();
            newPath.insert(newPath.end(), pathJIt->begin() + 1, pathJIt->end());

            if (checkInitialRoute(newPath)) {
                std::vector<int> targetI = *pathIIt;
                std::vector<int> targetJ = *pathJIt;

                currentPaths.erase(
                    std::remove(currentPaths.begin(), currentPaths.end(), targetI), currentPaths.end());
                currentPaths.erase(
                    std::remove(currentPaths.begin(), currentPaths.end(), targetJ), currentPaths.end());
                currentPaths.push_back(newPath);
            }
        }
    }

    std::vector<Route> routes;
    for (const auto& p : currentPaths) {
        double cost = 0;
        for (size_t k = 0; k < p.size() - 1; ++k)
            cost += instance.getDistance(p[k], p[k + 1]);
        cost += instance.chargeVehicleCost;
        routes.push_back({p, 'C', checkInitialRoute(p) ? cost : 1e9});
    }
    for (int c : instance.customers) {
        std::vector<int> p = {instance.sourceDepot, c, instance.sinkDepot};
        double cost = instance.getDistance(0, c) + instance.getDistance(c, instance.sinkDepot) +
                      instance.chargeVehicleCost;
        routes.push_back({p, 'C', checkInitialRoute(p) ? cost : 1e9});
    }
    return routes;
}
