// 基于弧使用量分支；保留原来的分支顺序、剪枝阈值和递归搜索。
#include "solver.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <numeric>

void Solver::branchAndBound(
    std::set<std::pair<int, int>> forbidden, int depth, std::vector<std::string> history) {
    checkTimeLimit();
    std::cout << "\n[B&B Depth " << depth << "] Current Branch: ";
    if (history.empty())
        std::cout << "Root\n";
    else {
        for (const auto& h : history)
            std::cout << h << " ";
        std::cout << "\n";
    }
    std::cout << "    Solving LP with " << forbidden.size() << " forbidden arcs...\n";

    rmp.setBranching(forbidden);

    ColumnGenerationResult lpResult = columnGeneration(forbidden);
    double lpVal = lpResult.objective;
    const std::vector<double>& routeValues = lpResult.routeValues;
    const std::vector<Route>& routes = lpResult.routes;

    if (depth == 0) {
        lowerBound = lpVal;
    }

    std::cout << "    [LP Solved] Current LP Obj: " << std::fixed << std::setprecision(2) << lpVal << "\n";

    if (lpVal > upperBound - 1e-4) {
        return;
    }

    std::map<std::pair<int, int>, double> arcUsage;
    for (size_t i = 0; i < routeValues.size(); ++i) {
        if (routeValues[i] > 1e-6) {
            const auto& p = routes[i].path;
            for (size_t k = 0; k < p.size() - 1; ++k)
                arcUsage[{p[k], p[k + 1]}] += routeValues[i];
        }
    }

    std::pair<int, int> branchArc = {-1, -1};
    double maxScore = -1.0;
    for (const auto& pair : arcUsage) {
        int u = pair.first.first, v = pair.first.second;
        if (std::find(instance.customers.begin(), instance.customers.end(), u) == instance.customers.end() &&
            std::find(instance.customers.begin(), instance.customers.end(), v) == instance.customers.end())
            continue;

        if (pair.second > 1e-4 && pair.second < 1 - 1e-4) {
            double score = std::min(pair.second, 1 - pair.second);
            if (score > maxScore) {
                maxScore = score;
                branchArc = pair.first;
            }
        }
    }

    bool isInteger = true;
    for (double v : routeValues)
        if (std::abs(v - std::round(v)) >= 1e-5)
            isInteger = false;

    if (branchArc.first == -1) {
        if (isInteger) {
            if (lpVal < upperBound) {
                std::vector<std::pair<Route, double>> newIntRoutes;
                for (size_t i = 0; i < routeValues.size(); ++i) {
                    if (routeValues[i] > 1e-6)
                        newIntRoutes.push_back({routes[i], routeValues[i]});
                }
                std::vector<std::pair<int, double>> activeVars;
                for (size_t i = 0; i < routeValues.size(); ++i)
                    if (routeValues[i] > 1e-6)
                        activeVars.push_back({static_cast<int>(i), routeValues[i]});
                auto conflicts = findConflicts(activeVars, routes);
                if (!conflicts.empty()) {
                    for (auto subset : conflicts) {
                        std::sort(subset.begin(), subset.end());
                        if (!addedConflicts.insert(subset).second)
                            throw std::runtime_error(
                                "An existing conflict cut is violated by the integer solution");
                        rmp.addPathConflictCut(subset);
                    }
                    branchAndBound(forbidden, depth, history);
                    return;
                }
                updateIncumbent(lpVal, newIntRoutes, lastSchedule);
            }
            return;
        } else {
            std::cout << "    [Fallback Branching] Executing variable branching...\n";
            int targetIdx = -1;
            for (size_t i = 0; i < routeValues.size(); ++i)
                if (routeValues[i] > 1e-4 && routeValues[i] < 1 - 1e-4) {
                    targetIdx = i;
                    break;
                }
            if (targetIdx != -1) {
                rmp.vars[targetIdx].set(GRB_DoubleAttr_UB, 0.0);
                auto h0 = history;
                h0.push_back("var_0");
                branchAndBound(forbidden, depth + 1, h0);
                rmp.vars[targetIdx].set(GRB_DoubleAttr_UB, GRB_INFINITY);

                rmp.vars[targetIdx].set(GRB_DoubleAttr_LB, 1.0);
                auto h1 = history;
                h1.push_back("var_1");
                branchAndBound(forbidden, depth + 1, h1);
                rmp.vars[targetIdx].set(GRB_DoubleAttr_LB, 0.0);
            }
            return;
        }
    }

    int u = branchArc.first, v = branchArc.second;

    std::set<std::pair<int, int>> forcedOut, forcedIn;
    if (std::find(instance.customers.begin(), instance.customers.end(), u) != instance.customers.end()) {
        for (int n : instance.nodes)
            if (n != v)
                forcedOut.insert({u, n});
    }
    if (std::find(instance.customers.begin(), instance.customers.end(), v) != instance.customers.end()) {
        for (int n : instance.nodes)
            if (n != u)
                forcedIn.insert({n, v});
    }

    std::set<std::pair<int, int>> branch1Forb = forbidden;
    branch1Forb.insert(forcedOut.begin(), forcedOut.end());
    branch1Forb.insert(forcedIn.begin(), forcedIn.end());
    auto h1 = history;
    h1.push_back("arc_1");
    branchAndBound(branch1Forb, depth + 1, h1);

    std::set<std::pair<int, int>> branch0Forb = forbidden;
    branch0Forb.insert(branchArc);
    auto h0 = history;
    h0.push_back("arc_0");
    branchAndBound(branch0Forb, depth + 1, h0);
}
