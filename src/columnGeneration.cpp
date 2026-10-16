// 列生成：主问题求解、割分离、联合调度检查、分层定价及 DSSR 更新。
#include "solver.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <numeric>

std::set<std::pair<int, int>> Solver::getReducedArcs(
    const std::map<int, double>& duals, const std::set<std::pair<int, int>>& forbidden, int maxNeighbors) {
    std::set<std::pair<int, int>> reducedArcs;
    std::map<int, std::vector<std::pair<int, double>>> outRanks, inRanks;

    std::vector<int> iNodes = {instance.sourceDepot};
    iNodes.insert(iNodes.end(), instance.chargeStations.begin(), instance.chargeStations.end());
    iNodes.insert(iNodes.end(), instance.swapStations.begin(), instance.swapStations.end());
    iNodes.insert(iNodes.end(), instance.customers.begin(), instance.customers.end());

    std::vector<int> jNodes = instance.chargeStations;
    jNodes.insert(jNodes.end(), instance.swapStations.begin(), instance.swapStations.end());
    jNodes.insert(jNodes.end(), instance.customers.begin(), instance.customers.end());
    jNodes.push_back(instance.sinkDepot);

    for (int i : iNodes) {
        for (int j : jNodes) {
            if (i == j || instance.infeasibleArcs.count({i, j}) || forbidden.count({i, j}))
                continue;
            double dVal = duals.count(j) ? duals.at(j) : 0.0;
            double cSwap = std::find(instance.swapStations.begin(), instance.swapStations.end(), j) !=
                                   instance.swapStations.end()
                               ? instance.swapCost
                               : 0.0;
            double rc = instance.getDistance(i, j) - dVal + cSwap;
            outRanks[i].push_back({j, rc});
            inRanks[j].push_back({i, rc});
        }
    }

    for (int i : iNodes) {
        std::sort(outRanks[i].begin(), outRanks[i].end(),
            [](const auto& a, const auto& b) { return a.second < b.second; });
        int limit = std::min(maxNeighbors, (int)outRanks[i].size());
        for (int k = 0; k < limit; ++k)
            reducedArcs.insert({i, outRanks[i][k].first});
    }
    for (int j : jNodes) {
        std::sort(inRanks[j].begin(), inRanks[j].end(),
            [](const auto& a, const auto& b) { return a.second < b.second; });
        int limit = std::min(maxNeighbors, (int)inRanks[j].size());
        for (int k = 0; k < limit; ++k)
            reducedArcs.insert({inRanks[j][k].first, j});
    }

    return reducedArcs;
}

ColumnGenerationResult Solver::columnGeneration(const std::set<std::pair<int, int>>& forbidden) {
    while (true) {
        checkTimeLimit();
        MasterSolution res;

        // 新的割会改变主问题解，先重复求解，随后使用最新对偶价定价。
        while (true) {
            auto rmpStart = std::chrono::high_resolution_clock::now();
            res = rmp.solve();
            auto rmpEnd = std::chrono::high_resolution_clock::now();
            std::cout << "    [Debug] rmp.solve took: " << std::fixed << std::setprecision(6)
                      << std::chrono::duration<double>(rmpEnd - rmpStart).count() << " s\n";

            if (res.objective == INFINITY)
                return {INFINITY, {}, {}};

            if (res.artificialSum > 1e-6) {
                std::cout << "      [RMP] Artificial coverage remaining: " << res.artificialSum
                          << "; continuing pricing.\n";
                break;
            }

            std::vector<std::pair<int, double>> activeVarsInfo;
            bool isInteger = true;
            for (size_t i = 0; i < res.routeValues.size(); ++i) {
                if (res.routeValues[i] > 1e-6) {
                    activeVarsInfo.push_back({(int)i, res.routeValues[i]});
                    if (std::abs(res.routeValues[i] - std::round(res.routeValues[i])) >= 1e-5)
                        isInteger = false;
                }
            }

            if (rmp.separateSubsetRowCuts(res.routeValues, res.routes, true)) {
                std::cout << "      [Accelerate] Triggering historical SRCs, resolving RMP...\n";
                continue;
            }

            if (rmp.separateCapacityCuts(res.routeValues, res.routes)) {
                std::cout << "      [Accelerate] Triggering Robust Cuts (CVRPSEP), resolving RMP...\n";
                continue;
            }

            if (isInteger && res.objective < upperBound) {
                auto conflictStart = std::chrono::high_resolution_clock::now();
                auto conflictingSubsets = findConflicts(activeVarsInfo, res.routes);
                auto conflictEnd = std::chrono::high_resolution_clock::now();
                double conflictTime = std::chrono::duration<double>(conflictEnd - conflictStart).count();

                std::cout << "      [Profile] Global schedule checking took: " << std::fixed
                          << std::setprecision(6) << conflictTime << " s\n";

                if (conflictingSubsets.empty()) {
                    std::vector<std::pair<Route, double>> newIntRoutes;
                    for (const auto& item : activeVarsInfo) {
                        newIntRoutes.push_back({res.routes[item.first], item.second});
                    }
                    updateIncumbent(res.objective, newIntRoutes, lastSchedule);
                    break;
                } else {
                    bool conflictFound = false;
                    for (auto subset : conflictingSubsets) {
                        std::sort(subset.begin(), subset.end());
                        if (addedConflicts.find(subset) == addedConflicts.end()) {
                            rmp.addPathConflictCut(subset);
                            addedConflicts.insert(subset);
                            conflictFound = true;
                        }
                    }
                    if (conflictFound) {
                        std::cout << "      [Conflict] Added conflict cuts, re-solving RMP...\n";
                        continue;
                    } else
                        throw std::runtime_error("An existing conflict cut is violated by the RMP solution");
                }
            } else {
                break;
            }
        }

        std::set<std::pair<std::vector<int>, char>> existingSigs;
        for (const auto& r : res.routes)
            existingSigs.insert({r.path, r.vehicleType});

        std::vector<SRCCutInfo> pricingCuts;
        for (const auto& cut : rmp.subsetRowCuts) {
            pricingCuts.push_back({cut.customerSet, cut.memorySet});
        }

        std::vector<RobustCutInfo> pricingRobustCuts;
        for (const auto& cut : rmp.capacityCuts) {
            pricingRobustCuts.push_back({cut.customerSet, cut.rhs});
        }

        auto pricingStart = std::chrono::high_resolution_clock::now();
        std::vector<Route> newLabels;

        struct PricingStrategy {
            int maxArcs;
            bool useCuts;
            std::string name;
        };

        // 先搜索缩减网络；未找到列时再搜索完整保留网络。
        std::vector<PricingStrategy> strategies = {{6, true, "L1 Aggressive (6 arcs, with cuts)"},
            {-1, true, "L3 Exact      (Full graph, with cuts)"}};

        int triggeredLevel = -1;

        for (size_t i = 0; i < strategies.size(); ++i) {
            const auto& strat = strategies[i];
            std::cout << "      [Pricing] Running " << strat.name << "...\n";

            std::set<std::pair<int, int>> activeArcs;
            const std::set<std::pair<int, int>>* arcsPtr = nullptr;
            if (strat.maxArcs != -1) {
                activeArcs = getReducedArcs(res.customerDuals, forbidden, strat.maxArcs);
                arcsPtr = &activeArcs;
            }

            const auto& currentCuts = strat.useCuts ? pricingCuts : std::vector<SRCCutInfo>();
            const auto& currentRobust = strat.useCuts ? pricingRobustCuts : std::vector<RobustCutInfo>();

            std::vector<double> currentDSrc = strat.useCuts ? res.subsetRowDuals : std::vector<double>();
            std::vector<double> currentDRobust = strat.useCuts ? res.capacityDuals : std::vector<double>();

            newLabels = solvePricing(instance, res.customerDuals, currentDSrc, currentDRobust, res.fleetDuals,
                forbidden, currentCuts, currentRobust, criticalNodes, existingSigs, arcsPtr);

            if (!newLabels.empty()) {
                triggeredLevel = i + 1;
                std::cout << "      [Heuristic] Found columns at Level " << triggeredLevel << ".\n";
                break;
            }
        }

        auto pricingEnd = std::chrono::high_resolution_clock::now();
        std::cout << "Found " << newLabels.size() << " improving routes, current Pricing took: " << std::fixed
                  << std::setprecision(2) << std::chrono::duration<double>(pricingEnd - pricingStart).count()
                  << " s\n";

        // 非初等路线用于更新关键客户集合，不直接加入主问题。
        if (!newLabels.empty()) {
            std::vector<Route> cleanLabels;
            std::set<int> cycleNodes;

            for (const auto& r : newLabels) {
                std::set<int> visitedCusts;
                bool hasCycle = false;
                for (int u : r.path) {
                    if (std::find(instance.customers.begin(), instance.customers.end(), u) !=
                        instance.customers.end()) {
                        if (visitedCusts.count(u)) {
                            cycleNodes.insert(u);
                            hasCycle = true;
                        }
                        visitedCusts.insert(u);
                    }
                }
                if (!hasCycle) {
                    cleanLabels.push_back(r);
                }
            }

            if (!cycleNodes.empty()) {
                for (int u : cycleNodes) {
                    for (int i : instance.nodes) {
                        instance.ngMasks[i].set(u);
                    }
                }
                std::cout << "      [DSSR] Detected cycles! Added customer(s) to all ng_masks: ";
                for (int u : cycleNodes)
                    std::cout << u << " ";
                std::cout << "\n";
            }

            if (!cleanLabels.empty()) {
                rmp.addColumns(cleanLabels);
            } else if (!cycleNodes.empty()) {
                std::cout << "      [DSSR] All new columns contained cycles. Retrying Pricing with updated "
                             "NG sets...\n";
                continue;
            }
        } else {
            if (res.artificialSum > 1e-6) {
                throw std::runtime_error(
                    "Pricing returned no new routes, but artificial customer coverage remains. "
                    "The penalized RMP objective cannot certify a bound or original-model infeasibility.");
            }
            if (!rmp.separateSubsetRowCuts(res.routeValues, res.routes, false)) {
                break;
            }
        }
    }

    MasterSolution finalRes = rmp.solve();
    if (finalRes.artificialSum > 1e-6) {
        throw std::runtime_error("Artificial coverage remains after column generation; no valid node bound.");
    }
    return {finalRes.objective, finalRes.routeValues, finalRes.routes};
}
