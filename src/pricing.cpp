// 双向标签扩展与拼接。保留原有缩减网络和完整网络定价逻辑。
#include "pricing.h"
#include "timeLimit.h"
#include <queue>
#include <algorithm>
#include <cmath>
#include <iostream>

std::vector<Route> solvePricing(const Instance& instance, const std::map<int, double>& duals,
    const std::vector<double>& dualsSrc, const std::vector<double>& dualsRobust,
    const std::map<char, double>& dualsFleet, const std::set<std::pair<int, int>>& forbidden,
    const std::vector<SRCCutInfo>& subsetRowCuts, const std::vector<RobustCutInfo>& capacityCuts,
    const std::set<int>& criticalNodes, std::set<std::pair<std::vector<int>, char>>& existingSigs,
    const std::set<std::pair<int, int>>* activeArcs) {

    double halfTime = (instance.readyTime[instance.sourceDepot] + instance.dueTime[instance.sinkDepot]) / 2.0;
    std::vector<int> initialCuts(subsetRowCuts.size(), 0);

    double upgradeCost =
        (instance.swapVehicleCost - dualsFleet.at('S')) - (instance.chargeVehicleCost - dualsFleet.at('C'));

    EnergyFunction fInit({{0, 0}, {instance.batteryCapacity, 0.0}}, instance.batteryCapacity);
    double baseCost = instance.chargeVehicleCost - dualsFleet.at('C');
    NodeMask startNg, startUnreach, startCrit;
    startNg.set(instance.sourceDepot);
    startUnreach.set(instance.sourceDepot);
    if (criticalNodes.count(instance.sourceDepot))
        startCrit.set(instance.sourceDepot);

    // 按约化成本、载重依次选择下一个标签，与原队列顺序相同。
    std::priority_queue<std::shared_ptr<ForwardLabel>, std::vector<std::shared_ptr<ForwardLabel>>,
        CompareForward>
        fwWorklist;
    std::map<int, ForwardBuckets> fwBuckets;

    auto startF = std::make_shared<ForwardLabel>(instance.sourceDepot, baseCost, 0.0, fInit, 'C', nullptr,
        startNg, initialCuts, startUnreach, startCrit, 0);
    fwWorklist.push(startF);
    fwBuckets[instance.sourceDepot][0.0].push_back(startF);

    EnergyFunction wInit({{0, instance.dueTime[instance.sinkDepot]},
                             {instance.batteryCapacity, instance.dueTime[instance.sinkDepot]}},
        instance.batteryCapacity);
    NodeMask bwStartNg, bwStartUnreach, bwStartCrit;
    bwStartNg.set(instance.sinkDepot);
    bwStartUnreach.set(instance.sinkDepot);
    if (criticalNodes.count(instance.sinkDepot))
        bwStartCrit.set(instance.sinkDepot);

    std::priority_queue<std::shared_ptr<BackwardLabel>, std::vector<std::shared_ptr<BackwardLabel>>,
        CompareBackward>
        bwWorklist;
    std::map<int, BackwardBuckets> bwBuckets;

    auto startB = std::make_shared<BackwardLabel>(instance.sinkDepot, 0.0, 0.0, wInit, 'C', nullptr,
        bwStartNg, initialCuts, bwStartUnreach, bwStartCrit, 0);
    bwWorklist.push(startB);
    bwBuckets[instance.sinkDepot][0.0].push_back(startB);

    while (!fwWorklist.empty()) {

        auto nowFw = std::chrono::high_resolution_clock::now();
        if (std::chrono::duration<double>(nowFw - solveStartTime).count() > solveTimeLimit) {
            std::cout << "      [Timeout] Forward pricing interrupted. Salvaging partial columns...\n";
            break;
        }

        auto curr = fwWorklist.top();
        fwWorklist.pop();
        double tMin = curr->energyFunc.points.empty() ? 1e9 : curr->energyFunc.points.front().second;
        if (tMin > halfTime)
            continue;

        std::vector<int> targets = instance.customers;
        targets.insert(targets.end(), instance.chargeStations.begin(), instance.chargeStations.end());
        targets.insert(targets.end(), instance.swapStations.begin(), instance.swapStations.end());
        targets.push_back(instance.sinkDepot);

        for (int j : targets) {
            if (j == curr->node)
                continue;
            if (activeArcs && activeArcs->find({curr->node, j}) == activeArcs->end())
                continue;
            if (instance.isInfeasible(curr->node, j))
                continue;

            bool isChargeSwap = (std::find(instance.chargeStations.begin(), instance.chargeStations.end(),
                                     j) != instance.chargeStations.end() ||
                                 std::find(instance.swapStations.begin(), instance.swapStations.end(), j) !=
                                     instance.swapStations.end());
            if (curr->unreachableMask.test(j) && !isChargeSwap)
                continue;
            if (forbidden.find({curr->node, j}) != forbidden.end())
                continue;
            if (curr->load + instance.demand[j] > instance.loadCapacity)
                continue;

            bool jCust = (std::find(instance.customers.begin(), instance.customers.end(), j) !=
                          instance.customers.end());
            if (jCust) {
                if (curr->ngMask.test(j))
                    continue;
                if (criticalNodes.count(j) && curr->criticalMask.test(j))
                    continue;
            }

            double dq = instance.getDistance(curr->node, j) * instance.consumptionRate;
            double dt = instance.getTravelTime(curr->node, j) + instance.serviceTime[curr->node];
            double arrivalEst = curr->energyFunc.query(dq) + dt;
            if (std::isnan(arrivalEst) || arrivalEst > instance.dueTime[j])
                continue;

            double extraC = instance.getDistance(curr->node, j);
            if (duals.count(j))
                extraC -= duals.at(j);
            for (size_t idx = 0; idx < capacityCuts.size(); ++idx) {
                if (capacityCuts[idx].customerSet.count(curr->node) &&
                    !capacityCuts[idx].customerSet.count(j))
                    extraC -= dualsRobust[idx];
            }

            EnergyFunction fNext = shiftAndClipForward(curr->energyFunc, dq, dt, instance.readyTime[j],
                instance.dueTime[j], instance.batteryCapacity);
            if (fNext.points.empty())
                continue;

            char nextVType = curr->vehicleType;
            EnergyFunction curF;
            if (std::find(instance.chargeStations.begin(), instance.chargeStations.end(), j) !=
                instance.chargeStations.end()) {
                curF =
                    chargeAlgorithmUpdate(fNext, instance.chargingFunctions.at(j), instance.batteryCapacity);
            } else if (std::find(instance.swapStations.begin(), instance.swapStations.end(), j) !=
                       instance.swapStations.end()) {
                double tDep = fNext.tMin + instance.swapTimes[j];
                curF =
                    EnergyFunction({{0, tDep}, {instance.batteryCapacity, tDep}}, instance.batteryCapacity);
                extraC += instance.swapCost;

                if (curr->vehicleType == 'C') {
                    nextVType = 'S';
                    extraC += upgradeCost;
                }
            } else {
                curF = fNext;
            }

            std::vector<int> newCuts = curr->cutStatus;
            if (jCust) {
                for (size_t idx = 0; idx < subsetRowCuts.size(); ++idx) {
                    if (subsetRowCuts[idx].customerSet.count(j)) {
                        newCuts[idx]++;
                        if (newCuts[idx] == 2) {
                            extraC -= dualsSrc[idx];
                            newCuts[idx] = 0;
                        }
                    }
                    if (!subsetRowCuts[idx].memorySet.count(j)) {
                        newCuts[idx] = 0;
                    }
                }
            }

            double newCost = curr->cost + extraC;
            NodeMask newUnreach = getUnreachableMaskFw(instance, curr.get(), j, curF);
            NodeMask newNg = curr->ngMask;
            NodeMask newCrit = curr->criticalMask;
            NodeMask newVisit = curr->visitMask;
            if (jCust) {
                newNg.set(j);
                newNg &= instance.ngMasks.at(j);
                newVisit.set(j);
            }
            if (criticalNodes.count(j))
                newCrit.set(j);

            auto newLab = std::make_shared<ForwardLabel>(j, newCost, curr->load + instance.demand[j], curF,
                nextVType, curr, newNg, newCuts, newUnreach, newCrit, newVisit);

            if (!checkIsDominated(newLab.get(), fwBuckets[j], dualsSrc)) {
                addAndFilterLabel(newLab, fwBuckets[j], dualsSrc);
                fwWorklist.push(newLab);
            }
        }
    }

    while (!bwWorklist.empty()) {

        auto nowBw = std::chrono::high_resolution_clock::now();
        if (std::chrono::duration<double>(nowBw - solveStartTime).count() > solveTimeLimit) {
            std::cout << "      [Timeout] Backward pricing interrupted. Salvaging partial columns...\n";
            break;
        }

        auto curr = bwWorklist.top();
        bwWorklist.pop();
        double tMax = curr->energyFunc.points.empty() ? -1e9 : curr->energyFunc.points.back().second;
        if (tMax <= halfTime)
            continue;

        std::vector<int> sources = instance.customers;
        sources.insert(sources.end(), instance.chargeStations.begin(), instance.chargeStations.end());
        sources.insert(sources.end(), instance.swapStations.begin(), instance.swapStations.end());
        sources.push_back(instance.sourceDepot);

        for (int i : sources) {
            if (i == curr->node)
                continue;
            if (activeArcs && activeArcs->find({i, curr->node}) == activeArcs->end())
                continue;
            if (instance.isInfeasible(i, curr->node))
                continue;

            bool isChargeSwap = (std::find(instance.chargeStations.begin(), instance.chargeStations.end(),
                                     i) != instance.chargeStations.end() ||
                                 std::find(instance.swapStations.begin(), instance.swapStations.end(), i) !=
                                     instance.swapStations.end());
            if (curr->unreachableMask.test(i) && !isChargeSwap)
                continue;
            if (forbidden.find({i, curr->node}) != forbidden.end())
                continue;
            if (curr->load + instance.demand[curr->node] > instance.loadCapacity)
                continue;

            bool iCust = (std::find(instance.customers.begin(), instance.customers.end(), i) !=
                          instance.customers.end());
            if (iCust) {
                if (curr->ngMask.test(i))
                    continue;
                if (criticalNodes.count(i) && curr->criticalMask.test(i))
                    continue;
            }

            double dq = instance.getDistance(i, curr->node) * instance.consumptionRate;
            double dt = instance.getTravelTime(i, curr->node) + instance.serviceTime[i];
            double arrivalEst = curr->energyFunc.query(instance.batteryCapacity - dq) - dt;
            if (std::isnan(arrivalEst) || arrivalEst < instance.readyTime[i])
                continue;

            double extraC = instance.getDistance(i, curr->node);
            if (duals.count(curr->node))
                extraC -= duals.at(curr->node);
            for (size_t idx = 0; idx < capacityCuts.size(); ++idx) {
                if (capacityCuts[idx].customerSet.count(i) &&
                    !capacityCuts[idx].customerSet.count(curr->node))
                    extraC -= dualsRobust[idx];
            }

            char nextVType = curr->vehicleType;
            if (std::find(instance.swapStations.begin(), instance.swapStations.end(), curr->node) !=
                instance.swapStations.end()) {
                extraC += instance.swapCost;

                if (curr->vehicleType == 'C') {
                    nextVType = 'S';
                    extraC += upgradeCost;
                }
            }

            EnergyFunction wCharge;
            if (std::find(instance.chargeStations.begin(), instance.chargeStations.end(), curr->node) !=
                instance.chargeStations.end()) {
                wCharge = backwardChargeAlgorithmUpdate(
                    curr->energyFunc, instance.chargingFunctions.at(curr->node), instance.batteryCapacity);
            } else if (std::find(instance.swapStations.begin(), instance.swapStations.end(), curr->node) !=
                       instance.swapStations.end()) {
                double tv = curr->energyFunc.query(instance.batteryCapacity) - instance.swapTimes[curr->node];
                wCharge = EnergyFunction({{0, tv}, {instance.batteryCapacity, tv}}, instance.batteryCapacity);
            } else {
                wCharge = curr->energyFunc;
            }

            EnergyFunction curW = shiftAndClipBackward(wCharge, dq, dt, instance.readyTime[curr->node],
                instance.dueTime[curr->node], instance.batteryCapacity);
            if (curW.points.empty())
                continue;

            std::vector<int> newCuts = curr->cutStatus;
            if (std::find(instance.customers.begin(), instance.customers.end(), curr->node) !=
                instance.customers.end()) {
                for (size_t idx = 0; idx < subsetRowCuts.size(); ++idx) {
                    if (subsetRowCuts[idx].customerSet.count(curr->node)) {
                        newCuts[idx]++;
                        if (newCuts[idx] == 2) {
                            extraC -= dualsSrc[idx];
                            newCuts[idx] = 0;
                        }
                    }
                    if (!subsetRowCuts[idx].memorySet.count(curr->node)) {
                        newCuts[idx] = 0;
                    }
                }
            }

            double newCost = curr->cost + extraC;
            NodeMask newUnreach = getUnreachableMaskBw(instance, curr.get(), i, curW);
            NodeMask newNg = curr->ngMask;
            NodeMask newCrit = curr->criticalMask;
            NodeMask newVisit = curr->visitMask;
            if (iCust) {
                newNg.set(i);
                newNg &= instance.ngMasks.at(i);
                newVisit.set(i);
            }
            if (criticalNodes.count(i))
                newCrit.set(i);

            auto newLab =
                std::make_shared<BackwardLabel>(i, newCost, curr->load + instance.demand[curr->node], curW,
                    nextVType, curr, newNg, newCuts, newUnreach, newCrit, newVisit);

            if (!checkIsDominatedBw(newLab.get(), bwBuckets[i], dualsSrc)) {
                addAndFilterLabelBw(newLab, bwBuckets[i], dualsSrc);
                bwWorklist.push(newLab);
            }
        }
    }

    std::vector<Route> validRoutes;
    double maxDiscount = std::max(0.0, upgradeCost);

    for (int node : instance.nodes) {
        if (fwBuckets.find(node) == fwBuckets.end() || bwBuckets.find(node) == bwBuckets.end())
            continue;
        if (fwBuckets[node].empty() || bwBuckets[node].empty())
            continue;

        std::vector<std::shared_ptr<ForwardLabel>> fwList;
        for (auto& p : fwBuckets[node])
            fwList.insert(fwList.end(), p.second.begin(), p.second.end());
        std::vector<std::shared_ptr<BackwardLabel>> bwList;
        for (auto& p : bwBuckets[node])
            bwList.insert(bwList.end(), p.second.begin(), p.second.end());

        if (fwList.empty() || bwList.empty())
            continue;
        std::sort(
            fwList.begin(), fwList.end(), [](const auto& a, const auto& b) { return a->cost < b->cost; });
        std::sort(
            bwList.begin(), bwList.end(), [](const auto& a, const auto& b) { return a->cost < b->cost; });

        double minBwCost = bwList[0]->cost;
        for (auto& fLab : fwList) {

            if (fLab->cost + minBwCost - maxDiscount > -1e-3)
                break;

            for (auto& bLab : bwList) {
                double crossSrcDiscount = 0.0;
                for (size_t idx = 0; idx < subsetRowCuts.size(); ++idx) {
                    if (fLab->cutStatus[idx] + bLab->cutStatus[idx] >= 2) {

                        if (fLab->cutStatus[idx] < 2 && bLab->cutStatus[idx] < 2) {
                            crossSrcDiscount += dualsSrc[idx];
                        }
                    }
                }
                double total = fLab->cost + bLab->cost - crossSrcDiscount;
                char finalVt = (fLab->vehicleType == 'S' || bLab->vehicleType == 'S') ? 'S' : 'C';

                if (fLab->vehicleType == 'S' && bLab->vehicleType == 'S') {
                    total -= upgradeCost;
                }

                if (total > -1e-3)
                    continue;
                if (fLab->load + bLab->load > instance.loadCapacity)
                    continue;

                NodeMask ngOverlap = fLab->ngMask & bLab->ngMask;
                ngOverlap.reset(node);
                if (ngOverlap.any())
                    continue;

                bool feasible = false;
                if (!fLab->energyFunc.points.empty() && !bLab->energyFunc.points.empty()) {
                    std::vector<double> allY;
                    for (auto& p : fLab->energyFunc.points)
                        allY.push_back(p.first);
                    for (auto& p : bLab->energyFunc.points)
                        allY.push_back(p.first);
                    std::sort(allY.begin(), allY.end());
                    allY.erase(std::unique(allY.begin(), allY.end(),
                                   [](double a, double b) { return std::abs(a - b) < 1e-6; }),
                        allY.end());
                    for (double y : allY) {
                        double fv = fLab->energyFunc.query(y);
                        double wv = bLab->energyFunc.query(y);
                        if (!std::isnan(fv) && !std::isnan(wv) && fv <= wv + 1e-6) {
                            feasible = true;
                            break;
                        }
                    }
                }
                if (!feasible)
                    continue;

                std::vector<int> fPath = fLab->getPath();
                std::vector<int> bPath = bLab->getPath();
                std::vector<int> full = fPath;
                if (bPath.size() > 1)
                    full.insert(full.end(), bPath.begin() + 1, bPath.end());

                if (existingSigs.find({full, finalVt}) == existingSigs.end()) {
                    double trueCost =
                        (finalVt == 'S' ? instance.swapVehicleCost : instance.chargeVehicleCost);
                    for (size_t k = 0; k < full.size() - 1; ++k)
                        trueCost += instance.getDistance(full[k], full[k + 1]);
                    for (int n : full) {
                        if (std::find(instance.swapStations.begin(), instance.swapStations.end(), n) !=
                            instance.swapStations.end())
                            trueCost += instance.swapCost;
                    }

                    validRoutes.push_back({full, finalVt, trueCost});
                    existingSigs.insert({full, finalVt});

                    if (validRoutes.size() >= 2000) {
                        return validRoutes;
                    }
                }
            }
        }
    }
    return validRoutes;
}
