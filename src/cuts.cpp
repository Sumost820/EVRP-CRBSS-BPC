// lm-SRC 与 CVRPSEP 容量割分离。
#include "master.h"
#include <algorithm>
#include <cmath>
#include <iostream>

namespace {

bool isSetFeasibleTw(const std::vector<int>& customerSet, const Instance& instance) {
    double totalD = 0;
    for (int c : customerSet)
        totalD += instance.demand[c];
    if (totalD > instance.loadCapacity)
        return false;

    std::vector<int> perm = customerSet;
    std::sort(perm.begin(), perm.end());
    do {
        bool feas = true;
        double t = instance.readyTime[instance.sourceDepot];
        int curr = instance.sourceDepot;
        for (int next : perm) {
            t = std::max(
                t + instance.time.at({curr, next}) + instance.serviceTime[curr], instance.readyTime[next]);
            if (t > instance.dueTime[next]) {
                feas = false;
                break;
            }
            curr = next;
        }
        if (feas) {
            t = std::max(t + instance.time.at({curr, instance.sinkDepot}) + instance.serviceTime[curr],
                instance.readyTime[instance.sinkDepot]);
            if (t <= instance.dueTime[instance.sinkDepot]) {
                return true;
            }
        }
    } while (std::next_permutation(perm.begin(), perm.end()));

    return false;
}
} // namespace

bool MasterProblem::separateSubsetRowCuts(
    const std::vector<double>& varsX, const std::vector<Route>& currentRoutes, bool checkOnlyPool) {
    std::vector<std::pair<double, int>> activeVars;
    bool isFractional = false;

    for (size_t i = 0; i < varsX.size(); ++i) {
        if (varsX[i] > 1e-4) {
            activeVars.push_back({varsX[i], static_cast<int>(i)});
            if (varsX[i] > 1e-4 && varsX[i] < 1.0 - 1e-4) {
                isFractional = true;
            }
        }
    }

    if (!isFractional)
        return false;

    std::vector<std::pair<double, std::set<int>>> customerPaths;
    for (const auto& item : activeVars) {
        double xVal = item.first;
        int rIdx = item.second;
        std::set<int> custs;
        for (int v : currentRoutes[rIdx].path) {
            if (std::find(instance.customers.begin(), instance.customers.end(), v) !=
                instance.customers.end()) {
                custs.insert(v);
            }
        }
        customerPaths.push_back({xVal, custs});
    }

    std::set<std::set<int>> candidateTriplets = globalSrcPool;

    if (!checkOnlyPool) {
        for (const auto& item : customerPaths) {
            const auto& custs = item.second;
            if (custs.size() >= 2) {
                std::vector<int> custList(custs.begin(), custs.end());
                for (size_t i = 0; i < custList.size(); ++i) {
                    for (size_t j = i + 1; j < custList.size(); ++j) {
                        int u = custList[i];
                        int v = custList[j];
                        for (int w : instance.customers) {
                            if (w != u && w != v) {
                                std::set<int> triplet = {u, v, w};
                                candidateTriplets.insert(triplet);
                            }
                        }
                    }
                }
            }
        }
    }

    std::vector<std::pair<double, std::set<int>>> violatedTriplets;
    for (const auto& triplet : candidateTriplets) {
        double lhs = 0.0;
        for (const auto& item : customerPaths) {
            double xVal = item.first;
            int intersectCount = 0;
            for (int node : item.second) {
                if (triplet.count(node))
                    intersectCount++;
            }
            lhs += xVal * (intersectCount / 2);
        }

        if (lhs > 1.05) {
            bool alreadyActive = false;
            for (const auto& cut : subsetRowCuts) {
                if (cut.customerSet == triplet) {
                    alreadyActive = true;
                    break;
                }
            }
            if (!alreadyActive) {
                violatedTriplets.push_back({lhs, triplet});
            }
        }
    }

    std::sort(violatedTriplets.begin(), violatedTriplets.end(),
        [](const auto& a, const auto& b) { return a.first > b.first; });

    int newCutsCount = 0;
    for (const auto& pair : violatedTriplets) {
        const auto& triplet = pair.second;
        globalSrcPool.insert(triplet);
        std::set<int> memory = triplet;

        for (int u : triplet) {
            if (instance.ngMasks.find(u) != instance.ngMasks.end()) {
                NodeMask mask = instance.ngMasks.at(u);
                for (size_t i = 0; i < instance.nodes.size(); ++i) {
                    if (mask.test(i))
                        memory.insert(i);
                }
            }
        }

        GRBLinExpr expr = 0;
        for (size_t i = 0; i < routes.size(); ++i) {
            int intersect = 0;
            for (int node : routes[i].path) {
                if (triplet.count(node))
                    intersect++;
            }
            if (intersect >= 2) {
                expr += (intersect / 2) * vars[i];
            }
        }

        std::string cname = "lm_src_" + std::to_string(subsetRowCuts.size());
        GRBConstr constr = model->addConstr(expr <= 1.0, cname);

        SubsetRowCut newEntry = {triplet, memory, constr};
        subsetRowCuts.push_back(newEntry);
        newCutsCount++;
    }

    if (newCutsCount > 0) {
        std::string poolType = checkOnlyPool ? "Global Pool" : "Deep Search";
        std::cout << "      [lm-SRc] add " << newCutsCount << " subset-row cuts (" << poolType << ")."
                  << std::endl;
        model->update();
        return true;
    }
    return false;
}

#include "memmod.h"
#include "basegrph.h"
#include "cnstrmgr.h"
#include "capsep.h"
bool MasterProblem::separateCapacityCuts(
    const std::vector<double>& varsX, const std::vector<Route>& currentRoutes) {
    int NoOfCustomers = instance.customers.size();

    std::map<int, int> nodeToSep;
    std::map<int, int> sepToNode;

    for (int i = 0; i < NoOfCustomers; ++i) {
        int origId = instance.customers[i];
        nodeToSep[origId] = i + 1;
        sepToNode[i + 1] = origId;
    }

    int SEP_DEPOT = NoOfCustomers + 1;
    nodeToSep[instance.sourceDepot] = SEP_DEPOT;
    nodeToSep[instance.sinkDepot] = SEP_DEPOT;

    std::vector<int> Demand(SEP_DEPOT + 1, 0);
    for (int i = 1; i <= NoOfCustomers; ++i) {
        Demand[i] = std::round(instance.demand[sepToNode[i]]);
    }

    std::map<std::pair<int, int>, double> undirectedX;

    for (size_t k = 0; k < varsX.size(); ++k) {
        if (varsX[k] > 1e-4) {
            const auto& p = currentRoutes[k].path;
            for (size_t i = 0; i < p.size() - 1; ++i) {
                int u = p[i], v = p[i + 1];

                if (nodeToSep.find(u) == nodeToSep.end() || nodeToSep.find(v) == nodeToSep.end())
                    continue;

                int sepU = nodeToSep[u];
                int sepV = nodeToSep[v];

                int minN = std::min(sepU, sepV);
                int maxN = std::max(sepU, sepV);
                if (minN != maxN) {
                    undirectedX[{minN, maxN}] += varsX[k];
                }
            }
        }
    }

    int EdgeCount = undirectedX.size();
    std::vector<int> EdgeTail(EdgeCount + 1);
    std::vector<int> EdgeHead(EdgeCount + 1);
    std::vector<double> EdgeX(EdgeCount + 1);

    int edgeIdx = 1;
    for (const auto& pair : undirectedX) {
        EdgeTail[edgeIdx] = pair.first.first;
        EdgeHead[edgeIdx] = pair.first.second;
        EdgeX[edgeIdx] = pair.second;
        edgeIdx++;
    }

    int newCutsCount = 0;

    CnstrMgrPointer MyCnstrMgr;
    CMGR_CreateCMgr(&MyCnstrMgr, 100);

    CnstrMgrPointer DummyExistingCuts;
    CMGR_CreateCMgr(&DummyExistingCuts, 100);

    char IntegerAndFeasible;
    double MaxViolation;

    CAPSEP_SeparateCapCuts(NoOfCustomers, Demand.data(), (int)instance.loadCapacity, EdgeCount,
        EdgeTail.data(), EdgeHead.data(), EdgeX.data(), DummyExistingCuts, 30, 0.01, &IntegerAndFeasible,
        &MaxViolation, MyCnstrMgr);

    for (int i = 0; i < MyCnstrMgr->Size; ++i) {

        int listSize = MyCnstrMgr->CPL[i]->IntListSize;
        std::set<int> customerSet;

        for (int j = 1; j <= listSize; ++j) {
            int sepNode = MyCnstrMgr->CPL[i]->IntList[j];
            customerSet.insert(sepToNode[sepNode]);
        }

        double dSum = 0;
        for (int n : customerSet)
            dSum += instance.demand[n];
        int rhsRcc = std::ceil(dSum / instance.loadCapacity);

        bool alreadyActive = false;
        for (const auto& cut : capacityCuts) {
            if (cut.customerSet == customerSet) {
                alreadyActive = true;
                break;
            }
        }

        if (!alreadyActive) {

            GRBLinExpr expr = 0;
            for (size_t r = 0; r < routes.size(); ++r) {
                int crossCount = 0;
                for (size_t k = 0; k < routes[r].path.size() - 1; ++k) {
                    if (customerSet.count(routes[r].path[k]) && !customerSet.count(routes[r].path[k + 1])) {
                        crossCount++;
                    }
                }
                if (crossCount > 0)
                    expr += crossCount * vars[r];
            }

            std::string cname = "cvrpsep_rcc_" + std::to_string(capacityCuts.size());
            GRBConstr constr = model->addConstr(expr >= rhsRcc, cname);
            capacityCuts.push_back({customerSet, rhsRcc, constr});
            newCutsCount++;
        }
    }

    CMGR_FreeMemCMgr(&MyCnstrMgr);
    CMGR_FreeMemCMgr(&DummyExistingCuts);

    if (newCutsCount > 0) {
        std::cout << "      [Robust Summary 1] CVRPSEP successfully injected " << newCutsCount
                  << " advanced RCCs.\n";
        model->update();
        return true;
    }
    return false;
}
