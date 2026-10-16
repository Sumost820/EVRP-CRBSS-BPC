// 受限主问题：客户覆盖、车队、列、人工变量和分支界。
#include "master.h"
#include <iostream>
#include <algorithm>
#include <cmath>

MasterProblem::MasterProblem(const Instance& instance) : instance(instance) {
    try {
        environment = std::make_unique<GRBEnv>(true);
        environment->set("OutputFlag", "0");
        environment->set("Threads", "1");

        environment->set("NodefileStart", "0.5");
        environment->start();

        model = std::make_unique<GRBModel>(*environment);

        for (int c : instance.customers) {
            std::string vname = "art_" + std::to_string(c);
            coverArts[c] = model->addVar(0.0, GRB_INFINITY, 1e6, GRB_CONTINUOUS, vname);
        }
        model->update();

        for (int c : instance.customers) {
            std::string cname = "cust_" + std::to_string(c);
            constrsCust[c] = model->addConstr(coverArts[c] == 1.0, cname);
        }

        constrsFleet['S'] = model->addConstr(GRBLinExpr(0.0) <= instance.swapVehicleCount, "fleet_S");
        constrsFleet['C'] = model->addConstr(GRBLinExpr(0.0) <= instance.chargeVehicleCount, "fleet_C");

        model->update();
    } catch (GRBException& e) {
        throw; // 由运行入口统一记录错误并返回失败状态。
    }
}

MasterProblem::~MasterProblem() {}

void MasterProblem::addPathConflictCut(const std::vector<int>& routeIndices) {
    GRBLinExpr expr = 0;
    for (int idx : routeIndices) {
        expr += vars[idx];
    }
    std::string cname = "conflict_cut_" + std::to_string(conflictCuts.size());
    GRBConstr constr = model->addConstr(expr <= routeIndices.size() - 1, cname);
    conflictCuts.push_back(constr);
    model->update();
}

void MasterProblem::addColumns(const std::vector<Route>& routeInfos) {
    for (const auto& rData : routeInfos) {
        const auto& path = rData.path;
        char vehicleType = rData.vehicleType;
        double cost = rData.cost;

        if (std::isnan(cost) || std::isinf(cost)) {
            cost = 1e9;
        }

        std::set<std::pair<int, int>> edges;
        for (size_t k = 0; k < path.size() - 1; ++k) {
            edges.insert({path[k], path[k + 1]});
        }

        bool containsForbidden = false;
        for (const auto& edge : edges) {
            if (curForbidden.count(edge)) {
                containsForbidden = true;
                break;
            }
        }
        double ub = containsForbidden ? 0.0 : GRB_INFINITY;

        GRBColumn col;
        for (int node : path) {
            if (std::find(instance.customers.begin(), instance.customers.end(), node) !=
                instance.customers.end()) {
                if (constrsCust.find(node) != constrsCust.end()) {
                    col.addTerm(1.0, constrsCust[node]);
                }
            }
        }

        if (constrsFleet.find(vehicleType) != constrsFleet.end()) {
            col.addTerm(1.0, constrsFleet[vehicleType]);
        }

        for (const auto& cut : subsetRowCuts) {
            int count = 0;
            for (int node : path) {
                if (cut.customerSet.count(node))
                    count++;
            }
            if (count >= 2) {
                col.addTerm(count / 2, cut.constr);
            }
        }

        for (const auto& cut : capacityCuts) {
            int crossCount = 0;
            for (size_t i = 0; i < path.size() - 1; ++i) {
                if (cut.customerSet.count(path[i]) && !cut.customerSet.count(path[i + 1])) {
                    crossCount++;
                }
            }
            if (crossCount > 0) {
                col.addTerm(crossCount, cut.constr);
            }
        }

        try {
            GRBVar var = model->addVar(0.0, ub, cost, GRB_CONTINUOUS, col, "");
            routes.push_back(rData);
            vars.push_back(var);
            colEdges.push_back(edges);
        } catch (GRBException& e) {
        }
    }
    model->update();
}

void MasterProblem::setBranching(const std::set<std::pair<int, int>>& forbiddenEdges) {
    std::set<std::pair<int, int>> newForbidden = forbiddenEdges;

    std::vector<std::pair<int, int>> toUnforbidVec;
    std::set_difference(curForbidden.begin(), curForbidden.end(), newForbidden.begin(), newForbidden.end(),
        std::back_inserter(toUnforbidVec));
    std::set<std::pair<int, int>> toUnforbid(toUnforbidVec.begin(), toUnforbidVec.end());

    std::vector<std::pair<int, int>> toForbidVec;
    std::set_difference(newForbidden.begin(), newForbidden.end(), curForbidden.begin(), curForbidden.end(),
        std::back_inserter(toForbidVec));
    std::set<std::pair<int, int>> toForbid(toForbidVec.begin(), toForbidVec.end());

    if (!toUnforbid.empty()) {
        for (size_t idx = 0; idx < colEdges.size(); ++idx) {
            bool hasUnforbid = false;
            for (const auto& edge : toUnforbid) {
                if (colEdges[idx].count(edge)) {
                    hasUnforbid = true;
                    break;
                }
            }
            if (hasUnforbid) {
                bool hasNewForbidden = false;
                for (const auto& edge : newForbidden) {
                    if (colEdges[idx].count(edge)) {
                        hasNewForbidden = true;
                        break;
                    }
                }
                if (!hasNewForbidden) {
                    vars[idx].set(GRB_DoubleAttr_UB, GRB_INFINITY);
                }
            }
        }
    }

    if (!toForbid.empty()) {
        for (size_t idx = 0; idx < colEdges.size(); ++idx) {
            bool hasForbid = false;
            for (const auto& edge : toForbid) {
                if (colEdges[idx].count(edge)) {
                    hasForbid = true;
                    break;
                }
            }
            if (hasForbid) {
                vars[idx].set(GRB_DoubleAttr_UB, 0.0);
            }
        }
    }

    curForbidden = newForbidden;
    model->update();
}

MasterSolution MasterProblem::solve() {
    MasterSolution sol;
    sol.objective = INFINITY;

    try {
        model->optimize();
        int status = model->get(GRB_IntAttr_Status);

        if (status == GRB_OPTIMAL || status == GRB_SUBOPTIMAL) {
            sol.objective = model->get(GRB_DoubleAttr_ObjVal);
            for (const auto& item : coverArts) {
                sol.artificialSum += std::max(0.0, item.second.get(GRB_DoubleAttr_X));
            }

            for (auto& var : vars) {
                sol.routeValues.push_back(var.get(GRB_DoubleAttr_X));
            }
            sol.routes = routes;

            for (int c : instance.customers) {
                sol.customerDuals[c] = constrsCust[c].get(GRB_DoubleAttr_Pi);
            }
            sol.fleetDuals['S'] = constrsFleet['S'].get(GRB_DoubleAttr_Pi);
            sol.fleetDuals['C'] = constrsFleet['C'].get(GRB_DoubleAttr_Pi);

            for (const auto& cut : subsetRowCuts) {
                sol.subsetRowDuals.push_back(cut.constr.get(GRB_DoubleAttr_Pi));
            }

            for (const auto& cut : capacityCuts) {
                sol.capacityDuals.push_back(cut.constr.get(GRB_DoubleAttr_Pi));
            }
        } else if (status == GRB_NUMERIC) {
            std::cout << "      [WARNING] Gurobi encountered severe numerical issues at this node."
                      << std::endl;
        }
    } catch (GRBException& e) {
    }
    return sol;
}

std::pair<double, std::vector<std::pair<Route, double>>> MasterProblem::solveAsMip() {
    std::vector<std::pair<Route, double>> bestIntRoutes;
    double objective = INFINITY;

    try {
        std::cout << "      [MIP Heuristic] Configuring RMP as a Mixed Integer Program...\n";

        std::vector<double> origLb(vars.size());
        std::vector<double> origUb(vars.size());
        std::vector<char> origVtype(vars.size());

        for (size_t i = 0; i < vars.size(); ++i) {
            origLb[i] = vars[i].get(GRB_DoubleAttr_LB);
            origUb[i] = vars[i].get(GRB_DoubleAttr_UB);
            origVtype[i] = vars[i].get(GRB_CharAttr_VType);

            vars[i].set(GRB_CharAttr_VType, GRB_BINARY);
            vars[i].set(GRB_DoubleAttr_LB, 0.0);
            vars[i].set(GRB_DoubleAttr_UB, 1.0);
        }

        model->set("TimeLimit", "60.0");
        model->set("MIPGap", "0.01");
        model->set("OutputFlag", "1");

        model->optimize();

        int status = model->get(GRB_IntAttr_Status);
        if (status == GRB_OPTIMAL || status == GRB_TIME_LIMIT || status == GRB_SUBOPTIMAL) {
            if (model->get(GRB_IntAttr_SolCount) > 0) {
                objective = model->get(GRB_DoubleAttr_ObjVal);
                for (size_t i = 0; i < vars.size(); ++i) {
                    if (vars[i].get(GRB_DoubleAttr_X) > 0.5) {
                        bestIntRoutes.push_back({routes[i], 1.0});
                    }
                }
            }
        }

        for (size_t i = 0; i < vars.size(); ++i) {
            vars[i].set(GRB_CharAttr_VType, origVtype[i]);
            vars[i].set(GRB_DoubleAttr_LB, origLb[i]);
            vars[i].set(GRB_DoubleAttr_UB, origUb[i]);
        }
        model->set("TimeLimit", "GRB_INFINITY");
        model->set("OutputFlag", "0");

    } catch (GRBException& e) {
        std::cerr << "      [MIP Heuristic Error] " << e.getMessage() << std::endl;
    }

    return {objective, bestIntRoutes};
}
