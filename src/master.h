#ifndef RMP_H
#define RMP_H

#include <vector>
#include <map>
#include <set>
#include <memory>
#include <tuple>
#include "gurobi_c++.h"
#include "instance.h"
#include "pricing.h"

struct MasterSolution {
    double objective;
    double artificialSum = 0.0;
    std::vector<double> routeValues;
    std::vector<Route> routes;
    std::map<int, double> customerDuals;
    std::map<char, double> fleetDuals;
    std::vector<double> subsetRowDuals;
    std::vector<double> capacityDuals;
};

struct SubsetRowCut {
    std::set<int> customerSet;
    std::set<int> memorySet;
    GRBConstr constr;
};

struct CapacityCut {
    std::set<int> customerSet;
    int rhs;
    GRBConstr constr;
};

class MasterProblem {
public:
    MasterProblem(const Instance& instance);
    ~MasterProblem();

    void addPathConflictCut(const std::vector<int>& routeIndices);
    void addColumns(const std::vector<Route>& routeInfos);
    void setBranching(const std::set<std::pair<int, int>>& forbiddenEdges);

    MasterSolution solve();
    bool separateSubsetRowCuts(const std::vector<double>& varsX, const std::vector<Route>& currentRoutes,
        bool checkOnlyPool = false);

    bool separateCapacityCuts(const std::vector<double>& varsX, const std::vector<Route>& currentRoutes);

    std::pair<double, std::vector<std::pair<Route, double>>> solveAsMip();

    std::vector<Route> routes;
    std::vector<GRBVar> vars;
    std::vector<SubsetRowCut> subsetRowCuts;
    std::vector<CapacityCut> capacityCuts;

private:
    const Instance& instance;

    std::unique_ptr<GRBEnv> environment;
    std::unique_ptr<GRBModel> model;

    std::map<int, GRBVar> coverArts;
    std::map<int, GRBConstr> constrsCust;
    std::map<char, GRBConstr> constrsFleet;
    std::vector<GRBConstr> conflictCuts;

    std::vector<std::set<std::pair<int, int>>> colEdges;
    std::set<std::pair<int, int>> curForbidden;

    std::set<std::set<int>> globalSrcPool;
};

#endif
