#pragma once
#include "instance.h"
#include "master.h"
#include "pricing.h"
#include "schedule.h"
#include "timeLimit.h"
#include <string>
#include <vector>
#include <set>

struct ColumnGenerationResult {
    double objective;
    std::vector<double> routeValues;
    std::vector<Route> routes;
};

// 保存一次求解的状态；算法实现按职责放在对应 cpp 中。
class Solver {
public:
    std::string instanceName;
    Solver(const std::string& dataFile, const std::string& resultDirectory, int chargersPerStation = 1);
    void solve();
    void printRouteDiagnostic(const std::vector<int>& path, char vehicleType);
    bool hasSolution() const { return !bestRoutes.empty(); }
    bool hasStopped() const { return stopped; }

private:
    Instance instance;
    MasterProblem rmp;
    double upperBound;
    double lowerBound = 0.0;
    std::vector<std::pair<Route, double>> bestRoutes;
    ScheduleResult lastSchedule, bestSchedule;
    std::set<int> criticalNodes;
    std::set<std::vector<int>> addedConflicts;
    std::chrono::time_point<std::chrono::high_resolution_clock> startTime;
    bool timedOut = false;
    bool stopped = false;
    std::string stopReason;
    double runtime = 0;
    std::string outputDirectory;

    void updateIncumbent(double objective, const std::vector<std::pair<Route, double>>& newRoutes,
        const ScheduleResult& schedule);
    bool checkInitialRoute(const std::vector<int>& path);
    std::vector<Route> createInitialRoutes();
    std::set<std::pair<int, int>> getReducedArcs(
        const std::map<int, double>& duals, const std::set<std::pair<int, int>>& forbidden, int maxNeighbors);
    std::vector<std::vector<int>> findConflicts(
        const std::vector<std::pair<int, double>>& activeVars, const std::vector<Route>& routes);
    ColumnGenerationResult columnGeneration(const std::set<std::pair<int, int>>& forbidden);
    void branchAndBound(std::set<std::pair<int, int>> forbidden, int depth, std::vector<std::string> history);
    void printRouteDetails(const Route& route, const std::vector<ScheduledVisit>& visits);
    void printSummary();
    void saveResult();
};
