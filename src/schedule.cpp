// 固定候选路线后联合检查全部车辆的补能安排，使用增量式分段线性充电。
// 每个站点仍为一个充电桩和一个换电设备；Unknown 不作为不可行证明。
#include "schedule.h"
#include "gurobi_c++.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>

namespace {
constexpr double checkTolerance = 1e-5;

bool contains(const std::vector<int>& nodes, int node) {
    return std::find(nodes.begin(), nodes.end(), node) != nodes.end();
}

double chargingTime(const Instance& instance, int node, double energy) {
    const auto& points = instance.chargingFunctions.at(node).points;
    energy = std::max(points.front().first, std::min(points.back().first, energy));
    for (size_t i = 1; i < points.size(); ++i) {
        if (energy <= points[i].first) {
            double fraction = (energy - points[i - 1].first) / (points[i].first - points[i - 1].first);
            return points[i - 1].second + fraction * (points[i].second - points[i - 1].second);
        }
    }
    return points.back().second;
}

bool validateSchedule(const Instance& instance, const ScheduleResult& result) {
    struct Occupation {
        int node;
        double start, finish;
    };
    std::vector<Occupation> occupations;
    for (const auto& route : result.visits) {
        for (size_t p = 0; p < route.size(); ++p) {
            const ScheduledVisit& visit = route[p];
            int node = visit.node;
            if (!std::isfinite(visit.arrivalTime) || !std::isfinite(visit.startTime) ||
                !std::isfinite(visit.departureTime) || !std::isfinite(visit.arrivalEnergy) ||
                !std::isfinite(visit.departureEnergy))
                return false;
            if (visit.arrivalTime < instance.readyTime[node] - checkTolerance ||
                visit.arrivalTime > instance.dueTime[node] + checkTolerance ||
                visit.startTime < visit.arrivalTime - checkTolerance ||
                visit.departureTime > instance.dueTime[instance.sinkDepot] + checkTolerance ||
                visit.arrivalEnergy < -checkTolerance || visit.departureEnergy < -checkTolerance ||
                visit.arrivalEnergy > instance.batteryCapacity + checkTolerance ||
                visit.departureEnergy > instance.batteryCapacity + checkTolerance)
                return false;
            double duration = instance.serviceTime[node];
            if (contains(instance.chargeStations, node)) {
                if (visit.departureEnergy < visit.arrivalEnergy - checkTolerance)
                    return false;
                duration = chargingTime(instance, node, visit.departureEnergy) -
                           chargingTime(instance, node, visit.arrivalEnergy);
                occupations.push_back({node, visit.startTime, visit.departureTime});
            } else if (contains(instance.swapStations, node)) {
                duration = instance.swapTimes[node];
                if (std::abs(visit.departureEnergy - instance.batteryCapacity) > checkTolerance)
                    return false;
                occupations.push_back({node, visit.startTime, visit.departureTime});
            } else {
                if (std::abs(visit.departureEnergy - visit.arrivalEnergy) > checkTolerance ||
                    std::abs(visit.startTime - visit.arrivalTime) > checkTolerance)
                    return false;
            }
            if (std::abs(visit.departureTime - visit.startTime - duration) > checkTolerance)
                return false;
            if (p > 0) {
                const ScheduledVisit& previous = route[p - 1];
                if (visit.arrivalTime < previous.departureTime + instance.getTravelTime(previous.node, node) -
                                            checkTolerance ||
                    std::abs(visit.arrivalEnergy - previous.departureEnergy +
                             instance.getDistance(previous.node, node) * instance.consumptionRate) >
                        checkTolerance)
                    return false;
            }
        }
    }
    for (size_t a = 0; a < occupations.size(); ++a) {
        for (size_t b = a + 1; b < occupations.size(); ++b) {
            const auto& first = occupations[a];
            const auto& second = occupations[b];

            if (first.node == second.node && first.start < second.finish - checkTolerance &&
                second.start < first.finish - checkTolerance)
                return false;
        }
    }
    return true;
}
} // namespace

ScheduleResult checkGlobalSchedule(
    const Instance& instance, const std::vector<Route>& routes, double timeLimit) {
    ScheduleResult result;
    const auto started = std::chrono::steady_clock::now();
    if (timeLimit <= 0) {
        result.message = "No time remains for global scheduling";
        return result;
    }
    try {

        std::set<int> selectedCustomers;
        for (const Route& route : routes) {
            if (route.path.size() < 3 || route.path.front() != instance.sourceDepot ||
                route.path.back() != instance.sinkDepot ||
                (route.vehicleType != 'C' && route.vehicleType != 'S')) {
                result.message = "Malformed route supplied to global scheduling";
                return result;
            }
            double load = 0;
            for (size_t p = 0; p < route.path.size(); ++p) {
                int node = route.path[p];
                if (node < 0 || node >= instance.nodeCount) {
                    result.message = "Route contains an invalid node";
                    return result;
                }
                if (p > 0 && p + 1 < route.path.size() &&
                    (node == instance.sourceDepot || node == instance.sinkDepot)) {
                    result.message = "Route contains an internal depot";
                    return result;
                }
                if (contains(instance.customers, node)) {
                    load += instance.demand[node];
                    if (!selectedCustomers.insert(node).second) {
                        result.status = ScheduleStatus::Infeasible;
                        result.message = "A customer is visited more than once";
                        return result;
                    }
                }
                if (route.vehicleType == 'C' && contains(instance.swapStations, node)) {
                    result.status = ScheduleStatus::Infeasible;
                    result.message = "A C-type route uses battery swapping";
                    return result;
                }
            }
            if (load > instance.loadCapacity + checkTolerance) {
                result.status = ScheduleStatus::Infeasible;
                result.message = "Route load exceeds capacity";
                return result;
            }
        }
        for (int node : instance.chargeStations) {
            const auto& points = instance.chargingFunctions.at(node).points;
            if (points.size() < 2 || std::abs(points.front().first) > 1e-8 ||
                std::abs(points.back().first - instance.batteryCapacity) > 1e-8)
                throw std::runtime_error("Charging curve must cover [0,Q]");
            for (size_t i = 1; i < points.size(); ++i)
                if (!(points[i].first > points[i - 1].first) || !(points[i].second >= points[i - 1].second))
                    throw std::runtime_error("Invalid charging curve");
        }
        GRBEnv env(true);
        env.set(GRB_IntParam_OutputFlag, 0);
        env.start();
        GRBModel model(env);
        model.set(GRB_IntParam_Threads, 1);
        model.set(GRB_IntParam_DualReductions, 0);
        model.set(GRB_DoubleParam_FeasibilityTol, 1e-8);
        model.set(GRB_DoubleParam_IntFeasTol, 1e-8);
        model.set(GRB_IntParam_SolutionLimit, 1);

        struct VisitVars {
            GRBVar arrival, arrivalEnergy;
            GRBLinExpr start, finish, departureEnergy;
        };
        std::vector<std::vector<VisitVars>> variables(routes.size());
        std::map<int, std::vector<std::pair<int, int>>> equipmentVisits;
        const double horizon = instance.dueTime[instance.sinkDepot];
        for (int r = 0; r < static_cast<int>(routes.size()); ++r) {
            const auto& path = routes[r].path;
            for (int p = 0; p < static_cast<int>(path.size()); ++p) {
                int node = path[p];
                std::string suffix = "_" + std::to_string(r) + "_" + std::to_string(p);
                VisitVars visit;
                visit.arrival = model.addVar(std::max(0.0, instance.readyTime[node]), instance.dueTime[node],
                    0, GRB_CONTINUOUS, "tau" + suffix);
                visit.arrivalEnergy =
                    model.addVar(0, instance.batteryCapacity, 0, GRB_CONTINUOUS, "y" + suffix);
                bool isStation =
                    contains(instance.chargeStations, node) || contains(instance.swapStations, node);
                if (isStation) {
                    visit.start = model.addVar(0, horizon, 0, GRB_CONTINUOUS, "w" + suffix);
                    visit.finish = model.addVar(0, horizon, 0, GRB_CONTINUOUS, "sigma" + suffix);
                    visit.departureEnergy =
                        model.addVar(0, instance.batteryCapacity, 0, GRB_CONTINUOUS, "z" + suffix);
                    model.addConstr(visit.start >= visit.arrival);
                } else {

                    visit.start = visit.arrival;
                    visit.finish = visit.arrival + instance.serviceTime[node];
                    visit.departureEnergy = visit.arrivalEnergy;
                }
                if (contains(instance.chargeStations, node)) {
                    const auto& points = instance.chargingFunctions.at(node).points;
                    int segmentCount = static_cast<int>(points.size()) - 1;
                    std::vector<GRBVar> lambdaArr, lambdaDep, thetaArr, thetaDep;
                    for (int m = 0; m < segmentCount; ++m) {
                        std::string segmentSuffix = suffix + "_" + std::to_string(m + 1);
                        lambdaArr.push_back(
                            model.addVar(0, 1, 0, GRB_CONTINUOUS, "lambdaArr" + segmentSuffix));
                        lambdaDep.push_back(
                            model.addVar(0, 1, 0, GRB_CONTINUOUS, "lambdaDep" + segmentSuffix));
                        thetaArr.push_back(model.addVar(0, 1, 0, GRB_BINARY, "thetaArr" + segmentSuffix));
                        thetaDep.push_back(model.addVar(0, 1, 0, GRB_BINARY, "thetaDep" + segmentSuffix));
                    }

                    std::string terminalSuffix = suffix + "_" + std::to_string(segmentCount + 1);
                    thetaArr.push_back(model.addVar(0, 0, 0, GRB_BINARY, "thetaArr" + terminalSuffix));
                    thetaDep.push_back(model.addVar(0, 0, 0, GRB_BINARY, "thetaDep" + terminalSuffix));

                    GRBLinExpr arrivalEnergy = 0, departureEnergy = 0, chargingDuration = 0;
                    for (int m = 0; m < segmentCount; ++m) {
                        double segmentEnergy = points[m + 1].first - points[m].first;
                        double segmentTime = points[m + 1].second - points[m].second;
                        arrivalEnergy += segmentEnergy * lambdaArr[m];
                        departureEnergy += segmentEnergy * lambdaDep[m];
                        chargingDuration += segmentTime * (lambdaDep[m] - lambdaArr[m]);

                        model.addConstr(lambdaArr[m] >= thetaArr[m + 1]);
                        model.addConstr(lambdaArr[m] <= thetaArr[m]);
                        model.addConstr(lambdaDep[m] >= thetaDep[m + 1]);
                        model.addConstr(lambdaDep[m] <= thetaDep[m]);
                    }
                    model.addConstr(visit.arrivalEnergy == arrivalEnergy);
                    model.addConstr(visit.departureEnergy == departureEnergy);
                    model.addConstr(visit.departureEnergy >= visit.arrivalEnergy);
                    model.addConstr(visit.finish == visit.start + chargingDuration);
                    equipmentVisits[node].push_back({r, p});
                } else if (contains(instance.swapStations, node)) {
                    model.addConstr(visit.departureEnergy == instance.batteryCapacity);
                    model.addConstr(visit.finish == visit.start + instance.swapTimes[node]);
                    equipmentVisits[node].push_back({r, p});
                }

                if (p > 0) {
                    const VisitVars& previous = variables[r].back();
                    model.addConstr(
                        visit.arrival >= previous.finish + instance.getTravelTime(path[p - 1], node));
                    model.addConstr(visit.arrivalEnergy ==
                                    previous.departureEnergy -
                                        instance.getDistance(path[p - 1], node) * instance.consumptionRate);
                }
                variables[r].push_back(visit);
            }
        }
        for (const auto& item : equipmentVisits) {
            const auto& visits = item.second;
            for (size_t a = 0; a < visits.size(); ++a) {
                for (size_t b = a + 1; b < visits.size(); ++b) {
                    const VisitVars& first = variables[visits[a].first][visits[a].second];
                    const VisitVars& second = variables[visits[b].first][visits[b].second];

                    if (visits[a].first == visits[b].first)
                        continue;
                    GRBVar order = model.addVar(0, 1, 0, GRB_BINARY);
                    model.addGenConstrIndicator(order, 1, second.start - first.finish, GRB_GREATER_EQUAL, 0);
                    model.addGenConstrIndicator(order, 0, first.start - second.finish, GRB_GREATER_EQUAL, 0);
                }
            }
        }
        double remaining =
            timeLimit - std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
        if (remaining <= 0) {
            result.message = "Global scheduling time budget exhausted during model construction";
            return result;
        }
        model.set(GRB_DoubleParam_TimeLimit, remaining);
        model.optimize();
        int status = model.get(GRB_IntAttr_Status);
        if (model.get(GRB_IntAttr_SolCount) > 0) {
            result.visits.resize(routes.size());
            for (size_t r = 0; r < routes.size(); ++r) {
                for (size_t p = 0; p < variables[r].size(); ++p) {
                    const VisitVars& visit = variables[r][p];
                    result.visits[r].push_back({routes[r].path[p], visit.arrival.get(GRB_DoubleAttr_X),
                        visit.start.getValue(), visit.finish.getValue(),
                        visit.arrivalEnergy.get(GRB_DoubleAttr_X), visit.departureEnergy.getValue()});
                }
            }
            if (validateSchedule(instance, result)) {
                result.status = ScheduleStatus::Feasible;
                result.message = "All selected routes have a common feasible schedule";
            } else {
                result.visits.clear();
                result.message = "Extracted schedule failed numerical validation";
            }
        } else if (status == GRB_INFEASIBLE) {
            result.status = ScheduleStatus::Infeasible;
            result.message = "Selected routes are jointly infeasible";
        } else {
            result.message = "Global scheduling unresolved; Gurobi status " + std::to_string(status);
        }
    } catch (const GRBException& error) {
        result.message = "Global scheduling Gurobi error " + std::to_string(error.getErrorCode()) + ": " +
                         error.getMessage();
    } catch (const std::exception& error) {
        result.message = std::string("Global scheduling error: ") + error.what();
    }
    return result;
}
