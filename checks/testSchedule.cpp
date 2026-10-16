#include "schedule.h"
#include <iostream>
#include <stdexcept>

static Instance makeCase(const Instance& seed, int nodeCount, double horizon) {
    Instance instance = seed;
    instance.nodeCount = nodeCount;
    instance.sourceDepot = 0; instance.sinkDepot = nodeCount - 1;
    instance.batteryCapacity = 10; instance.loadCapacity = 100; instance.consumptionRate = 1;
    instance.xCoord.assign(nodeCount, 0); instance.yCoord.assign(nodeCount, 0);
    instance.demand.assign(nodeCount, 0); instance.readyTime.assign(nodeCount, 0);
    instance.dueTime.assign(nodeCount, horizon); instance.serviceTime.assign(nodeCount, 0); instance.swapTimes.assign(nodeCount, 0);
    instance.distMatrix.assign(nodeCount * nodeCount, 0);
    instance.timeMatrix.assign(nodeCount * nodeCount, 0);
    instance.chargeStations.clear(); instance.swapStations.clear(); instance.customers.clear();
    instance.chargingFunctions.clear();
    return instance;
}

static void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

static ScheduleResult expect(const Instance& instance, const std::vector<Route>& routes,
    ScheduleStatus expected, const std::string& name) {
    ScheduleResult result = checkGlobalSchedule(instance, routes, 20);
    require(result.status == expected, name + ": " + result.message);
    std::cout << "PASS: " << name << "\n";
    return result;
}

int main(int argc, char* argv[]) {
    try {
        require(argc == 2, "Supply the seed instance path");
        Instance seed(argv[1]);

        Instance order = makeCase(seed, 7, 12);
        order.swapStations = {1}; order.swapTimes[1] = 2;
        order.customers = {2, 3, 4, 5};
        order.readyTime[2] = order.dueTime[2] = 0;
        order.readyTime[4] = order.dueTime[4] = 1;
        order.readyTime[5] = order.dueTime[5] = 3;
        std::vector<Route> orderRoutes = {{{0, 2, 1, 3, 6}, 'S', 0}, {{0, 4, 1, 5, 6}, 'S', 0}};
        auto orderResult = expect(order, orderRoutes, ScheduleStatus::Feasible, "swap ordering may reverse arrival order");
        require(orderResult.visits[0][2].startTime >= orderResult.visits[1][2].departureTime - 1e-5, "B must be served before A");

        Instance joint = makeCase(seed, 9, 3);
        joint.swapStations = {1, 2}; joint.swapTimes[1] = joint.swapTimes[2] = 1;
        joint.customers = {3, 4, 5, 6, 7};
        joint.readyTime[4] = joint.dueTime[4] = 0; joint.readyTime[5] = joint.dueTime[5] = 1;
        joint.readyTime[6] = joint.dueTime[6] = 2; joint.readyTime[7] = joint.dueTime[7] = 3;
        Route through = {{0, 1, 3, 2, 8}, 'S', 0};
        Route blockA = {{0, 4, 1, 5, 8}, 'S', 0};
        Route blockB = {{0, 6, 2, 7, 8}, 'S', 0};
        expect(joint, {through, blockA}, ScheduleStatus::Feasible, "station A projection feasible");
        expect(joint, {through, blockB}, ScheduleStatus::Feasible, "station B projection feasible");
        expect(joint, {through, blockA, blockB}, ScheduleStatus::Infeasible, "independent station feasibility is not joint feasibility");

        Instance parallel = makeCase(seed, 8, 3);
        parallel.chargeStations = {1}; parallel.swapStations = {2}; parallel.swapTimes[2] = 3;
        parallel.chargingFunctions[1] = EnergyFunction({{0, 0}, {5, 1}, {10, 3}}, 10);
        parallel.customers = {3, 4, 5, 6};
        parallel.readyTime[3] = parallel.dueTime[3] = parallel.readyTime[5] = parallel.dueTime[5] = 0;
        parallel.readyTime[4] = parallel.dueTime[4] = parallel.readyTime[6] = parallel.dueTime[6] = 3;
        parallel.distMatrix[0 * 8 + 3] = parallel.distMatrix[0 * 8 + 5] = 10;
        parallel.distMatrix[1 * 8 + 4] = parallel.distMatrix[2 * 8 + 6] = 10;
        Route charge = {{0, 3, 1, 4, 7}, 'C', 0};
        Route swap = {{0, 5, 2, 6, 7}, 'S', 0};
        auto parallelResult = expect(parallel, {charge, swap}, ScheduleStatus::Feasible, "charging and swapping use independent equipment");
        require(std::abs(parallelResult.visits[0][2].startTime) < 1e-5 &&
            std::abs(parallelResult.visits[1][2].startTime) < 1e-5, "Both operations must start at zero");
        parallel.distMatrix[1 * 8 + 6] = 10;
        Route otherCharge = {{0, 5, 1, 6, 7}, 'C', 0};
        expect(parallel, {charge, otherCharge}, ScheduleStatus::Infeasible, "one charger cannot serve two simultaneous charges");

        Instance partial = makeCase(seed, 4, 0.6);
        partial.chargeStations = {1}; partial.customers = {2};
        partial.chargingFunctions[1] = EnergyFunction({{0, 0}, {5, 1}, {10, 3}}, 10);
        partial.distMatrix[0 * 4 + 1] = 6;
        partial.distMatrix[1 * 4 + 2] = 6;
        Route partialRoute = {{0, 1, 2, 3}, 'C', 0};
        auto partialResult = expect(partial, {partialRoute}, ScheduleStatus::Feasible, "nonlinear partial charge crossing a breakpoint");
        require(std::abs(partialResult.visits[0][1].departureEnergy - 6) < 1e-5, "Partial charge must end at energy 6");
        partial.dueTime.assign(4, 0.5);
        expect(partial, {partialRoute}, ScheduleStatus::Infeasible, "insufficient time for required nonlinear charge");

        Instance repeated = makeCase(seed, 4, 2);
        repeated.swapStations = {1}; repeated.swapTimes[1] = 1; repeated.customers = {2};
        expect(repeated, {{{0, 1, 2, 1, 3}, 'S', 0}}, ScheduleStatus::Feasible, "repeated visits to the same device use separate variables");
        auto unknown = checkGlobalSchedule(repeated, {{{0, 1, 2, 1, 3}, 'S', 0}}, 0);
        require(unknown.status == ScheduleStatus::Unknown, "No time must return UNKNOWN, never INFEASIBLE");
        std::cout << "PASS: exhausted checking budget returns UNKNOWN\n";
        std::cout << "All 10 scheduling regression checks passed.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << "\n";
        return 1;
    }
}
