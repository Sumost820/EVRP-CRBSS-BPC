#include "schedule.h"
#include <iostream>
#include <stdexcept>

static Instance makeCase(const Instance& seed, int nodeCount, double horizon) {
    Instance instance = seed;
    instance.chargersPerStation = 1;
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
        parallel.chargersPerStation = 2;
        auto twoChargers = expect(parallel, {charge, otherCharge}, ScheduleStatus::Feasible,
            "two chargers serve two simultaneous charges");
        require(twoChargers.visits[0][2].chargerId != twoChargers.visits[1][2].chargerId,
            "Simultaneous positive charges must use different chargers");
        Route otherSwap = {{0, 3, 2, 4, 7}, 'S', 0};
        parallel.distMatrix[2 * 8 + 4] = 10;
        expect(parallel, {swap, otherSwap}, ScheduleStatus::Infeasible,
            "two chargers do not increase swap capacity");

        Instance multi = makeCase(seed, 10, 3);
        multi.chargeStations = {1}; multi.swapStations = {2}; multi.swapTimes[2] = 3;
        multi.chargingFunctions[1] = EnergyFunction({{0, 0}, {5, 1}, {10, 3}}, 10);
        multi.customers = {3, 4, 5, 6, 7, 8};
        std::vector<Route> threeCharges;
        for (int customer : {3, 5, 7}) {
            multi.readyTime[customer] = multi.dueTime[customer] = 0;
            multi.readyTime[customer + 1] = multi.dueTime[customer + 1] = 3;
            multi.distMatrix[customer] = 10;
            multi.distMatrix[1 * 10 + customer + 1] = 10;
            threeCharges.push_back({{0, customer, 1, customer + 1, 9}, 'C', 0});
        }
        multi.chargersPerStation = 2;
        expect(multi, threeCharges, ScheduleStatus::Infeasible, "two chargers cannot serve three simultaneous charges");
        multi.chargersPerStation = 3;
        expect(multi, threeCharges, ScheduleStatus::Feasible, "three chargers serve three simultaneous charges");
        multi.chargersPerStation = 1000000;
        expect(multi, threeCharges, ScheduleStatus::Feasible, "surplus chargers need no extra assignment variables");
        multi.chargersPerStation = 2;
        auto mixed = threeCharges;
        mixed[2].path[2] = 2; mixed[2].vehicleType = 'S';
        multi.distMatrix[2 * 10 + 8] = 10;
        expect(multi, mixed, ScheduleStatus::Feasible, "two charges and one swap operate simultaneously");

        // The same charger can be reused immediately when a charge ends.
        Instance sequential = parallel;
        sequential.chargersPerStation = 1;
        sequential.dueTime.assign(8, 6);
        sequential.readyTime[3] = sequential.dueTime[3] = 0;
        sequential.readyTime[4] = sequential.dueTime[4] = 3;
        sequential.readyTime[5] = sequential.dueTime[5] = 3;
        sequential.readyTime[6] = sequential.dueTime[6] = 6;
        expect(sequential, {charge, otherCharge}, ScheduleStatus::Feasible, "back-to-back charges reuse one charger");

        Instance zero = parallel;
        zero.chargersPerStation = 1;
        zero.readyTime[5] = zero.dueTime[5] = 1;
        zero.readyTime[6] = zero.dueTime[6] = 1;
        zero.distMatrix[0 * 8 + 5] = zero.distMatrix[1 * 8 + 6] = 0;
        expect(zero, {charge, otherCharge}, ScheduleStatus::Feasible,
            "zero-duration charge inside another charge consumes no capacity");

        Instance separate = parallel;
        separate.chargersPerStation = 1;
        separate.swapStations.clear(); separate.chargeStations = {1, 2};
        separate.chargingFunctions[2] = separate.chargingFunctions.at(1);
        Route remoteCharge = {{0, 5, 2, 6, 7}, 'C', 0};
        expect(separate, {charge, remoteCharge}, ScheduleStatus::Feasible,
            "charger capacity is independent at each station");

        for (int invalid : {0, -1}) {
            bool rejected = false;
            try { Instance invalidInstance(argv[1], invalid); }
            catch (const std::invalid_argument&) { rejected = true; }
            require(rejected, "Instance must reject nonpositive charger counts");
        }
        multi.chargersPerStation = 0;
        expect(multi, threeCharges, ScheduleStatus::Unknown, "invalid capacity is not an infeasibility proof");

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
        std::cout << "All scheduling regression checks passed.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << "\n";
        return 1;
    }
}
