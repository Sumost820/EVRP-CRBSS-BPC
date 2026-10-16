#ifndef INSTANCE_H
#define INSTANCE_H

#include <vector>
#include <map>
#include <set>
#include <string>
#include <bitset>
#include <cstdint>
#include "energy.h"

const int ngSize = 8;
const int maxNodes = 256; // 与原程序相同，包含客户、电站模式节点及两个仓库节点。
typedef std::bitset<maxNodes> NodeMask;

struct Instance {
    double swapCost = 0.1;
    double swapTime = 10;
    double swapVehicleCost = 0.1;
    double chargeVehicleCost = 0;
    int swapVehicleCount = 10;
    int chargeVehicleCount = 10;

    double batteryCapacity = 0, loadCapacity = 0, consumptionRate = 0, inverseChargeRate = 0, speed = 0;

    std::vector<double> xCoord, yCoord, demand, readyTime, dueTime, serviceTime, swapTimes;
    int sourceDepot;
    int sinkDepot;

    std::vector<int> nodes;
    // 同一物理电站构造充电和换电两种节点，节点可在不同路线中重复使用。
    std::vector<int> chargeStations;
    std::vector<int> swapStations;
    std::vector<int> customers;

    std::map<int, std::vector<std::pair<double, double>>> rawChargingData;
    std::map<int, EnergyFunction> chargingFunctions;

    int nodeCount = 0;
    std::vector<double> distMatrix;
    std::vector<double> timeMatrix;
    std::vector<uint8_t> infeasibleMat;

    inline double getDistance(int i, int j) const { return distMatrix[i * nodeCount + j]; }
    inline double getTravelTime(int i, int j) const { return timeMatrix[i * nodeCount + j]; }
    inline bool isInfeasible(int i, int j) const { return infeasibleMat[i * nodeCount + j] == 1; }

    std::map<std::pair<int, int>, double> distance;
    std::map<std::pair<int, int>, double> time;
    std::set<std::pair<int, int>> infeasibleArcs;

    std::map<int, NodeMask> ngMasks;

    Instance(const std::string& dataFile);

private:
    void readData(const std::string& dataFile);
    void preprocessArcs();
    void initializeNgMasks();
};

#endif
