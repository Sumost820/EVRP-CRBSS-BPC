// 算例读入、电站节点构造、充电曲线缩放及不可行弧预处理。
#include "instance.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <vector>
#include <map>
#include <string>

Instance::Instance(const std::string& dataFile, int chargersPerStation)
    : chargersPerStation(chargersPerStation) {
    if (chargersPerStation < 1)
        throw std::invalid_argument("chargersPerStation must be a positive integer");
    readData(dataFile);
    preprocessArcs();
    initializeNgMasks();
}

void Instance::readData(const std::string& dataFile) {
    std::ifstream file(dataFile);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open file: " + dataFile);
    }

    std::vector<double> rawXc, rawYc, rawD, rawE, rawL, rawS;
    std::vector<int> stationIndices;
    std::vector<std::vector<double>> rawLens, rawSlps;

    batteryCapacity = 79.69;
    inverseChargeRate = 3.39;
    loadCapacity = 200.0;
    consumptionRate = 1.0;
    speed = 1.0;

    std::string line;
    bool meetEmpty = false;
    bool firstLine = true;

    while (std::getline(file, line)) {
        if (firstLine) {
            firstLine = false;
            continue;
        }

        line.erase(line.begin(),
            std::find_if(line.begin(), line.end(), [](unsigned char ch) { return !std::isspace(ch); }));
        if (line.empty()) {
            meetEmpty = true;
            continue;
        }

        if (!meetEmpty) {
            std::istringstream iss(line);
            std::string id, typeStr;
            double x, y, dVal, eVal, lVal, sVal;

            if (!(iss >> id >> typeStr >> x >> y >> dVal >> eVal >> lVal >> sVal))
                continue;

            bool isStat = (typeStr == "f");
            rawXc.push_back(x);
            rawYc.push_back(y);
            rawD.push_back(dVal);
            rawE.push_back(eVal);
            rawL.push_back(lVal);
            rawS.push_back(sVal);

            if (isStat) {
                stationIndices.push_back(rawXc.size() - 1);
                std::vector<double> lList, sList;

                size_t start = line.find('[');

                if (start == std::string::npos) {
                    std::string nextLine;
                    if (std::getline(file, nextLine)) {
                        line += " " + nextLine;
                        start = line.find('[');
                    }
                }

                size_t end = line.find(']');
                if (start != std::string::npos && end != std::string::npos) {
                    std::string listStr = line.substr(start + 1, end - start - 1);
                    size_t pos = 0;
                    double prev = 0.0;
                    while ((pos = listStr.find('(')) != std::string::npos) {
                        size_t pos2 = listStr.find(')', pos);
                        if (pos2 == std::string::npos)
                            break;
                        std::string tupleStr = listStr.substr(pos + 1, pos2 - pos - 1);
                        size_t comma = tupleStr.find(',');
                        double lv = std::stod(tupleStr.substr(0, comma));
                        double slp = std::stod(tupleStr.substr(comma + 1));

                        lList.push_back(lv - prev);
                        sList.push_back(slp);
                        prev = lv;
                        listStr = listStr.substr(pos2 + 1);
                    }
                }
                rawLens.push_back(lList);
                rawSlps.push_back(sList);
            }
        } else {
            size_t start = line.find('/');
            if (start != std::string::npos) {
                size_t end = line.find('/', start + 1);
                if (end != std::string::npos) {
                    double val = std::stod(line.substr(start + 1, end - start - 1));
                    if (line.find("Q") == 0)
                        batteryCapacity = val;
                    else if (line.find("C") == 0)
                        loadCapacity = val;
                    else if (line.find("r") == 0)
                        consumptionRate = val;
                    else if (line.find("g") == 0)
                        inverseChargeRate = val;
                    else if (line.find("v") == 0)
                        speed = val;
                }
            }
        }
    }

    xCoord.push_back(rawXc[0]);
    yCoord.push_back(rawYc[0]);
    demand.push_back(rawD[0]);
    readyTime.push_back(rawE[0]);
    dueTime.push_back(rawL[0]);
    serviceTime.push_back(rawS[0]);
    swapTimes.push_back(0.0);
    sourceDepot = 0;

    double qOld = 16000.0;

    std::vector<std::pair<double, double>> fastOld = {
        {13600, 0.000023}, {15200, 0.000050}, {16000, 0.000150}};
    std::vector<std::pair<double, double>> normalOld = {
        {13600, 0.000046}, {15200, 0.000094}, {16000, 0.000300}};
    std::vector<std::pair<double, double>> slowOld = {
        {13600, 0.000093}, {15200, 0.000175}, {16000, 0.000625}};

    double tNormalOld = 0.0;
    double prevBOld = 0.0;
    for (const auto& bp : normalOld) {
        tNormalOld += (bp.first - prevBOld) * bp.second;
        prevBOld = bp.first;
    }

    double tNormalNew = batteryCapacity * inverseChargeRate;

    double kQ = batteryCapacity / qOld;
    double kT = tNormalNew / tNormalOld;
    double kRate = kT / kQ;

    auto scaleFunc = [&](const std::vector<std::pair<double, double>>& oldFunc) {
        std::vector<std::pair<double, double>> newFunc;
        newFunc.push_back({0.0, 0.0});

        double cq = 0.0;
        double ct = 0.0;
        double pB = 0.0;

        for (const auto& bp : oldFunc) {
            double bOld = bp.first;
            double rOld = bp.second;

            double bDiff = bOld - pB;
            double ln = bDiff * kQ;
            double sp = rOld * kRate;

            cq += ln;
            ct += ln * sp;
            newFunc.push_back({cq, ct});

            pB = bOld;
        }
        return newFunc;
    };

    std::map<std::string, std::vector<std::pair<double, double>>> scaledFunctions;
    scaledFunctions["fast"] = scaleFunc(fastOld);
    scaledFunctions["normal"] = scaleFunc(normalOld);
    scaledFunctions["slow"] = scaleFunc(slowOld);

    double fastFullChargeTime = scaledFunctions["fast"].back().second;
    double newSwapTime = fastFullChargeTime / 5;

    std::cout << "\n========================================" << std::endl;
    std::cout << "[Instance Info] 电池容量 (Q): " << batteryCapacity << std::endl;
    std::cout << "[Instance Info] Fast 0->满电 充电时间: " << fastFullChargeTime << std::endl;
    std::cout << "[Instance Info] 换电站 换电时间: " << newSwapTime << std::endl;
    std::cout << "========================================\n" << std::endl;
    std::cout << "\n--- Updated Fast Charging Function Breakpoints ---" << std::endl;
    for (size_t i = 0; i < scaledFunctions["fast"].size(); ++i) {
        std::cout << "  Point " << i << " -> "
                  << "Charge Level (cq): " << scaledFunctions["fast"][i].first
                  << ", Total Time (ct): " << scaledFunctions["fast"][i].second << std::endl;
    }
    std::cout << "--------------------------------------------------\n" << std::endl;

    swapTime = newSwapTime;

    for (size_t i = 0; i < stationIndices.size(); ++i) {
        int idx = stationIndices[i];
        int sid = xCoord.size();
        chargeStations.push_back(sid);
        xCoord.push_back(rawXc[idx]);
        yCoord.push_back(rawYc[idx]);
        demand.push_back(0);
        readyTime.push_back(rawE[idx]);
        dueTime.push_back(rawL[idx]);
        serviceTime.push_back(rawS[idx]);
        swapTimes.push_back(0.0);

        auto pts = scaledFunctions["fast"];
        rawChargingData[sid] = pts;
        chargingFunctions[sid] = EnergyFunction(pts, batteryCapacity);
    }

    for (size_t i = 0; i < stationIndices.size(); ++i) {
        int idx = stationIndices[i];
        int sid = xCoord.size();
        swapStations.push_back(sid);
        xCoord.push_back(rawXc[idx]);
        yCoord.push_back(rawYc[idx]);
        demand.push_back(0);
        readyTime.push_back(rawE[idx]);
        dueTime.push_back(rawL[idx]);
        serviceTime.push_back(rawS[idx]);
        swapTimes.push_back(newSwapTime);
    }

    for (size_t i = 1; i < rawXc.size(); ++i) {
        if (std::find(stationIndices.begin(), stationIndices.end(), i) == stationIndices.end()) {
            int cid = xCoord.size();
            customers.push_back(cid);
            xCoord.push_back(rawXc[i]);
            yCoord.push_back(rawYc[i]);
            demand.push_back(rawD[i]);
            readyTime.push_back(rawE[i]);
            dueTime.push_back(rawL[i]);
            serviceTime.push_back(rawS[i]);
            swapTimes.push_back(0.0);
        }
    }

    sinkDepot = xCoord.size();
    xCoord.push_back(rawXc[0]);
    yCoord.push_back(rawYc[0]);
    demand.push_back(0);
    readyTime.push_back(rawE[0]);
    dueTime.push_back(rawL[0]);
    serviceTime.push_back(0);
    swapTimes.push_back(0.0);

    for (size_t i = 0; i < xCoord.size(); ++i)
        nodes.push_back(i);

    for (int i : nodes) {
        for (int j : nodes) {
            distance[{i, j}] = std::hypot(xCoord[i] - xCoord[j], yCoord[i] - yCoord[j]);
            time[{i, j}] = distance[{i, j}] / speed;
        }
    }

    nodeCount = xCoord.size();

    distMatrix.assign(nodeCount * nodeCount, 0.0);
    timeMatrix.assign(nodeCount * nodeCount, 0.0);
    infeasibleMat.assign(nodeCount * nodeCount, 0);

    for (int i = 0; i < nodeCount; ++i) {
        for (int j = 0; j < nodeCount; ++j) {
            double distance = std::hypot(xCoord[i] - xCoord[j], yCoord[i] - yCoord[j]);
            distMatrix[i * nodeCount + j] = distance;
            timeMatrix[i * nodeCount + j] = distance / speed;

            this->distance[{i, j}] = distance;
            time[{i, j}] = distance / speed;
        }
    }
}

void Instance::preprocessArcs() {
    std::vector<int> iNodes = {sourceDepot};
    iNodes.insert(iNodes.end(), chargeStations.begin(), chargeStations.end());
    iNodes.insert(iNodes.end(), swapStations.begin(), swapStations.end());
    iNodes.insert(iNodes.end(), customers.begin(), customers.end());

    std::vector<int> jNodes = chargeStations;
    jNodes.insert(jNodes.end(), swapStations.begin(), swapStations.end());
    jNodes.insert(jNodes.end(), customers.begin(), customers.end());
    jNodes.push_back(sinkDepot);

    for (int i : iNodes) {
        for (int j : jNodes) {
            if (i == j)
                continue;
            if (i == sourceDepot && j == sinkDepot) {
                infeasibleMat[i * nodeCount + j] = 1;
                infeasibleArcs.insert({i, j});
                continue;
            }

            bool iCust = std::find(customers.begin(), customers.end(), i) != customers.end();
            bool jCust = std::find(customers.begin(), customers.end(), j) != customers.end();

            if (iCust && jCust && demand[i] + demand[j] > loadCapacity) {
                infeasibleMat[i * nodeCount + j] = 1;
                infeasibleArcs.insert({i, j});
                continue;
            }
            if (getDistance(i, j) * consumptionRate > batteryCapacity) {
                infeasibleMat[i * nodeCount + j] = 1;
                infeasibleArcs.insert({i, j});
                continue;
            }
            if (readyTime[i] + serviceTime[i] + getTravelTime(i, j) > dueTime[j]) {
                infeasibleMat[i * nodeCount + j] = 1;
                infeasibleArcs.insert({i, j});
                continue;
            }

            bool iCharge = std::find(chargeStations.begin(), chargeStations.end(), i) != chargeStations.end();
            bool jSwap = std::find(swapStations.begin(), swapStations.end(), j) != swapStations.end();
            if (iCharge && jSwap && i == j - (int)chargeStations.size()) {
                infeasibleMat[i * nodeCount + j] = 1;
                infeasibleArcs.insert({i, j});
                continue;
            }

            bool iSwap = std::find(swapStations.begin(), swapStations.end(), i) != swapStations.end();
            bool jCharge = std::find(chargeStations.begin(), chargeStations.end(), j) != chargeStations.end();
            if (iSwap && jCharge && j == i - (int)chargeStations.size()) {
                infeasibleMat[i * nodeCount + j] = 1;
                infeasibleArcs.insert({i, j});
                continue;
            }

            bool iIsStation =
                (std::find(chargeStations.begin(), chargeStations.end(), i) != chargeStations.end()) ||
                (std::find(swapStations.begin(), swapStations.end(), i) != swapStations.end());
            bool jIsStation =
                (std::find(chargeStations.begin(), chargeStations.end(), j) != chargeStations.end()) ||
                (std::find(swapStations.begin(), swapStations.end(), j) != swapStations.end());
            if (iIsStation && jIsStation) {
                infeasibleMat[i * nodeCount + j] = 1;
                infeasibleArcs.insert({i, j});
                continue;
            }
        }
    }
}

void Instance::initializeNgMasks() {
    for (int i : nodes) {
        std::vector<std::pair<int, double>> dists;
        for (int j : customers) {
            if (j != i)
                dists.push_back({j, getDistance(i, j)});
        }
        std::sort(
            dists.begin(), dists.end(), [](const std::pair<int, double>& a, const std::pair<int, double>& b) {
                return a.second < b.second;
            });

        NodeMask mask;
        mask.reset();
        for (size_t k = 0; k < std::min((size_t)ngSize, dists.size()); ++k) {
            mask.set(dists[k].first);
        }
        mask.set(i);
        ngMasks[i] = mask;
    }
    std::cout << "\n=== ng_masks for first 5 customers ===\n";
    for (int i = 0; i < (int)customers.size(); ++i) {
        int c = customers[i];
        std::cout << "Customer " << c << " ng_mask: ";
        for (int j : customers) {
            if (ngMasks[c].test(j))
                std::cout << j << " ";
        }
        std::cout << std::endl;
    }
}
