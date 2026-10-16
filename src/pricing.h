#ifndef PRICING_H
#define PRICING_H

#include "instance.h"
#include "energy.h"
#include <vector>
#include <memory>
#include <set>
#include <map>
#include <cstdint>

struct SRCCutInfo {
    std::set<int> customerSet;
    std::set<int> memorySet;
};

struct RobustCutInfo {
    std::set<int> customerSet;
    int rhs;
};

struct Route {
    std::vector<int> path;
    char vehicleType;
    double cost;
};

struct ForwardLabel {
    int node;
    double cost;
    double load;
    EnergyFunction energyFunc;
    char vehicleType;
    std::shared_ptr<ForwardLabel> parent; // 多个子标签共用前驱，回溯时恢复完整路线。

    NodeMask ngMask; // 用位集合保存节点状态，保留原来的支配与加速逻辑。
    NodeMask unreachableMask;
    NodeMask criticalMask;
    NodeMask visitMask;

    std::vector<int> cutStatus;

    ForwardLabel(int n, double c, double l, const EnergyFunction& ef, char vehicleType,
        std::shared_ptr<ForwardLabel> p = nullptr, NodeMask ng = 0, const std::vector<int>& cuts = {},
        NodeMask unreach = 0, NodeMask crit = 0, NodeMask visit = 0);

    bool isDominatedBy(const ForwardLabel* other, const std::vector<double>& dualsSrc) const;
    std::vector<int> getPath() const;
};

struct CompareForward {
    bool operator()(const std::shared_ptr<ForwardLabel>& a, const std::shared_ptr<ForwardLabel>& b) const {
        if (std::abs(a->cost - b->cost) > 1e-9)
            return a->cost > b->cost;
        return a->load > b->load;
    }
};

struct BackwardLabel {
    int node;
    double cost;
    double load;
    EnergyFunction energyFunc;
    char vehicleType;
    std::shared_ptr<BackwardLabel> parent;

    NodeMask ngMask;
    NodeMask unreachableMask;
    NodeMask criticalMask;
    NodeMask visitMask;

    std::vector<int> cutStatus;

    BackwardLabel(int n, double c, double l, const EnergyFunction& ef, char vehicleType,
        std::shared_ptr<BackwardLabel> p = nullptr, NodeMask ng = 0, const std::vector<int>& cuts = {},
        NodeMask unreach = 0, NodeMask crit = 0, NodeMask visit = 0);

    bool isDominatedBy(const BackwardLabel* other, const std::vector<double>& dualsSrc) const;
    std::vector<int> getPath() const;
};

struct CompareBackward {
    bool operator()(const std::shared_ptr<BackwardLabel>& a, const std::shared_ptr<BackwardLabel>& b) const {
        if (std::abs(a->cost - b->cost) > 1e-9)
            return a->cost > b->cost;
        return a->load > b->load;
    }
};

using ForwardBuckets = std::map<double, std::vector<std::shared_ptr<ForwardLabel>>>;
using BackwardBuckets = std::map<double, std::vector<std::shared_ptr<BackwardLabel>>>;

bool checkIsDominated(ForwardLabel* newLabel, ForwardBuckets& buckets, const std::vector<double>& dualsSrc);
bool checkIsDominatedBw(
    BackwardLabel* newLabel, BackwardBuckets& buckets, const std::vector<double>& dualsSrc);
void addAndFilterLabel(
    std::shared_ptr<ForwardLabel> newLabel, ForwardBuckets& buckets, const std::vector<double>& dualsSrc);
void addAndFilterLabelBw(
    std::shared_ptr<BackwardLabel> newLabel, BackwardBuckets& buckets, const std::vector<double>& dualsSrc);
NodeMask getUnreachableMaskFw(
    const Instance& instance, const ForwardLabel* current, int nextNode, const EnergyFunction& nextFunction);
NodeMask getUnreachableMaskBw(
    const Instance& instance, const BackwardLabel* current, int nextNode, const EnergyFunction& nextFunction);

std::vector<Route> solvePricing(const Instance& instance, const std::map<int, double>& duals,
    const std::vector<double>& dualsSrc, const std::vector<double>& dualsRobust,
    const std::map<char, double>& dualsFleet, const std::set<std::pair<int, int>>& forbidden,
    const std::vector<SRCCutInfo>& subsetRowCuts, const std::vector<RobustCutInfo>& capacityCuts,
    const std::set<int>& criticalNodes, std::set<std::pair<std::vector<int>, char>>& existingSigs,
    const std::set<std::pair<int, int>>* activeArcs = nullptr);

#endif
