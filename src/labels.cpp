// 标签状态、支配规则、载重分桶和不可达节点更新。
#include "pricing.h"
#include <queue>
#include <cmath>
#include <algorithm>
#include "energy.h"
#include <iostream>
#include <sstream>
#include <iomanip>

ForwardLabel::ForwardLabel(int n, double c, double l, const EnergyFunction& ef, char vehicleType,
    std::shared_ptr<ForwardLabel> p, NodeMask ng, const std::vector<int>& cuts, NodeMask unreach,
    NodeMask crit, NodeMask visit)
    : node(n), cost(c), load(l), energyFunc(ef), vehicleType(vehicleType), parent(p), ngMask(ng),
      unreachableMask(unreach), criticalMask(crit), visitMask(visit), cutStatus(cuts) {}

std::vector<int> ForwardLabel::getPath() const {
    std::vector<int> path;
    const ForwardLabel* curr = this;
    while (curr) {
        path.push_back(curr->node);
        curr = curr->parent.get();
    }
    std::reverse(path.begin(), path.end());
    return path;
}

bool ForwardLabel::isDominatedBy(const ForwardLabel* other, const std::vector<double>& dualsSrc) const {
    if (this == other)
        return false;

    if (this->vehicleType != other->vehicleType)
        return false;

    if (cost < other->cost - 1e-6 || load < other->load - 1e-6)
        return false;

    if ((other->ngMask & ngMask) != other->ngMask)
        return false;
    if ((other->criticalMask & criticalMask) != other->criticalMask)
        return false;

    double penaltyCompensation = 0.0;
    for (size_t i = 0; i < dualsSrc.size(); ++i) {
        if (dualsSrc[i] < -1e-6) {
            if (other->cutStatus[i] > cutStatus[i]) {
                penaltyCompensation += dualsSrc[i];
            }
        }
    }
    if (other->cost - penaltyCompensation > cost + 1e-6)
        return false;

    if (other->energyFunc.points.empty())
        return false;
    if (energyFunc.points.empty())
        return true;

    const auto& pts1 = energyFunc.points;
    const auto& pts2 = other->energyFunc.points;
    size_t i = 0, j = 0;
    while (i < pts1.size() && j < pts2.size()) {
        double v1 = pts1[i].first;
        double v2 = pts2[j].first;
        double yEval;
        if (std::abs(v1 - v2) < 1e-6) {
            yEval = v1;
            i++;
            j++;
        } else if (v1 < v2) {
            yEval = v1;
            i++;
        } else {
            yEval = v2;
            j++;
        }
        double tS = energyFunc.query(yEval);
        double tO = other->energyFunc.query(yEval);
        if (!std::isnan(tS) && (std::isnan(tO) || tO > tS + 1e-6))
            return false;
    }
    while (i < pts1.size()) {
        double yEval = pts1[i].first;
        double tS = pts1[i].second;
        double tO = other->energyFunc.query(yEval);
        if (!std::isnan(tS) && (std::isnan(tO) || tO > tS + 1e-6))
            return false;
        i++;
    }
    while (j < pts2.size()) {
        double yEval = pts2[j].first;
        if (yEval >= energyFunc.yMin - 1e-6 && yEval <= energyFunc.yMax + 1e-6) {
            double tS = energyFunc.query(yEval);
            double tO = pts2[j].second;
            if (!std::isnan(tS) && (std::isnan(tO) || tO > tS + 1e-6))
                return false;
        }
        j++;
    }
    return true;
}

BackwardLabel::BackwardLabel(int n, double c, double l, const EnergyFunction& ef, char vehicleType,
    std::shared_ptr<BackwardLabel> p, NodeMask ng, const std::vector<int>& cuts, NodeMask unreach,
    NodeMask crit, NodeMask visit)
    : node(n), cost(c), load(l), energyFunc(ef), vehicleType(vehicleType), parent(p), ngMask(ng),
      unreachableMask(unreach), criticalMask(crit), visitMask(visit), cutStatus(cuts) {}

std::vector<int> BackwardLabel::getPath() const {
    std::vector<int> path;
    const BackwardLabel* curr = this;
    while (curr) {
        path.push_back(curr->node);
        curr = curr->parent.get();
    }
    return path;
}

bool BackwardLabel::isDominatedBy(const BackwardLabel* other, const std::vector<double>& dualsSrc) const {
    if (this == other)
        return false;

    if (this->vehicleType != other->vehicleType)
        return false;

    if (cost < other->cost - 1e-6 || load < other->load - 1e-6)
        return false;

    if ((other->ngMask & ngMask) != other->ngMask)
        return false;
    if ((other->criticalMask & criticalMask) != other->criticalMask)
        return false;

    double penaltyCompensation = 0.0;
    for (size_t i = 0; i < dualsSrc.size(); ++i) {
        if (dualsSrc[i] < -1e-6) {
            if (other->cutStatus[i] > cutStatus[i]) {
                penaltyCompensation += dualsSrc[i];
            }
        }
    }
    if (other->cost - penaltyCompensation > cost + 1e-6)
        return false;

    if (other->energyFunc.points.empty())
        return false;
    if (energyFunc.points.empty())
        return true;

    const auto& pts1 = energyFunc.points;
    const auto& pts2 = other->energyFunc.points;
    size_t i = 0, j = 0;
    while (i < pts1.size() && j < pts2.size()) {
        double v1 = pts1[i].first;
        double v2 = pts2[j].first;
        double yEval;
        if (std::abs(v1 - v2) < 1e-6) {
            yEval = v1;
            i++;
            j++;
        } else if (v1 < v2) {
            yEval = v1;
            i++;
        } else {
            yEval = v2;
            j++;
        }
        double tS = energyFunc.query(yEval);
        double tO = other->energyFunc.query(yEval);
        if (!std::isnan(tS) && (std::isnan(tO) || tO < tS - 1e-6))
            return false;
    }
    while (i < pts1.size()) {
        double yEval = pts1[i].first;
        double tS = pts1[i].second;
        double tO = other->energyFunc.query(yEval);
        if (!std::isnan(tS) && (std::isnan(tO) || tO < tS - 1e-6))
            return false;
        i++;
    }
    while (j < pts2.size()) {
        double yEval = pts2[j].first;
        if (yEval >= energyFunc.yMin - 1e-6 && yEval <= energyFunc.yMax + 1e-6) {
            double tS = energyFunc.query(yEval);
            double tO = pts2[j].second;
            if (!std::isnan(tS) && (std::isnan(tO) || tO < tS - 1e-6))
                return false;
        }
        j++;
    }
    return true;
}

bool checkIsDominated(ForwardLabel* newLab, ForwardBuckets& buckets, const std::vector<double>& dualsSrc) {
    for (auto& pair : buckets) {
        double bLoad = pair.first;
        if (bLoad > newLab->load)
            continue;
        for (auto& extLab : pair.second) {
            if (extLab->cost > newLab->cost + 1e-6)
                break;
            if (newLab->isDominatedBy(extLab.get(), dualsSrc))
                return true;
        }
    }
    return false;
}

void addAndFilterLabel(
    std::shared_ptr<ForwardLabel> newLab, ForwardBuckets& buckets, const std::vector<double>& dualsSrc) {
    auto it = buckets.begin();
    while (it != buckets.end()) {
        if (it->first < newLab->load) {
            ++it;
            continue;
        }
        std::vector<std::shared_ptr<ForwardLabel>> survivors;
        for (auto& extLab : it->second) {
            if (!extLab->isDominatedBy(newLab.get(), dualsSrc)) {
                survivors.push_back(extLab);
            }
        }
        if (survivors.empty()) {
            it = buckets.erase(it);
        } else {
            it->second = survivors;
            ++it;
        }
    }
    auto& bucket = buckets[newLab->load];
    auto pos = std::lower_bound(bucket.begin(), bucket.end(), newLab,
        [](const std::shared_ptr<ForwardLabel>& a, const std::shared_ptr<ForwardLabel>& b) {
            return a->cost < b->cost;
        });
    bucket.insert(pos, newLab);
}

bool checkIsDominatedBw(
    BackwardLabel* newLab, BackwardBuckets& buckets, const std::vector<double>& dualsSrc) {
    for (auto& pair : buckets) {
        double bLoad = pair.first;
        if (bLoad > newLab->load)
            continue;
        for (auto& extLab : pair.second) {
            if (extLab->cost > newLab->cost + 1e-6)
                break;
            if (newLab->isDominatedBy(extLab.get(), dualsSrc))
                return true;
        }
    }
    return false;
}

void addAndFilterLabelBw(
    std::shared_ptr<BackwardLabel> newLab, BackwardBuckets& buckets, const std::vector<double>& dualsSrc) {
    auto it = buckets.begin();
    while (it != buckets.end()) {
        if (it->first < newLab->load) {
            ++it;
            continue;
        }
        std::vector<std::shared_ptr<BackwardLabel>> survivors;
        for (auto& extLab : it->second) {
            if (!extLab->isDominatedBy(newLab.get(), dualsSrc)) {
                survivors.push_back(extLab);
            }
        }
        if (survivors.empty()) {
            it = buckets.erase(it);
        } else {
            it->second = survivors;
            ++it;
        }
    }
    auto& bucket = buckets[newLab->load];
    auto pos = std::lower_bound(bucket.begin(), bucket.end(), newLab,
        [](const std::shared_ptr<BackwardLabel>& a, const std::shared_ptr<BackwardLabel>& b) {
            return a->cost < b->cost;
        });
    bucket.insert(pos, newLab);
}

NodeMask getUnreachableMaskFw(
    const Instance& instance, const ForwardLabel* curr, int nextNode, const EnergyFunction& nextEf) {
    NodeMask newUn = curr->unreachableMask;
    if (std::find(instance.customers.begin(), instance.customers.end(), nextNode) !=
        instance.customers.end()) {
        newUn.set(nextNode);
    }
    double tBound = nextEf.points.empty() ? NAN : nextEf.points.front().second;
    if (std::isnan(tBound))
        return newUn;
    for (int jPrime : instance.customers) {
        if (newUn.test(jPrime))
            continue;
        if (curr->load + instance.demand[nextNode] + instance.demand[jPrime] > instance.loadCapacity ||
            tBound + instance.getTravelTime(nextNode, jPrime) > instance.dueTime[jPrime]) {
            newUn.set(jPrime);
        }
    }
    return newUn;
}

NodeMask getUnreachableMaskBw(
    const Instance& instance, const BackwardLabel* curr, int nextNode, const EnergyFunction& nextEf) {
    NodeMask newUn = curr->unreachableMask;
    if (std::find(instance.customers.begin(), instance.customers.end(), nextNode) !=
        instance.customers.end()) {
        newUn.set(nextNode);
    }
    double tBound = nextEf.points.empty() ? NAN : nextEf.points.back().second;
    if (std::isnan(tBound))
        return newUn;
    for (int jPrime : instance.customers) {
        if (newUn.test(jPrime))
            continue;
        if (curr->load + instance.demand[curr->node] + instance.demand[nextNode] + instance.demand[jPrime] >
                instance.loadCapacity ||
            tBound - instance.getTravelTime(jPrime, nextNode) < instance.readyTime[jPrime]) {
            newUn.set(jPrime);
        }
    }
    return newUn;
}
