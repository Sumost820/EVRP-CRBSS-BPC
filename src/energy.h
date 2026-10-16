#ifndef UTILS_H
#define UTILS_H

#include <vector>
#include <utility>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <string>

const double epsilon = 1e-6;

std::vector<std::pair<double, double>> simplifyPoints(
    const std::vector<std::pair<double, double>>& points, double tol = 1e-6);

struct EnergyFunction {
    std::vector<std::pair<double, double>> points;
    double qMax;
    double yMin;
    double yMax;
    double tMin;

    EnergyFunction() : qMax(0), yMin(1e9), yMax(-1e9), tMin(1e9) {}
    EnergyFunction(const std::vector<std::pair<double, double>>& pts, double qMax);

    double query(double yVal) const;
};

EnergyFunction chargeAlgorithmUpdate(const EnergyFunction& fTW, const EnergyFunction& hInv, double qMax);
EnergyFunction backwardChargeAlgorithmUpdate(
    const EnergyFunction& wNext, const EnergyFunction& hInv, double qMax);

EnergyFunction shiftAndClipForward(
    const EnergyFunction& currFunc, double dq, double dt, double ei, double li, double qMax);
EnergyFunction shiftAndClipBackward(
    const EnergyFunction& currFunc, double dq, double dt, double ej, double lj, double qMax);

#endif
