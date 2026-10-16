// 时间—电量函数：插值、充电更新、前向和后向时间窗处理。
#include "energy.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <chrono>

std::vector<std::pair<double, double>> simplifyPoints(
    const std::vector<std::pair<double, double>>& points, double tol) {
    if (points.size() <= 2)
        return points;
    std::vector<std::pair<double, double>> finalPts;
    finalPts.reserve(points.size());
    for (const auto& p : points) {
        while (finalPts.size() >= 2) {
            const auto& p1 = finalPts[finalPts.size() - 2];
            const auto& p2 = finalPts[finalPts.size() - 1];
            double crossProduct = (p2.second - p1.second) * (p.first - p2.first) -
                                  (p.second - p2.second) * (p2.first - p1.first);
            if (std::abs(crossProduct) < tol) {
                finalPts.pop_back();
            } else {
                break;
            }
        }
        finalPts.push_back(p);
    }
    return finalPts;
}

EnergyFunction::EnergyFunction(const std::vector<std::pair<double, double>>& pts, double qMax) : qMax(qMax) {
    points = pts;
    std::sort(points.begin(), points.end(), [](const auto& a, const auto& b) { return a.first < b.first; });

    if (!points.empty()) {
        yMin = points.front().first;
        yMax = points.back().first;
        tMin = 1e9;
        for (const auto& p : points) {
            if (p.second < tMin)
                tMin = p.second;
        }
    } else {
        yMin = 1e9;
        yMax = -1e9;
        tMin = 1e9;
    }
}

double EnergyFunction::query(double yVal) const {
    if (points.empty())
        return std::nan("");
    if (yVal < yMin - epsilon || yVal > yMax + epsilon)
        return std::nan("");

    auto it = std::lower_bound(points.begin(), points.end(), yVal,
        [](const std::pair<double, double>& p, double val) { return p.first < val - 1e-9; });

    if (it == points.begin()) {
        return (std::abs(yVal - yMin) <= epsilon) ? it->second : std::nan("");
    }
    if (it == points.end()) {
        return (std::abs(yVal - yMax) <= epsilon) ? points.back().second : std::nan("");
    }

    auto prev = std::prev(it);
    double x0 = prev->first, y0 = prev->second;
    double x1 = it->first, y1 = it->second;

    if (std::abs(x1 - x0) < 1e-9)
        return y0;
    return y0 + (yVal - x0) * (y1 - y0) / (x1 - x0);
}

EnergyFunction chargeAlgorithmUpdate(const EnergyFunction& fTW, const EnergyFunction& hInv, double qMax) {
    if (fTW.points.empty())
        return EnergyFunction({}, qMax);
    double yFMax = fTW.points.back().first;
    double tFMin = fTW.points.front().second;
    double tFMax = fTW.points.back().second;

    auto getHVal = [&](double y) { return hInv.query(y) + tFMin; };
    auto getFVal = [&](double y) { return fTW.query(y); };

    std::vector<double> allY;
    for (auto& p : fTW.points)
        allY.push_back(p.first);
    for (auto& p : hInv.points) {
        if (p.first <= qMax)
            allY.push_back(p.first);
    }
    allY.push_back(qMax);
    std::sort(allY.begin(), allY.end());
    allY.erase(
        std::unique(allY.begin(), allY.end(), [](double a, double b) { return std::abs(a - b) < 1e-6; }),
        allY.end());

    std::vector<std::pair<double, double>> rawPoints;
    for (size_t j = 0; j < allY.size() - 1; ++j) {
        double y0 = allY[j], y1 = allY[j + 1];
        double f0 = getFVal(y0), f1 = getFVal(y1);
        double h0 = getHVal(y0), h1 = getHVal(y1);

        if (std::isnan(f0) && y0 >= yFMax - 1e-6)
            f0 = tFMax + (h0 - getHVal(yFMax));
        if (std::isnan(f1) && y1 >= yFMax - 1e-6)
            f1 = tFMax + (h1 - getHVal(yFMax));

        double v0 = (!std::isnan(f0)) ? f0 : 1e9;
        rawPoints.push_back({y0, std::min(v0, h0)});

        if (!std::isnan(f0) && !std::isnan(f1)) {
            if ((f0 - h0) * (f1 - h1) < -1e-7) {
                double kf = (f1 - f0) / (y1 - y0);
                double kh = (h1 - h0) / (y1 - y0);
                if (std::abs(kf - kh) > 1e-9) {
                    double yMid = y0 + (h0 - f0) / (kf - kh);
                    rawPoints.push_back({yMid, getHVal(yMid)});
                }
            }
        }
    }
    double lastY = allY.back();
    double fvL = getFVal(lastY);
    if (std::isnan(fvL) && lastY >= yFMax - 1e-6)
        fvL = tFMax + (getHVal(lastY) - getHVal(yFMax));
    double vLast = (!std::isnan(fvL)) ? fvL : 1e9;
    rawPoints.push_back({lastY, std::min(vLast, getHVal(lastY))});

    std::sort(rawPoints.begin(), rawPoints.end());
    std::vector<std::pair<double, double>> uniqueFinal;
    for (auto& p : rawPoints) {
        if (uniqueFinal.empty() || std::abs(p.first - uniqueFinal.back().first) > 1e-6) {
            double tVal = p.second;
            if (!uniqueFinal.empty())
                tVal = std::max(tVal, uniqueFinal.back().second);
            uniqueFinal.push_back({p.first, tVal});
        }
    }
    return EnergyFunction(simplifyPoints(uniqueFinal), qMax);
}

EnergyFunction backwardChargeAlgorithmUpdate(
    const EnergyFunction& wNext, const EnergyFunction& hInv, double qMax) {
    if (wNext.points.empty())
        return EnergyFunction({}, qMax);
    std::vector<double> allY;
    for (auto& p : wNext.points)
        allY.push_back(p.first);
    for (auto& p : hInv.points) {
        if (p.first <= qMax)
            allY.push_back(p.first);
    }
    allY.push_back(0.0);
    allY.push_back(qMax);
    std::sort(allY.begin(), allY.end());
    allY.erase(
        std::unique(allY.begin(), allY.end(), [](double a, double b) { return std::abs(a - b) < 1e-6; }),
        allY.end());

    std::vector<std::pair<double, double>> rawPoints;
    double currentMaxG = -1e9;
    for (int j = (int)allY.size() - 1; j > 0; --j) {
        double y0 = allY[j - 1], y1 = allY[j];
        double w0 = wNext.query(y0), w1 = wNext.query(y1);
        double h0 = hInv.query(y0), h1 = hInv.query(y1);

        double g0 = !std::isnan(w0) ? w0 - h0 : -1e9;
        double g1 = !std::isnan(w1) ? w1 - h1 : -1e9;

        if (g1 > currentMaxG)
            currentMaxG = g1;
        if (currentMaxG > -1e8)
            rawPoints.push_back({y1, h1 + currentMaxG});

        if (g0 > currentMaxG && g1 <= currentMaxG && g0 > -1e8 && g1 > -1e8) {
            double yMid = y0 + (y1 - y0) * (g0 - currentMaxG) / (g0 - g1);
            double hMid = hInv.query(yMid);
            rawPoints.push_back({yMid, hMid + currentMaxG});
        }
    }
    double yFirst = allY[0];
    double wFirst = wNext.query(yFirst);
    double hFirst = hInv.query(yFirst);
    double gFirst = !std::isnan(wFirst) ? wFirst - hFirst : -1e9;
    if (gFirst > currentMaxG)
        currentMaxG = gFirst;
    if (currentMaxG > -1e8)
        rawPoints.push_back({yFirst, hFirst + currentMaxG});

    std::reverse(rawPoints.begin(), rawPoints.end());
    std::vector<std::pair<double, double>> uniqueFinal;
    for (auto& p : rawPoints) {
        if (uniqueFinal.empty() || std::abs(p.first - uniqueFinal.back().first) > 1e-6) {
            double tVal = p.second;
            if (!uniqueFinal.empty())
                tVal = std::max(tVal, uniqueFinal.back().second);
            uniqueFinal.push_back({p.first, tVal});
        }
    }
    return EnergyFunction(simplifyPoints(uniqueFinal), qMax);
}

EnergyFunction shiftAndClipForward(
    const EnergyFunction& currFunc, double dq, double dt, double ei, double li, double qMax) {
    std::vector<double> depYs = {dq, qMax};
    for (auto& p : currFunc.points) {
        if (p.first > dq && p.first < qMax)
            depYs.push_back(p.first);
    }
    std::sort(depYs.begin(), depYs.end());
    depYs.erase(
        std::unique(depYs.begin(), depYs.end(), [](double a, double b) { return std::abs(a - b) < 1e-6; }),
        depYs.end());

    std::vector<std::pair<double, double>> rawPts;
    for (double yDep : depYs) {
        double tDep = currFunc.query(yDep);
        if (!std::isnan(tDep)) {
            rawPts.push_back({yDep - dq, tDep + dt});
        }
    }
    if (rawPts.empty())
        return EnergyFunction({}, qMax);

    std::vector<std::pair<double, double>> finalPts;
    for (size_t k = 0; k < rawPts.size(); ++k) {
        double y = rawPts[k].first, t = rawPts[k].second;
        if (!finalPts.empty() && k > 0) {
            double prevY = rawPts[k - 1].first, prevT = rawPts[k - 1].second;
            if (prevT < ei && t > ei) {
                if (std::abs(t - prevT) > 1e-9) {
                    double yIntE = prevY + (ei - prevT) * (y - prevY) / (t - prevT);
                    finalPts.push_back({yIntE, ei});
                }
            }
            if (prevT <= li && t > li) {
                if (std::abs(t - prevT) > 1e-9) {
                    double yIntL = prevY + (li - prevT) * (y - prevY) / (t - prevT);
                    finalPts.push_back({yIntL, li});
                }
                break;
            }
        }
        if (t <= li + 1e-7)
            finalPts.push_back({y, std::max(t, ei)});
    }
    if (finalPts.empty())
        return EnergyFunction({}, qMax);

    std::vector<std::pair<double, double>> uniqueF;
    for (auto& p : finalPts) {
        if (uniqueF.empty() || std::abs(p.first - uniqueF.back().first) > 1e-6) {
            double tVal = std::max(p.second, uniqueF.empty() ? 0.0 : uniqueF.back().second);
            uniqueF.push_back({p.first, tVal});
        }
    }
    return EnergyFunction(simplifyPoints(uniqueF), qMax);
}

EnergyFunction shiftAndClipBackward(
    const EnergyFunction& currFunc, double dq, double dt, double ej, double lj, double qMax) {
    if (dq > qMax + 1e-7)
        return EnergyFunction({}, qMax);

    std::vector<double> evalYLeave;
    evalYLeave.push_back(dq);
    for (const auto& p : currFunc.points) {
        double yl = p.first + dq;
        if (dq < yl && yl < qMax)
            evalYLeave.push_back(yl);
    }
    evalYLeave.push_back(qMax);

    std::sort(evalYLeave.begin(), evalYLeave.end());
    evalYLeave.erase(std::unique(evalYLeave.begin(), evalYLeave.end(),
                         [](double a, double b) { return std::abs(a - b) < 1e-6; }),
        evalYLeave.end());

    std::vector<std::pair<double, double>> rawWPts;
    for (double yl : evalYLeave) {
        double tJ = currFunc.query(yl - dq);
        rawWPts.push_back({yl, tJ});
    }

    std::vector<std::pair<double, double>> clippedPts;
    for (size_t k = 0; k < rawWPts.size(); ++k) {
        double y = rawWPts[k].first;
        double t = rawWPts[k].second;
        if (std::isnan(t))
            continue;

        if (k > 0) {
            double yPrev = rawWPts[k - 1].first;
            double tPrev = rawWPts[k - 1].second;
            if (!std::isnan(tPrev) && t > tPrev + 1e-9) {
                if (tPrev < ej && t >= ej) {
                    double yInt = yPrev + (ej - tPrev) * (y - yPrev) / (t - tPrev);
                    clippedPts.push_back({yInt, ej});
                }
                if (tPrev <= lj && t > lj) {
                    double yInt = yPrev + (lj - tPrev) * (y - yPrev) / (t - tPrev);
                    clippedPts.push_back({yInt, lj});
                }
            }
        }

        if (t >= ej - 1e-7 && t <= lj + 1e-7) {
            clippedPts.push_back({y, t});
        } else if (t > lj + 1e-7) {
            clippedPts.push_back({y, lj});
        }
    }

    std::vector<std::pair<double, double>> validWPts;
    for (const auto& p : clippedPts) {
        double y = p.first;
        double tLeaveI = p.second - dt;

        if (validWPts.empty()) {
            validWPts.push_back({y, tLeaveI});
        } else {
            double prevY = validWPts.back().first;
            double prevT = validWPts.back().second;
            if (std::abs(y - prevY) <= 1e-6) {
                validWPts.back().second = std::max(prevT, tLeaveI);
            } else {
                tLeaveI = std::max(tLeaveI, prevT);
                validWPts.push_back({y, tLeaveI});
            }
        }
    }
    return EnergyFunction(simplifyPoints(validWPts), qMax);
}
