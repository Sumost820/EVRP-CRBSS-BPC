#pragma once

#include "instance.h"
#include "pricing.h"

enum class ScheduleStatus {
    Feasible,
    Infeasible,
    Unknown
};

struct ScheduledVisit {
    int node = -1;
    double arrivalTime = 0;
    double startTime = 0;
    double departureTime = 0;
    double arrivalEnergy = 0;
    double departureEnergy = 0;
    int chargerId = -1; // 充电访问使用站内 1-based 桩编号；其他节点为 -1。
};

struct ScheduleResult {
    ScheduleStatus status = ScheduleStatus::Unknown;
    std::string message;
    std::vector<std::vector<ScheduledVisit>> visits;
};

ScheduleResult checkGlobalSchedule(
    const Instance& instance, const std::vector<Route>& routes, double timeLimit);
