#pragma once

#include <cstdint>

namespace RIDERAPTOR {

constexpr int noVehicleId = INFTY;

struct StopInsertionInfo {
    int32_t vehicleId;
    int32_t insertionPosition;
    int32_t distTo;
    int32_t distFrom;
    int32_t detour;
    int32_t leeway;
    int32_t minDepTime;
    int32_t maxDepTime;
    int32_t minArrTime;
    int32_t reverseDistTo;
    int32_t reverseDistFrom;

    StopInsertionInfo()
        : vehicleId(noVehicleId)
    {
    }
};

} // namespace RIDERAPTOR
