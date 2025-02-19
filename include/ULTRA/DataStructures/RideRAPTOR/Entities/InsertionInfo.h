#pragma once

#include "../../../Helpers/Types.h"
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

    StopInsertionInfo(int32_t vehicleId, int32_t insertionPosition, int32_t distTo, int32_t distFrom, int32_t detour, int32_t leeway, int32_t minDepTime, int32_t maxDepTime, int32_t minArrTime, int32_t reverseDistTo, int32_t reverseDistFrom)
        : vehicleId(vehicleId)
        , insertionPosition(insertionPosition)
        , distTo(distTo)
        , distFrom(distFrom)
        , detour(detour)
        , leeway(leeway)
        , minDepTime(minDepTime)
        , maxDepTime(maxDepTime)
        , minArrTime(minArrTime)
        , reverseDistTo(reverseDistTo)
        , reverseDistFrom(reverseDistFrom)
    {
    }
};

} // namespace RIDERAPTOR
