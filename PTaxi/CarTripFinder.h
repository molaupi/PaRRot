#pragma once
#include "CarResult.h"
#include "Common/Constants.h"
#include "KARRI/Algorithms/KaRRi/RequestState/RequestState.h"
#include "KARRI/Tools/ThreadSafeRandom.h"

namespace parrot {
    using namespace karri;

    // Computes a car trip for a given request, i.e., a trip that consists of only walking from origin to destination.
    class CarTripFinder {
        static bool isCarAllowed(const Request &req) {
            return req.allowPrivateCarProbability > 0.0 &&
                   ThreadSafeRandom::randomNumber() < req.allowPrivateCarProbability;
        }

    public:
        CarTripFinder() = default;

        CarResult findCarTrip(const RequestState &requestState, stats::CarPerformanceStats &) {
            if (!isCarAllowed(requestState.originalRequest))
                return {INFTY};
            return {requestState.originalReqDirectDist};
        }

    private:
    };
}
