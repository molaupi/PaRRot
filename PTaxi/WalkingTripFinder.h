#pragma once
#include "WalkingResult.h"
#include "KARRI/Algorithms/CH/CH.h"
#include "KARRI/Algorithms/KaRRi/BaseObjects/Request.h"
#include "KARRI/Algorithms/KaRRi/RequestState/RequestState.h"

namespace parrot {

    using namespace karri;

    // Computes a walking trip for a given request, i.e., a trip that consists of only walking from origin to destination.
    template<
        typename VehInputGraphT,
        typename PsgInputGraphT,
        typename PsgCHEnvT
    >
    class WalkingTripFinder {
    public:
        WalkingTripFinder(const VehInputGraphT &vehInputGraph,
                          const PsgInputGraphT &psgInputGraph,
                          const PsgCHEnvT &psgChEnv)
            : vehInputGraph(vehInputGraph),
              psgInputGraph(psgInputGraph),
              psgCh(psgChEnv.getCH()),
              psgChQuery(psgChEnv.template getFullCHQuery<>()) {
        }

        WalkingResult findWalkingTrip(const RequestState &requestState, stats::WalkPerformanceStats &stats) {
            KaRRiTimer timer;
            const auto &request = requestState.originalRequest;
            const int originPsgEdge = vehInputGraph.toPsgEdge(request.origin);
            const int destPsgEdge = vehInputGraph.toPsgEdge(request.destination);
            int walkingDist = INFTY;
            int travelTimeOfDestEdge = INFTY;
            if (originPsgEdge == destPsgEdge) {
                walkingDist = 0;
                travelTimeOfDestEdge = 0;
            } else {
                const int source = psgInputGraph.edgeHead(originPsgEdge);
                const int target = psgInputGraph.edgeTail(destPsgEdge);
                const int offset = psgInputGraph.travelTime(originPsgEdge);
                psgChQuery.run(psgCh.rank(source), psgCh.rank(target));
                walkingDist = psgChQuery.getDistance() + offset;
                travelTimeOfDestEdge = offset;
            }
            const WalkingResult res = {walkingDist, CostCalculator::calcCostForNotUsingVehicle(walkingDist, travelTimeOfDestEdge)};
            const int64_t time = timer.elapsed<std::chrono::nanoseconds>();
            stats.time += time;
            return res;
        }

    private:
        const VehInputGraphT &vehInputGraph;
        const PsgInputGraphT &psgInputGraph;
        const CH &psgCh;
        PsgCHEnvT::template FullCHQuery<> psgChQuery;
    };
}
