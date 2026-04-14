#pragma once
#include "WalkingResult.h"
#include "KARRI/Algorithms/CH/CH.h"
#include "KARRI/Algorithms/KaRRi/BaseObjects/Request.h"
#include "KARRI/Algorithms/KaRRi/RequestState/RequestState.h"

namespace karri {
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
                          const PsgCHEnvT &psgChEnv,
                          const RouteState &routeState)
            : vehInputGraph(vehInputGraph),
              psgInputGraph(psgInputGraph),
              psgCh(psgChEnv.getCH()),
              psgChQuery(psgChEnv.template getFullCHQuery<>()),
              calc(routeState) {
        }

        WalkingResult findWalkingTrip(const RequestState &requestState, stats::WalkPerformanceStats &stats) {
            KaRRiTimer timer;
            const auto &request = requestState.originalRequest;
            const int originPsgEdge = vehInputGraph.toPsgEdge(request.origin);
            const int destPsgEdge = vehInputGraph.toPsgEdge(request.destination);
            const int source = psgInputGraph.edgeHead(originPsgEdge);
            const int target = psgInputGraph.edgeTail(destPsgEdge);
            const int offset = psgInputGraph.travelTime(originPsgEdge);
            psgChQuery.run(psgCh.rank(source), psgCh.rank(target));
            const auto walkingDist = psgChQuery.getDistance() + offset;
            const WalkingResult res = {walkingDist, calc.calcCostForNotUsingVehicle(walkingDist, offset)};
            const int64_t time = timer.elapsed<std::chrono::nanoseconds>();
            stats.time += time;
            return res;
        }

    private:
        const VehInputGraphT &vehInputGraph;
        const PsgInputGraphT &psgInputGraph;
        const CH &psgCh;
        PsgCHEnvT::template FullCHQuery<> psgChQuery;
        CostCalculator calc;
    };
}
