#pragma once

#include <vector>

#include <KARRI/Algorithms/KaRRi/RequestState/RequestState.h>
#include "KARRI/Algorithms/KaRRi/Stats/PerformanceStats.h"
#include "PTLeg/PTResult.h"

namespace karri {

    // Computes a PT journey with walking but without taxi legs.
    template<
        typename PTQueryT,
        typename PTAlgorithmT
    >
    class PTJourneyFinder {
    public:
        PTJourneyFinder(const std::vector<PTQueryT> &queries,
                     PTAlgorithmT &ptAlgorithm)
            : queries(queries),
              ptAlgorithm(ptAlgorithm) {
        }

        PTResult findBestJourney(const RequestState & requestState,
                                 stats::PtPerformanceStats &stats) {
            const auto query = queries[requestState.originalRequest.requestId];
            const int originPsgEdge = query.originPsgEdge;
            const int originVehEdge = query.originVehEdge;
            const int destPsgEdge = query.destinationPsgEdge;
            const int destVehEdge = query.destinationVehEdge;

            ptAlgorithm.run(originPsgEdge, originVehEdge, destPsgEdge, destVehEdge, query.departureTime, stats);
            const auto ptOnlyParetoFront = ptAlgorithm.getJourneys();
            auto [journey, cost] = chooseBestJourney(ptOnlyParetoFront, requestState.originalRequest.requestTime);
            PTResult ptOnlyResponse(journey, cost);

            return ptOnlyResponse;
        }

    private:
        const std::vector<PTQueryT> &queries;
        PTAlgorithmT &ptAlgorithm;
    };
}
