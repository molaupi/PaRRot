#pragma once

#include <vector>

#include "KARRI/Algorithms/KaRRi/Stats/PerformanceStats.h"
#include "PTLeg/PTResult.h"

namespace karri {

    // Computes a PT journey with walking but without taxi legs.
    template<
        typename PTQueryT,
        typename PTAlgorithmWithTaxiT
    >
    class PTJourneyFinder {
    public:
        PTJourneyFinder(const std::vector<PTQueryT> &queries,
                     PTAlgorithmWithTaxiT &ptAlgorithmWithTaxi)
            : queries(queries),
              ptAlgorithmWithTaxi(ptAlgorithmWithTaxi) {
        }

        PTResult findBestJourney(const RequestState & requestState,
                                 stats::PtPerformanceStats &stats) {
            const auto query = queries[requestState.originalRequest.requestId];
            const int originPsgEdge = query.originPsgEdge;
            const int originVehEdge = query.originVehEdge;
            const int destPsgEdge = query.destinationPsgEdge;
            const int destVehEdge = query.destinationVehEdge;

            ptAlgorithmWithTaxi.run(originPsgEdge, originVehEdge, destPsgEdge, destVehEdge, query.departureTime, stats);
            const auto ptOnlyParetoFront = ptAlgorithmWithTaxi.getJourneys();
            auto journey = chooseBestJourney(ptOnlyParetoFront);
            PTResult ptOnlyResponse(journey);

            return ptOnlyResponse;
        }

    private:
        const std::vector<PTQueryT> &queries;
        PTAlgorithmWithTaxiT &ptAlgorithmWithTaxi;
    };
}
