#pragma once

#include "Station/Station.h"
#include "Stats/IntermediateResultStats.h"
#include <KARRI/Algorithms/CH/CH.h>
#include <KARRI/Algorithms/KaRRi/RequestState/RelevantPDLocs.h>

#include "KaRRiBaseInfo.h"
#include "KARRI/Algorithms/KaRRi/PDDistanceQueries/PDDistances.h"
#include <TaxiResult.h>

namespace karri {
    // Given a ride request r, this facility finds the optimal assignment of r to the route of a ride-pooling
    // vehicle and a pickup and dropoff location, according to the current state of all vehicle routes.
    template<
        typename OrdAssignmentsT,
        typename PbnsAssignmentsT,
        typename PalsAssignmentsT,
        typename DalsAssignmentsT,
        typename RepositioningAssignmentsT
    >
    class TaxiTripFinder {
    public:
        TaxiTripFinder(OrdAssignmentsT &ordinaryAssigments,
                            PbnsAssignmentsT &pbnsAssignments,
                            PalsAssignmentsT &palsAssignments,
                            DalsAssignmentsT &dalsAssignments,
                            RepositioningAssignmentsT &repositioningAssignments)
            : ordAssignments(ordinaryAssigments),
              pbnsAssignments(pbnsAssignments),
              palsAssignments(palsAssignments),
              dalsAssignments(dalsAssignments),
              repositioningAssignments(repositioningAssignments) {
        }

        TaxiResult findBestAssignment(const RequestState &requestState, const KaRRiBaseInfo &baseInfo, stats::TaxiPerformanceStats &stats) {
            TaxiResult result;

            initializeComponentsForRequest(requestState, baseInfo.pdLocs, stats);

            // Try PALS assignments:
            palsAssignments.findAssignments(requestState, baseInfo.pdDistances, baseInfo.pdLocs, result,
                                            stats.palsAssignmentsStats);

            // Try ordinary assignments:
            ordAssignments.findAssignments(baseInfo.relOrdinaryPickups, baseInfo.relOrdinaryDropoffs,
                                           requestState, baseInfo.pdDistances, baseInfo.pdLocs,
                                           result, stats.ordAssignmentsStats);

            // Try DALS assignments:
            dalsAssignments.findAssignments(baseInfo.relOrdinaryPickups, baseInfo.relPickupsBeforeNextStop,
                                            requestState, baseInfo.pdLocs, result, stats.dalsAssignmentsStats);

            // Try PBNS assignments:
            pbnsAssignments.findAssignments(baseInfo.relPickupsBeforeNextStop, baseInfo.relOrdinaryDropoffs,
                                            baseInfo.relDropoffsBeforeNextStop, requestState,
                                            baseInfo.pdDistances, baseInfo.pdLocs, result, stats.pbnsAssignmentsStats);

            // Try repositioning assignments (vehicles currently being repositioned):
            repositioningAssignments.findAssignments(requestState, baseInfo.pdDistances, baseInfo.pdLocs, result,
                                                     stats.repositioningAssignmentsStats);

            return result;
        }

        void initializeComponentsForRequest(const RequestState &requestState, const PDLocs &pdLocs,
                                            stats::TaxiPerformanceStats &stats) {
            // Initialize components according to new request state:
            ordAssignments.init(requestState, pdLocs, stats.ordAssignmentsStats);
            pbnsAssignments.init(requestState, pdLocs, stats.pbnsAssignmentsStats);
            palsAssignments.init(requestState, pdLocs, stats.palsAssignmentsStats);
            dalsAssignments.init(requestState, pdLocs, stats.dalsAssignmentsStats);
            repositioningAssignments.init();
        }


        // Tries ordinary assignments where pickup and dropoff are inserted between existing stops.
        OrdAssignmentsT &ordAssignments;
        // Tries PBNS assignments where pickup (and possibly dropoff) is inserted before the next vehicle stop.
        PbnsAssignmentsT &pbnsAssignments;
        // Tries PALS assignments where pickup and dropoff are inserted after the last stop.
        PalsAssignmentsT &palsAssignments;
        // Tries DALS assignments where only the dropoff is inserted after the last stop.
        DalsAssignmentsT &dalsAssignments;
        // Tries repositioning assignments using a vehicle that is currently being repositioned.
        RepositioningAssignmentsT &repositioningAssignments;
    };
}
