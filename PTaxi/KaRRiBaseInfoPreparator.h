#pragma once
#include "KaRRiBaseInfo.h"
#include "KARRI/Algorithms/KaRRi/RouteState.h"

namespace karri {
    template<
        typename VehicleInputGraphT,
        typename VehCHEnvT,
        typename FeasibleEllipticDistancesT,
        typename PDLocsFinderT,
        typename PdLocsAtExistingStopsFinderT,
        typename EllipticBchSearchesT,
        typename PdDistanceSearchesT,
        typename RelevantPdLocsFilterT
    >
    class KaRRiBaseInfoPreparator {

    public:
        KaRRiBaseInfoPreparator(const VehicleInputGraphT &vehInputGraph,
                                const VehCHEnvT &vehChEnv,
                                const Fleet &fleet,
                                const RouteState &routeState,
                                PDLocsFinderT &pdLocsFinder,
                                PdLocsAtExistingStopsFinderT &pdLocsAtExistingStopsFinder,
                                EllipticBchSearchesT &ellipticBchSearches,
                                PdDistanceSearchesT &pdDistanceSearches)
            : vehInputGraph(vehInputGraph),
              feasibleEllipticPickups(fleet.size(), routeState),
              feasibleEllipticDropoffs(fleet.size(), routeState),
              relevantPdLocsFilter(fleet, vehInputGraph, vehChEnv, routeState),
              pdLocsFinder(pdLocsFinder),
              pdLocsAtExistingStopsFinder(pdLocsAtExistingStopsFinder),
              ellipticBchSearches(ellipticBchSearches),
              pdDistanceSearches(pdDistanceSearches) {
        }

        KaRRiBaseInfo prepareBaseInfo(const RequestState &requestState, stats::TaxiPerformanceStats &stats) {
            KaRRiBaseInfo bi;

            const auto &req = requestState.originalRequest;

            // Generate PDLocs
            bi.pdLocs = pdLocsFinder.findPDLocs(req.origin, req.destination, stats.initializationStats);
            stats.numPickups = bi.pdLocs.numPickups();
            stats.numDropoffs = bi.pdLocs.numDropoffs();

            initializeForRequest(requestState, bi.pdLocs, stats);

            // Run PD-distance queries
            bi.pdDistances = pdDistanceSearches.run(requestState, bi.pdLocs, stats.pdDistancesStats);

            // Run Elliptic BCH searches
            ellipticBchSearches.run(feasibleEllipticPickups, feasibleEllipticDropoffs, requestState,
                                    bi.pdLocs, stats.ellipticBchStats);

            // Filter elliptic BCH search results
            const int minDirectPdDist = bi.pdDistances.getMinDirectDistance();
            bi.relOrdinaryPickups = relevantPdLocsFilter.filterOrdinaryPickups(
                feasibleEllipticPickups, requestState, bi.pdLocs, minDirectPdDist, stats.ordAssignmentsStats);
            bi.relPickupsBeforeNextStop = relevantPdLocsFilter.filterPickupsBeforeNextStop(
                feasibleEllipticPickups, requestState, bi.pdLocs, minDirectPdDist, stats.pbnsAssignmentsStats);
            bi.relOrdinaryDropoffs = relevantPdLocsFilter.filterOrdinaryDropoffs(
                feasibleEllipticDropoffs, requestState, bi.pdLocs, minDirectPdDist, stats.ordAssignmentsStats);
            bi.relDropoffsBeforeNextStop = relevantPdLocsFilter.filterDropoffsBeforeNextStop(
                feasibleEllipticDropoffs, requestState, bi.pdLocs, minDirectPdDist, stats.pbnsAssignmentsStats);

            return bi;
        }

    private:
        void initializeForRequest(const RequestState &requestState, const PDLocs &pdLocs,
                                  stats::TaxiPerformanceStats &stats) {
            feasibleEllipticPickups.init(pdLocs.numPickups(), stats.ellipticBchStats);
            auto pickupsAtExistingStops = pdLocsAtExistingStopsFinder.template findPDLocsAtExistingStops<PICKUP>(
                pdLocs.pickups, stats.ellipticBchStats);
            feasibleEllipticPickups.initializeDistancesForPdLocsAtExistingStops(
                std::move(pickupsAtExistingStops), vehInputGraph, stats.ellipticBchStats);

            feasibleEllipticDropoffs.init(pdLocs.numDropoffs(), stats.ellipticBchStats);
            auto dropoffsAtExistingStops = pdLocsAtExistingStopsFinder.template findPDLocsAtExistingStops<DROPOFF>(
                pdLocs.dropoffs, stats.ellipticBchStats);
            feasibleEllipticDropoffs.initializeDistancesForPdLocsAtExistingStops(
                std::move(dropoffsAtExistingStops), vehInputGraph, stats.ellipticBchStats);

            // Initialize components according to new request state:
            ellipticBchSearches.init(requestState, pdLocs, stats.ellipticBchStats);
            pdDistanceSearches.init(requestState, pdLocs, stats.pdDistancesStats);
        }

        const VehicleInputGraphT &vehInputGraph;

        FeasibleEllipticDistancesT feasibleEllipticPickups;
        FeasibleEllipticDistancesT feasibleEllipticDropoffs;
        RelevantPdLocsFilterT relevantPdLocsFilter;

        PDLocsFinderT &pdLocsFinder;
        PdLocsAtExistingStopsFinderT &pdLocsAtExistingStopsFinder;
        EllipticBchSearchesT &ellipticBchSearches;
        PdDistanceSearchesT &pdDistanceSearches;
    };
}
