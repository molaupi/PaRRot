#pragma once
#include "KaRRiBaseInfo.h"
#include "KARRI/Algorithms/KaRRi/RouteState.h"

namespace karri {
    template<
        typename VehicleInputGraphT,
        typename VehCHEnvT,
        typename EllipticBCHSearchResultT,
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
        routeState(routeState),
              pickupEllipticResult(fleet.size(), routeState),
              dropoffEllipticResult(fleet.size(), routeState),
              relevantPdLocsFilter(fleet, vehInputGraph, vehChEnv, routeState),
              pdLocsFinder(pdLocsFinder),
              pdLocsAtExistingStopsFinder(pdLocsAtExistingStopsFinder),
              ellipticBchSearches(ellipticBchSearches),
              pdDistanceSearches(pdDistanceSearches) {
        }

        KaRRiBaseInfo prepareBaseInfo(const RequestState &requestState, const bool isSecondTaxiLeg,
                                      stats::TaxiPrepStats &stats) {
            KaRRiBaseInfo bi;

            const auto &req = requestState.originalRequest;

            // Generate PDLocs
            bi.pdLocs = pdLocsFinder.findPDLocs(req.origin, req.destination, isSecondTaxiLeg,
                                                stats.initializationStats);
            stats.numPickups = bi.pdLocs.numPickups();
            stats.numDropoffs = bi.pdLocs.numDropoffs();

            initializeForRequest(requestState, bi.pdLocs, stats);

            // Run PD-distance queries
            bi.pdDistances = pdDistanceSearches.run(requestState, bi.pdLocs, stats.pdDistancesStats);

            // Run Elliptic BCH searches
            ellipticBchSearches.run(pickupEllipticResult, dropoffEllipticResult, requestState,
                                    bi.pdLocs, stats.ellipticBchStats);

            // Find feasible PDLocs in elliptic BCH search results
            bi.feasiblePickups.construct(pickupEllipticResult, bi.pdLocs.pickups, routeState, requestState, stats.filterOrdinaryPdLocsStats);
            bi.feasibleDropoffs.construct(dropoffEllipticResult, bi.pdLocs.dropoffs, routeState, requestState, stats.filterOrdinaryPdLocsStats);

            // Find relevant PDLocs for (non-paired) ordinary and PBNS assignments
            bi.relOrdinaryPickups = relevantPdLocsFilter.getRelevantOrdinaryPickups(
                bi.feasiblePickups, requestState, bi.pdLocs, stats.filterOrdinaryPdLocsStats);
            bi.relPickupsBeforeNextStop = relevantPdLocsFilter.getRelevantPickupsBeforeNextStop(
                bi.feasiblePickups, requestState, bi.pdLocs, stats.filterBnsPdLocsStats);
            bi.relOrdinaryDropoffs = relevantPdLocsFilter.getRelevantOrdinaryDropoffs(
                bi.feasibleDropoffs, requestState, bi.pdLocs, stats.filterOrdinaryPdLocsStats);

            return bi;
        }

    private:
        void initializeForRequest(const RequestState &requestState, const PDLocs &pdLocs,
                                  stats::TaxiPrepStats &stats) {
            pickupEllipticResult.init(pdLocs.numPickups(), stats.ellipticBchStats);
            auto pickupsAtExistingStops = pdLocsAtExistingStopsFinder.template findPDLocsAtExistingStops<PICKUP>(
                pdLocs.pickups, stats.ellipticBchStats);
            pickupEllipticResult.initializeDistancesForPdLocsAtExistingStops(
                std::move(pickupsAtExistingStops), vehInputGraph, stats.ellipticBchStats);

            dropoffEllipticResult.init(pdLocs.numDropoffs(), stats.ellipticBchStats);
            auto dropoffsAtExistingStops = pdLocsAtExistingStopsFinder.template findPDLocsAtExistingStops<DROPOFF>(
                pdLocs.dropoffs, stats.ellipticBchStats);
            dropoffEllipticResult.initializeDistancesForPdLocsAtExistingStops(
                std::move(dropoffsAtExistingStops), vehInputGraph, stats.ellipticBchStats);

            // Initialize components according to new request state:
            ellipticBchSearches.init(requestState, pdLocs, stats.ellipticBchStats);
            pdDistanceSearches.init(requestState, pdLocs, stats.pdDistancesStats);
        }

        const VehicleInputGraphT &vehInputGraph;
        const RouteState &routeState;

        EllipticBCHSearchResultT pickupEllipticResult;
        EllipticBCHSearchResultT dropoffEllipticResult;
        RelevantPdLocsFilterT relevantPdLocsFilter;

        PDLocsFinderT &pdLocsFinder;
        PdLocsAtExistingStopsFinderT &pdLocsAtExistingStopsFinder;
        EllipticBchSearchesT &ellipticBchSearches;
        PdDistanceSearchesT &pdDistanceSearches;
    };
}
