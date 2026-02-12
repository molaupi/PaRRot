#pragma once

#include "PTAndTaxiTriple.h"
#include "Station/Station.h"
#include "Stats/IntermediateResultStats.h"
#include <KARRI/Algorithms/CH/CH.h>
#include <KARRI/Algorithms/KaRRi/RequestState/RelevantPDLocs.h>

namespace karri {
    // Core of the PTaxi algorithm: Given a ride request r, this facility finds the optimal assignment of r to the route
    // of a vehicle and a pickup and dropoff location, according to the current state of all vehicle routes.
    template<
        typename FeasibleEllipticDistancesT,
        typename RequestStateInitializerT,
        typename PDLocsFinderT,
        typename PdLocsAtExistingStopsFinderT,
        typename EllipticBchSearchesT,
        typename FfPdDistanceSearchesT,
        typename OrdAssignmentsT,
        typename PbnsAssignmentsT,
        typename PalsAssignmentsT,
        typename DalsAssignmentsT,
        typename RelevantPdLocsFilterT,
        typename VehicleInputGraphT,
        typename VehCHEnvT,
        typename StationBucketsEnvT,
        typename StationBCHQueryT,
        typename PALSToStationsT,
        typename StationsInEllipseT,
        typename OrdinaryToStationsT,
        typename DALSToStationsT,
        typename PBNSToStationsT,
        typename PTQueryT,
        typename PTAlgorithmWithTaxiT,
        typename TaxiLegApproximationT
    >
    class PTAndTaxiTripFinder {
        using PDDistancesT = FfPdDistanceSearchesT::PDDistancesT;

    public:
        PTAndTaxiTripFinder(RequestStateInitializerT &requestStateInitializer,
                            PDLocsFinderT &pdLocsFinder,
                            PdLocsAtExistingStopsFinderT &pdLocsAtExistingStopsFinder,
                            FeasibleEllipticDistancesT &feasibleEllipticPickups,
                            FeasibleEllipticDistancesT &feasibleEllipticDropoffs,
                            EllipticBchSearchesT &ellipticBchSearches,
                            FfPdDistanceSearchesT &ffPdDistanceSearches,
                            OrdAssignmentsT &ordinaryAssigments,
                            PbnsAssignmentsT &pbnsAssignments,
                            PalsAssignmentsT &palsAssignments,
                            DalsAssignmentsT &dalsAssignments,
                            RelevantPdLocsFilterT &relevantPdLocsFilter,
                            const VehicleInputGraphT &vehInputGraph,
                            const VehCHEnvT &vehChEnv,
                            const Fleet &fleet,
                            RouteState &routeState,
                            PTStations stations,
                            const std::vector<PTQueryT> &queries,
                            StationBucketsEnvT &stationBucketsEnv,
                            PALSToStationsT &palsToStations,
                            StationsInEllipseT &stationsInEllipse,
                            DALSToStationsT &dalsToStations,
                            PBNSToStationsT &pbnsToStations,
                            PTAlgorithmWithTaxiT &ptAlgorithmWithTaxi)
            : feasibleEllipticPickups(feasibleEllipticPickups),
              feasibleEllipticDropoffs(feasibleEllipticDropoffs),
              requestStateInitializer(requestStateInitializer),
              pdLocsFinder(pdLocsFinder),
              pdLocsAtExistingStopsFinder(pdLocsAtExistingStopsFinder),
              ellipticBchSearches(ellipticBchSearches),
              ffPDDistanceSearches(ffPdDistanceSearches),
              ordAssignments(ordinaryAssigments),
              pbnsAssignments(pbnsAssignments),
              palsAssignments(palsAssignments),
              dalsAssignments(dalsAssignments),
              relevantPdLocsFilter(relevantPdLocsFilter),
              vehInputGraph(vehInputGraph),
              vehCh(vehChEnv.getCH()),
              routeState(routeState),
              fleet(fleet),
              stations(stations),
              queries(queries),
              stationBucketsEnv(stationBucketsEnv),
              stationBCH(vehInputGraph, vehChEnv, routeState, stationBucketsEnv, stations.size()),
              palsToStations(palsToStations),
              stationsInEllipse(stationsInEllipse),
              ordinaryToStations(fleet, routeState),
              dalsToStations(dalsToStations),
              pbnsToStations(pbnsToStations),
              ptAlgorithmWithTaxi(ptAlgorithmWithTaxi),
              taxiLegApproximation(vehInputGraph, vehChEnv, stationBucketsEnv, stations.size()),
              bestCost(INFTY),
              intermediateLogger(LogManager<std::ofstream>::getLogger(stats::IntermediateResultStats::LOGGER_NAME,
                                                                      "request_id," +
                                                                      std::string(
                                                                          stats::IntermediateResultStats::LOGGER_COLS))),
              firstTaxiLegLogger(LogManager<std::ofstream>::getLogger(stats::FirstTaxiLegResultStats::LOGGER_NAME,
                                                                      "request_id," +
                                                                      std::string(
                                                                          stats::FirstTaxiLegResultStats::LOGGER_COLS))),
              ptLogger(LogManager<std::ofstream>::getLogger(stats::PTResultStats::LOGGER_NAME,
                                                            "request_id," +
                                                            std::string(stats::PTResultStats::LOGGER_COLS))) {
        }

        PTAndTaxiTriple findBestAssignment(const Request &req, stats::RequestReceiveStats &stats) {
            // Initialize finder for this request, find PD locations:
            // TODO: initializationStats are for whole request not just taxi only
            RequestState rs = requestStateInitializer.initializeRequestState(
                req, req.requestTime, stats.taxiOnlyStats.initializationStats);
            const int maxTripTime = rs.getOriginalReqMaxTripTime();
            PDLocs pdLocs = pdLocsFinder.findPDLocs(req.origin, req.destination,
                                                    stats.taxiOnlyStats.initializationStats);
            stats.taxiOnlyStats.numPickups = pdLocs.numPickups();
            stats.taxiOnlyStats.numDropoffs = pdLocs.numDropoffs();
            stats.taxiFirstLegStats.numPickups = pdLocs.numPickups();
            stats.taxiFirstLegStats.numDropoffs = pdLocs.numDropoffs();

            RelevantPDLocs relOrdinaryPickups(fleet.size());
            RelevantPDLocs relPickupsBns(fleet.size());

            // Taxi only leg and invalid taxi leg
            findBestTaxiAssignment(rs, pdLocs, relOrdinaryPickups, relPickupsBns, stats.taxiOnlyStats);

            const auto query = queries[req.requestId];
            const int originPsgEdge = query.originPsgEdge;
            const int originVehEdge = query.originVehEdge;
            const int destPsgEdge = query.destinationPsgEdge;
            const int destVehEdge = query.destinationVehEdge;

            ptAlgorithmWithTaxi.run(originPsgEdge, originVehEdge, destPsgEdge, destVehEdge, query.departureTime, stats.ptOnlyStats);
            auto ptOnlyParetoFront = ptAlgorithmWithTaxi.getJourneys();

            PTResult ptOnlyResponse(ptOnlyParetoFront, maxTripTime);

            size_t ptOnlyLegCount = ptOnlyResponse.getBestJourney().size();

            const bool taxiOnlyHasBetterCost = rs.getBestCost() < ptOnlyResponse.getBestCost();
            bestCost = taxiOnlyHasBetterCost ? rs.getBestCost() : ptOnlyResponse.getBestCost();

            const FirstTaxiLegResult &firstTaxiLeg = runFirstTaxiSharingLeg(
                rs, pdLocs, relOrdinaryPickups, relPickupsBns, stats.taxiFirstLegStats);


            taxiLegApproximation.findDistancesFromStationsToDest(req.destination, maxTripTime,
                                                                 stats.taxiFirstLegStats.stationBchStats);
            const auto &distFromStations = taxiLegApproximation.getDistancesFromStations();

            ptAlgorithmWithTaxi.runWithTaxi(originPsgEdge, originVehEdge, destPsgEdge, destVehEdge,
                                            query.departureTime, firstTaxiLeg, distFromStations, stats.ptWithTaxiStats);
            auto ptLegParetoFront = ptAlgorithmWithTaxi.getJourneys();
            PTResult ptLegResponse(ptLegParetoFront, maxTripTime);

            size_t ptLegCount = ptLegResponse.getBestJourney().size();

            // first taxi leg + PT journey + 2nd taxi leg approximation
            IntermediateResult<TaxiLegApproximationT> intermediateResult(req.requestTime,
                                                                         maxTripTime,
                                                                         firstTaxiLeg,
                                                                         ptLegResponse,
                                                                         taxiLegApproximation
            );

            constexpr const char *InsertionTypes[] = {"PALS", "DALS", "DALS_PBNS", "ORDINARY", "PBNS", "UNDEFINED"};
            // LOGS: Cost of taxi, PT, combined; arrivalTimes

            intermediateLogger << req.requestId << ", "
                    << rs.getBestCost() << ", "
                    << ptOnlyResponse.getBestCost() << ", "
                    << intermediateResult.getBestCost() << ", "
                    << intermediateResult.getFirstTaxiLegCost() << ", "
                    << InsertionTypes[intermediateResult.getFirstTaxiLegInsertionType()] << ", "
                    << intermediateResult.getPTLegCost() << ", "
                    << intermediateResult.getSecondTaxiLegCost() << ", "
                    << rs.getArrivalTime(routeState) << ", "
                    << ptOnlyResponse.getArrivalTime() << ", "
                    << intermediateResult.getArrivalTime() << "\n";

            const auto firstTaxiLegResults = firstTaxiLeg.getValidResults();
            const size_t validResultsCount = firstTaxiLegResults.size();

            int sumCost = 0, sumArrivalTime = 0;
            int insertionTypeCounts[] = {0, 0, 0, 0, 0, 0}; // PALS, DALS, DALS_PBNS, ORDINARY, PBNS, UNDEFINED

            for (const auto &result: firstTaxiLegResults) {
                sumCost += result.bestCost;
                sumArrivalTime += result.arrivalTime;
                ++insertionTypeCounts[result.insertionType];
            }

            const double averageCost = firstTaxiLegResults.empty()
                                           ? 0.0
                                           : static_cast<double>(sumCost) / validResultsCount;
            const double averageArrivalTime = firstTaxiLegResults.empty()
                                                  ? 0.0
                                                  : static_cast<double>(sumArrivalTime) / validResultsCount;

            firstTaxiLegLogger << req.requestId << ", "
                    << averageCost << ", "
                    << averageArrivalTime << ", "
                    << insertionTypeCounts[0] << ", "
                    << insertionTypeCounts[1] << ", "
                    << insertionTypeCounts[2] << ", "
                    << insertionTypeCounts[3] << ", "
                    << insertionTypeCounts[4] << ", "
                    << insertionTypeCounts[5] << ", "
                    << validResultsCount << "\n";

            ptLogger << req.requestId << ", "
                    << ptOnlyResponse.getBestCost() << ", "
                    << ptOnlyLegCount << ", "
                    << intermediateResult.getPTLegCost() << ", "
                    << ptLegCount << "\n";


            const bool combinationIsBestCost = intermediateResult.getBestCost() < bestCost;
            bestCost = combinationIsBestCost ? intermediateResult.getBestCost() : bestCost;

            if (combinationIsBestCost) {
                auto &chosenFirstTaxiLeg = intermediateResult.getFirstTaxiLeg();
                // No first taxi leg
                if (chosenFirstTaxiLeg.insertionType == UNDEFINED || chosenFirstTaxiLeg.bestCost == INFTY) {
                    return PTAndTaxiTriple(RequestState(), ptLegResponse, true,
                                           0, intermediateResult.getPTLegCost(),
                                           intermediateResult.getSecondTaxiLegCost());
                }
                rs.tryAssignmentWithKnownCost(chosenFirstTaxiLeg.bestAssignment, chosenFirstTaxiLeg.bestCost);
                return PTAndTaxiTriple(
                    rs,
                    ptLegResponse,
                    true,
                    intermediateResult.getFirstTaxiLegCost(),
                    intermediateResult.getPTLegCost(),
                    intermediateResult.getSecondTaxiLegCost()
                );
            }

            if (taxiOnlyHasBetterCost) {
                PTResult invalidPTResponse;
                return PTAndTaxiTriple(rs, invalidPTResponse, false, rs.getBestCost(),
                                       0, 0);
            }

            return PTAndTaxiTriple(RequestState(), ptOnlyResponse, false, 0,
                                   ptOnlyResponse.getBestCost(), 0);
        }

        RequestState findBestSecondTaxiLeg(const Request& req, const int now, stats::TaxiPerformanceStats &stats) {
            RequestState rs = requestStateInitializer.initializeRequestState(
                req, now, stats.initializationStats);
            PDLocs pdLocs = pdLocsFinder.findPDLocs(req.origin, req.destination, stats.initializationStats);
            stats.numPickups = pdLocs.numPickups();
            stats.numDropoffs = pdLocs.numDropoffs();
            RelevantPDLocs relOrdinaryPickups(fleet.size());
            RelevantPDLocs relPickupsBns(fleet.size());

            findBestTaxiAssignment(rs, pdLocs, relOrdinaryPickups, relPickupsBns, stats);

            return rs;
        }

    private:
        void findBestTaxiAssignment(RequestState &rs, const PDLocs &pdLocs,
                                    RelevantPDLocs &relOrdinaryPickups,
                                    RelevantPDLocs &relPickupsBeforeNextStop,
                                    stats::TaxiPerformanceStats &stats) {
            initializeComponentsForRequest(rs, pdLocs, stats);

            // Compute PD distances:
            const auto ffPdDistances = ffPDDistanceSearches.run(rs, pdLocs, stats.pdDistancesStats);

            // Try PALS assignments:
            palsAssignments.findAssignments(rs, ffPdDistances, pdLocs, stats.palsAssignmentsStats);

            // Run elliptic BCH searches (populates feasibleEllipticPickups and feasibleEllipticDropoffs):
            ellipticBchSearches.run(feasibleEllipticPickups, feasibleEllipticDropoffs, rs, pdLocs,
                                    stats.ellipticBchStats);

            // Filter feasible PD-locations between ordinary stops:
            relOrdinaryPickups = relevantPdLocsFilter.filterOrdinaryPickups(
                feasibleEllipticPickups, rs, pdLocs,
                stats.ordAssignmentsStats);
            const auto &relOrdinaryDropoffs = relevantPdLocsFilter.filterOrdinaryDropoffs(feasibleEllipticDropoffs,
                rs, pdLocs, stats.ordAssignmentsStats);

            // Try ordinary assignments:
            ordAssignments.findAssignments(relOrdinaryPickups, relOrdinaryDropoffs, rs, ffPdDistances, pdLocs,
                                           stats.ordAssignmentsStats);

            // Filter feasible pickups before next stops:
            relPickupsBeforeNextStop = relevantPdLocsFilter.filterPickupsBeforeNextStop(
                feasibleEllipticPickups, rs, pdLocs, stats.pbnsAssignmentsStats);

            // Try DALS assignments:
            dalsAssignments.findAssignments(relOrdinaryPickups, relPickupsBeforeNextStop, rs, pdLocs,
                                            stats.dalsAssignmentsStats);

            // Filter feasible dropoffs before next stop:
            const auto relDropoffsBeforeNextStop = relevantPdLocsFilter.filterDropoffsBeforeNextStop(
                feasibleEllipticDropoffs, rs, pdLocs, stats.pbnsAssignmentsStats);

            // Try PBNS assignments:
            pbnsAssignments.findAssignments(relPickupsBeforeNextStop, relOrdinaryDropoffs, relDropoffsBeforeNextStop,
                                            rs, ffPdDistances, pdLocs, stats.pbnsAssignmentsStats);
        }

        FirstTaxiLegResult runFirstTaxiSharingLeg(const RequestState &rs, const PDLocs &pdLocs,
                                                  const RelevantPDLocs &relOrdinaryPickups,
                                                  const RelevantPDLocs &relPickupsBeforeNextStop,
                                                  stats::TaxiPerformanceStats &stats) {
            FirstTaxiLegResult firstTaxiLegResult(routeState, rs, stations.size());

            runStationBCH(rs, pdLocs, stats.stationBchStats);
            runPALS(rs, pdLocs, stats.palsAssignmentsStats, firstTaxiLegResult);
            runOrdinary(rs, pdLocs, relOrdinaryPickups, stats.ordAssignmentsStats, firstTaxiLegResult);
            runDALS(rs, pdLocs, relOrdinaryPickups, relPickupsBeforeNextStop, stats.dalsAssignmentsStats,
                    firstTaxiLegResult);
            runPBNS(rs, pdLocs, relPickupsBeforeNextStop, stats.pbnsAssignmentsStats, firstTaxiLegResult);

            // -> assignment with best cost for each PT station
            return firstTaxiLegResult;
        }

        void runStationBCH(const RequestState &rs, const PDLocs &pdLocs, stats::StationBchPerformanceStats &stats) {
            // Run BCH queries from origin to all stations reachable pickups from origin from KaRRi
            stationBCH.setExternalCostUpperBound(bestCost);
            stationBCH.runBchQueries(rs, pdLocs, stats);
        }

        void runPALS(const RequestState &rs, const PDLocs &pdLocs, stats::PalsAssignmentsPerformanceStats &stats,
                     FirstTaxiLegResult &firstTaxiLegResult) {
            // last stop -> pickups
            // PALS Individual BCH
            palsToStations.setExternalCostUpperBound(bestCost, firstTaxiLegResult.getWorstCostForAllStations());
            palsToStations.tryPickupAfterLastStop(rs, pdLocs, stationBCH.getTentativeDistances(),
                                                  stationBCH.getStationsSeen(), stations, stats, firstTaxiLegResult);
        }

        void runOrdinary(const RequestState &rs, const PDLocs &pdLocs,
                         const RelevantPDLocs &relOrdinaryPickpus, stats::OrdAssignmentsPerformanceStats &stats,
                         FirstTaxiLegResult &firstTaxiLegResult) {
            ordinaryToStations.enumerateAssignments(rs, pdLocs, relOrdinaryPickpus, stations, stationsInEllipse,
                                                    stationBCH.getTentativeDistances(), stats, firstTaxiLegResult);
        }

        void runDALS(const RequestState &rs, const PDLocs &pdLocs,
                     const RelevantPDLocs &relOrdinaryPickups, const RelevantPDLocs &relPickupsBeforeNextStop,
                     stats::DalsAssignmentsPerformanceStats &stats,
                     FirstTaxiLegResult &firstTaxiLegResult) {
            dalsToStations.setExternalCostUpperBound(bestCost, firstTaxiLegResult.getWorstCostForAllStations());
            dalsToStations.tryDropoffAfterLastStop(rs, pdLocs, relOrdinaryPickups, relPickupsBeforeNextStop, stats,
                                                   firstTaxiLegResult);
        }

        void runPBNS(const RequestState &rs, const PDLocs &pdLocs,
                     const RelevantPDLocs &relPickupsBns,
                     stats::PbnsAssignmentsPerformanceStats &stats,
                     FirstTaxiLegResult &firstTaxiLegResult) {
            pbnsToStations.setExternalCostUpperBound(bestCost, firstTaxiLegResult.getWorstCostForAllStations());
            pbnsToStations.findAssignments(rs, pdLocs, relPickupsBns, stations, stationsInEllipse,
                                           stationBCH.getTentativeDistances(), stats, firstTaxiLegResult);
        }

        void initializeComponentsForRequest(const RequestState &requestState, const PDLocs &pdLocs,
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
            ffPDDistanceSearches.init(requestState, pdLocs, stats.pdDistancesStats);
            ordAssignments.init(requestState, pdLocs, stats.ordAssignmentsStats);
            pbnsAssignments.init(requestState, pdLocs, stats.pbnsAssignmentsStats);
            palsAssignments.init(requestState, pdLocs, stats.palsAssignmentsStats);
            dalsAssignments.init(requestState, pdLocs, stats.dalsAssignmentsStats);
        }

        // Assignment Finder components
        FeasibleEllipticDistancesT &feasibleEllipticPickups;
        FeasibleEllipticDistancesT &feasibleEllipticDropoffs;

        RequestStateInitializerT &requestStateInitializer;
        PDLocsFinderT &pdLocsFinder; // Finds possible pickup and dropoff locations for a given request
        const PdLocsAtExistingStopsFinderT &pdLocsAtExistingStopsFinder;
        // Identifies pd locs that coincide with existing stops
        EllipticBchSearchesT &ellipticBchSearches;
        // Elliptic BCH searches that find distances between existing stops and PD-locations (except after last stop).
        FfPdDistanceSearchesT &ffPDDistanceSearches;
        // PD-distance searches that compute distances from pickups to dropoffs.
        OrdAssignmentsT &ordAssignments;
        // Tries ordinary assignments where pickup and dropoff are inserted between existing stops.
        PbnsAssignmentsT &pbnsAssignments;
        // Tries PBNS assignments where pickup (and possibly dropoff) is inserted before the next vehicle stop.
        PalsAssignmentsT &palsAssignments;
        // Tries PALS assignments where pickup and dropoff are inserted after the last stop.
        DalsAssignmentsT &dalsAssignments;
        // Tries DALS assignments where only the dropoff is inserted after the last stop.
        RelevantPdLocsFilterT &relevantPdLocsFilter;
        // Additionally filters feasible pickups/dropoffs found by elliptic BCH searches.

        const VehicleInputGraphT &vehInputGraph;
        const CH &vehCh;

        const Fleet &fleet;
        RouteState &routeState;

        PTStations stations;
        StationBucketsEnvT &stationBucketsEnv;
        StationBCHQueryT stationBCH;
        PALSToStationsT &palsToStations;

        StationsInEllipseT &stationsInEllipse;
        OrdinaryToStationsT ordinaryToStations;

        DALSToStationsT &dalsToStations;
        PBNSToStationsT &pbnsToStations;

        const std::vector<PTQueryT> &queries;
        PTAlgorithmWithTaxiT &ptAlgorithmWithTaxi;

        TaxiLegApproximationT taxiLegApproximation;

        int bestCost;

        std::ofstream &intermediateLogger;
        std::ofstream &firstTaxiLegLogger;
        std::ofstream &ptLogger;
    };
}
