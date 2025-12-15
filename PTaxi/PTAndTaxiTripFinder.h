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
            typename PTAlgorithmT,
            typename PTAlgorithmWithTaxiT,
            typename TaxiLegApproximationT
    >
    class PTAndTaxiTripFinder {
    using PDDistancesT = FfPdDistanceSearchesT::PDDistancesT;


    public:
    
        PTAndTaxiTripFinder(RequestStateInitializerT &requestStateInitializer,
                            PDLocsFinderT &pdLocsFinder,
                            PdLocsAtExistingStopsFinderT &pdLocsAtExistingStopsFinder,
                            FeasibleEllipticDistancesT& feasibleEllipticPickups,
                            FeasibleEllipticDistancesT& feasibleEllipticDropoffs,
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
                            PTAlgorithmT &ptAlgorithm,
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
                  ptAlgorithm(ptAlgorithm),
                  ptAlgorithmWithTaxi(ptAlgorithmWithTaxi),
                  taxiLegApproximation(vehInputGraph, vehChEnv, stationBucketsEnv, stations.size()),
                  curRelOrdinaryPickups(fleet.size()),
                  curRelPickupsBns(fleet.size()), 
                  bestCost(INFTY),
                  intermediateLogger(LogManager<std::ofstream>::getLogger(stats::IntermediateResultStats::LOGGER_NAME,
                                                "request_id," +
                                                std::string(stats::IntermediateResultStats::LOGGER_COLS))),
                  firstTaxiLegLogger(LogManager<std::ofstream>::getLogger(stats::FirstTaxiLegResultStats::LOGGER_NAME,
                                                "request_id," +
                                                std::string(stats::FirstTaxiLegResultStats::LOGGER_COLS))),
                  ptLogger(LogManager<std::ofstream>::getLogger(stats::PTResultStats::LOGGER_NAME,
                                                "request_id," +
                                                std::string(stats::PTResultStats::LOGGER_COLS))) {}

        PTAndTaxiTriple findBestAssignment(const Request &req) {
            // Taxi only leg and invalid taxi leg
            auto taxiOnlyResponse = findBestTaxiAssignment(req);
            RequestState invalidTaxiResponse;
            std::pair<RequestState, stats::DispatchingPerformanceStats> invalidTaxiResponseWithStats{invalidTaxiResponse, stats::DispatchingPerformanceStats()};
            
            const auto query = queries[req.requestId];
            ptAlgorithm.run(query.source, query.departureTime, query.target);
            auto ptOnlyParetoFront = ptAlgorithm.getJourneys();

            PTResult ptOnlyResponse(ptOnlyParetoFront, curReqState);
            PTResult invalidPTResponse;

            size_t ptOnlyLegCount = ptOnlyResponse.getBestJourney().size();

            const bool taxiOnlyHasBetterCost = taxiOnlyResponse.first.getBestCost() < ptOnlyResponse.getBestCost();
            bestCost = taxiOnlyHasBetterCost ? taxiOnlyResponse.first.getBestCost() : ptOnlyResponse.getBestCost();

            const FirstTaxiLegResult &firstTaxiLeg = runFirstTaxiSharingLeg(taxiOnlyResponse.second);

            const int maxTripTime = taxiOnlyResponse.first.getOriginalReqMaxTripTime();

            taxiLegApproximation.findDistancesFromStationsToDest(req.destination, maxTripTime);
            const auto &distFromStations = taxiLegApproximation.getDistancesFromStations();

            ptAlgorithmWithTaxi.run(query.source, query.departureTime, query.target, firstTaxiLeg, distFromStations);
            auto ptLegParetoFront = ptAlgorithmWithTaxi.getJourneys();
            PTResult ptLegResponse(ptLegParetoFront, curReqState);

            size_t ptLegCount = ptLegResponse.getBestJourney().size();

            // first taxi leg + PT journey + 2nd taxi leg approximation
            IntermediateResult<TaxiLegApproximationT> intermediateResult(req.requestTime, 
                                                                        maxTripTime, 
                                                                        firstTaxiLeg, 
                                                                        ptLegResponse, 
                                                                        taxiLegApproximation
                                                                    );

            constexpr const char* InsertionTypes[] = {"PALS", "DALS", "DALS_PBNS", "ORDINARY", "PBNS", "UNDEFINED"};
            // LOGS: Cost of taxi, PT, combined; arrivalTimes

            intermediateLogger << req.requestId << ", "
                    << taxiOnlyResponse.first.getBestCost() << ", "
                    << ptOnlyResponse.getBestCost() << ", "
                    << intermediateResult.getBestCost() << ", "
                    << intermediateResult.getFirstTaxiLegCost() << ", "
                    << InsertionTypes[intermediateResult.getFirstTaxiLegInsertionType()] << ", "
                    << intermediateResult.getPTLegCost() << ", "
                    << intermediateResult.getSecondTaxiLegCost() << ", "
                    << taxiOnlyResponse.first.getArrivalTime(routeState) << ", "
                    << ptOnlyResponse.getArrivalTime() << ", "
                    << intermediateResult.getArrivalTime() << "\n";

            const auto firstTaxiLegResults = firstTaxiLeg.getValidResults();
            const size_t validResultsCount = firstTaxiLegResults.size();

            int sumCost = 0, sumArrivalTime = 0;
            int insertionTypeCounts[] = {0, 0, 0, 0, 0, 0}; // PALS, DALS, DALS_PBNS, ORDINARY, PBNS, UNDEFINED

            for (const auto& result : firstTaxiLegResults) {
                sumCost += result.bestCost;
                sumArrivalTime += result.arrivalTime;
                insertionTypeCounts[result.insertionType]++;
            }

            const double averageCost = firstTaxiLegResults.empty() ? 0.0 : static_cast<double>(sumCost) / validResultsCount;
            const double averageArrivalTime = firstTaxiLegResults.empty() ? 0.0 : static_cast<double>(sumArrivalTime) / validResultsCount;
            
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
                auto &firstTaxiLeg = intermediateResult.getFirstTaxiLeg();
                // No first taxi leg
                if (firstTaxiLeg.insertionType == UNDEFINED && firstTaxiLeg.bestCost == INFTY) {
                    return PTAndTaxiTriple(invalidTaxiResponseWithStats, ptLegResponse, true);
                }
                curReqState.tryAssignmentWithKnownCost(firstTaxiLeg.bestAssignment, firstTaxiLeg.bestCost);
                return PTAndTaxiTriple(
                    {curReqState, taxiOnlyResponse.second},
                    ptLegResponse,
                    true
                );
            } 
            
            if (taxiOnlyHasBetterCost) {
                return PTAndTaxiTriple(taxiOnlyResponse, invalidPTResponse, false);
            } else {
                return PTAndTaxiTriple(invalidTaxiResponseWithStats, ptOnlyResponse, false);
            }
        }

        std::pair<RequestState, stats::DispatchingPerformanceStats> findBestTaxiAssignment(const Request &req) {

            // Initialize finder for this request, find PD locations:
            std::pair<RequestState, stats::DispatchingPerformanceStats> initRequestState = requestStateInitializer.initializeRequestState(req);
            RequestState rs = std::move(initRequestState.first);
            stats::DispatchingPerformanceStats stats = std::move(initRequestState.second);
            PDLocs pdLocs = pdLocsFinder.findPDLocs(req.origin, req.destination, stats.initializationStats);
            stats.numPickups = pdLocs.numPickups();
            stats.numDropoffs = pdLocs.numDropoffs();
            initializeComponentsForRequest(rs, pdLocs, stats);
            
            stats::DispatchingPerformanceStats taxiOnlyStats = stats;
            curPdLocs = pdLocs;
            curReqState = rs;

            // Compute PD distances:
            const auto ffPdDistances = ffPDDistanceSearches.run(rs, pdLocs, taxiOnlyStats.pdDistancesStats);

            // Try PALS assignments:
            palsAssignments.findAssignments(rs, ffPdDistances, pdLocs, taxiOnlyStats.palsAssignmentsStats);

            // Run elliptic BCH searches (populates feasibleEllipticPickups and feasibleEllipticDropoffs):
            ellipticBchSearches.run(feasibleEllipticPickups, feasibleEllipticDropoffs, rs, pdLocs, taxiOnlyStats.ellipticBchStats);

            // Filter feasible PD-locations between ordinary stops:
            const auto &relOrdinaryPickups = relevantPdLocsFilter.filterOrdinaryPickups(feasibleEllipticPickups, rs, pdLocs,
                                                                                       taxiOnlyStats.ordAssignmentsStats);
            const auto &relOrdinaryDropoffs = relevantPdLocsFilter.filterOrdinaryDropoffs(feasibleEllipticDropoffs,
                                                                                         rs, pdLocs, taxiOnlyStats.ordAssignmentsStats);


            curRelOrdinaryPickups = relOrdinaryPickups;

            // Try ordinary assignments:
            ordAssignments.findAssignments(relOrdinaryPickups, relOrdinaryDropoffs, rs, ffPdDistances, pdLocs, taxiOnlyStats.ordAssignmentsStats);

            // Filter feasible pickups before next stops:
            const auto &relPickupsBeforeNextStop = relevantPdLocsFilter.filterPickupsBeforeNextStop(
                    feasibleEllipticPickups, rs, pdLocs, taxiOnlyStats.pbnsAssignmentsStats);

            curRelPickupsBns = relPickupsBeforeNextStop;

            // Try DALS assignments:
            dalsAssignments.findAssignments(relOrdinaryPickups, relPickupsBeforeNextStop, rs, pdLocs, taxiOnlyStats.dalsAssignmentsStats);

            // Filter feasible dropoffs before next stop:
            const auto relDropoffsBeforeNextStop = relevantPdLocsFilter.filterDropoffsBeforeNextStop(
                    feasibleEllipticDropoffs, rs, pdLocs, taxiOnlyStats.pbnsAssignmentsStats);

            // Try PBNS assignments:
            pbnsAssignments.findAssignments(relPickupsBeforeNextStop, relOrdinaryDropoffs, relDropoffsBeforeNextStop,
                                            rs, ffPdDistances, pdLocs, taxiOnlyStats.pbnsAssignmentsStats);

            return {rs, stats};
        }

    private:

        FirstTaxiLegResult runFirstTaxiSharingLeg(stats::DispatchingPerformanceStats &stats) {
            FirstTaxiLegResult firstTaxiLegResult(routeState, curReqState, stations.size());

            runPALS(curReqState, stats.palsAssignmentsStats, firstTaxiLegResult);
            runOrdinary(curReqState, stats.ordAssignmentsStats, firstTaxiLegResult);
            runDALS(curReqState, stats.dalsAssignmentsStats, firstTaxiLegResult);
            runPBNS(curReqState, stats.pbnsAssignmentsStats, firstTaxiLegResult);

            // -> assignment with best cost for each PT station
            return firstTaxiLegResult;
        }

        void runPALS(RequestState &rs, stats::PalsAssignmentsPerformanceStats &stats, FirstTaxiLegResult &firstTaxiLegResult) {
            // Run BCH queries from origin to all stations
            // reachable pickups from origin from KaRRi
            stationBCH.setExternalCostUpperBound(bestCost);
            stationBCH.runBchQueries(rs, curPdLocs);
                        
            // last stop -> pickups
            // PALS Individual BCH
            palsToStations.setExternalCostUpperBound(bestCost, firstTaxiLegResult.getWorstCostForAllStations());
            palsToStations.tryPickupAfterLastStop(rs, curPdLocs, stationBCH.getTentativeDistances(), stations, stats, firstTaxiLegResult);
        }

        void runOrdinary(RequestState &rs, stats::OrdAssignmentsPerformanceStats &stats, FirstTaxiLegResult &firstTaxiLegResult) {
            ordinaryToStations.enumerateAssignments(rs, curPdLocs, curRelOrdinaryPickups, stations, stationsInEllipse, stationBCH.getTentativeDistances(), stats, firstTaxiLegResult);
        }

        void runDALS(RequestState &rs, stats::DalsAssignmentsPerformanceStats &stats, FirstTaxiLegResult &firstTaxiLegResult) {
            dalsToStations.setExternalCostUpperBound(bestCost, firstTaxiLegResult.getWorstCostForAllStations());
            dalsToStations.tryDropoffAfterLastStop(rs, curPdLocs, curRelOrdinaryPickups, curRelPickupsBns, stats, firstTaxiLegResult);
        }

        void runPBNS(RequestState &rs, stats::PbnsAssignmentsPerformanceStats &stats, FirstTaxiLegResult &firstTaxiLegResult) {
            pbnsToStations.setExternalCostUpperBound(bestCost, firstTaxiLegResult.getWorstCostForAllStations());
            pbnsToStations.findAssignments(rs, curPdLocs, curRelPickupsBns, stations, stationsInEllipse, stationBCH.getTentativeDistances(), stats, firstTaxiLegResult);
        }

        void initializeComponentsForRequest(const RequestState& requestState, const PDLocs &pdLocs, stats::DispatchingPerformanceStats& stats) {
            feasibleEllipticPickups.init(pdLocs.numPickups(), stats.ellipticBchStats);
            auto pickupsAtExistingStops = pdLocsAtExistingStopsFinder.template findPDLocsAtExistingStops<PICKUP>(pdLocs.pickups, stats.ellipticBchStats);
            feasibleEllipticPickups.initializeDistancesForPdLocsAtExistingStops(std::move(pickupsAtExistingStops), vehInputGraph, stats.ellipticBchStats);

            feasibleEllipticDropoffs.init(pdLocs.numDropoffs(), stats.ellipticBchStats);
            auto dropoffsAtExistingStops = pdLocsAtExistingStopsFinder.template findPDLocsAtExistingStops<DROPOFF>(pdLocs.dropoffs, stats.ellipticBchStats);
            feasibleEllipticDropoffs.initializeDistancesForPdLocsAtExistingStops(std::move(dropoffsAtExistingStops), vehInputGraph, stats.ellipticBchStats);

            // Initialize components according to new request state:
            ellipticBchSearches.init(requestState, pdLocs, stats.ellipticBchStats);
            ffPDDistanceSearches.init(requestState, pdLocs, stats.pdDistancesStats);
            ordAssignments.init(requestState, pdLocs, stats.ordAssignmentsStats);
            pbnsAssignments.init(requestState, pdLocs, stats.pbnsAssignmentsStats);
            palsAssignments.init(requestState, pdLocs, stats.palsAssignmentsStats);
            dalsAssignments.init(requestState, pdLocs, stats.dalsAssignmentsStats);
        }

        // Assignment Finder components
        FeasibleEllipticDistancesT& feasibleEllipticPickups;
        FeasibleEllipticDistancesT& feasibleEllipticDropoffs;

        RequestStateInitializerT &requestStateInitializer;
        PDLocsFinderT &pdLocsFinder; // Finds possible pickup and dropoff locations for a given request
        const PdLocsAtExistingStopsFinderT &pdLocsAtExistingStopsFinder; // Identifies pd locs that coincide with existing stops
        EllipticBchSearchesT &ellipticBchSearches; // Elliptic BCH searches that find distances between existing stops and PD-locations (except after last stop).
        FfPdDistanceSearchesT &ffPDDistanceSearches; // PD-distance searches that compute distances from pickups to dropoffs.
        OrdAssignmentsT &ordAssignments; // Tries ordinary assignments where pickup and dropoff are inserted between existing stops.
        PbnsAssignmentsT &pbnsAssignments; // Tries PBNS assignments where pickup (and possibly dropoff) is inserted before the next vehicle stop.
        PalsAssignmentsT &palsAssignments; // Tries PALS assignments where pickup and dropoff are inserted after the last stop.
        DalsAssignmentsT &dalsAssignments; // Tries DALS assignments where only the dropoff is inserted after the last stop.
        RelevantPdLocsFilterT &relevantPdLocsFilter; // Additionally filters feasible pickups/dropoffs found by elliptic BCH searches.

        const VehicleInputGraphT &vehInputGraph;
        const CH &vehCh;

        const Fleet &fleet;
        RouteState &routeState;

        RelevantPDLocs curRelOrdinaryPickups;
        RelevantPDLocs curRelPickupsBns;
        PDLocs curPdLocs;

        PTStations stations;
        StationBucketsEnvT &stationBucketsEnv;
        StationBCHQueryT stationBCH;
        PALSToStationsT &palsToStations;

        StationsInEllipseT &stationsInEllipse;
        OrdinaryToStationsT ordinaryToStations;

        DALSToStationsT &dalsToStations;
        PBNSToStationsT &pbnsToStations;

        const std::vector<PTQueryT> &queries;
        PTAlgorithmT &ptAlgorithm;
        PTAlgorithmWithTaxiT &ptAlgorithmWithTaxi;

        TaxiLegApproximationT taxiLegApproximation;
        
        RequestState curReqState;
        int bestCost;

        std::ofstream &intermediateLogger;
        std::ofstream &firstTaxiLegLogger;
        std::ofstream &ptLogger;
    };
}