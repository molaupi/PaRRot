#pragma once

#include "PTAndTaxiTriple.h"
#include "Station.h"
#include "IntermediateResultStats.h"
#include <KARRI/Algorithms/CH/CH.h>
#include <KARRI/Algorithms/KaRRi/RequestState/RelevantPDLocs.h>
#include <ULTRA/DataStructures/Queries/Queries.h>
#include <ULTRA/Helpers/Vector/Permutation.h>

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
            typename PsgInputGraphT,
            typename PsgCHEnvT,
            typename StationBucketsEnvT,
            typename StationBCHQueryT,
            typename PALSToStationsT,
            typename StationsInEllipseT,
            typename OrdinaryToStationsT,
            typename DALSToStationsT,
            typename PBNSToStationsT,
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
                            const PsgInputGraphT &psgInputGraph,
                            const PsgCHEnvT &psgChEnv,
                            const Fleet &fleet,
                            RouteState &routeState,
                            PTStations stations,
                            StationBucketsEnvT &stationBucketsEnv,
                            PALSToStationsT &palsToStations,
                            StationsInEllipseT &stationsInEllipse,
                            DALSToStationsT &dalsToStations,
                            PBNSToStationsT &pbnsToStations,
                            PTAlgorithmT &ptAlgorithm,
                            Order &order,
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
                  psgInputGraph(psgInputGraph),
                  psgCh(psgChEnv.getCH()),
                  vehCh(vehChEnv.getCH()),
                  routeState(routeState),
                  fleet(fleet),
                  stations(stations),
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
                  chOrder(order),
                  curRelOrdinaryPickups(fleet.size()),
                  curRelPickupsBns(fleet.size()), 
                  bestCost(INFTY) {}

        PTAndTaxiTriple findBestAssignment(const Request &req) {
            // Taxi only leg and invalid taxi leg
            auto taxiOnlyResponse = findBestTaxiAssignment(req);
            RequestState invalidTaxiResponse;
            std::pair<RequestState, stats::DispatchingPerformanceStats> invalidTaxiResponseWithStats{invalidTaxiResponse, stats::DispatchingPerformanceStats()};
            
            VertexQuery query = convertKARRIRequestToULTRAQuery(req);
            ptAlgorithm.run(query.source, query.departureTime, query.target);
            auto ptOnlyParetoFront = ptAlgorithm.getJourneys();
            PTResult ptOnlyResponse(ptOnlyParetoFront, curReqState);
            PTResult invalidPTResponse;

            size_t ptOnlyLegCount = ptOnlyResponse.getBestJourney().size();

            const bool taxiOnlyHasBetterCost = taxiOnlyResponse.first.getBestCost() < ptOnlyResponse.getBestCost();
            bestCost = taxiOnlyHasBetterCost ? taxiOnlyResponse.first.getBestCost() : ptOnlyResponse.getBestCost();

            const auto &firstTaxiLeg = runFirstTaxiSharingLeg(req);

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
            LogManager<std::ofstream>::getLogger(stats::IntermediateResultStats::LOGGER_NAME,
                                                "request_id," +
                                                std::string(stats::IntermediateResultStats::LOGGER_COLS))
                    << req.requestId << ", "
                    << taxiOnlyResponse.first.getBestCost() << ", "
                    << ptOnlyResponse.getBestCost() << ", "
                    << intermediateResult.getBestCost() << ", "
                    << intermediateResult.getFirstTaxiLegCost() << ", "
                    << intermediateResult.getPTLegCost() << ", "
                    << intermediateResult.getSecondTaxiLegCost() << ", "
                    << taxiOnlyResponse.first.getArrivalTime(routeState) << ", "
                    << ptOnlyResponse.getArrivalTime() << ", "
                    << intermediateResult.getArrivalTime() << "\n";
            
             LogManager<std::ofstream>::getLogger("first_taxi_leg_results.csv",
                                                "request_id,insertion_type,valid_results_count\n")
                    << req.requestId << ", "
                    << InsertionTypes[intermediateResult.getFirstTaxiLegInsertionType()] << ", "
                    << firstTaxiLeg.countValidResults() << "\n";

            LogManager<std::ofstream>::getLogger("pt_results.csv",
                                                "request_id,pt_only_leg_count,pt_and_taxi_leg_count\n")
                    << req.requestId << ", "
                    << ptOnlyLegCount << ", "
                    << ptLegCount << "\n";

            const bool combinationIsBestCost = intermediateResult.getBestCost() < bestCost;

            // TODO: IntermediateResult -> PTAndTaxiTriple
            // if (combinationIsBestCost) return PTAndTaxiTriple();
            
            // Return the combined results
            if (taxiOnlyHasBetterCost) {
                return PTAndTaxiTriple(taxiOnlyResponse, invalidPTResponse, invalidTaxiResponse);
            } else {
                return PTAndTaxiTriple(invalidTaxiResponseWithStats, ptOnlyResponse, invalidTaxiResponse);
            }
        }

        std::pair<RequestState, stats::DispatchingPerformanceStats> findBestTaxiAssignment(const Request &req) {

            // Initialize finder for this request, find PD locations:
            std::pair<RequestState, stats::DispatchingPerformanceStats> initRequestState = requestStateInitializer.initializeRequestState(req);
            RequestState& rs = initRequestState.first;
            stats::DispatchingPerformanceStats& stats = initRequestState.second;
            PDLocs pdLocs = pdLocsFinder.findPDLocs(req.origin, req.destination, stats.initializationStats);
            stats.numPickups = pdLocs.numPickups();
            stats.numDropoffs = pdLocs.numDropoffs();
            initializeComponentsForRequest(rs, pdLocs, stats);

            curPdLocs = pdLocs;
            curReqState = rs;
            curStats = stats;

            // Compute PD distances:
            const auto ffPdDistances = ffPDDistanceSearches.run(rs, pdLocs, stats.pdDistancesStats);

            // Try PALS assignments:
            palsAssignments.findAssignments(rs, ffPdDistances, pdLocs, stats.palsAssignmentsStats);

            // Run elliptic BCH searches (populates feasibleEllipticPickups and feasibleEllipticDropoffs):
            ellipticBchSearches.run(feasibleEllipticPickups, feasibleEllipticDropoffs, rs, pdLocs, stats.ellipticBchStats);

            // Filter feasible PD-locations between ordinary stops:
            const auto &relOrdinaryPickups = relevantPdLocsFilter.filterOrdinaryPickups(feasibleEllipticPickups, rs, pdLocs,
                                                                                       stats.ordAssignmentsStats);
            const auto &relOrdinaryDropoffs = relevantPdLocsFilter.filterOrdinaryDropoffs(feasibleEllipticDropoffs,
                                                                                         rs, pdLocs, stats.ordAssignmentsStats);


            curRelOrdinaryPickups = relOrdinaryPickups;

            // Try ordinary assignments:
            ordAssignments.findAssignments(relOrdinaryPickups, relOrdinaryDropoffs, rs, ffPdDistances, pdLocs, stats.ordAssignmentsStats);

            // Filter feasible pickups before next stops:
            const auto &relPickupsBeforeNextStop = relevantPdLocsFilter.filterPickupsBeforeNextStop(
                    feasibleEllipticPickups, rs, pdLocs, stats.pbnsAssignmentsStats);

            curRelPickupsBns = relPickupsBeforeNextStop;

            // Try DALS assignments:
            dalsAssignments.findAssignments(relOrdinaryPickups, relPickupsBeforeNextStop, rs, pdLocs, stats.dalsAssignmentsStats);

            // Filter feasible dropoffs before next stop:
            const auto relDropoffsBeforeNextStop = relevantPdLocsFilter.filterDropoffsBeforeNextStop(
                    feasibleEllipticDropoffs, rs, pdLocs, stats.pbnsAssignmentsStats);

            // Try PBNS assignments:
            pbnsAssignments.findAssignments(relPickupsBeforeNextStop, relOrdinaryDropoffs, relDropoffsBeforeNextStop,
                                            rs, ffPdDistances, pdLocs, stats.pbnsAssignmentsStats);

            return {rs, stats};
        }

    private:

        VertexQuery convertKARRIRequestToULTRAQuery(const Request &req) {
            const auto origin = psgCh.rank(psgInputGraph.edgeHead(vehInputGraph.toPsgEdge(req.origin)));
            const auto destination = psgCh.rank(psgInputGraph.edgeHead(vehInputGraph.toPsgEdge(req.destination)));
            const auto requestTime = req.requestTime / 10;
            const Vertex originVertex = Vertex(chOrder[origin]);
            const Vertex destinationVertex = Vertex(chOrder[destination]);
            return VertexQuery(originVertex, destinationVertex, requestTime);
        }

        FirstTaxiLegResult runFirstTaxiSharingLeg(const Request &req) {
            RequestState rs = curReqState;
            FirstTaxiLegResult firstTaxiLegResult(routeState, rs, stations.size());
            stats::DispatchingPerformanceStats& stats = curStats;

            runPALS(rs, stats.palsAssignmentsStats, firstTaxiLegResult);
            runOrdinary(rs, stats.ordAssignmentsStats, firstTaxiLegResult);
            runDALS(rs, stats.dalsAssignmentsStats, firstTaxiLegResult);
            runPBNS(rs, stats.pbnsAssignmentsStats, firstTaxiLegResult);

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
        const PsgInputGraphT &psgInputGraph;
        const CH &psgCh;
        const CH &vehCh;

        const Fleet &fleet;
        RouteState &routeState;

        Order &chOrder;
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

        PTAlgorithmT &ptAlgorithm;
        PTAlgorithmWithTaxiT &ptAlgorithmWithTaxi;

        TaxiLegApproximationT taxiLegApproximation;
        
        RequestState curReqState;
        stats::DispatchingPerformanceStats curStats;
        int bestCost;
    };
}