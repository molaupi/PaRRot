#pragma once

#include "PTAndTaxiTriple.h"
#include "Station.h"
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
            typename PTAlgorithmT
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
                            PTAlgorithmT &ptAlgorithm,
                            Order &order)
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
                  ptAlgorithm(ptAlgorithm),
                  chOrder(order), 
                  bestCost(INFTY),
                  relPickups(fleet.size()) {}

        PTAndTaxiTriple findBestAssignment(const Request &req) {
            // Taxi only leg and invalid taxi leg
            auto taxiOnlyResponse = findBestTaxiAssignment(req);
            RequestState invalidTaxiResponse;
            
            VertexQuery query = convertKARRIRequestToULTRAQuery(req);
            ptAlgorithm.run(query.source, query.departureTime, query.target);
            
            // PT only leg and invalid PT leg
            auto paretoFront = ptAlgorithm.getJourneys();
            PTResult ptOnlyResponse(paretoFront, taxiOnlyResponse);
            PTResult invalidPTResponse;

            const bool taxiOnlyHasBestCost = taxiOnlyResponse.getBestCost() < ptOnlyResponse.getBestCost();
            bestCost = taxiOnlyHasBestCost ? taxiOnlyResponse.getBestCost() : ptOnlyResponse.getBestCost();

            auto firstTaxiLeg = runFirstTaxiSharingLeg(req);
            
            // Return the combined results
            if (taxiOnlyHasBestCost) {
                return PTAndTaxiTriple(taxiOnlyResponse, invalidPTResponse, invalidTaxiResponse);
            } else {
                return PTAndTaxiTriple(invalidTaxiResponse, ptOnlyResponse, invalidTaxiResponse);
            }
        }

        RequestState findBestTaxiAssignment(const Request &req) {

            // Initialize finder for this request, find PD locations:
            RequestState rs = requestStateInitializer.initializeRequestState(req);
            stats::DispatchingPerformanceStats& stats = rs.stats();
            PDLocs pdLocs = pdLocsFinder.findPDLocs(req.origin, req.destination, stats.initializationStats);
            stats.numPickups = pdLocs.numPickups();
            stats.numDropoffs = pdLocs.numDropoffs();
            initializeComponentsForRequest(rs, pdLocs, stats);

            relevantPdLocs = pdLocs;
            curReqState = rs;

            // Compute PD distances:
            const auto ffPdDistances = ffPDDistanceSearches.run(rs, pdLocs, stats.pdDistancesStats);

            // Try PALS assignments:
            palsAssignments.findAssignments(rs, ffPdDistances, pdLocs, stats.palsAssignmentsStats);

            // Run elliptic BCH searches (populates feasibleEllipticPickups and feasibleEllipticDropoffs):
            ellipticBchSearches.run(feasibleEllipticPickups, feasibleEllipticDropoffs, rs, pdLocs, stats.ellipticBchStats);

            // Filter feasible PD-locations between ordinary stops:
            const auto relOrdinaryPickups = relevantPdLocsFilter.filterOrdinaryPickups(feasibleEllipticPickups, rs, pdLocs,
                                                                                       stats.ordAssignmentsStats);
            const auto relOrdinaryDropoffs = relevantPdLocsFilter.filterOrdinaryDropoffs(feasibleEllipticDropoffs,
                                                                                         rs, pdLocs, stats.ordAssignmentsStats);


            relPickups = relOrdinaryPickups;
            // Try ordinary assignments:
            ordAssignments.findAssignments(relOrdinaryPickups, relOrdinaryDropoffs, rs, ffPdDistances, pdLocs, stats.ordAssignmentsStats);

            // Filter feasible pickups before next stops:
            const auto relPickupsBeforeNextStop = relevantPdLocsFilter.filterPickupsBeforeNextStop(
                    feasibleEllipticPickups, rs, pdLocs, stats.pbnsAssignmentsStats);

            // Try DALS assignments:
            dalsAssignments.findAssignments(relOrdinaryPickups, relPickupsBeforeNextStop, rs, pdLocs, stats.dalsAssignmentsStats);

            // Filter feasible dropoffs before next stop:
            const auto relDropoffsBeforeNextStop = relevantPdLocsFilter.filterDropoffsBeforeNextStop(
                    feasibleEllipticDropoffs, rs, pdLocs, stats.pbnsAssignmentsStats);

            // Try PBNS assignments:
            pbnsAssignments.findAssignments(relPickupsBeforeNextStop, relOrdinaryDropoffs, relDropoffsBeforeNextStop,
                                            rs, ffPdDistances, pdLocs, stats.pbnsAssignmentsStats);

            return rs;
        }

    private:

        VertexQuery convertKARRIRequestToULTRAQuery(const Request &req) {
            const auto origin = psgCh.rank(psgInputGraph.edgeHead(vehInputGraph.toPsgEdge(req.origin)));
            const auto destination = psgCh.rank(psgInputGraph.edgeHead(vehInputGraph.toPsgEdge(req.destination)));
            const auto requestTime = req.requestTime;
            const Vertex originVertex = Vertex(chOrder[origin]);
            const Vertex destinationVertex = Vertex(chOrder[destination]);
            return VertexQuery(originVertex, destinationVertex, requestTime);
        }

        RequestState runFirstTaxiSharingLeg(const Request &req) {
            RequestState rs = curReqState;
            stats::DispatchingPerformanceStats& stats = rs.stats();

            runPALS(rs, stats.palsAssignmentsStats);
            runOrdinary(rs, stats.ordAssignmentsStats);

            // -> assignment with best cost
            return rs;
        }

        void runPALS(RequestState &rs, stats::PalsAssignmentsPerformanceStats &stats) {
            // Run BCH queries from origin to all stations
            // reachable pickups from origin from KaRRi
            stationBCH.setExternalCostUpperBound(bestCost);
            stationBCH.runBchQueries(relevantPdLocs, rs);
                        
            // last stop -> pickups
            // PALS Individual BCH
            // neu laufen lassen mit eigenen pruning für alle stations
            palsToStations.setExternalCostUpperBound(bestCost);
            palsToStations.tryPickupAfterLastStop(rs, stationBCH.getTentativeDistances(), relevantPdLocs, stations, stats);
        }

        void runOrdinary(RequestState &rs, stats::OrdAssignmentsPerformanceStats &stats) {
            ordinaryToStations.enumerateAssignments(rs, relevantPdLocs, relPickups, stations, stationsInEllipse, stationBCH.getTentativeDistances(), stats);
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
        PDLocs relevantPdLocs;
        RelevantPDLocs relPickups;

        PTStations stations;
        StationBucketsEnvT &stationBucketsEnv;
        StationBCHQueryT stationBCH;
        PALSToStationsT &palsToStations;

        StationsInEllipseT &stationsInEllipse;
        OrdinaryToStationsT ordinaryToStations;

        PTAlgorithmT &ptAlgorithm;
        
        RequestState curReqState;
        int bestCost;
    };
}