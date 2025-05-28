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
            typename AssignmentFinderT,
            typename VehicleInputGraphT,
            typename VehCHEnvT,
            typename PsgInputGraphT,
            typename PsgCHEnvT,
            typename StationBucketsEnvT,
            typename StationBCHQueryT,
            typename PALSToStationsT,
            typename PTAlgorithmT
    >
    class PTAndTaxiTripFinder {

    public:
    
        PTAndTaxiTripFinder(AssignmentFinderT &assignmentFinder,
                            const VehicleInputGraphT &vehInputGraph,
                            const VehCHEnvT &vehChEnv,
                            const PsgInputGraphT &psgInputGraph,
                            const PsgCHEnvT &psgChEnv,
                            const Fleet &fleet,
                            PTStations stations,
                            StationBucketsEnvT &stationBucketsEnv,
                            PALSToStationsT &palsToStations,
                            PTAlgorithmT &ptAlgorithm,
                            Order &order)
                : assignmentFinder(assignmentFinder), 
                  vehInputGraph(vehInputGraph),
                  psgInputGraph(psgInputGraph),
                  psgCh(psgChEnv.getCH()),
                  vehCh(vehChEnv.getCH()),
                  stations(stations),
                  stationBucketsEnv(stationBucketsEnv),
                  stationBCH(vehInputGraph, vehChEnv, stationBucketsEnv, stations.size()),
                  palsToStations(palsToStations),
                  ptAlgorithm(ptAlgorithm),
                  chOrder(order), 
                  bestCost(INFTY),
                  relOrdinaryPickups(fleet.size()) {}

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

    private:

        VertexQuery convertKARRIRequestToULTRAQuery(const Request &req) {
            const auto origin = psgCh.rank(psgInputGraph.edgeHead(vehInputGraph.toPsgEdge(req.origin)));
            const auto destination = psgCh.rank(psgInputGraph.edgeHead(vehInputGraph.toPsgEdge(req.destination)));
            const auto requestTime = req.requestTime;
            const Vertex originVertex = Vertex(chOrder[origin]);
            const Vertex destinationVertex = Vertex(chOrder[destination]);
            return VertexQuery(originVertex, destinationVertex, requestTime);
        }

        RequestState findBestTaxiAssignment(const Request &req) {
            // Initialize finder for this request, find PD locations:
            RequestState rs = assignmentFinder.initializeRequestState(req);
            stats::DispatchingPerformanceStats& stats = rs.stats();
            
            PDLocs pdLocs = assignmentFinder.findPDLocs(req, stats.initializationStats);
            stats.numPickups = pdLocs.numPickups();
            stats.numDropoffs = pdLocs.numDropoffs();
            assignmentFinder.initializeComponentsForRequest(rs, pdLocs, stats);

            relevantPdLocs = pdLocs;
            curReqState = rs;

            auto ffPdDistances = assignmentFinder.computePDDistances(rs, pdLocs, stats.pdDistancesStats);

            assignmentFinder.tryPALSAssignments(rs, pdLocs, ffPdDistances, stats.palsAssignmentsStats);
            assignmentFinder.runEllipticBCHSearches(rs, pdLocs, stats.ellipticBchStats);

            auto relOrdinaryPdLocs = assignmentFinder.filterOrdinaryPDLocs(rs, pdLocs, stats.ordAssignmentsStats);

            relOrdinaryPickups = relOrdinaryPdLocs.first;

            return assignmentFinder.findBestAssignment(rs, pdLocs, ffPdDistances, relOrdinaryPdLocs.first, relOrdinaryPdLocs.second, stats);
        }

        RequestState runFirstTaxiSharingLeg(const Request &req) {
            RequestState rs = curReqState;
            stats::DispatchingPerformanceStats& stats = rs.stats();

            runPALS(rs, stats.palsAssignmentsStats);
            runOrdinary(rs, stats.ordAssignmentsStats);

            // -> assignment with earliest arrival time (bestAssignment + bestArrivalTime)
            return rs;
        }

        void runPALS(RequestState &rs, stats::PalsAssignmentsPerformanceStats &stats) {
            // Run BCH queries from origin to all stations
            // reachable pickups from origin from KaRRi
            stationBCH.runBchQueries(relevantPdLocs);
                        
            // last stop -> pickups
            // PALS Individual BCH
            // neu laufen lassen mit eigenen pruning für alle stations
            palsToStations.setExternalCostUpperBound(bestCost);
            palsToStations.tryPickupAfterLastStop(rs, stationBCH.getTentativeDistances(), relevantPdLocs, stations, stats);
        }

        void runOrdinary(RequestState &rs, stats::OrdAssignmentsPerformanceStats &stats) {
            for (const auto &vehId : relOrdinaryPickups.getVehiclesWithRelevantPDLocs()) {
                std::cout << "Vehicle with relevant pickup: " << vehId << std::endl;
            }
        }

        AssignmentFinderT &assignmentFinder;
        const VehicleInputGraphT &vehInputGraph;
        const PsgInputGraphT &psgInputGraph;
        const CH &psgCh;
        const CH &vehCh;

        Order &chOrder;
        PDLocs relevantPdLocs;
        RelevantPDLocs relOrdinaryPickups;

        PTStations stations;
        StationBucketsEnvT &stationBucketsEnv;
        StationBCHQueryT stationBCH;
        PALSToStationsT &palsToStations;

        PTAlgorithmT &ptAlgorithm;
        
        RequestState curReqState;
        int bestCost;
    };
}