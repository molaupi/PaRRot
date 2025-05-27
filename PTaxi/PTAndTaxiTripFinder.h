#pragma once

#include "PTAndTaxiTriple.h"
#include "StationBCHQuery.h"
#include "Station.h"
#include <KARRI/Algorithms/CH/CH.h>
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
            typename PALSToStationsT,
            typename PTAlgorithmT
    >
    class PTAndTaxiTripFinder {

    public:
    using StationBCH = StationBCHQuery<VehicleInputGraphT, VehCHEnvT, StationBucketsEnvT>;
    
        PTAndTaxiTripFinder(AssignmentFinderT &assignmentFinder,
                            const VehicleInputGraphT &vehInputGraph,
                            const VehCHEnvT &vehChEnv,
                            const PsgInputGraphT &psgInputGraph,
                            const PsgCHEnvT &psgChEnv,
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
                  bestCost(INFTY) {}

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
            // Pull this out to use in first taxi leg as well
            PDLocs pdLocs = assignmentFinder.findPDLocs(req, stats.initializationStats);
            stats.numPickups = pdLocs.numPickups();
            stats.numDropoffs = pdLocs.numDropoffs();
            assignmentFinder.initializeComponentsForRequest(rs, pdLocs, stats);

            relevantPdLocs = pdLocs;

            return assignmentFinder.findBestAssignment(rs, pdLocs, stats);
        }

        RequestState runFirstTaxiSharingLeg(const Request &req) {
            RequestState rs;
            stats::DispatchingPerformanceStats& stats = rs.stats();

            // Run BCH queries from origin to all stations
            // reachable pickups from origin from KaRRi
            stationBCH.runBchQueries(relevantPdLocs);
                        
            // last stop -> pickups
            // PALS Individual BCH
            // neu laufen lassen mit eigenen pruning für alle stations
            palsToStations.tryPickupAfterLastStop(rs, stationBCH.getTentativeDistances(), relevantPdLocs, stations, stats.palsAssignmentsStats);

            // -> assignment with earliest arrival time (explicit the earliest arrival time + taxi assignment)
            return rs;
        }

        AssignmentFinderT &assignmentFinder;
        const VehicleInputGraphT &vehInputGraph;
        const PsgInputGraphT &psgInputGraph;
        const CH &psgCh;
        const CH &vehCh;

        Order &chOrder;
        PDLocs relevantPdLocs;

        PTStations stations;
        StationBucketsEnvT &stationBucketsEnv;
        StationBCH stationBCH;
        PALSToStationsT &palsToStations;

        PTAlgorithmT &ptAlgorithm;

        int bestCost;
    };
}