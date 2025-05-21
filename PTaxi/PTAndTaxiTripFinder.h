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
                  ptAlgorithm(ptAlgorithm),
                  chOrder(order) {}

        PTAndTaxiTriple findBestAssignment(const Request &req) {
            // Taxi only leg and invalid taxi leg
            auto taxiOnlyResponse = assignmentFinder.findBestAssignment(req);
            RequestState invalidTaxiResponse;
            
            VertexQuery query = convertKARRIRequestToULTRAQuery(req);
            ptAlgorithm.run(query.source, query.departureTime, query.target);
            
            // PT only leg and invalid PT leg
            PTResult ptOnlyResponse(ptAlgorithm.getEarliestJourney(query.target), taxiOnlyResponse);
            PTResult invalidPTResponse;

            auto firstTaxiLeg = runFirstTaxiSharingLeg(req);
            
            // Return the combined results
            if (taxiOnlyResponse.getBestCost() < ptOnlyResponse.getBestCost()) {
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

        RequestState runFirstTaxiSharingLeg(const Request &req) {
            // Run BCH queries from origin to all stations
            PDLocs pdLocs;
            const PDLoc originPickup = {
                req.requestId, // pickup id
                req.origin, // pickup location in road network
                vehCh.rank(vehInputGraph.toPsgEdge(req.origin)), // pickup location in passenger road network
                0, // walking time from origin to this pickup
                0, // vehicle driving time from this pickup to the origin
                0 // vehicle driving time from origin to this pickup
            };
            pdLocs.pickups.push_back(originPickup);
            
            stationBCH.runBchQueries(pdLocs);
                        
            RequestState empty;
            return empty;
        }

        AssignmentFinderT &assignmentFinder;
        const VehicleInputGraphT &vehInputGraph;
        const PsgInputGraphT &psgInputGraph;
        const CH &psgCh;
        const CH &vehCh;
        Order &chOrder;

        PTStations stations;
        StationBucketsEnvT &stationBucketsEnv;
        StationBCH stationBCH;

        PTAlgorithmT &ptAlgorithm;
    };
}