#pragma once

#include "PTAndTaxiTriple.h"
#include <KARRI/Algorithms/CH/CH.h>
#include <ULTRA/DataStructures/Queries/Queries.h>
#include <ULTRA/Helpers/Vector/Permutation.h>

namespace karri {


    // Core of the PTaxi algorithm: Given a ride request r, this facility finds the optimal assignment of r to the route
    // of a vehicle and a pickup and dropoff location, according to the current state of all vehicle routes.
    template<
            typename AssignmentFinderT,
            typename VehicleInputGraphT,
            typename PsgInputGraphT,
            typename PsgCHEnvT,
            typename PTAlgorithmT
    >
    class PTAndTaxiTripFinder {

    public:
    
        PTAndTaxiTripFinder(AssignmentFinderT &assignmentFinder,
                            const VehicleInputGraphT &vehInputGraph,
                            const PsgInputGraphT &psgInputGraph,
                            const PsgCHEnvT &psgChEnv,
                            PTAlgorithmT &ptAlgorithm,
                            Order &order
                        )
                : assignmentFinder(assignmentFinder), 
                  vehInputGraph(vehInputGraph),
                  psgInputGraph(psgInputGraph),
                  psgCh(psgChEnv.getCH()),
                  ptAlgorithm(ptAlgorithm),
                  chOrder(order) {}

        PTAndTaxiTriple findBestAssignment(const Request &req) {
            // First taxi leg and invalid second taxi leg
            auto taxiOnlyResponse = assignmentFinder.findBestAssignment(req);
            RequestState invalidTaxiResponse;
            
            // PT leg
            VertexQuery query = convertKARRIRequestToULTRAQuery(req);
            ptAlgorithm.run(query.source, query.departureTime, query.target);

            PTResult ptOnlyResponse(ptAlgorithm.getEarliestJourney(query.target), taxiOnlyResponse);
            
            // Return the combined results
            return PTAndTaxiTriple(taxiOnlyResponse, ptOnlyResponse, invalidTaxiResponse);
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

        AssignmentFinderT &assignmentFinder;
        const VehicleInputGraphT &vehInputGraph;
        const PsgInputGraphT &psgInputGraph;
        const CH &psgCh;
        Order &chOrder;

        PTAlgorithmT &ptAlgorithm;
    };
}