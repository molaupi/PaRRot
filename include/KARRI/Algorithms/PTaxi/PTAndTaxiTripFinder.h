#pragma once

#include "PTAndTaxiTriple.h"
#include "../CH/CH.h"
#include <ULTRA/DataStructures/Queries/Queries.h>

namespace karri {


    // Core of the PTaxi algorithm: Given a ride request r, this facility finds the optimal assignment of r to the route
    // of a vehicle and a pickup and dropoff location, according to the current state of all vehicle routes.
    template<
            typename AssignmentFinderT,
            typename VehicleInputGraphT,
            typename PsgInputGraphT,
            typename PsgCHEnvT
    >
    class PTAndTaxiTripFinder {

    public:
    
        PTAndTaxiTripFinder(AssignmentFinderT &assignmentFinder,
                            const VehicleInputGraphT &vehInputGraph,
                            const PsgInputGraphT &psgInputGraph,
                            const PsgCHEnvT &psgChEnv)
                : assignmentFinder(assignmentFinder), 
                  vehInputGraph(vehInputGraph),
                  psgInputGraph(psgInputGraph),
                  psgCh(psgChEnv.getCH()) {}

        PTAndTaxiTriple findBestAssignment(const Request &req) {
            // First taxi leg
            // TODO: make sure request state is trivially copyable by using changes in mt_karri_batch
            auto taxiOnlyResult = assignmentFinder.findBestAssignment(req);
            RequestState invalidTaxiResponse;
            PTResult invalidPTResponse(false);
            
            // Return the combined results
            return PTAndTaxiTriple(taxiOnlyResult, invalidPTResponse, invalidTaxiResponse);
        }

    private:

        VertexQuery convertKARRIRequestToULTRAQuery(const Request &req) {
            const auto origin = psgCh.rank(psgInputGraph.edgeHead(vehInputGraph.toPsgEdge(req.origin)));
            const auto destination = psgCh.rank(psgInputGraph.edgeHead(vehInputGraph.toPsgEdge(req.destination)));
            const auto requestTime = req.requestTime;
            return VertexQuery(origin, destination, requestTime);
        }

        AssignmentFinderT &assignmentFinder;
        const VehicleInputGraphT &vehInputGraph;
        const PsgInputGraphT &psgInputGraph;
        const CH &psgCh;

    };
}