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

        void transformCHFromKARRIToULTRA() {
            // convert from KARRI:CH to ULTRA::CH (Patricks Code)

            /*
            auto& karriPsgCH = psgCh;
            auto& karriPsgUpCHGraph = karriPsgCH.upwardGraph();
            auto& karriPsgDownCHGraph = karriPsgCH.downwardGraph();

            assert(karriPsgUpCHGraph.numVertices() == karriPsgDownCHGraph.numVertices());

            CHConstructionGraph upPsgCHGraph;
            CHConstructionGraph downPsgCHGraph;

            upPsgCHGraph.addVertices(karriPsgUpCHGraph.numVertices());
            upPsgCHGraph.reserve(karriPsgUpCHGraph.numVertices(), karriPsgUpCHGraph.numEdges());

            FORALL_VALID_EDGES(karriPsgUpCHGraph, v, e)
            {
                auto edgeHandle = upPsgCHGraph.addEdge(Vertex(v), Vertex(karriPsgUpCHGraph.edgeHead(e)));
                edgeHandle.set(Weight, karriPsgUpCHGraph.traversalCost(e));
                // since UnpackingInfoAttribute is defined a little weird (via the two edges directly, rather than the ViaVertex itself), we need to get the Vertex
                auto& unpackingInfoOfEdge = karriPsgUpCHGraph.unpackingInfo(e);
                if (unpackingInfoOfEdge.second == INVALID_EDGE) {
                    // this case, the edge was an input edge => set invalid vertex as viavertex
                    edgeHandle.set(ViaVertex, noVertex);
                } else {
                    // otherwise take the FromVertex / edgeTail from the first edge
                    assert(unpackingInfoOfEdge.first < karriPsgDownCHGraph.numEdges());
                    edgeHandle.set(ViaVertex, Vertex(karriPsgDownCHGraph.edgeHead(unpackingInfoOfEdge.first)));
                }
            }

            downPsgCHGraph.addVertices(karriPsgDownCHGraph.numVertices());
            downPsgCHGraph.reserve(karriPsgDownCHGraph.numVertices(), karriPsgDownCHGraph.numEdges());

            FORALL_VALID_EDGES(karriPsgDownCHGraph, v, e)
            {
                auto edgeHandle = downPsgCHGraph.addEdge(Vertex(v), Vertex(karriPsgDownCHGraph.edgeHead(e)));
                edgeHandle.set(Weight, karriPsgDownCHGraph.traversalCost(e));
                // TODO
                edgeHandle.set(ViaVertex, noVertex);
                /* // since UnpackingInfoAttribute is defined a little weird (via the two edges directly, rather than the ViaVertex itself), we need to get the Vertex */
                /* auto& unpackingInfoOfEdge = karriPsgDownCHGraph.unpackingInfo(e); */
                /* if (unpackingInfoOfEdge.second == INVALID_EDGE) { */
                /*     // this case, the edge was an input edge => set invalid vertex as viavertex */
                /*     edgeHandle.set(ViaVertex, noVertex); */
                /* } else { */
                /*     // otherwise take the FromVertex / edgeTail from the first edge */
                /*     assert(unpackingInfoOfEdge.first < karriPsgUpCHGraph.numEdges()); */
                /*     edgeHandle.set(ViaVertex, Vertex(karriPsgUpCHGraph.edgeHead(unpackingInfoOfEdge.first))); */
                /* } 
            }

            CH::CH psgCh(std::move(upPsgCHGraph), std::move(downPsgCHGraph));
            */
        }

        AssignmentFinderT &assignmentFinder;
        const VehicleInputGraphT &vehInputGraph;
        const PsgInputGraphT &psgInputGraph;
        const CH &psgCh;

    };
}