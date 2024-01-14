#pragma once

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "../../Algorithms/CH/CH.h"
#include "../../Algorithms/CH/Query/BucketQuery.h"
#include "../../DataStructures/Attributes/AttributeNames.h"
#include "../../DataStructures/Graph/Classes/DynamicGraph.h"
#include "../../DataStructures/Graph/Classes/GraphWrapper.h"
#include "../../DataStructures/Graph/Graph.h"
#include "../../DataStructures/Graph/Utils/Conversion.h"
#include "../../DataStructures/RAPTOR/Data.h"
#include "../../Shell/Shell.h"
// TODO adapt the ch providers
#include "../../../KARRI/Algorithms/KaRRi/BaseObjects/Request.h"
#include "../../../KARRI/Algorithms/KaRRi/BaseObjects/Vehicle.h"
/* #include "../../Algorithms/Loud/CHProviders/StandardCHProvider.h" */
/* #include "../../Algorithms/Loud/LoudDispatcher.h" */
/* #include "../../Algorithms/Loud/MatSimDispatcher.h" */
/* #include "../../Algorithms/Loud/TransportSimulation.h" */
/* #include "../../Algorithms/Loud/VehicleLocators/StandaloneVehicleLocator.h" */
#include "../../../KARRI/Tools/Constants.h"
#include "../../../KARRI/Tools/Logging/LogManager.h"
#include "../../../KARRI/Tools/StringHelpers.h"

#include "../../DataStructures/RideRAPTOR/Entities/Journey.h"
#include "../../Helpers/Vector/Permutation.h"
#include "DistanceMatrix.h"
#include "Entities/InsertionInfo.h"
#include "Profiler.h"

namespace RIDERAPTOR {

using RoadGraph = GraphWrapper<KaRRiGraph>;
// what this?
using CHProvider = Loud::StandardCHProvider<RoadGraph, ImplementationDetail::TravelTimeType>;
using Dispatcher = Loud::LoudDispatcher<RoadGraph, CHProvider, std::ofstream>;

inline TransferGraph getOverheadGraph(RAPTOR::Data& raptorData,
    TransferGraph& graph) noexcept
{
    DynamicTransferGraph temp;
    Graph::copy(graph, temp);
    for (const StopId stop : raptorData.stops()) {
        temp.addVertex(temp.vertexRecord(stop));
    }
    Permutation permutation(Construct::Id, temp.numVertices());
    for (const StopId stop : raptorData.stops()) {
        const size_t newStopId = stop + graph.numVertices();
        permutation[stop] = newStopId;
        permutation[newStopId] = stop;
    }
    temp.applyVertexPermutation(permutation);
    for (const StopId stop : raptorData.stops()) {
        const Vertex stopVertex(stop + graph.numVertices());
        temp.addEdge(stop, stopVertex).set(TravelTime, 0);
        temp.addEdge(stopVertex, stop).set(TravelTime, 0);
    }
    TransferGraph result;
    Graph::move(std::move(temp), result);
    return result;
}

struct RideRaptorParameter {
    RideRaptorParameter() { }

    RideRaptorParameter(int stopTime, double alpha, int beta, int maxIdleTime,
        int maxWalkTime)
        : stopTime(stopTime)
        , alpha(alpha)
        , beta(beta)
        , maxIdleTime(maxIdleTime)
        , maxWalkTime(maxWalkTime)
    {
    }

    RideRaptorParameter(const std::string& fileName,
        const std::string& separator = ".")
    {
        readFrom(fileName, separator);
    }

    int stopTime;
    double alpha;
    int beta;
    int maxIdleTime;
    int maxWalkTime;

    void readFrom(const std::string& fileName,
        const std::string& separator = ".")
    {
        IO::deserialize(fileName + separator + "parameter", stopTime, alpha, beta,
            maxIdleTime, maxWalkTime);
    }

    void writeTo(const std::string& fileName,
        const std::string& separator = ".")
    {
        IO::serialize(fileName + separator + "parameter", stopTime, alpha, beta,
            maxIdleTime, maxWalkTime);
    }
};

class Data {
public:
    Data(const std::string& fileName, const RAPTOR::Data& raptorData,
        Dispatcher& dispatcher, CH::CH walkingCH = CH::CH(),
        const int maxWalkTime = 600,
        const DistanceMatrix& distanceMatrix = DistanceMatrix(),
        const std::string& separator = ".", const Profiler& = Profiler())
        : Data(raptorData, dispatcher, walkingCH, maxWalkTime, distanceMatrix)
    {
        readRideDataFrom(fileName, separator);
    }

    Data(const RAPTOR::Data& raptorData, Dispatcher& disp,
        CH::CH walkingCH = CH::CH(), const int maxWalkTime = 600,
        const DistanceMatrix& distanceMatrix = DistanceMatrix(),
        const Profiler& profilerTemplate = Profiler())
        : raptorData(raptorData)
        , disp(disp)
        , walkingCH(walkingCH)
        , distanceMatrix(distanceMatrix)
        , maxWalkTime(maxWalkTime)
        , stopCounter(disp.fleet.size(), 0)
        , accumulatedNumStops(disp.fleet.size() + 1)
        ,
        // necessary to allow to add target later
        sourceStopDummy(raptorData.numberOfStops())
        , targetStopDummy(raptorData.numberOfStops() + 1)
        , numberOfStops(raptorData.numberOfStops() + 2)
        , profiler(profilerTemplate)
    {
        profiler.registerPhases(
            { PHASE_BUILDTRANSFERGRAPH, PHASE_BCHSEARCHES, PHASE_SORT_EDGES });
        profiler.registerMetrics(
            { METRIC_NEARBYVEHICLEROUTES, METRIC_VEHICLES, METRIC_STATIONS,
                METRIC_BCHSEARCHES, METRIC_VEHICLESTOPS, METRIC_VERTICES, METRIC_EDGES,
                METRIC_PICKUPEDGES, METRIC_DROPOFFEDGES, METRIC_NUMBER_OF_INSERTIONS,
                METRIC_MAX_STATIONS_PER_VEHICLE });
        profiler.initialize();
        profiler.countMetric(METRIC_VEHICLES, disp.fleet.size());
        profiler.countMetric(METRIC_STATIONS, raptorData.numberOfStops());
    }

    void buildRideTransferGraph()
    {
        profiler.start();

        accumulatedNumStops = disp.getNumStopsForVehicles();
        profiler.countMetric(METRIC_VEHICLESTOPS, accumulatedNumStops.back());
        stopCounter.clear();
        stopCounter.resize(accumulatedNumStops.back());

        profiler.startPhase(PHASE_BUILDTRANSFERGRAPH);
        ConstructionGraph constructionGraph;
        constructionGraph.addVertices(numberOfStops);
        constructionGraph.addVertices(accumulatedNumStops.back());

        for (int vehId = 0; vehId < accumulatedNumStops.size() - 1; vehId++) {
            const int numStops = accumulatedNumStops[vehId + 1] - accumulatedNumStops[vehId];
            for (int pos = 0; pos < numStops; pos++) {
                const auto vehicleStopVertex = getVertexFromVehiclePosition(vehId, pos);
                constructionGraph.set(VehicleId, vehicleStopVertex, vehId);
                constructionGraph.addEdge(vehicleStopVertex, Vertex(targetStopDummy))
                    .set(Weight, INFTY);
                constructionGraph.addEdge(Vertex(sourceStopDummy), vehicleStopVertex)
                    .set(Weight, INFTY);
            }
        }
        profiler.donePhase(PHASE_BUILDTRANSFERGRAPH);

        profiler.startPhase(PHASE_BCHSEARCHES);
        for (const auto stop : raptorData.stops()) {
            const auto vehicles = disp.runBCHSearchFromStop(stop, stop);
            profiler.countMetric(METRIC_BCHSEARCHES, 2);
            profiler.countMetric(METRIC_NEARBYVEHICLEROUTES, vehicles.size());
            insertVehicleEdges(vehicles, stop, constructionGraph);
        }
        profiler.donePhase(PHASE_BCHSEARCHES);

        profiler.startPhase(PHASE_SORT_EDGES);
        constructionGraph.sortEdgesReverse(Weight);
        profiler.donePhase(PHASE_SORT_EDGES);

        profiler.startPhase(PHASE_BUILDTRANSFERGRAPH);
        Graph::move(std::move(constructionGraph), rideTransferGraph);
        profiler.donePhase(PHASE_BUILDTRANSFERGRAPH);

        calculateStatistics();

        profiler.done();
    }

    /*void extendRideTransferGraph(StopId sourceStop, Edge sourceEdge, StopId
    targetStop, Edge targetEdge) { if (raptorData.isStop(sourceStop) &&
    raptorData.isStop(targetStop)) { return;
        }

        ConstructionGraph constructionGraph;

        Graph::move(std::move(rideTransferGraph), constructionGraph);

        if (!raptorData.isStop(sourceStop)) {

            const auto sourceTail = disp.inputGraph.get(FromVertex, sourceEdge);
            const auto sourceHead = disp.inputGraph.get(ToVertex, sourceEdge);
            const auto travelTime = disp.inputGraph.get(TravelTime, sourceEdge);

            const auto insertionInfos = disp.runBCHSearchFromStop(sourceTail,
    sourceHead, travelTime); insertSourceEdges(insertionInfos, sourceStop);
        }

        if (!raptorData.isStop(targetStop)) {
            constructionGraph.deleteAllIncomingEdges(targetStop);
            constructionGraph.deleteAllOutgoingEdges(targetStop);

            const auto targetTail = disp.inputGraph.get(FromVertex, targetEdge);
            const auto targetHead = disp.inputGraph.get(ToVertex, targetEdge);
            const auto travelTime = disp.inputGraph.get(TravelTime, targetEdge);

            const auto insertionInfos = disp.runBCHSearchFromStop(targetTail,
    targetHead, travelTime); insertVehicleEdges(insertionInfos, targetStop,
    constructionGraph);
        }

        constructionGraph.sortEdgesReverse(Weight);
        Graph::move(std::move(constructionGraph), rideTransferGraph);
    }*/

    void extendRideTransferGraph(StopId sourceStop, Edge sourceEdge,
        StopId targetStop, Edge targetEdge)
    {
        if (raptorData.isStop(sourceStop) && raptorData.isStop(targetStop)) {
            return;
        }

        if (!raptorData.isStop(sourceStop)) {
            const auto sourceTail = disp.inputGraph.get(FromVertex, sourceEdge);
            const auto sourceHead = disp.inputGraph.get(ToVertex, sourceEdge);
            const auto travelTime = disp.inputGraph.get(TravelTime, sourceEdge);

            const auto insertionInfos = disp.runBCHSearchFromStop(sourceTail, sourceHead, travelTime);
            insertSourceEdges(insertionInfos, sourceStop);
        }

        if (!raptorData.isStop(targetStop)) {
            const auto targetTail = disp.inputGraph.get(FromVertex, targetEdge);
            const auto targetHead = disp.inputGraph.get(ToVertex, targetEdge);
            const auto travelTime = disp.inputGraph.get(TravelTime, targetEdge);

            const auto insertionInfos = disp.runBCHSearchFromStop(targetTail, targetHead, travelTime);
            insertTargetEdges(insertionInfos, targetStop);
        }
    }

    inline std::vector<std::string> journeyToText(
        const Journey& journey) const noexcept
    {
        std::vector<std::string> text;
        for (const JourneyLeg& leg : journey) {
            std::stringstream line;
            if (leg.usesRoute) {
                line << "Take "
                     << GTFS::TypeNames[raptorData.routeData[leg.routeId].type];
                line << ": " << raptorData.routeData[leg.routeId].name << " ["
                     << leg.routeId << "] ";
                line << "from " << raptorData.stopData[leg.from].name << " ["
                     << leg.from << "] ";
                line << "departing at " << String::secToTime(leg.departureTime) << " ["
                     << leg.departureTime << "], ";
                line << "to " << raptorData.stopData[leg.to].name << " [" << leg.to
                     << "] ";
                line << "arrive at " << String::secToTime(leg.arrivalTime) << " ["
                     << leg.arrivalTime << "] ";
            } else if (leg.from == leg.to) {
                line << "Wait at " << raptorData.stopData[leg.from].name << " ["
                     << leg.from << "], ";
                line << "minimal waiting time: "
                     << String::secToString(leg.arrivalTime - leg.departureTime) << ".";
            } else if (leg.usesRide) {
                line << "Take " << disp.fleet[leg.vehicleId] << " [" << leg.vehicleId
                     << "] ";
                line << "from "
                     << (raptorData.isStop(leg.from)
                                ? raptorData.stopData[leg.from].name
                                : "Edge")
                     << " [" << leg.from << "] ";
                line << "departing at " << String::secToTime(leg.departureTime) << " ["
                     << leg.departureTime << "], ";
                line << "to "
                     << (raptorData.isStop(leg.to) ? raptorData.stopData[leg.to].name
                                                   : "Edge")
                     << " [" << leg.to << "], ";
                line << "arrive at " << String::secToTime(leg.arrivalTime) << " ["
                     << leg.arrivalTime << "] ";
            } else {
                line << "Walk from "
                     << (raptorData.isStop(leg.from)
                                ? raptorData.stopData[leg.from].name
                                : "Edge")
                     << " [" << leg.from << "] ";
                line << "to "
                     << (raptorData.isStop(leg.to) ? raptorData.stopData[leg.to].name
                                                   : "Edge")
                     << " [" << leg.to << "], ";
                line << "start at " << String::secToTime(leg.departureTime) << " ["
                     << leg.departureTime << "] ";
                line << "and arrive at " << String::secToTime(leg.arrivalTime) << " ["
                     << leg.arrivalTime << "] ";
                line << "(" << String::secToString(leg.arrivalTime - leg.departureTime)
                     << ").";
            }
            text.emplace_back(line.str());
        }
        return text;
    }

    void print()
    {
        std::cout << "Num Ride Edges: " << rideTransferGraph.numEdges()
                  << std::endl;

        for (int vehId = 0; vehId < accumulatedNumStops.size() - 1; vehId++) {
            const int numStops = accumulatedNumStops[vehId + 1] - accumulatedNumStops[vehId];
            for (int pos = 0; pos < numStops; pos++) {
                std::cout << vehId << "-" << pos << ": "
                          << stopCounter[accumulatedNumStops[vehId] + pos] << ", ";
            }
            std::cout << "\n";
        }
    }

    int getVehicleFromVertex(const Vertex vertex)
    {
        return rideTransferGraph.get(VehicleId, vertex);
    }

    Vertex getVertexFromVehiclePosition(const int vehicleId, const int position)
    {
        return Vertex(numberOfStops + accumulatedNumStops[vehicleId] + position);
    }

    inline const Profiler& getProfiler() const noexcept { return profiler; }

    void readRideDataFrom(const std::string& fileName,
        const std::string& separator = ".")
    {
        disp.readFrom(fileName + separator + "dispatcher", separator);
        IO::deserialize(fileName + separator + "stopCounter", stopCounter);
        IO::deserialize(fileName + separator + "accumulatedNumStops",
            accumulatedNumStops);
        rideTransferGraph.readBinary(fileName + separator + "rideTransferGraph");
    }

    void writeRideDataTo(const std::string& fileName,
        const std::string& separator = ".")
    {
        disp.writeTo(fileName + separator + "dispatcher", separator);
        IO::serialize(fileName + separator + "stopCounter", stopCounter);
        IO::serialize(fileName + separator + "accumulatedNumStops",
            accumulatedNumStops);
        rideTransferGraph.writeBinary(fileName + separator + "rideTransferGraph");
    }

private:
    template <bool INSERT_OUTGOING = true>
    void insertVehicleEdges(const std::vector<StopInsertionInfo> insertionInfos,
        const StopId stop,
        ConstructionGraph& constructionGraph)
    {
        for (const auto insertionInfo : insertionInfos) {
            auto edge = constructionGraph
                            .addEdge(Vertex(stop), getVertexFromVehiclePosition(insertionInfo.vehicleId, insertionInfo.insertionPosition))
                            .set(Weight, insertionInfo.leeway);
            edge.set(InsertionInfo, insertionInfo);
            profiler.countMetric(METRIC_PICKUPEDGES);

            if (INSERT_OUTGOING) {
                const auto& occupancies = disp.occupanciesFor(insertionInfo.vehicleId);
                const auto& capacity = disp.fleet[insertionInfo.vehicleId].seatingCapacity;

                for (int pos = insertionInfo.insertionPosition; pos >= 0; pos--) {
                    if (occupancies[pos] == capacity)
                        break;
                    auto reverseEdge = constructionGraph
                                           .addEdge(getVertexFromVehiclePosition(insertionInfo.vehicleId,
                                                        pos),
                                               Vertex(stop))
                                           .set(Weight, (pos == insertionInfo.insertionPosition ? insertionInfo.detour : 0) + insertionInfo.leeway);
                    reverseEdge.set(InsertionInfo, insertionInfo);
                    profiler.countMetric(METRIC_DROPOFFEDGES);
                }
            }
            stopCounter[accumulatedNumStops[insertionInfo.vehicleId] + insertionInfo.insertionPosition]++;
        }
    }

    void insertSourceEdges(const std::vector<StopInsertionInfo> insertionInfos,
        const Vertex sourceVertex)
    {
        for (const auto edge : rideTransferGraph.edgesFrom(sourceVertex)) {
            rideTransferGraph.set(Weight, edge, INFTY);
            StopInsertionInfo info;
            rideTransferGraph.set(InsertionInfo, edge, info);
        }

        for (const auto insertionInfo : insertionInfos) {
            auto edge = rideTransferGraph.findEdge(
                sourceVertex,
                getVertexFromVehiclePosition(insertionInfo.vehicleId,
                    insertionInfo.insertionPosition));
            rideTransferGraph.set(Weight, edge, insertionInfo.leeway);
            rideTransferGraph.set(InsertionInfo, edge, insertionInfo);
            profiler.countMetric(METRIC_PICKUPEDGES);

            stopCounter[accumulatedNumStops[insertionInfo.vehicleId] + insertionInfo.insertionPosition]++;
        }
    }

    void insertTargetEdges(const std::vector<StopInsertionInfo> insertionInfos,
        const Vertex targetVertex)
    {
        for (auto vertex = numberOfStops; vertex < rideTransferGraph.numVertices();
             vertex++) {
            const auto edge = rideTransferGraph.findEdge(Vertex(vertex), targetVertex);
            rideTransferGraph.set(Weight, edge, INFTY);
            StopInsertionInfo info;
            rideTransferGraph.set(InsertionInfo, edge, info);
        }

        for (const auto insertionInfo : insertionInfos) {
            const auto& occupancies = disp.occupanciesFor(insertionInfo.vehicleId);
            const auto& capacity = disp.fleet[insertionInfo.vehicleId].seatingCapacity;

            for (int pos = insertionInfo.insertionPosition; pos >= 0; pos--) {
                if (occupancies[pos] == capacity)
                    break;
                auto reverseEdge = rideTransferGraph.findEdge(
                    getVertexFromVehiclePosition(insertionInfo.vehicleId, pos),
                    targetVertex);
                rideTransferGraph.set(
                    Weight, reverseEdge,
                    (pos == insertionInfo.insertionPosition ? insertionInfo.detour
                                                            : 0)
                        + insertionInfo.leeway);
                rideTransferGraph.set(InsertionInfo, reverseEdge, insertionInfo);
                profiler.countMetric(METRIC_DROPOFFEDGES);
            }

            stopCounter[accumulatedNumStops[insertionInfo.vehicleId] + insertionInfo.insertionPosition]++;
        }
    }

    void calculateStatistics()
    {
        int maxCount = 0;
        for (int vehId = 0; vehId < accumulatedNumStops.size() - 1; vehId++) {
            const int numStops = accumulatedNumStops[vehId + 1] - accumulatedNumStops[vehId];
            int count = 0;
            for (int pos = 0; pos < numStops; pos++) {
                count += stopCounter[accumulatedNumStops[vehId] + pos];
            }
            if (maxCount < count)
                maxCount = count;
        }

        profiler.countMetric(METRIC_MAX_STATIONS_PER_VEHICLE, maxCount);

        int insertions = 0;
        for (const auto stop : raptorData.stops()) {
            for (const auto pickupEdge : rideTransferGraph.edgesFrom(stop)) {
                const auto vehicleVertex = rideTransferGraph.get(ToVertex, pickupEdge);
                insertions += rideTransferGraph.outDegree(vehicleVertex);
            }
        }
        profiler.countMetric(METRIC_NUMBER_OF_INSERTIONS, insertions);

        profiler.countMetric(METRIC_VERTICES, rideTransferGraph.numVertices());
        profiler.countMetric(METRIC_EDGES, rideTransferGraph.numEdges());
    }

public:
    const RAPTOR::Data& raptorData;
    Dispatcher& disp;
    RideTransferGraph rideTransferGraph;
    const DistanceMatrix& distanceMatrix;
    CH::CH walkingCH;
    const int maxWalkTime;

    const StopId sourceStopDummy;
    const StopId targetStopDummy;

private:
    std::vector<int> stopCounter;
    std::vector<int> accumulatedNumStops;
    const int numberOfStops;
    Profiler profiler;
};
} // namespace RIDERAPTOR
