#pragma once

#include <iostream>
#include <string>
#include <vector>

#include <ULTRA/DataStructures/Container/Map.h>
#include <ULTRA/DataStructures/Container/IndexedSet.h>
#include <ULTRA/DataStructures/RAPTOR/Data.h>
#include <ULTRA/DataStructures/RAPTOR/Entities/EarliestArrivalTime.h>
#include <ULTRA/DataStructures/RAPTOR/Entities/ArrivalLabel.h>
#include <ULTRA/Algorithms/RAPTOR/InitialTransfers.h>
#include <ULTRA/Algorithms/RAPTOR/Profiler.h>

#include "PTResult.h"
#include "../Station/Station.h"

namespace RAPTOR {

template <typename LabelSetT, typename PROFILER = NoProfiler, bool PREVENT_DIRECT_WALKING = false,
    typename INITIAL_TRANSFERS = BucketCHInitialTransfers>
class TaxiULTRARAPTOR {
public:
    using Profiler = PROFILER;
    static constexpr bool PreventDirectWalking = PREVENT_DIRECT_WALKING;
    using InitialTransferType = INITIAL_TRANSFERS;
    using InitialTransferGraph = typename InitialTransferType::Graph;
    static constexpr bool SeparateRouteAndTransferEntries = PreventDirectWalking;
    static constexpr int RoundFactor = SeparateRouteAndTransferEntries ? 2 : 1;
    using ArrivalTime = EarliestArrivalTime<SeparateRouteAndTransferEntries>;
    using Type = TaxiULTRARAPTOR<LabelSetT, Profiler, PreventDirectWalking, InitialTransferType>;
    using SourceType = Vertex;

    using DistanceLabel = typename LabelSetT::DistanceLabel;

private:
    struct EarliestArrivalLabel {
        EarliestArrivalLabel()
            : arrivalTime(never)
            , parentDepartureTime(never)
            , parent(noVertex)
            , usesRoute(false)
            , routeId(noRouteId)
            , usesTaxi(false)
        {
        }
        int arrivalTime;
        int parentDepartureTime;
        Vertex parent;
        bool usesRoute;
        bool usesTaxi;
        union {
            RouteId routeId;
            Edge transferId;
        };
    };
    using Round = std::vector<EarliestArrivalLabel>;

public:
    TaxiULTRARAPTOR(const Data& data, InitialTransferType& initialTransfers, const PTStations &stations,
        const int psgNumEdges,
        const Profiler& profilerTemplate = Profiler())
        : data(data)
        , initialTransfers(initialTransfers)
        , earliestArrival(data.numberOfStops() + 1)
        , stopsUpdatedByRoute(data.numberOfStops() + 1)
        , stopsUpdatedByTransfer(data.numberOfStops() + 1)
        , routesServingUpdatedStops(data.numberOfRoutes())
        , originPsgEdge(INVALID_ID)
        , originVehEdge(INVALID_ID)
        , destinationPsgEdge(INVALID_ID)
        , destinationVehEdge(INVALID_ID)
        , targetStop(noStop)
        , sourceDepartureTime(never)
        , profiler(profilerTemplate)
        , stations(stations)
        , psgEdgeToStation(psgNumEdges, INVALID_ID)
    {
        Assert(data.hasImplicitBufferTimes(),
            "Departure buffer times have to be implicit!");
        profiler.registerExtraRounds(
            { EXTRA_ROUND_CLEAR, EXTRA_ROUND_INITIALIZATION });
        profiler.registerPhases(
            { PHASE_INITIALIZATION, PHASE_COLLECT, PHASE_SCAN, PHASE_TRANSFERS });
        profiler.registerMetrics({ METRIC_ROUTES, METRIC_ROUTE_SEGMENTS,
            METRIC_EDGES, METRIC_STOPS_BY_TRIP,
            METRIC_STOPS_BY_TRANSFER });
        profiler.initialize();
        initializeEdgeToStationMappings();
    }

    template <typename ATTRIBUTE>
    TaxiULTRARAPTOR(const Data& data, const InitialTransferGraph& forwardGraph,
        const InitialTransferGraph& backwardGraph, const ATTRIBUTE weight, const PTStations &stations,
        const std::string& fileName = "", const Profiler& profilerTemplate = Profiler())
        : TaxiULTRARAPTOR(data,
            InitialTransferType(forwardGraph, backwardGraph,
                data.numberOfStops(), weight, fileName),
            stations,
            profilerTemplate)
    {
    }

    TaxiULTRARAPTOR(const Data& data, const ULTRACH::CH& chData, const PTStations &stations,
        const std::string& fileName = "", const Profiler& profilerTemplate = Profiler())
        : TaxiULTRARAPTOR(data, chData.forward, chData.backward, Weight, stations,
            fileName, profilerTemplate)
    {
    }

    TaxiULTRARAPTOR(const Data& data, const TransferGraph& forwardGraph,
        const TransferGraph& backwardGraph,
        const Profiler& profilerTemplate = Profiler())
        : TaxiULTRARAPTOR(data, forwardGraph, backwardGraph, TravelTime,
            profilerTemplate)
    {
    }

    inline void run(const int originPsgEdge, const int originVehEdge, 
        const int destPsgEdge, const int destVehEdge,
        const int departureTime, const size_t maxRounds = INFTY) noexcept
    {
        profiler.start();
        profiler.startExtraRound(EXTRA_ROUND_CLEAR);
        clear();
        profiler.doneRound();

        profiler.startExtraRound(EXTRA_ROUND_INITIALIZATION);
        profiler.startPhase();
        initialize(originPsgEdge, originVehEdge, destPsgEdge, destVehEdge, departureTime);
        profiler.donePhase(PHASE_INITIALIZATION);
        profiler.startPhase();
        relaxInitialTransfers(departureTime);
        profiler.donePhase(PHASE_TRANSFERS);
        profiler.doneRound();

        for (size_t i = 0; i < maxRounds; i++) {
            profiler.startRound();
            profiler.startPhase();
            startNewRound();
            profiler.donePhase(PHASE_INITIALIZATION);
            profiler.startPhase();
            collectRoutesServingUpdatedStops();
            profiler.donePhase(PHASE_COLLECT);
            profiler.startPhase();
            scanRoutes();
            profiler.donePhase(PHASE_SCAN);
            if (stopsUpdatedByRoute.empty()) {
                profiler.doneRound();
                break;
            }
            if constexpr (SeparateRouteAndTransferEntries) {
                profiler.startPhase();
                startNewRound();
                profiler.donePhase(PHASE_INITIALIZATION);
            }
            profiler.startPhase();
            relaxIntermediateTransfers();
            profiler.donePhase(PHASE_TRANSFERS);
            profiler.doneRound();
        }
        profiler.done();
    }

    // without initialization phase
    inline void runWithTaxi(const int departureTime,
        const karri::FirstTaxiLegResult &firstTaxiLeg, const std::vector<DistanceLabel> &distFromStations,
        const size_t maxRounds = INFTY) noexcept
    {
        profiler.start();
        profiler.startExtraRound(EXTRA_ROUND_CLEAR);
        clear();
        profiler.doneRound();

        startNewRound();

        profiler.startPhase();
        relaxInitialTransfers(departureTime, &firstTaxiLeg);
        profiler.donePhase(PHASE_TRANSFERS);
        profiler.doneRound();

        for (size_t i = 0; i < maxRounds; i++) {
            profiler.startRound();
            profiler.startPhase();
            startNewRound();
            profiler.donePhase(PHASE_INITIALIZATION);
            profiler.startPhase();
            collectRoutesServingUpdatedStops();
            profiler.donePhase(PHASE_COLLECT);
            profiler.startPhase();
            scanRoutes();
            profiler.donePhase(PHASE_SCAN);
            if (stopsUpdatedByRoute.empty()) {
                profiler.doneRound();
                break;
            }
            if constexpr (SeparateRouteAndTransferEntries) {
                profiler.startPhase();
                startNewRound();
                profiler.donePhase(PHASE_INITIALIZATION);
            }
            profiler.startPhase();
            relaxIntermediateTransfers(distFromStations);
            profiler.donePhase(PHASE_TRANSFERS);
            profiler.doneRound();
        }
        profiler.done();
    }

    inline std::vector<Journey> getJourneys() const noexcept
    {
        return getJourneys(targetStop);
    }

    inline std::vector<Journey> getJourneys(const StopId stop) const noexcept
    {
        std::vector<Journey> journeys;
        for (size_t i = 0; i < rounds.size(); i += RoundFactor) {
            getJourney(journeys, i, stop);
        }
        return journeys;
    }

    inline Journey getEarliestJourney(const StopId stop) const noexcept
    {
        std::vector<Journey> journeys = getJourneys(stop);
        return journeys.empty() ? Journey() : journeys.back();
    }

    inline std::vector<ArrivalLabel> getArrivals() const noexcept
    {
        return getArrivals(targetStop);
    }

    inline std::vector<ArrivalLabel> getArrivals(
        const StopId stopId) const noexcept
    {
        std::vector<ArrivalLabel> labels;
        for (size_t i = 0; i < rounds.size(); i += RoundFactor) {
            getArrival(labels, i, stopId);
        }
        return labels;
    }

    inline std::vector<int> getArrivalTimes() const noexcept
    {
        return getArrivalTimes(targetStop);
    }

    inline std::vector<int> getArrivalTimes(const StopId stop) const noexcept
    {
        std::vector<int> arrivalTimes;
        for (size_t i = 0; i < rounds.size(); i += RoundFactor) {
            getArrivalTime(arrivalTimes, i, stop);
        }
        return arrivalTimes;
    }

    inline bool reachable(const StopId stop) const noexcept
    {
        return earliestArrival[stop].getArrivalTime() < never;
    }

    inline int getEarliestArrivalTime(const StopId stop) const noexcept
    {
        return earliestArrival[stop].getArrivalTime();
    }

    inline int getEarliestArrivalTime() const noexcept
    {
        return getEarliestArrivalTime(targetStop);
    }

    inline int getEarliestArrivalNumberOfTrips() const noexcept
    {
        const int eat = getEarliestArrivalTime();
        for (size_t i = rounds.size() - 1; i < rounds.size(); i -= RoundFactor) {
            if (rounds[i][targetStop].arrivalTime == eat)
                return i;
        }
        return -1;
    }

    inline int getWalkingArrivalTime() const noexcept
    {
        return sourceDepartureTime + convertToULTRATime(initialTransfers.getDistance());
    }

    inline int getWalkingArrivalTime(const StopId stop) const noexcept
    {
        return sourceDepartureTime + convertToULTRATime(initialTransfers.getForwardDistance(stop));
    }

    inline int getWalkingTravelTime() const noexcept
    {
        return convertToULTRATime(initialTransfers.getDistance());
    }

    inline int getWalkingTravelTime(const StopId stop) const noexcept
    {
        return convertToULTRATime(initialTransfers.getDistance(stop));
    }

    inline int getDirectTransferTime() const noexcept
    {
        return convertToULTRATime(initialTransfers.getDistance());
    }

    inline std::vector<Vertex> getStopPath(const StopId stop) const noexcept
    {
        return journeyToPath(getJourneys(stop).back());
    }

    inline std::vector<Vertex> getStopPath() const noexcept
    {
        return getStopPath(targetStop);
    }


    inline Profiler& getProfiler() noexcept { return profiler; }

    inline int getArrivalTime(const StopId stop,
        const size_t numberOfTrips) const noexcept
    {
        size_t round = numberOfTrips * RoundFactor;
        if constexpr (SeparateRouteAndTransferEntries) {
            if ((round + 1 < rounds.size()) && (rounds[round + 1][stop].arrivalTime < rounds[round][stop].arrivalTime))
                round++;
        }
        Assert(
            rounds[round][stop].arrivalTime < never,
            "No label found for stop " << stop << " in round " << round << "!");
        return rounds[round][stop].arrivalTime;
    }

    template <bool RESET_CAPACITIES = false>
    inline void clear() noexcept
    {
        stopsUpdatedByRoute.clear();
        stopsUpdatedByTransfer.clear();
        routesServingUpdatedStops.clear();
        targetStop = StopId(data.numberOfStops());
        sourceDepartureTime = never;
        if constexpr (RESET_CAPACITIES) {
            std::vector<Round>().swap(rounds);
            std::vector<int>(earliestArrival.size(), never).swap(earliestArrival);
        } else {
            rounds.clear();
            Vector::fill(earliestArrival);
        }
    }

    inline void reset() noexcept { clear<true>(); }

private:

    inline void initializeEdgeToStationMappings() noexcept {
        for (const auto& station : stations) {
            psgEdgeToStation[station.psgEdgeId] = station.stationId;
        }
    }

    inline void initialize(const int originPsgEdge_, const int originVehEdge_,
        const int destPsgEdge_, const int destVehEdge_, const int departureTime) noexcept
    {
        originPsgEdge = originPsgEdge_;
        originVehEdge = originVehEdge_;
        destinationPsgEdge = destPsgEdge_;
        destinationVehEdge = destVehEdge_;

        if (destPsgEdge_ >= 0 && destPsgEdge_ < psgEdgeToStation.size()) {
            int stationId = psgEdgeToStation[destPsgEdge_];
            if (stationId != INVALID_ID) {
                targetStop = StopId(stationId);
            }
        }
        
        sourceDepartureTime = departureTime;
        startNewRound();
        
        // Initialize source stop if the origin edge maps to a station
        if (originPsgEdge_ >= 0 && originPsgEdge_ < psgEdgeToStation.size()) {
            int stationId = psgEdgeToStation[originPsgEdge_];
            if (stationId != INVALID_ID && data.isStop(StopId(stationId))) {
                StopId sourceStop = StopId(stationId);
                arrivalByRoute(sourceStop, sourceDepartureTime);
                currentRound()[sourceStop].parent = noVertex; // Sentinel: this stop was reached from origin
                currentRound()[sourceStop].parentDepartureTime = sourceDepartureTime;
                currentRound()[sourceStop].usesRoute = false;
                currentRound()[sourceStop].usesTaxi = false;
                if constexpr (!SeparateRouteAndTransferEntries)
                    stopsUpdatedByTransfer.insert(sourceStop);
            }
        }
        
        if constexpr (SeparateRouteAndTransferEntries)
            startNewRound();
    }

    inline void collectRoutesServingUpdatedStops() noexcept
    {
        for (const StopId stop : stopsUpdatedByTransfer) {
            Assert(data.isStop(stop), "Stop " << stop << " is out of range!");
            const int arrivalTime = previousRound()[stop].arrivalTime;
            Assert(arrivalTime < never, "Updated stop has arrival time = never!");
            for (const RouteSegment& route : data.routesContainingStop(stop)) {
                Assert(data.isRoute(route.routeId),
                    "Route " << route.routeId << " is out of range!");
                Assert(data.stopIds[data.firstStopIdOfRoute[route.routeId] + route.stopIndex] == stop,
                    "RAPTOR data contains invalid route segments!");
                if (route.stopIndex + 1 == data.numberOfStopsInRoute(route.routeId))
                    continue;
                if (data.lastTripOfRoute(route.routeId)[route.stopIndex].departureTime < arrivalTime)
                    continue;
                if (routesServingUpdatedStops.contains(route.routeId)) {
                    routesServingUpdatedStops[route.routeId] = std::min(
                        routesServingUpdatedStops[route.routeId], route.stopIndex);
                } else {
                    routesServingUpdatedStops.insert(route.routeId, route.stopIndex);
                }
            }
        }
    }

    inline void scanRoutes() noexcept
    {
        stopsUpdatedByRoute.clear();
        for (const RouteId route : routesServingUpdatedStops.getKeys()) {
            profiler.countMetric(METRIC_ROUTES);
            StopIndex stopIndex = routesServingUpdatedStops[route];
            const size_t tripSize = data.numberOfStopsInRoute(route);
            Assert(stopIndex < tripSize - 1,
                "Cannot scan a route starting at/after the last stop (Route: "
                    << route << ", StopIndex: " << stopIndex
                    << ", TripSize: " << tripSize << ")!");

            const StopId* stops = data.stopArrayOfRoute(route);
            const StopEvent* trip = data.lastTripOfRoute(route);
            StopId stop = stops[stopIndex];
            Assert(
                trip[stopIndex].departureTime >= previousRound()[stop].arrivalTime,
                "Cannot scan a route after the last trip has departed (Route: "
                    << route << ", Stop: " << stop << ", StopIndex: " << stopIndex
                    << ", Time: " << previousRound()[stop].arrivalTime
                    << ", LastDeparture: " << trip[stopIndex].departureTime << ")!");

            StopIndex parentIndex = stopIndex;
            const StopEvent* firstTrip = data.firstTripOfRoute(route);
            while (stopIndex < tripSize - 1) {
                while ((trip > firstTrip) && ((trip - tripSize + stopIndex)->departureTime >= previousRound()[stop].arrivalTime)) {
                    trip -= tripSize;
                    parentIndex = stopIndex;
                }
                stopIndex++;
                stop = stops[stopIndex];
                profiler.countMetric(METRIC_ROUTE_SEGMENTS);
                if (arrivalByRoute(stop, trip[stopIndex].arrivalTime)) {
                    EarliestArrivalLabel& label = currentRound()[stop];
                    label.parent = stops[parentIndex];
                    label.parentDepartureTime = trip[parentIndex].departureTime;
                    label.usesRoute = true;
                    label.usesTaxi = false;
                    label.routeId = route;
                }
            }
        }
    }

    inline void relaxInitialTransfers(const int sourceDepartureTime, const karri::FirstTaxiLegResult *firstTaxiLeg = nullptr) noexcept
    {
        // Pass edge IDs to initialTransfers for BCH-based distance computation
        initialTransfers.template run<!PreventDirectWalking>(originPsgEdge,
            destinationPsgEdge);
        for (const Vertex stop : initialTransfers.getForwardPOIs()) {
            if (stop == targetStop)
                continue;
            Assert(data.isStop(stop), "Reached POI " << stop << " is not a stop!");
            Assert(initialTransfers.getForwardDistance(stop) != INFTY,
                "Vertex " << stop << " was not reached!");
            const int arrivalTime = sourceDepartureTime + convertToULTRATime(initialTransfers.getForwardDistance(stop));
            if (arrivalByTransfer(StopId(stop), arrivalTime)) {
                EarliestArrivalLabel& label = currentRound()[stop];
                label.parent = noVertex; // Sentinel: this stop was reached from origin
                label.parentDepartureTime = sourceDepartureTime;
                label.usesRoute = false;
                label.usesTaxi = false;
                label.transferId = noEdge;
            }
        }

        // Extension for first taxi leg
        if (firstTaxiLeg != nullptr) {
            for (const auto &station : stations) {
                if (firstTaxiLeg->getWorstCostForAllStations() == INFTY) 
                    break; // no stations reached by taxi
                
                const int stationId = station.stationId;
                const Vertex targetStop = Vertex(stationId);
                const StopId targetStopId = StopId(targetStop);
                const auto taxiResult = firstTaxiLeg->getResultForStation(stationId);
    
                // Check if this is the destination station
                bool isDestStation = (destinationPsgEdge >= 0 && destinationPsgEdge < psgEdgeToStation.size() && 
                                       psgEdgeToStation[destinationPsgEdge] == stationId);
                // Check if this is the origin station
                bool isOriginStation = (originPsgEdge >= 0 && originPsgEdge < psgEdgeToStation.size() && 
                                         psgEdgeToStation[originPsgEdge] == stationId);
                
                if (isOriginStation || isDestStation || taxiResult.bestCost == INFTY)
                    continue;
                Assert(data.isStop(targetStop), "Taxi station " << targetStop << " is not a stop!");
                const int arrivalTime = convertToULTRATime(taxiResult.arrivalTime);
                if (arrivalByTransfer(targetStopId, arrivalTime)) {
                    EarliestArrivalLabel& label = currentRound()[targetStop];
                    label.parent = noVertex; // Sentinel: this stop was reached from origin via taxi
                    label.parentDepartureTime = sourceDepartureTime;
                    label.usesRoute = false;
                    label.usesTaxi = true;
                    label.transferId = noEdge;
                }
            }
        }

        if constexpr (!PreventDirectWalking) {
            if (initialTransfers.getDistance() != INFTY) {
                const int arrivalTime = sourceDepartureTime + convertToULTRATime(initialTransfers.getDistance());
                if (arrivalByTransfer(targetStop, arrivalTime)) {
                    EarliestArrivalLabel& label = currentRound()[targetStop];
                    label.parent = noVertex; // Sentinel: target reached directly from origin
                    label.parentDepartureTime = sourceDepartureTime;
                    label.usesRoute = false;
                    label.usesTaxi = false;
                    label.transferId = noEdge;
                }
            }
        }
    }

    inline void relaxIntermediateTransfers(const std::vector<DistanceLabel> &distFromStations = {}) noexcept
    {
        stopsUpdatedByTransfer.clear();
        routesServingUpdatedStops.clear();
        for (const StopId stop : stopsUpdatedByRoute) {
            const int earliestArrivalTime = SeparateRouteAndTransferEntries
                ? previousRound()[stop].arrivalTime
                : currentRound()[stop].arrivalTime;
            for (const Edge edge : data.transferGraph.edgesFrom(stop)) {
                const StopId toStop = StopId(data.transferGraph.get(ToVertex, edge));
                if (toStop == targetStop)
                    continue;
                profiler.countMetric(METRIC_EDGES);
                const int arrivalTime = earliestArrivalTime + data.transferGraph.get(TravelTime, edge);
                Assert(data.isStop(data.transferGraph.get(ToVertex, edge)),
                    "Graph contains edges to non stop vertices!");
                if (arrivalByTransfer(toStop, arrivalTime)) {
                    EarliestArrivalLabel& label = currentRound()[toStop];
                    label.parent = stop;
                    label.parentDepartureTime = earliestArrivalTime;
                    label.usesRoute = false;
                    label.usesTaxi = false;
                    label.transferId = edge;
                }
            }
            if (initialTransfers.getBackwardDistance(stop) != INFTY) {
                const int arrivalTime = earliestArrivalTime + convertToULTRATime(initialTransfers.getBackwardDistance(stop));
                if (arrivalByTransfer(targetStop, arrivalTime)) {
                    EarliestArrivalLabel& label = currentRound()[targetStop];
                    label.parent = stop;
                    label.parentDepartureTime = earliestArrivalTime;
                    label.usesRoute = false;
                    label.usesTaxi = false;
                    label.transferId = noEdge;
                }
            }

            // Extension for second taxi leg
            if (!distFromStations.empty()) {
                const int stationId = stop.value();
                auto station = stations[stationId];
                // ensure that no second taxi leg is used if station edge id == destination edge id
                if (stationId != INVALID_ID && station.vehEdgeId != destinationVehEdge) {
                    const int taxiTravelDistance = convertToULTRATime(distFromStations[stationId][0]);
                    const int arrivalTime = earliestArrivalTime + taxiTravelDistance;
                    if (arrivalByTransfer(targetStop, arrivalTime)) {
                        EarliestArrivalLabel& label = currentRound()[targetStop];
                        label.parent = stop;
                        label.parentDepartureTime = earliestArrivalTime;
                        label.usesRoute = false;
                        label.usesTaxi = true;
                        label.transferId = noEdge;
    
                    }
                }
            }
            
            if constexpr (SeparateRouteAndTransferEntries) {
                if (arrivalByTransfer(stop, earliestArrivalTime)) {
                    EarliestArrivalLabel& label = currentRound()[stop];
                    label.parent = stop;
                    label.parentDepartureTime = earliestArrivalTime;
                    label.usesRoute = false;
                    label.usesTaxi = false;
                }
            } else {
                stopsUpdatedByTransfer.insert(stop);
            }
        }
    }

    inline Round& currentRound() noexcept
    {
        Assert(!rounds.empty(),
            "Cannot return current round, because no round exists!");
        return rounds.back();
    }

    inline Round& previousRound() noexcept
    {
        Assert(
            rounds.size() >= 2,
            "Cannot return previous round, because less than two rounds exist!");
        return rounds[rounds.size() - 2];
    }

    inline void startNewRound() noexcept
    {
        rounds.emplace_back(data.numberOfStops() + 1);
    }

    inline bool arrivalByRoute(const StopId stop, const int time) noexcept
    {
        Assert(data.isStop(stop), "Stop " << stop << " is out of range!");
        if (earliestArrival[targetStop].getArrivalTimeByRoute() <= time)
            return false;
        if (earliestArrival[stop].getArrivalTimeByRoute() <= time)
            return false;
        profiler.countMetric(METRIC_STOPS_BY_TRIP);
        currentRound()[stop].arrivalTime = time;
        earliestArrival[stop].setArrivalTimeByRoute(time);
        stopsUpdatedByRoute.insert(stop);
        return true;
    }

    inline bool arrivalByTransfer(const StopId stop, const int time) noexcept
    {
        Assert(data.isStop(stop) || stop == targetStop,
            "Stop " << stop << " is out of range!");
        if (earliestArrival[targetStop].getArrivalTimeByTransfer() <= time)
            return false;
        if (earliestArrival[stop].getArrivalTimeByTransfer() <= time)
            return false;
        profiler.countMetric(METRIC_STOPS_BY_TRANSFER);
        currentRound()[stop].arrivalTime = time;
        earliestArrival[stop].setArrivalTimeByTransfer(time);
        if (data.isStop(stop))
            stopsUpdatedByTransfer.insert(stop);
        return true;
    }

    inline void getJourney(std::vector<Journey>& journeys, size_t round,
        StopId stop) const noexcept
    {
        if constexpr (SeparateRouteAndTransferEntries) {
            if ((round + 1 < rounds.size()) && (rounds[round + 1][stop].arrivalTime < rounds[round][stop].arrivalTime))
                round++;
        }
        if (rounds[round][stop].arrivalTime >= (journeys.empty() ? never : journeys.back().back().arrivalTime))
            return;
        Journey journey;
        do {
            Assert(
                round != size_t(-1),
                "Backtracking parent pointers did not pass through the source stop!");
            const EarliestArrivalLabel& label = rounds[round][stop];
            
            // Check if this is an initial transfer from origin (parent == noVertex)
            if (label.parent == noVertex) {
                // This leg starts from the origin - use noVertex as 'from' to signal origin
                journey.emplace_back(noVertex, stop, label.parentDepartureTime,
                    label.arrivalTime, label.usesRoute, label.routeId, label.usesTaxi);
                break; // Reached origin, journey reconstruction complete
            }
            
            // Regular intermediate leg: both parent and stop are valid stops
            journey.emplace_back(label.parent, stop, label.parentDepartureTime,
                label.arrivalTime, label.usesRoute, label.routeId, label.usesTaxi);
            
            Assert(data.isStop(label.parent),
                "Backtracking parent pointers reached a non-stop vertex ("
                    << label.parent << ")!");
            
            stop = StopId(label.parent);
            if constexpr (SeparateRouteAndTransferEntries) {
                round--;
            } else {
                if (label.usesRoute)
                    round--;
            }
        } while (true);
        
        journeys.emplace_back(Vector::reverse(journey));
    }

    inline void getArrival(std::vector<ArrivalLabel>& labels, size_t round,
        const StopId stop) const noexcept
    {
        if constexpr (SeparateRouteAndTransferEntries) {
            if ((round + 1 < rounds.size()) && (rounds[round + 1][stop].arrivalTime < rounds[round][stop].arrivalTime))
                round++;
        }
        if (rounds[round][stop].arrivalTime >= (labels.empty() ? never : labels.back().arrivalTime))
            return;
        labels.emplace_back(rounds[round][stop].arrivalTime, round / RoundFactor);
    }

    inline void getArrivalTime(std::vector<int>& labels, size_t round,
        const StopId stop) const noexcept
    {
        if constexpr (SeparateRouteAndTransferEntries) {
            if ((round + 1 < rounds.size()) && (rounds[round + 1][stop].arrivalTime < rounds[round][stop].arrivalTime))
                round++;
        }
        labels.emplace_back(std::min(rounds[round][stop].arrivalTime,
            (labels.empty()) ? (never) : (labels.back())));
    }

private:

    const int convertToULTRATime(const int timeInOneTenthSeconds) const {
        return timeInOneTenthSeconds / 10;
    }

    const Data& data;
    const PTStations &stations;

    InitialTransferType& initialTransfers;

    std::vector<Round> rounds;

    std::vector<ArrivalTime> earliestArrival;

    IndexedSet<false, StopId> stopsUpdatedByRoute;
    IndexedSet<false, StopId> stopsUpdatedByTransfer;
    IndexedMap<StopIndex, false, RouteId> routesServingUpdatedStops;

    int originPsgEdge;         // Origin edge ID in passenger graph
    int originVehEdge;         // Origin edge ID in vehicle graph
    int destinationPsgEdge;    // Destination edge ID in passenger graph
    int destinationVehEdge;    // Destination edge ID in vehicle graph
    StopId targetStop;         // Target stop ID - set as number of stops if destination is not a stop
    int sourceDepartureTime;

    // Fast lookup maps: edge ID -> station ID
    std::vector<int> psgEdgeToStation;

    Profiler profiler;
};

} // namespace RAPTOR
