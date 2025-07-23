#pragma once

#include <iostream>
#include <string>
#include <vector>

#include "../../DataStructures/Container/Map.h"
#include "../../DataStructures/Container/Set.h"
#include "../../DataStructures/RAPTOR/Data.h"
#include "../../DataStructures/RAPTOR/Entities/EarliestArrivalTime.h"
#include "../../DataStructures/RAPTOR/Entities/ArrivalLabel.h"
#include "InitialTransfers.h"
#include "Profiler.h"

namespace RAPTOR {

template <typename PROFILER = NoProfiler, bool PREVENT_DIRECT_WALKING = false,
    typename INITIAL_TRANSFERS = BucketCHInitialTransfers>
class MultiSourceULTRARAPTOR {
public:
    using Profiler = PROFILER;
    static constexpr bool PreventDirectWalking = PREVENT_DIRECT_WALKING;
    using InitialTransferType = INITIAL_TRANSFERS;
    using InitialTransferGraph = typename InitialTransferType::Graph;
    static constexpr bool SeparateRouteAndTransferEntries = PreventDirectWalking;
    static constexpr int RoundFactor = SeparateRouteAndTransferEntries ? 2 : 1;
    using ArrivalTime = EarliestArrivalTime<SeparateRouteAndTransferEntries>;
    using Type = MultiSourceULTRARAPTOR<Profiler, PreventDirectWalking, InitialTransferType>;
    using SourceType = Vertex;

    struct TaxiSource {
        Vertex originVertex;        // Original pickup location
        StopId ptStopId;           // PT stop reached by taxi
        int arrivalTime;           // Time to reach PT stop via taxi
        int taxiCost;              // Cost of taxi leg
        int taxiDistance;          // Distance of taxi leg
        int taxiId;                // Taxi vehicle identifier
        
        TaxiSource(Vertex origin, StopId stop, int time, int cost = 0, int distance = 0, int id = -1) :
            originVertex(origin), ptStopId(stop), arrivalTime(time), taxiCost(cost), 
            taxiDistance(distance), taxiId(id) {}
    };

    struct TaxiTarget {
        Vertex destinationVertex; // Final destination
        int travelTime;           // Time from PT stop to destination
        int directDistance;       // Distance of final taxi leg
        StopId fromStopId;        // Specific PT stop (noStop means all stops)
        
        TaxiTarget(Vertex destination, int time, int distance = 0, StopId fromStop = noStop) :
            destinationVertex(destination), travelTime(time),
            directDistance(distance), fromStopId(fromStop) {}
    };

private:
    struct EarliestArrivalLabel {
        EarliestArrivalLabel()
            : arrivalTime(never)
            , parentDepartureTime(never)
            , parent(noVertex)
            , usesRoute(false)
            , routeId(noRouteId)
            , sourceTaxiId(-1)
            , targetTaxiId(-1)
            , totalTaxiCost(0)
            , isFromTaxi(false)
            , isToTaxi(false)
        {
        }
        int arrivalTime;
        int parentDepartureTime;
        Vertex parent;
        bool usesRoute;
        union {
            RouteId routeId;
            Edge transferId;
        };

        // Taxi-specific extensions
        int sourceTaxiId;          // Which taxi brought us to this stop
        int targetTaxiId;          // Which taxi will take us to destination
        int totalTaxiCost;         // Total cost so far
        bool isFromTaxi;           // Whether this leg came from taxi
        bool isToTaxi;           // Whether this leg came from taxi
    };

    using Round = std::vector<EarliestArrivalLabel>;

public:
    MultiSourceULTRARAPTOR(const Data& data, const InitialTransferType initialTransfers,
        const Profiler& profilerTemplate = Profiler())
        : data(data)
        , initialTransfers(initialTransfers)
        , earliestArrival(data.numberOfStops() + 1)
        , stopsUpdatedByRoute(data.numberOfStops() + 1)
        , stopsUpdatedByTransfer(data.numberOfStops() + 1)
        , routesServingUpdatedStops(data.numberOfRoutes())
        , sourceVertex(noVertex)
        , targetVertex(noVertex)
        , targetStop(noStop)
        , sourceDepartureTime(never)
        , profiler(profilerTemplate)
    {
        AssertMsg(data.hasImplicitBufferTimes(),
            "Departure buffer times have to be implicit!");
        profiler.registerExtraRounds(
            { EXTRA_ROUND_CLEAR, EXTRA_ROUND_INITIALIZATION });
        profiler.registerPhases(
            { PHASE_INITIALIZATION, PHASE_COLLECT, PHASE_SCAN, PHASE_TRANSFERS });
        profiler.registerMetrics({ METRIC_ROUTES, METRIC_ROUTE_SEGMENTS,
            METRIC_EDGES, METRIC_STOPS_BY_TRIP,
            METRIC_STOPS_BY_TRANSFER });
        profiler.initialize();
    }

    template <typename ATTRIBUTE>
    MultiSourceULTRARAPTOR(const Data& data, const InitialTransferGraph& forwardGraph,
        const InitialTransferGraph& backwardGraph, const ATTRIBUTE weight,
        const std::string& fileName = "", const Profiler& profilerTemplate = Profiler())
        : MultiSourceULTRARAPTOR(data,
            InitialTransferType(forwardGraph, backwardGraph,
                data.numberOfStops(), weight, fileName),
            profilerTemplate)
    {
    }

    template <typename T = CHGraph, typename = std::enable_if_t<Meta::Equals<T, CHGraph>() && Meta::Equals<T, InitialTransferGraph>()>>
    MultiSourceULTRARAPTOR(const Data& data, const ULTRACH::CH& chData,
        const std::string& fileName = "", const Profiler& profilerTemplate = Profiler())
        : MultiSourceULTRARAPTOR(data, chData.forward, chData.backward, Weight,
            fileName, profilerTemplate)
    {
    }

    template <
        typename T = TransferGraph,
        typename = std::enable_if_t<Meta::Equals<T, TransferGraph>() && Meta::Equals<T, InitialTransferGraph>()>>
    MultiSourceULTRARAPTOR(const Data& data, const TransferGraph& forwardGraph,
        const TransferGraph& backwardGraph,
        const Profiler& profilerTemplate = Profiler())
        : MultiSourceULTRARAPTOR(data, forwardGraph, backwardGraph, TravelTime,
            profilerTemplate)
    {
    }

    inline void run(const Vertex source, const int departureTime,
        const Vertex target,
        const size_t maxRounds = INFTY) noexcept
    {
        profiler.start();
        profiler.startExtraRound(EXTRA_ROUND_CLEAR);
        clear();
        profiler.doneRound();

        profiler.startExtraRound(EXTRA_ROUND_INITIALIZATION);
        profiler.startPhase();
        initialize(source, departureTime, target);
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

    inline std::vector<Journey> getJourneys() const noexcept
    {
        return getJourneys(targetStop);
    }

    inline std::vector<Journey> getJourneys(const Vertex vertex) const noexcept
    {
        const StopId target = (vertex == targetVertex) ? (targetStop) : (StopId(vertex));
        std::vector<Journey> journeys;
        for (size_t i = 0; i < rounds.size(); i += RoundFactor) {
            getJourney(journeys, i, target);
        }
        return journeys;
    }

    inline Journey getEarliestJourney(const Vertex vertex) const noexcept
    {
        std::vector<Journey> journeys = getJourneys(vertex);
        return journeys.empty() ? Journey() : journeys.back();
    }

    inline std::vector<ArrivalLabel> getArrivals() const noexcept
    {
        return getArrivals(targetStop);
    }

    inline std::vector<ArrivalLabel> getArrivals(
        const Vertex vertex) const noexcept
    {
        const StopId target = (vertex == targetVertex) ? (targetStop) : (StopId(vertex));
        std::vector<ArrivalLabel> labels;
        for (size_t i = 0; i < rounds.size(); i += RoundFactor) {
            getArrival(labels, i, target);
        }
        return labels;
    }

    inline std::vector<int> getArrivalTimes() const noexcept
    {
        return getArrivalTimes(targetStop);
    }

    inline std::vector<int> getArrivalTimes(const Vertex vertex) const noexcept
    {
        const StopId target = (vertex == targetVertex) ? (targetStop) : (StopId(vertex));
        std::vector<int> arrivalTimes;
        for (size_t i = 0; i < rounds.size(); i += RoundFactor) {
            getArrivalTime(arrivalTimes, i, target);
        }
        return arrivalTimes;
    }

    inline bool reachable(const Vertex vertex) const noexcept
    {
        const StopId target = (vertex == targetVertex) ? (targetStop) : (StopId(vertex));
        return earliestArrival[target].getArrivalTime() < never;
    }

    inline int getEarliestArrivalTime(const Vertex vertex) const noexcept
    {
        const StopId target = (vertex == targetVertex) ? (targetStop) : (StopId(vertex));
        return earliestArrival[target].getArrivalTime();
    }

    inline int getEarliestArrivalTime() const noexcept
    {
        return getEarliestArrivalTime(targetVertex);
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
        return sourceDepartureTime + initialTransfers.getDistance();
    }

    inline int getWalkingArrivalTime(const Vertex vertex) const noexcept
    {
        return sourceDepartureTime + initialTransfers.getForwardDistance(vertex);
    }

    inline int getWalkingTravelTime() const noexcept
    {
        return initialTransfers.getDistance();
    }

    inline int getWalkingTravelTime(const Vertex vertex) const noexcept
    {
        return initialTransfers.getDistance(vertex);
    }

    inline int getDirectTransferTime() const noexcept
    {
        return initialTransfers.getDistance();
    }

    inline std::vector<Vertex> getPath(const Vertex vertex) const noexcept
    {
        const StopId target = (vertex == targetVertex) ? (targetStop) : (StopId(vertex));
        return journeyToPath(getJourneys(target).back());
    }

    inline std::vector<Vertex> getPath() const noexcept
    {
        return getPath(targetVertex);
    }

    inline std::vector<std::string> getRouteDescription(
        const Vertex vertex) const noexcept
    {
        const StopId target = (vertex == targetVertex) ? (targetStop) : (StopId(vertex));
        return data.journeyToText(getJourneys(target).back());
    }

    inline Profiler& getProfiler() noexcept { return profiler; }

    inline int getArrivalTime(const Vertex vertex,
        const size_t numberOfTrips) const noexcept
    {
        const StopId target = (vertex == targetVertex) ? (targetStop) : (StopId(vertex));
        size_t round = numberOfTrips * RoundFactor;
        if constexpr (SeparateRouteAndTransferEntries) {
            if ((round + 1 < rounds.size()) && (rounds[round + 1][target].arrivalTime < rounds[round][target].arrivalTime))
                round++;
        }
        AssertMsg(
            rounds[round][target].arrivalTime < never,
            "No label found for stop " << target << " in round " << round << "!");
        return rounds[round][target].arrivalTime;
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
    inline void initialize(const Vertex source, const int departureTime,
        const Vertex target) noexcept
    {
        sourceVertex = source;
        targetVertex = target;
        if (data.isStop(target)) {
            targetStop = StopId(target);
        }
        sourceDepartureTime = departureTime;
        startNewRound();
        if (data.isStop(source)) {
            arrivalByRoute(StopId(source), sourceDepartureTime);
            currentRound()[source].parent = source;
            currentRound()[source].parentDepartureTime = sourceDepartureTime;
            currentRound()[source].usesRoute = false;
            if constexpr (!SeparateRouteAndTransferEntries)
                stopsUpdatedByTransfer.insert(StopId(source));
        }
        if constexpr (SeparateRouteAndTransferEntries)
            startNewRound();
    }

    inline void initializeMultiSourceTaxi(
        const std::vector<TaxiSource>& taxiSources,
        const std::vector<TaxiTarget>& taxiTargets,
        const std::vector<Vertex>& targets
    ) noexcept {

        // Store taxi information
        activeTaxiSources = taxiSources;
        activeTaxiTargets = taxiTargets;
        targetVertices = targets;
        
        // Build lookup maps for efficiency
        stopToTaxiSourceMap.clear();
        destinationToTaxiTargetMap.clear();
        
        for (size_t i = 0; i < taxiSources.size(); ++i) {
            stopToTaxiSourceMap[taxiSources[i].ptStopId].push_back(i);
        }
        
        for (size_t i = 0; i < taxiTargets.size(); ++i) {
            destinationToTaxiTargetMap[taxiTargets[i].destinationVertex] = i;
        }
        
        for (const Vertex target : targets) {
            if (data.isStop(target)) {
                targetStops.push_back(StopId(target));
            } else {
                // For non-PT targets, we need special handling
                // Could use extended stop space or different approach
                targetStops.push_back(StopId(data.numberOfStops() + targetStops.size()));
            }
        }

        // Extend earliestArrival array if needed
        size_t requiredSize = data.numberOfStops() + 1 + targetStops.size();
        if (earliestArrival.size() < requiredSize) {
            earliestArrival.resize(requiredSize);
        }
        
        // Initialize first round
        startNewRound();
        
        // Set arrival times at all taxi-reachable PT stops
        for (const TaxiSource& source : taxiSources) {
            Assert(data.isStop(source.ptStopId), "Taxi source " << source.ptStopId << " is not a valid PT stop!");
            
            if (arrivalByRoute(source.ptStopId, source.arrivalTime)) {
                EarliestArrivalLabel& label =
                    static_cast<EarliestArrivalLabel&>(currentRound()[source.ptStopId]);

                // Set basic arrival information
                label.parent = source.originVertex;
                label.parentDepartureTime = source.arrivalTime; // Taxi departure time would be earlier
                label.usesRoute = false;
                label.transferId = noEdge;
                
                // Set taxi-specific information
                label.sourceTaxiId = source.taxiVehicleId;
                label.totalTaxiCost = source.taxiCost;
                label.isFromTaxi = true;
                
                // Mark for processing in next round
                if constexpr (!SeparateRouteAndTransferEntries) {
                    stopsUpdatedByTransfer.insert(source.ptStopId);
                }
            }
        }
        
        // Initialize second round if needed
        if constexpr (SeparateRouteAndTransferEntries) startNewRound();
    }

    inline void collectRoutesServingUpdatedStops() noexcept
    {
        for (const StopId stop : stopsUpdatedByTransfer) {
            AssertMsg(data.isStop(stop), "Stop " << stop << " is out of range!");
            const int arrivalTime = previousRound()[stop].arrivalTime;
            AssertMsg(arrivalTime < never, "Updated stop has arrival time = never!");
            for (const RouteSegment& route : data.routesContainingStop(stop)) {
                AssertMsg(data.isRoute(route.routeId),
                    "Route " << route.routeId << " is out of range!");
                AssertMsg(data.stopIds[data.firstStopIdOfRoute[route.routeId] + route.stopIndex] == stop,
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
            AssertMsg(stopIndex < tripSize - 1,
                "Cannot scan a route starting at/after the last stop (Route: "
                    << route << ", StopIndex: " << stopIndex
                    << ", TripSize: " << tripSize << ")!");

            const StopId* stops = data.stopArrayOfRoute(route);
            const StopEvent* trip = data.lastTripOfRoute(route);
            StopId stop = stops[stopIndex];
            AssertMsg(
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
                    label.routeId = route;
                }
            }
        }
    }

    inline void relaxInitialTransfers(const int sourceDepartureTime) noexcept
    {
        initialTransfers.template run<!PreventDirectWalking>(sourceVertex,
            targetVertex);
        for (const Vertex stop : initialTransfers.getForwardPOIs()) {
            if (stop == targetStop)
                continue;
            AssertMsg(data.isStop(stop), "Reached POI " << stop << " is not a stop!");
            AssertMsg(initialTransfers.getForwardDistance(stop) != INFTY,
                "Vertex " << stop << " was not reached!");
            const int arrivalTime = sourceDepartureTime + initialTransfers.getForwardDistance(stop);
            if (arrivalByTransfer(StopId(stop), arrivalTime)) {
                EarliestArrivalLabel& label = currentRound()[stop];
                label.parent = sourceVertex;
                label.parentDepartureTime = sourceDepartureTime;
                label.usesRoute = false;
                label.transferId = noEdge;
            }
        }
        // walk to destination
        if constexpr (!PreventDirectWalking) {
            if (initialTransfers.getDistance() != INFTY) {
                const int arrivalTime = sourceDepartureTime + initialTransfers.getDistance();
                if (arrivalByTransfer(targetStop, arrivalTime)) {
                    EarliestArrivalLabel& label = currentRound()[targetStop];
                    label.parent = sourceVertex;
                    label.parentDepartureTime = sourceDepartureTime;
                    label.usesRoute = false;
                    label.transferId = noEdge;
                }
            }
        }
    }

    inline void relaxMultiSourceInitialTransfers(const int sourceDepartureTime) noexcept {
        // For each taxi origin point, also allow direct walking to PT stations
        
        std::set<Vertex> processedOrigins; // Avoid duplicate computation
        
        for (const TaxiSource& taxiSource : activeTaxiSources) {
            if (processedOrigins.find(taxiSource.originVertex) != processedOrigins.end()) {
                continue;
            }
            processedOrigins.insert(taxiSource.originVertex);
            
            Vertex referenceTarget = targetVertices.empty() ? noVertex : targetVertices[0];
            initialTransfers.template run<!PreventDirectWalking>(taxiSource.originVertex, referenceTarget);
            
            // Process all reachable PT stops by walking
            for (const Vertex stop : initialTransfers.getForwardPOIs()) {
                // Skip target stops - handle them separately
                bool isTarget = false;
                for (const Vertex target : targetVertices) {
                    if (stop == target) {
                        isTarget = true;
                        break;
                    }
                }
                if (isTarget) continue;
                
                Assert(data.isStop(stop), "Reached POI " << stop << " is not a stop!");
                Assert(initialTransfers.getForwardDistance(stop) != INFTY, "Vertex " << stop << " was not reached!");
                
                const int walkingTime = initialTransfers.getForwardDistance(stop);
                const int walkingArrivalTime = sourceDepartureTime + walkingTime;
                
                if (arrivalByTransfer(StopId(stop), walkingArrivalTime)) {
                    EarliestArrivalLabel& label = currentRound()[stop];
                    label.parent = taxiSource.originVertex;
                    label.parentDepartureTime = sourceDepartureTime;
                    label.usesRoute = false;
                    label.transferId = noEdge;
                    
                    // This is walking, not taxi - no taxi cost involved
                    label.sourceTaxiId = -1;
                    label.totalTaxiCost = 0;
                    label.isFromTaxi = false;
                    label.isToTaxi = false;
                }
            }
            
            // Handle direct walking to target destinations 
            if constexpr (!PreventDirectWalking) {
                for (size_t targetIdx = 0; targetIdx < targetVertices.size(); ++targetIdx) {
                    const Vertex targetVertex = targetVertices[targetIdx];
                    
                    // Check if target is directly reachable by walking
                    if (initialTransfers.getDistance(targetVertex) != INFTY) {
                        const int walkingTime = initialTransfers.getDistance(targetVertex);
                        const int walkingDepartureTime = taxiSource.arrivalTime;
                        const int walkingArrivalTime = walkingDepartureTime + walkingTime;
                        
                        const StopId targetStopId = targetStops[targetIdx];
                        if (arrivalByTransfer(targetStopId, walkingArrivalTime)) {
                            EarliestArrivalLabel& label = currentRound()[targetStopId];
                            label.parent = taxiSource.originVertex;
                            label.parentDepartureTime = walkingDepartureTime;
                            label.usesRoute = false;
                            label.transferId = noEdge;
                            
                            // Pure walking solution - no taxi cost
                            label.sourceTaxiId = -1;
                            label.totalTaxiCost = 0;
                            label.isFromTaxi = false;
                            label.isToTaxi = false;
                        }
                    }
                }
            }
        }
    }

    inline void relaxIntermediateTransfers() noexcept
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
                AssertMsg(data.isStop(data.transferGraph.get(ToVertex, edge)),
                    "Graph contains edges to non stop vertices!");
                if (arrivalByTransfer(toStop, arrivalTime)) {
                    EarliestArrivalLabel& label = currentRound()[toStop];
                    label.parent = stop;
                    label.parentDepartureTime = earliestArrivalTime;
                    label.usesRoute = false;
                    label.transferId = edge;
                }
            }
            // Final Transfer
            // Approximation for taxi distances from all stations
            if (initialTransfers.getBackwardDistance(stop) != INFTY) {
                const int arrivalTime = earliestArrivalTime + initialTransfers.getBackwardDistance(stop);
                if (arrivalByTransfer(targetStop, arrivalTime)) {
                    EarliestArrivalLabel& label = currentRound()[targetStop];
                    label.parent = stop;
                    label.parentDepartureTime = earliestArrivalTime;
                    label.usesRoute = false;
                    label.transferId = noEdge;
                }
            }
            if constexpr (SeparateRouteAndTransferEntries) {
                if (arrivalByTransfer(stop, earliestArrivalTime)) {
                    EarliestArrivalLabel& label = currentRound()[stop];
                    label.parent = stop;
                    label.parentDepartureTime = earliestArrivalTime;
                    label.usesRoute = false;
                }
            } else {
                stopsUpdatedByTransfer.insert(stop);
            }
        }
    }

    inline Round& currentRound() noexcept
    {
        AssertMsg(!rounds.empty(),
            "Cannot return current round, because no round exists!");
        return rounds.back();
    }

    inline Round& previousRound() noexcept
    {
        AssertMsg(
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
        AssertMsg(data.isStop(stop), "Stop " << stop << " is out of range!");
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
        AssertMsg(data.isStop(stop) || stop == targetStop,
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
            AssertMsg(
                round != size_t(-1),
                "Backtracking parent pointers did not pass through the source stop!");
            const EarliestArrivalLabel& label = rounds[round][stop];
            journey.emplace_back(label.parent, stop, label.parentDepartureTime,
                label.arrivalTime, label.usesRoute, label.routeId);
            AssertMsg(data.isStop(label.parent) || label.parent == sourceVertex,
                "Backtracking parent pointers reached a vertex ("
                    << label.parent << ")!");
            stop = StopId(label.parent);
            if constexpr (SeparateRouteAndTransferEntries) {
                round--;
            } else {
                if (label.usesRoute)
                    round--;
            }
        } while (journey.back().from != sourceVertex);
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
    const Data& data;

    InitialTransferType initialTransfers;

    std::vector<Round> rounds;

    std::vector<ArrivalTime> earliestArrival;

    IndexedSet<false, StopId> stopsUpdatedByRoute;
    IndexedSet<false, StopId> stopsUpdatedByTransfer;
    IndexedMap<StopIndex, false, RouteId> routesServingUpdatedStops;

    Vertex sourceVertex;
    Vertex targetVertex;
    StopId targetStop;
    int sourceDepartureTime;

    Profiler profiler;

    // Additional storage for taxi information
    std::vector<TaxiSource> activeTaxiSources;
    std::vector<TaxiTarget> activeTaxiTargets;
    std::unordered_map<StopId, std::vector<size_t>> stopToTaxiSourceMap;
    std::unordered_map<Vertex, size_t> destinationToTaxiTargetMap;
    
    // Track multiple potential target vertices
    std::vector<Vertex> targetVertices;
    std::vector<StopId> targetStops;
};

} // namespace RAPTOR
