#pragma once

#include <iostream>
#include <string>
#include <vector>

#include "../../../KARRI/Algorithms/KaRRi/BaseObjeccts/RideRAPTORRequest.h"
#include "../../DataStructures/Container/Map.h"
#include "../../DataStructures/Container/Set.h"
#include "../../DataStructures/RAPTOR/Data.h"
#include "../../DataStructures/RAPTOR/Entities/EarliestArrivalTime.h"
#include "../../DataStructures/RideRAPTOR/Data.h"
#include "../../DataStructures/RideRAPTOR/Entities/Journey.h"
#include "../RAPTOR/InitialTransfers.h"
#include "../RAPTOR/Profiler.h"

namespace RAPTOR {

struct RideOptimizationFlags {
    bool UseTimePruning;
    bool UseLeewayPruning;
    bool UseApproximation;
    bool UseDistanceMatrix;
};

template <bool TARGET_PRUNING, typename PROFILER = NoProfiler,
    bool TRANSITIVE = false, bool USE_MIN_TRANSFER_TIMES = false>
class RideRAPTOR {
public:
    static constexpr bool TargetPruning = TARGET_PRUNING;
    using Profiler = PROFILER;
    static constexpr bool Transitive = TRANSITIVE;
    static constexpr bool UseMinTransferTimes = USE_MIN_TRANSFER_TIMES;

    static constexpr bool SeparateRouteAndTransferEntries = !Transitive | UseMinTransferTimes;
    static constexpr int RoundFactor = SeparateRouteAndTransferEntries ? 2 : 1;
    using ArrivalTime = EarliestArrivalTime<SeparateRouteAndTransferEntries>;
    using Type = RideRAPTOR<TargetPruning, Profiler, Transitive, UseMinTransferTimes>;
    using InitialTransferGraph = TransferGraph;
    using SourceType = StopId;
    using Journey = RIDERAPTOR::Journey;

private:
    struct EarliestArrivalLabel {
        EarliestArrivalLabel()
            : arrivalTime(never)
            , parentDepartureTime(never)
            , parent(noStop)
            , usesRoute(false)
            , usesRide(false)
            , routeId(noRouteId)
        {
        }
        int arrivalTime;
        int parentDepartureTime;
        StopId parent;
        bool usesRoute;
        bool usesRide;
        union {
            RouteId routeId;
            Edge transferId;
            int vehicleId;
        };

        inline bool dominates(const EarliestArrivalLabel& other) const noexcept
        {
            if (arrivalTime > other.arrivalTime)
                return false;
            return true;
        }
    };
    using Round = std::vector<EarliestArrivalLabel>;

public:
    RideRAPTOR(RIDERAPTOR::Data& data,
        RideOptimizationFlags optimizationFlags = { true, true, true, true },
        const Profiler& profilerTemplate = Profiler())
        : data(data)
        , earliestArrival(data.raptorData.numberOfStops() + 2)
        , stopsUpdatedByRoute(data.raptorData.numberOfStops() + 2)
        , stopsUpdatedByTransfer(data.raptorData.numberOfStops() + 2)
        , routesServingUpdatedStops(data.raptorData.numberOfRoutes())
        , initialWalkingTransfers(data.walkingCH, FORWARD,
              data.raptorData.numberOfStops(),
              data.maxWalkTime)
        , sourceEdge(noEdge)
        , sourceVertex(noVertex)
        , sourceStop(noStop)
        , targetEdge(noEdge)
        , targetVertex(noVertex)
        , targetStop(noStop)
        , sourceDepartureTime(never)
        , walkingDistance(INFTY)
        , profiler(profilerTemplate)
        , optimizationFlags(optimizationFlags)
    {
        if constexpr (UseMinTransferTimes) {
            AssertMsg(!data.raptorData.hasImplicitBufferTimes(),
                "Either min transfer times have to be used OR departure buffer "
                "times have to be implicit!");
        } else {
            AssertMsg(data.raptorData.hasImplicitBufferTimes(),
                "Either min transfer times have to be used OR departure buffer "
                "times have to be implicit!");
        }
        profiler.registerExtraRounds(
            { EXTRA_ROUND_CLEAR, EXTRA_ROUND_INITIALIZATION });
        profiler.registerPhases(
            { PHASE_INITIALIZATION, PHASE_INIT_RIDE, PHASE_INIT_WALK, PHASE_COLLECT,
                PHASE_SCAN, PHASE_TRANSFERS_WALKING, PHASE_TRANSFERS_RIDESHARING });
        profiler.registerMetrics(
            { METRIC_ROUTES, METRIC_ROUTE_SEGMENTS, METRIC_EDGES,
                METRIC_DIRECT_WALKING, METRIC_STOPS_BY_TRIP, METRIC_STOPS_BY_TRANSFER,
                METRIC_TRIED_RIDES, METRIC_FEASIBLE_RIDES,
                METRIC_STOPS_BY_RIDETRANSFER, METRIC_CHSEARCHES, METRIC_TIME_PRUNING,
                METRIC_LEEWAY_PRUNING, METRIC_FILTERED_BY_APPROX, METRIC_PICKUP_EDGES,
                METRIC_FILTERED_PICKUP_EDGES, METRIC_TOTAL_DE
                /*, METRIC_PLACEHOLDER_3, METRIC_PLACEHOLDER_4*/ });
        profiler.initialize();
    }

    template <typename ATTRIBUTE>
    RideRAPTOR(RIDERAPTOR::Data& data, const InitialTransferGraph&,
        const InitialTransferGraph&, const ATTRIBUTE,
        const Profiler& = Profiler())
        : RideRAPTOR(data)
    {
    }

    inline void run(const Edge source, const int departureTime, const Edge target,
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
        data.extendRideTransferGraph(sourceStop, sourceEdge, targetStop,
            targetEdge);
        profiler.donePhase(PHASE_INIT_RIDE);
        profiler.startPhase();
        if (!data.raptorData.isStop(targetStop) || !data.raptorData.isStop(sourceStop)) {
            initialWalkingTransfers.run(sourceVertex, targetVertex);
        }
        profiler.donePhase(PHASE_INIT_WALK);
        profiler.startPhase();
        relaxInitialWalkingTransfer();
        profiler.donePhase(PHASE_TRANSFERS_WALKING);
        profiler.startPhase();
        relaxRideTransfers();
        profiler.donePhase(PHASE_TRANSFERS_RIDESHARING);
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
            relaxTransfers();
            profiler.donePhase(PHASE_TRANSFERS_WALKING);
            profiler.startPhase();
            relaxRideTransfers();
            profiler.donePhase(PHASE_TRANSFERS_RIDESHARING);
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
        const StopId stop) const noexcept
    {
        AssertMsg(data.raptorData.isStop(stop),
            "The StopId " << stop << " does not correspond to any stop!");
        std::vector<ArrivalLabel> labels;
        for (size_t i = 0; i < rounds.size(); i += RoundFactor) {
            getArrival(labels, i, stop);
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

    inline int getWalkingArrivalTime() const noexcept
    {
        return sourceDepartureTime + walkingDistance;
    }

    inline int getWalkingTravelTime() const noexcept { return walkingDistance; }

    inline std::vector<Vertex> getPath(const StopId stop) const
    {
        return journeyToPath(getJourneys(stop).back());
    }

    inline std::vector<std::string> getRouteDescription(const StopId stop) const
    {
        return data.raptorData.journeyToText(getJourneys(stop).back());
    }

    template <bool RESET_CAPACITIES = false>
    inline void clear() noexcept
    {
        stopsUpdatedByRoute.clear();
        stopsUpdatedByTransfer.clear();
        routesServingUpdatedStops.clear();
        sourceEdge = noEdge;
        targetEdge = noEdge;
        sourceVertex = noVertex;
        targetVertex = noVertex;
        sourceStop = StopId(data.raptorData.numberOfStops());
        targetStop = StopId(data.raptorData.numberOfStops() + 1);
        sourceDepartureTime = never;
        walkingDistance = INFTY;
        if constexpr (RESET_CAPACITIES) {
            std::vector<Round>().swap(rounds);
            std::vector<ArrivalTime>(earliestArrival.size(), never)
                .swap(earliestArrival);
        } else {
            rounds.clear();
            Vector::fill(earliestArrival);
        }
    }

    inline void reset() noexcept { clear<true>(); }

    inline const Profiler& getProfiler() const noexcept { return profiler; }

    inline int getArrivalTime(const StopId stop,
        const size_t numberOfTrips) const noexcept
    {
        size_t round = numberOfTrips * RoundFactor;
        if constexpr (SeparateRouteAndTransferEntries) {
            if ((round + 1 < rounds.size()) && (rounds[round + 1][stop].arrivalTime < rounds[round][stop].arrivalTime))
                round++;
        }
        AssertMsg(
            rounds[round][stop].arrivalTime < never,
            "No label found for stop " << stop << " in round " << round << "!");
        return rounds[round][stop].arrivalTime;
    }

private:
    inline void initialize(const Edge source, const int departureTime,
        const Edge target) noexcept
    {
        sourceDepartureTime = departureTime;
        sourceEdge = source;
        sourceVertex = data.disp.inputGraph.get(FromVertex, source);
        if (data.raptorData.isStop(sourceVertex)) {
            sourceStop = StopId(sourceVertex);
        }

        targetEdge = target;
        targetVertex = data.disp.inputGraph.get(FromVertex, target);
        if (data.raptorData.isStop(targetVertex)) {
            targetStop = StopId(targetVertex);
        }

        startNewRound();
        arrivalByRoute(sourceStop, sourceDepartureTime);
        currentRound()[sourceStop].parent = sourceStop;
        currentRound()[sourceStop].parentDepartureTime = sourceDepartureTime;
        currentRound()[sourceStop].usesRoute = false;
        currentRound()[sourceStop].usesRide = false;
        if constexpr (SeparateRouteAndTransferEntries)
            startNewRound();
    }

    inline void collectRoutesServingUpdatedStops() noexcept
    {
        for (const StopId stop : stopsUpdatedByTransfer) {
            AssertMsg(data.raptorData.isStop(stop),
                "Stop " << stop << " is out of range!");
            const int arrivalTime = previousRound()[stop].arrivalTime;
            AssertMsg(arrivalTime < never, "Updated stop has arrival time = never!");
            for (const RouteSegment& route :
                data.raptorData.routesContainingStop(stop)) {
                AssertMsg(data.raptorData.isRoute(route.routeId),
                    "Route " << route.routeId << " is out of range!");
                AssertMsg(
                    data.raptorData
                            .stopIds[data.raptorData.firstStopIdOfRoute[route.routeId] + route.stopIndex]
                        == stop,
                    "RAPTOR data contains invalid route segments!");
                if (route.stopIndex + 1 == data.raptorData.numberOfStopsInRoute(route.routeId))
                    continue;
                if (data.raptorData.lastTripOfRoute(route.routeId)[route.stopIndex]
                        .departureTime
                    < arrivalTime)
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
            const size_t tripSize = data.raptorData.numberOfStopsInRoute(route);
            AssertMsg(stopIndex < tripSize - 1,
                "Cannot scan a route starting at/after the last stop (Route: "
                    << route << ", StopIndex: " << stopIndex
                    << ", TripSize: " << tripSize << ")!");

            const StopId* stops = data.raptorData.stopArrayOfRoute(route);
            const StopEvent* trip = data.raptorData.lastTripOfRoute(route);
            StopId stop = stops[stopIndex];
            AssertMsg(
                trip[stopIndex].departureTime >= previousRound()[stop].arrivalTime,
                "Cannot scan a route after the last trip has departed (Route: "
                    << route << ", Stop: " << stop << ", StopIndex: " << stopIndex
                    << ", Time: " << previousRound()[stop].arrivalTime
                    << ", LastDeparture: " << trip[stopIndex].departureTime << ")!");

            StopIndex parentIndex = stopIndex;
            const StopEvent* firstTrip = data.raptorData.firstTripOfRoute(route);
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

    inline void relaxInitialWalkingTransfer()
    {
        for (const Vertex stop : initialWalkingTransfers.getForwardPOIs()) {
            if (stop == targetStop || stop == sourceStop)
                continue;
            AssertMsg(data.raptorData.isStop(stop),
                "Reached POI " << stop << " is not a stop!");
            AssertMsg(initialWalkingTransfers.getForwardDistance(stop) != INFTY,
                "Vertex " << stop << " was not reached!");

            profiler.countMetric(METRIC_DIRECT_WALKING);

            const int arrivalTime = sourceDepartureTime + initialWalkingTransfers.getForwardDistance(stop);
            if (arrivalByTransfer(StopId(stop), arrivalTime)) {
                EarliestArrivalLabel& label = currentRound()[stop];
                label.parent = sourceStop;
                label.parentDepartureTime = sourceDepartureTime;
                label.usesRoute = false;
                label.usesRide = false;
                label.transferId = noEdge;
            }
        }

        if (initialWalkingTransfers.getDistance() <= data.maxWalkTime) {
            profiler.countMetric(METRIC_DIRECT_WALKING);
            const int arrivalTime = sourceDepartureTime + initialWalkingTransfers.getDistance();
            if (arrivalByTransfer(targetStop, arrivalTime)) {
                walkingDistance = initialWalkingTransfers.getDistance();
                EarliestArrivalLabel& label = currentRound()[targetStop];
                label.parent = sourceStop;
                label.parentDepartureTime = sourceDepartureTime;
                label.usesRoute = false;
                label.usesRide = false;
                label.transferId = noEdge;
            }
        }

        if (data.raptorData.isStop(sourceStop)) {
            relaxTransfers();
        }
    }

    inline void relaxTransfers() noexcept
    {
        stopsUpdatedByTransfer.clear();
        routesServingUpdatedStops.clear();
        for (const StopId stop : stopsUpdatedByRoute) {
            const int earliestArrivalTime = SeparateRouteAndTransferEntries
                ? previousRound()[stop].arrivalTime
                : currentRound()[stop].arrivalTime;
            for (const Edge edge : data.raptorData.transferGraph.edgesFrom(stop)) {
                profiler.countMetric(METRIC_EDGES);
                const int arrivalTime = earliestArrivalTime + data.raptorData.transferGraph.get(TravelTime, edge);
                AssertMsg(data.raptorData.isStop(
                              data.raptorData.transferGraph.get(ToVertex, edge)),
                    "Graph contains edges to non stop vertices!");
                const StopId toStop = StopId(data.raptorData.transferGraph.get(ToVertex, edge));
                if (!data.raptorData.isStop(toStop))
                    continue; // Necessary because of to many vertices in transfer graph
                if (arrivalByTransfer(toStop, arrivalTime)) {
                    EarliestArrivalLabel& label = currentRound()[toStop];
                    label.parent = stop;
                    label.parentDepartureTime = earliestArrivalTime;
                    label.usesRoute = false;
                    label.usesRide = false;
                    label.transferId = edge;
                }
            }

            if (initialWalkingTransfers.getBackwardDistance(stop) != INFTY) {
                profiler.countMetric(METRIC_DIRECT_WALKING);

                const int arrivalTime = earliestArrivalTime + initialWalkingTransfers.getBackwardDistance(stop);
                if (arrivalByTransfer(targetStop, arrivalTime)) {
                    EarliestArrivalLabel& label = currentRound()[targetStop];
                    label.parent = stop;
                    label.parentDepartureTime = earliestArrivalTime;
                    label.usesRoute = false;
                    label.usesRide = false;
                    label.transferId = noEdge;
                }
            }

            if constexpr (SeparateRouteAndTransferEntries) {
                const int arrivalTime = earliestArrivalTime + getMinTransferTime(stop);
                if (arrivalByTransfer(stop, arrivalTime)) {
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

    inline void relaxRideTransfers() noexcept
    {
        for (const StopId fromStop : stopsUpdatedByRoute) {
            const int earliestArrivalTime = SeparateRouteAndTransferEntries
                ? previousRound()[fromStop].arrivalTime
                : currentRound()[fromStop].arrivalTime;
            int pickup = data.disp.inputGraph.beginEdgeFrom(Vertex(fromStop));
            if (fromStop == sourceStop) {
                pickup = sourceEdge;
            }
            for (const Edge edgeFrom : data.rideTransferGraph.edgesFrom(fromStop)) {
                const auto vehicleVertex = data.rideTransferGraph.get(ToVertex, edgeFrom);
                const auto vehId = data.getVehicleFromVertex(vehicleVertex);
                const auto pickupInfo = data.rideTransferGraph.get(InsertionInfo, edgeFrom);
                if (pickupInfo.vehicleId == RIDERAPTOR::noVehicleId)
                    continue;

                profiler.countMetric(METRIC_TOTAL_DE,
                    data.rideTransferGraph.outDegree(vehicleVertex));

                if (optimizationFlags.UseTimePruning && (pickupInfo.minDepTime > earliestArrival[targetStop].getArrivalTimeByTransfer() || pickupInfo.maxDepTime < earliestArrivalTime)) {
                    profiler.countMetric(METRIC_FILTERED_PICKUP_EDGES);
                    continue;
                }

                const auto pickupDetour = data.disp.getPickupDetour(earliestArrivalTime, pickupInfo);
                profiler.countMetric(METRIC_PICKUP_EDGES);

                for (const Edge edgeTo :
                    data.rideTransferGraph.edgesFrom(vehicleVertex)) {
                    const auto toStop = StopId(data.rideTransferGraph.get(ToVertex, edgeTo));
                    if (toStop == sourceStop || toStop == fromStop) {
                        continue;
                    }
                    const auto areStops = data.raptorData.isStop(fromStop) && data.raptorData.isStop(toStop);

                    const auto dropoffInfo = data.rideTransferGraph.get(InsertionInfo, edgeTo);
                    if (dropoffInfo.vehicleId == RIDERAPTOR::noVehicleId)
                        continue;
                    const auto latestArrival = std::min(earliestArrival[toStop].getArrivalTimeByTransfer(),
                        earliestArrival[targetStop].getArrivalTimeByTransfer());

                    if (optimizationFlags.UseLeewayPruning && areStops && pickupDetour > data.rideTransferGraph.get(Weight, edgeTo)) {
                        profiler.countMetric(METRIC_LEEWAY_PRUNING);
                        break;
                    }

                    if (optimizationFlags.UseTimePruning && (dropoffInfo.minArrTime > latestArrival)) {
                        profiler.countMetric(METRIC_TIME_PRUNING);
                        continue;
                    }

                    int dropoff = data.disp.inputGraph.beginEdgeFrom(Vertex(toStop));
                    if (toStop == targetStop) {
                        dropoff = targetEdge;
                    }

                    const bool useDistance = optimizationFlags.UseDistanceMatrix && areStops;
                    karri::RideRAPTORRequest req;
                    req.pickupSpot = pickup;
                    req.dropoffSpot = dropoff;
                    req.minDepTime = earliestArrivalTime;
                    req.vehicleId = vehId;
                    req.directDistance = useDistance ? data.distanceMatrix.getDistance(fromStop, toStop)
                                                     : INFTY;
                    req.pickupInfo = pickupInfo;
                    req.dropoffInfo = dropoffInfo;

                    if (optimizationFlags.UseApproximation && !useDistance && !data.disp.isFeasible(req, latestArrival)) {
                        profiler.countMetric(METRIC_FILTERED_BY_APPROX);
                        continue;
                    }

                    profiler.countMetric(METRIC_TRIED_RIDES);
                    if (!useDistance)
                        profiler.startExtraTimer();
                    const auto [departureTime, arrivalTime] = useDistance ? data.disp.tryInsertionForVehicle<true>(req)
                                                                          : data.disp.tryInsertionForVehicle<false>(req);
                    if (!useDistance)
                        profiler.countMetric(METRIC_CHSEARCHES);
                    if (!useDistance)
                        profiler.doneExtraTimer();
                    if (arrivalTime != INFTY)
                        profiler.countMetric(METRIC_FEASIBLE_RIDES);

                    if (arrivalByTransfer(toStop, arrivalTime)) {
                        profiler.countMetric(METRIC_STOPS_BY_RIDETRANSFER);
                        EarliestArrivalLabel& label = currentRound()[toStop];
                        label.parent = fromStop;
                        label.parentDepartureTime = departureTime;
                        label.usesRoute = false;
                        label.usesRide = true;
                        label.vehicleId = vehId;
                    }
                }
            }
        }
    }

    inline int getMinTransferTime(const StopId stop) const noexcept
    {
        if constexpr (!UseMinTransferTimes) {
            suppressUnusedParameterWarning(stop);
            return 0;
        } else {
            return data.raptorData.stopData[stop].minTransferTime;
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
        rounds.emplace_back(data.raptorData.numberOfStops() + 2);
    }

    inline bool arrivalByRoute(const StopId stop, const int time) noexcept
    {
        if constexpr (TargetPruning)
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
        if constexpr (TargetPruning)
            if (earliestArrival[targetStop].getArrivalTimeByTransfer() <= time)
                return false;
        if (earliestArrival[stop].getArrivalTimeByTransfer() <= time)
            return false;
        profiler.countMetric(METRIC_STOPS_BY_TRANSFER);
        currentRound()[stop].arrivalTime = time;
        earliestArrival[stop].setArrivalTimeByTransfer(time);
        if (data.raptorData.isStop(stop))
            stopsUpdatedByTransfer.insert(stop); // Because of targetStop
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
            if (stop == targetStop) {
                journey.emplace_back(label.parent, Vertex(targetEdge),
                    label.parentDepartureTime, label.arrivalTime,
                    label.usesRoute, label.usesRide, label.routeId);
            } else {
                journey.emplace_back(label.parent, stop, label.parentDepartureTime,
                    label.arrivalTime, label.usesRoute, label.usesRide,
                    label.routeId);
            }
            stop = label.parent;
            if constexpr (SeparateRouteAndTransferEntries) {
                round--;
            } else {
                if (label.usesRoute)
                    round--;
            }
        } while (journey.back().from != sourceStop);
        // Necessary to display correct Id of source
        if (!data.raptorData.isStop(sourceStop)) {
            journey.back().from = Vertex(sourceEdge);
        }
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
    RIDERAPTOR::Data& data;
    std::vector<Round> rounds;

    std::vector<ArrivalTime> earliestArrival;

    IndexedSet<false, StopId> stopsUpdatedByRoute;
    IndexedSet<false, StopId> stopsUpdatedByTransfer;
    IndexedMap<StopIndex, false, RouteId> routesServingUpdatedStops;

    BucketCHInitialTransfers initialWalkingTransfers;

    Edge sourceEdge;
    Vertex sourceVertex;
    StopId sourceStop;
    Edge targetEdge;
    Vertex targetVertex;
    StopId targetStop;
    int sourceDepartureTime;
    int walkingDistance;

    Profiler profiler;
    RideOptimizationFlags optimizationFlags;
};

} // namespace RAPTOR
