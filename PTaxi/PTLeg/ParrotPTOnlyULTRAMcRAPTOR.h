#pragma once

#include <vector>

#include <ULTRA/DataStructures/Container/IndexedSet.h>
#include <ULTRA/DataStructures/Container/Map.h>
#include <ULTRA/DataStructures/RAPTOR/Data.h>
#include <ULTRA/DataStructures/RAPTOR/Entities/ArrivalLabel.h>
#include <ULTRA/DataStructures/RAPTOR/Entities/Bags.h>

#include <ULTRA/Algorithms/RAPTOR/Profiler.h>

namespace RAPTOR {
    template<typename InitialTransfersT, typename PROFILER = NoProfiler>
    class ParrotPTOnlyULTRAMcRAPTOR {
    public:
        using Profiler = PROFILER;
        using Type = ParrotPTOnlyULTRAMcRAPTOR<InitialTransfersT, Profiler>;
        using InitialTransferGraph = CHGraph;
        using SourceType = Vertex;

    private:

        static constexpr StopId originSentinelStop = StopId(static_cast<u_int32_t>(-2));
        static_assert(originSentinelStop.value() != noStop.value(), "Origin sentinel stop must be different from noStop.");

        struct Label {
            Label() : arrivalTime(never), walkingDistance(INFTY), parentStop(noStop), parentIndex(-1),
                      parentDepartureTime(never), routeId(noRouteId) {
            }

            Label(const Label &parentLabel, const StopId stop, const size_t parentIndex) : arrivalTime(
                    parentLabel.arrivalTime),
                walkingDistance(parentLabel.walkingDistance),
                parentStop(stop),
                parentIndex(parentIndex),
                parentDepartureTime(parentLabel.arrivalTime),
                transferId(noEdge) {
            }

            Label(const int departureTime, const StopId sourceStop) : arrivalTime(departureTime),
                                                                      walkingDistance(0),
                                                                      parentStop(sourceStop),
                                                                      parentIndex(-1),
                                                                      parentDepartureTime(departureTime),
                                                                      routeId(noRouteId) {
            }

            int arrivalTime;
            int walkingDistance;

            StopId parentStop;
            size_t parentIndex;
            int parentDepartureTime;

            union {
                RouteId routeId;
                Edge transferId;
            };

            inline bool dominates(const Label &other) const noexcept {
                return arrivalTime <= other.arrivalTime && walkingDistance <= other.walkingDistance;
            }
        };

        struct BestLabel {
            BestLabel() : arrivalTime(never), walkingDistance(INFTY) {
            }

            BestLabel(const int arrivalTime, const int walkingDistance) : arrivalTime(arrivalTime),
                                                                          walkingDistance(walkingDistance) {
            }

            template<typename LABEL>
            BestLabel(const LABEL &label) : arrivalTime(label.arrivalTime),
                                            walkingDistance(label.walkingDistance) {
            }

            template<typename LABEL>
            inline bool dominates(const LABEL &other) const noexcept {
                return arrivalTime <= other.arrivalTime && walkingDistance <= other.walkingDistance;
            }

            int arrivalTime;
            int walkingDistance;
        };

        struct RouteLabel {
            const StopEvent *trip;
            int walkingDistance;
            StopIndex parentStop;
            size_t parentIndex;

            inline bool dominates(const RouteLabel &other) const noexcept {
                return trip <= other.trip && walkingDistance <= other.walkingDistance;
            }
        };

        using BagType = Bag<Label>;
        using BestBagType = Bag<BestLabel>;
        using Round = std::vector<BagType>;
        using RouteBagType = RouteBag<RouteLabel>;

    public:
        ParrotPTOnlyULTRAMcRAPTOR(const Data &data,
                                  InitialTransfersT &initialTransfers, const PTStations &stations,
                                  const int psgNumEdges,
                                  const Profiler &profilerTemplate = Profiler()) : data(data),
            stations(stations),
            psgEdgeToStation(psgNumEdges, INVALID_ID),
            initialTransfers(initialTransfers),
            bestLabels(data.numberOfStops() + 1),
            stopsUpdatedByRoute(data.numberOfStops()),
            stopsUpdatedByTransfer(data.numberOfStops()),
            routesServingUpdatedStops(data.numberOfRoutes()),
        sourceStop(noStop),
            targetStop(noStop),
            sourceDepartureTime(never),
            profiler(profilerTemplate) {
            Assert(data.hasImplicitBufferTimes(), "Departure buffer times have to be implicit!");
            profiler.registerExtraRounds({EXTRA_ROUND_CLEAR, EXTRA_ROUND_INITIALIZATION});
            profiler.registerPhases({PHASE_INITIALIZATION, PHASE_COLLECT, PHASE_SCAN, PHASE_TRANSFERS});
            profiler.registerMetrics({
                METRIC_ROUTES, METRIC_ROUTE_SEGMENTS, METRIC_EDGES, METRIC_STOPS_BY_TRIP, METRIC_STOPS_BY_TRANSFER
            });
            profiler.initialize();
            initializeEdgeToStationMappings();
        }

        inline void run(const int originPsgEdge, int,
                         const int destinationPsgEdge, int,
                         const int departureTime,
                         karri::stats::PtPerformanceStats &stats,
                         const size_t maxRounds = INFTY) noexcept {
            KaRRiTimer timer;
            profiler.start();
            profiler.startExtraRound(EXTRA_ROUND_CLEAR);
            clear();
            profiler.doneRound();

            profiler.startExtraRound(EXTRA_ROUND_INITIALIZATION);
            profiler.startPhase();
            initialize(originPsgEdge, destinationPsgEdge, departureTime, stats);
            profiler.donePhase(PHASE_INITIALIZATION);
            stats.roundInitializationTime += timer.elapsed<std::chrono::nanoseconds>();

            timer.restart();
            profiler.startPhase();
            relaxInitialTransfers(originPsgEdge, destinationPsgEdge, stats);
            profiler.donePhase(PHASE_TRANSFERS);
            profiler.doneRound();
            stats.relaxInitialTransfersTime += timer.elapsed<std::chrono::nanoseconds>();

            for (size_t i = 0; i < maxRounds; i++) {
                ++stats.numRounds;
                timer.restart();
                profiler.startRound();
                profiler.startPhase();
                startNewRound();
                profiler.donePhase(PHASE_INITIALIZATION);
                stats.roundInitializationTime += timer.elapsed<std::chrono::nanoseconds>();
                timer.restart();
                profiler.startPhase();
                collectRoutesServingUpdatedStops();
                profiler.donePhase(PHASE_COLLECT);
                stats.collectRoutesTime += timer.elapsed<std::chrono::nanoseconds>();
                timer.restart();
                profiler.startPhase();
                scanRoutes(stats);
                profiler.donePhase(PHASE_SCAN);
                stats.scanRoutesTime += timer.elapsed<std::chrono::nanoseconds>();
                if (stopsUpdatedByRoute.empty()) {
                    profiler.doneRound();
                    break;
                }
                timer.restart();
                profiler.startPhase();
                startNewRound();
                profiler.donePhase(PHASE_INITIALIZATION);
                stats.roundInitializationTime += timer.elapsed<std::chrono::nanoseconds>();
                timer.restart();
                profiler.startPhase();
                relaxIntermediateTransfers(stats);
                profiler.donePhase(PHASE_TRANSFERS);
                stats.relaxIntermediateTransfersTime += timer.elapsed<std::chrono::nanoseconds>();
                profiler.doneRound();
            }
            profiler.done();
        }

        inline std::vector<Journey> getJourneys() const noexcept {
            std::vector<Journey> journeys;
            for (size_t round = 0; round < rounds.size(); round += 2) {
                const size_t trueRound = std::min(round + 1, rounds.size() - 1);
                for (size_t i = 0; i < rounds[trueRound][targetStop].size(); i++) {
                    getJourney(journeys, trueRound, targetStop, i);
                }
            }
            return journeys;
        }

        inline std::vector<WalkingParetoLabel> getResults() const noexcept {
            std::vector<WalkingParetoLabel> result;
            for (size_t round = 0; round < rounds.size(); round += 2) {
                const size_t trueRound = std::min(round + 1, rounds.size() - 1);
                for (const Label &label: rounds[trueRound][targetStop].labels) {
                    result.emplace_back(label, round / 2);
                }
            }
            return result;
        }

        template<bool RESET_CAPACITIES = false>
        inline void clear() noexcept {
            stopsUpdatedByRoute.clear();
            stopsUpdatedByTransfer.clear();
            routesServingUpdatedStops.clear();
            sourceStop = noStop;
            targetStop = StopId(data.numberOfStops());
            sourceDepartureTime = never;
            if constexpr (RESET_CAPACITIES) {
                std::vector<Round>().swap(rounds);
                std::vector<BestBagType>(bestLabels.size()).swap(bestLabels);
            } else {
                rounds.clear();
                Vector::fill(bestLabels);
            }
        }

        inline void reset() noexcept {
            clear<true>();
        }

        inline const Profiler &getProfiler() const noexcept {
            return profiler;
        }

    private:
        inline void initializeEdgeToStationMappings() noexcept {
            for (const auto &station: stations) {
                if (station.psgEdgeId == INVALID_EDGE)
                    continue;
                psgEdgeToStation[station.psgEdgeId] = station.stationId;
            }
        }

        inline void initialize(const int originPsgEdge, const int destinationPsgEdge, const int departureTime,
                               karri::stats::PtPerformanceStats &stats) noexcept {

            targetStop = StopId(data.numberOfStops());
            if (destinationPsgEdge >= 0 && destinationPsgEdge < psgEdgeToStation.size()) {
                int stationId = psgEdgeToStation[destinationPsgEdge];
                if (stationId != INVALID_ID) {
                    targetStop = StopId(stationId);
                }
            }

            sourceDepartureTime = departureTime;
            startNewRound();

            // Initialize source stop if the origin edge maps to a station
            sourceStop = originSentinelStop;
            if (originPsgEdge >= 0 && originPsgEdge < psgEdgeToStation.size()) {
                int stationId = psgEdgeToStation[originPsgEdge];
                if (stationId != INVALID_ID && data.isStop(StopId(stationId))) {
                    sourceStop = StopId(stationId);
                    Label initialLabel(sourceDepartureTime, noStop); // Sentinel parent since we are at the source stop
                    arrivalByRoute(sourceStop, initialLabel, stats);
                }
            }
            startNewRound();
        }

        inline void collectRoutesServingUpdatedStops() noexcept {
            for (const StopId stop: stopsUpdatedByTransfer) {
                Assert(data.isStop(stop), "Stop " << stop << " is out of range!");
                for (const RouteSegment &route: data.routesContainingStop(stop)) {
                    Assert(data.isRoute(route.routeId), "Route " << route.routeId << " is out of range!");
                    Assert(data.stopIds[data.firstStopIdOfRoute[route.routeId] + route.stopIndex] == stop,
                           "RAPTOR data contains invalid route segments!");
                    if (route.stopIndex + 1 == data.numberOfStopsInRoute(route.routeId)) continue;
                    if (routesServingUpdatedStops.contains(route.routeId)) {
                        routesServingUpdatedStops[route.routeId] = std::min(
                            routesServingUpdatedStops[route.routeId], route.stopIndex);
                    } else {
                        routesServingUpdatedStops.insert(route.routeId, route.stopIndex);
                    }
                }
            }
        }

        inline void scanRoutes(karri::stats::PtPerformanceStats &stats) noexcept {
            stopsUpdatedByRoute.clear();
            for (const RouteId route: routesServingUpdatedStops.getKeys()) {
                profiler.countMetric(METRIC_ROUTES);
                ++stats.numRoutesScanned;
                StopIndex stopIndex = routesServingUpdatedStops[route];
                const size_t tripSize = data.numberOfStopsInRoute(route);
                Assert(stopIndex < tripSize - 1,
                       "Cannot scan a route starting at/after the last stop (Route: " << route << ", StopIndex: " <<
                       stopIndex << ", TripSize: " << tripSize << ")!");

                const u_int32_t routeIdInt = route.value();

                const StopId *stops = data.stopArrayOfRoute(route);
                StopId stop = stops[stopIndex];

                const StopEvent *firstTrip = data.firstTripOfRoute(route);
                const StopEvent *lastTrip = data.lastTripOfRoute(route);

                RouteBagType routeBag;

                while (stopIndex < tripSize - 1) {
                    for (size_t i = 0; i < previousRound()[stop].size(); i++) {
                        const Label &label = previousRound()[stop][i];
                        const StopEvent *trip = firstTrip;
                        while ((trip < lastTrip) && (trip[stopIndex].departureTime < label.arrivalTime)) {
                            trip += tripSize;
                        }
                        if (trip[stopIndex].departureTime < label.arrivalTime) continue;

                        RouteLabel newLabel;
                        newLabel.trip = trip;
                        newLabel.walkingDistance = label.walkingDistance;
                        newLabel.parentStop = stopIndex;
                        newLabel.parentIndex = i;
                        routeBag.merge(newLabel);
                    }
                    stopIndex++;
                    stop = stops[stopIndex];
                    profiler.countMetric(METRIC_ROUTE_SEGMENTS);
                    ++stats.numRouteSegmentsScanned;
                    for (const RouteLabel &label: routeBag.labels) {
                        Label newLabel;
                        newLabel.arrivalTime = label.trip[stopIndex].arrivalTime;
                        newLabel.walkingDistance = label.walkingDistance;
                        newLabel.parentStop = stops[label.parentStop];
                        newLabel.parentIndex = label.parentIndex;
                        newLabel.parentDepartureTime = label.trip[label.parentStop].departureTime;
                        newLabel.routeId = route;
                        arrivalByRoute(stop, newLabel, stats);
                    }
                }
            }
        }

        inline void relaxInitialTransfers(const int originPsgEdge, const int destinationPsgEdge, karri::stats::PtPerformanceStats &stats) noexcept {
            // If the origin edge maps to a station, it is considered reached by initial transfer, too
            if (sourceStop != originSentinelStop) {
                KASSERT(psgEdgeToStation[originPsgEdge] == sourceStop, "Source stop does not match mapping from origin edge!");
                stopsUpdatedByTransfer.insert(sourceStop);
                currentRound()[sourceStop].resize(1);
                currentRound()[sourceStop][0] = Label(previousRound()[sourceStop][0], sourceStop, 0);
            }
            initialTransfers.template run<true>(originPsgEdge, destinationPsgEdge);
            for (const Vertex stop: initialTransfers.getForwardPOIs()) {
                if (stop == targetStop) continue;
                Assert(data.isStop(stop), "Reached POI " << stop << " is not a stop!");
                Assert(initialTransfers.getForwardDistance(stop) != INFTY, "Vertex " << stop << " was not reached!");
                Label newLabel;
                newLabel.arrivalTime = sourceDepartureTime + parrot::karriToULTRATime(initialTransfers.getForwardDistance(stop));
                newLabel.walkingDistance = parrot::karriToULTRATime(initialTransfers.getForwardDistance(stop));
                newLabel.parentStop = sourceStop;
                newLabel.parentIndex = 0;
                newLabel.parentDepartureTime = sourceDepartureTime;
                newLabel.transferId = noEdge;
                arrivalByTransfer(StopId(stop), newLabel, stats);
            }
            if (initialTransfers.getDistance() != INFTY) {
                Label newLabel;
                newLabel.arrivalTime = sourceDepartureTime + parrot::karriToULTRATime(initialTransfers.getDistance());
                newLabel.walkingDistance = parrot::karriToULTRATime(initialTransfers.getDistance());
                newLabel.parentStop = sourceStop;
                newLabel.parentIndex = 0;
                newLabel.parentDepartureTime = sourceDepartureTime;
                newLabel.transferId = noEdge;
                arrivalByTransfer(targetStop, newLabel, stats);
            }
        }

        inline void relaxIntermediateTransfers(karri::stats::PtPerformanceStats &stats) noexcept {
            stopsUpdatedByTransfer.clear();
            routesServingUpdatedStops.clear();
            for (const StopId stop: stopsUpdatedByRoute) {
                stopsUpdatedByTransfer.insert(stop);
                const BagType &bag = previousRound()[stop];
                currentRound()[stop].resize(bag.size());
                for (size_t i = 0; i < bag.size(); i++) {
                    currentRound()[stop][i] = Label(bag[i], stop, i);
                }
            }

            for (const StopId stop: stopsUpdatedByRoute) {
                const BagType &bag = previousRound()[stop];
                for (const Edge edge: data.transferGraph.edgesFrom(stop)) {
                    profiler.countMetric(METRIC_EDGES);
                    ++stats.numTransferEdgesRelaxed;
                    const StopId toStop = StopId(data.transferGraph.get(ToVertex, edge));
                    Assert(data.isStop(toStop), "Graph contains edges to non-stop vertices!");
                    const int travelTime = data.transferGraph.get(TravelTime, edge);
                    for (size_t i = 0; i < bag.size(); i++) {
                        Label newLabel;
                        newLabel.arrivalTime = bag[i].arrivalTime + travelTime;
                        newLabel.walkingDistance = bag[i].walkingDistance + travelTime;
                        newLabel.parentStop = stop;
                        newLabel.parentIndex = i;
                        newLabel.parentDepartureTime = bag[i].arrivalTime;
                        newLabel.transferId = edge;
                        arrivalByTransfer(toStop, newLabel, stats);
                    }
                }
                if (initialTransfers.getBackwardDistance(stop) != INFTY) {
                    const int travelTime = initialTransfers.getBackwardDistance(stop);
                    for (size_t i = 0; i < bag.size(); i++) {
                        Label newLabel;
                        newLabel.arrivalTime = bag[i].arrivalTime + parrot::karriToULTRATime(travelTime);
                        newLabel.walkingDistance = bag[i].walkingDistance + parrot::karriToULTRATime(travelTime);
                        newLabel.parentStop = stop;
                        newLabel.parentIndex = i;
                        newLabel.parentDepartureTime = bag[i].arrivalTime;
                        newLabel.transferId = noEdge;
                        arrivalByTransfer(targetStop, newLabel, stats);
                    }
                }
            }
        }

        inline Round &currentRound() noexcept {
            Assert(!rounds.empty(), "Cannot return current round, because no round exists!");
            return rounds.back();
        }

        inline Round &previousRound() noexcept {
            Assert(rounds.size() >= 2, "Cannot return previous round, because less than two rounds exist!");
            return rounds[rounds.size() - 2];
        }

        inline void startNewRound() noexcept {
            rounds.emplace_back(data.numberOfStops() + 1);
        }

        inline void arrival(const StopId stop, const Label &label, IndexedSet<false, StopId> &updatedStops,
        Metric metric, int64_t &numImproved) noexcept {
            Assert(data.isStop(stop) || stop == targetStop, "Stop " << stop << " is out of range!");
            if (bestLabels[targetStop].dominates(label)) return;
            if (!bestLabels[stop].merge(BestLabel(label))) return;
            profiler.countMetric(metric);
            ++numImproved;
            currentRound()[stop].mergeUndominated(label);
            Assert(bestLabels[stop].dominates(currentRound()[stop]), "Best bag does not dominate current bag!");
            if (data.isStop(stop)) updatedStops.insert(stop);
        }

        inline void arrivalByTransfer(const StopId stop, const Label &label,
                                   karri::stats::PtPerformanceStats &stats) noexcept {
            arrival(stop, label, stopsUpdatedByTransfer, METRIC_STOPS_BY_TRANSFER, stats.numStopsImprovedByTransfer);
        }

        inline void arrivalByRoute(const StopId stop, const Label &label,
                                   karri::stats::PtPerformanceStats &stats) noexcept {
            arrival(stop, label, stopsUpdatedByRoute, METRIC_STOPS_BY_TRIP, stats.numStopsImprovedByTrip);
        }

        inline void getJourney(std::vector<Journey> &journeys, size_t round, StopId stop, size_t index) const noexcept {
            Journey journey;
            do {
                Assert(round != size_t(-1), "Backtracking parent pointers did not pass through the source stop!");
                const Label &label = rounds[round][stop][index];
                journey.emplace_back(label.parentStop, stop, label.parentDepartureTime, label.arrivalTime,
                                     round % 2 == 0, label.routeId);
                stop = label.parentStop;
                index = label.parentIndex;
                round--;
            } while (journey.back().from != sourceStop);
            journeys.emplace_back(Vector::reverse(journey));
        }

    private:
        const Data &data;
        const PTStations &stations;
        // Fast lookup maps: edge ID -> station ID
        std::vector<int> psgEdgeToStation;

        InitialTransfersT &initialTransfers;

        std::vector<Round> rounds;

        std::vector<BestBagType> bestLabels;

        IndexedSet<false, StopId> stopsUpdatedByRoute;
        IndexedSet<false, StopId> stopsUpdatedByTransfer;
        IndexedMap<StopIndex, false, RouteId> routesServingUpdatedStops;


        StopId sourceStop; // Stop if origin edge maps to a station, originSentinelStop otherwise
        StopId targetStop;
        int sourceDepartureTime;

        Profiler profiler;
    };
}
