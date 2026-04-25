#pragma once

#include <vector>

#include <Common/TimeConversion.h>
#include <ULTRA/DataStructures/Container/IndexedSet.h>
#include <ULTRA/DataStructures/Container/Map.h>
#include <ULTRA/DataStructures/RAPTOR/Data.h>
#include <ULTRA/DataStructures/RAPTOR/Entities/ArrivalLabel.h>
#include <ULTRA/DataStructures/RAPTOR/Entities/Bags.h>

#include "ULTRA/Algorithms/RAPTOR/Profiler.h"


namespace RAPTOR {
    // ULTRA-RAPTOR specialization that uses initial labels based on access ride-pooling trips and finds the
    // journey with smallest cost according to a cost function.
    // Works like multi-criteria ULTRA-RAPTOR to deal with the fact that a ride-pooling trip with a later arrival time
    // might have a smaller cost than a ride-pooling trip with an earlier arrival time.
    // Uses target pruning based on only the cost though.
    template<typename InitialTransfersT, typename PROFILER = RAPTOR::NoProfiler>
    class ParrotCombinedULTRAMcRAPTOR {
    public:
        using Profiler = PROFILER;
        using Type = ParrotCombinedULTRAMcRAPTOR<InitialTransfersT, Profiler>;
        using SourceType = Vertex;

    private:

        static constexpr StopId originSentinelStop = StopId(static_cast<u_int32_t>(-2));
        static_assert(originSentinelStop.value() != noStop.value(), "Origin sentinel stop must be different from noStop.");

        struct Label {
            Label() : arrivalTime(never), cost(INFTY), parentStop(noStop), parentIndex(-1), parentDepartureTime(never),
                      usesRoute(false), usesTaxi(false), routeId(noRouteId) {
            }

            Label(const Label &parentLabel, const StopId stop, const size_t parentIndex) : arrivalTime(
                    parentLabel.arrivalTime),
                cost(parentLabel.cost),
                parentStop(stop),
                parentIndex(parentIndex),
                parentDepartureTime(parentLabel.arrivalTime),
                usesRoute(false),
                usesTaxi(false),
                transferId(noEdge) {
            }

            Label(const int departureTime, const StopId sourceStop) : arrivalTime(departureTime),
                                                                      cost(0),
                                                                      parentStop(sourceStop),
                                                                      parentIndex(-1),
                                                                      parentDepartureTime(departureTime),
                                                                      usesRoute(false),
                                                                      usesTaxi(false),
                                                                      routeId(noRouteId) {
            }

            int arrivalTime;
            int cost; // Cost of the journey up to this point, including the access ride-pooling trip (if any)

            StopId parentStop;
            size_t parentIndex;
            int parentDepartureTime;
            bool usesRoute;
            bool usesTaxi;

            union {
                RouteId routeId;
                Edge transferId;
            };

            inline bool dominates(const Label &other) const noexcept {
                return arrivalTime <= other.arrivalTime && cost <= other.cost;
            }
        };

        struct BestLabel {
            BestLabel() : arrivalTime(never), cost(INFTY) {
            }

            BestLabel(const int arrivalTime, const int cost) : arrivalTime(arrivalTime),
                                                               cost(cost) {
            }

            template<typename LABEL>
            BestLabel(const LABEL &label) : arrivalTime(label.arrivalTime),
                                            cost(label.cost) {
            }

            template<typename LABEL>
            inline bool dominates(const LABEL &other) const noexcept {
                return arrivalTime <= other.arrivalTime && cost <= other.cost;
            }

            int arrivalTime;
            int cost;
        };

        struct RouteLabel {
            StopEvent const *trip;
            int costWithoutTripCost;
            StopIndex parentStop;
            size_t parentIndex;

            inline bool dominates(const RouteLabel &other) const noexcept {
                return trip <= other.trip && costWithoutTripCost <= other.costWithoutTripCost;
            }
        };

        using BagType = Bag<Label>;
        using BestBagType = Bag<BestLabel>;
        using Round = std::vector<BagType>;
        using RouteBagType = RouteBag<RouteLabel>;

    public:
        ParrotCombinedULTRAMcRAPTOR(const Data &data,
                            InitialTransfersT &initialTransfers, const PTStations &stations,
                            const int psgNumEdges,
                            const Profiler &profilerTemplate = Profiler())
            : data(data),
              stations(stations),
              psgEdgeToStation(psgNumEdges, INVALID_ID),
              initialTransfers(initialTransfers),
              bestLabels(data.numberOfStops() + 1),
              stopsUpdatedByRoute(data.numberOfStops()),
              stopsUpdatedByTransfer(data.numberOfStops()),
              routesServingUpdatedStops(data.numberOfRoutes()),
              targetStop(noStop),
              sourceDepartureTime(never),
              bestCostAtTarget(INFTY),
              profiler(profilerTemplate) {
            KASSERT(data.hasImplicitBufferTimes(), "Departure buffer times have to be implicit!");
            profiler.registerExtraRounds({EXTRA_ROUND_CLEAR, EXTRA_ROUND_INITIALIZATION});
            profiler.registerPhases({PHASE_INITIALIZATION, PHASE_COLLECT, PHASE_SCAN, PHASE_TRANSFERS});
            profiler.registerMetrics({
                METRIC_ROUTES, METRIC_ROUTE_SEGMENTS, METRIC_EDGES, METRIC_STOPS_BY_TRIP, METRIC_STOPS_BY_TRANSFER
            });
            profiler.initialize();
            initializeEdgeToStationMappings();
        }


        void runWithTaxi(const int origPsgEdge, const int origVehEdge,
                         const int destPsgEdge, const int destVehEdge,
                         const int departureTime,
                         const karri::FirstTaxiLegResult &firstTaxiLeg, const std::vector<int> &distFromStations,
                         const int ptOnlyUpperBoundCost,
                         karri::stats::PtPerformanceStats &stats, const size_t maxRounds = INFTY) noexcept {
            run<true>(origPsgEdge, origVehEdge, destPsgEdge, destVehEdge, departureTime, firstTaxiLeg, distFromStations,
                      ptOnlyUpperBoundCost, stats, maxRounds);
        }

        // void run(const int origPsgEdge, const int origVehEdge,
        //                     const int destPsgEdge, const int destVehEdge,
        //                     const int departureTime,
        //                     karri::stats::PtPerformanceStats &stats, const size_t maxRounds = INFTY) noexcept {
        //     constexpr struct {} dummy;
        //     run<false>(origPsgEdge, origVehEdge, destPsgEdge, destVehEdge, departureTime, dummy, dummy,
        //                INFTY, stats, maxRounds);
        // }

        bool hasFoundJourney() const noexcept {
            return !bestLabels[targetStop].empty();
        }

        struct BestJourney {
            Journey journey = {};
            int cost = INFTY;
            // If journey uses access RP trip, this gives the index of the trip among pareto-optimal access RP trips at the station
            int accessRpTripIndex = INVALID_INDEX;
        };

        BestJourney getJourneyWithBestCost() const noexcept {
            // KASSERT(bestLabels[targetStop].size() == 1, "Target has more than one best label.");
            int smallestCost = INFTY;
            size_t bestRound = rounds.size();
            size_t bestParentIndex = 0;
            for (size_t round = 0; round < rounds.size(); round += 2) {
                const size_t trueRound = std::min(round + 1, rounds.size() - 1);
                for (size_t i = 0; i < rounds[trueRound][targetStop].size(); i++) {
                    const Label &currentLabel = rounds[trueRound][targetStop][i];
                    if (currentLabel.cost < smallestCost) {
                        smallestCost = currentLabel.cost;
                        bestRound = trueRound;
                        bestParentIndex = i;
                    }
                }
            }
            if (smallestCost == INFTY)
                return {};
            // KASSERT(smallestCost < INFTY, "No label found for target stop!");

            // Sets bestParentIndex to parent index of label at first station. Can be used to reconstruct correct access RP trip.
            const auto journey = getJourney(bestRound, targetStop, bestParentIndex);
            KASSERT(!journey.empty(), "Backtracking the journey for the target stop did not yield a valid journey!");

            // If journey with best cost does not use RP at all, we consider combined journey infeasible
            if (!journey.front().usesTaxi && !journey.back().usesTaxi)
                return {};

            int accessRpTripIndex = INVALID_INDEX;
            if (journey.front().usesTaxi) {
                accessRpTripIndex = static_cast<int>(bestParentIndex);
            }
            return {journey, smallestCost, accessRpTripIndex};
        }



        inline const Profiler &getProfiler() const noexcept {
            return profiler;
        }

    private:

        template<bool USE_TAXI_ACCESS_AND_EGRESS, typename FirstTaxiLegResultT, typename DistFromStationsT>
        requires (!USE_TAXI_ACCESS_AND_EGRESS || (std::same_as<FirstTaxiLegResultT, karri::FirstTaxiLegResult> && std::same_as<DistFromStationsT, std::vector<int>>))
        void run(const int origPsgEdge, const int origVehEdge,
                        const int destPsgEdge, const int destVehEdge,
                        const int departureTime,
                        const FirstTaxiLegResultT &firstTaxiLeg, const DistFromStationsT &distFromStations,
                        const int ptOnlyUpperBoundCost,
                        karri::stats::PtPerformanceStats &stats, const size_t maxRounds = INFTY) noexcept {
            KaRRiTimer timer;
            profiler.start();
            profiler.startExtraRound(EXTRA_ROUND_CLEAR);
            clear();
            profiler.doneRound();

            profiler.startExtraRound(EXTRA_ROUND_INITIALIZATION);
            profiler.startPhase();
            initialize(origPsgEdge, origVehEdge, destPsgEdge, destVehEdge, departureTime, ptOnlyUpperBoundCost, stats);
            profiler.donePhase(PHASE_INITIALIZATION);

            stats.roundInitializationTime += timer.elapsed<std::chrono::nanoseconds>();
            timer.restart();

            profiler.startPhase();
            relaxInitialTransfers(stats);
            if constexpr (USE_TAXI_ACCESS_AND_EGRESS) {
                relaxInitialTransfersByTaxi(firstTaxiLeg, stats);
            }
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
                if constexpr (USE_TAXI_ACCESS_AND_EGRESS) {
                    relaxFinalTransfersByTaxi(distFromStations, stats);
                }
                profiler.donePhase(PHASE_TRANSFERS);
                stats.relaxIntermediateTransfersTime += timer.elapsed<std::chrono::nanoseconds>();
                profiler.doneRound();
            }
            profiler.done();
        }

        template<bool RESET_CAPACITIES = false>
        inline void clear() noexcept {
            stopsUpdatedByRoute.clear();
            stopsUpdatedByTransfer.clear();
            routesServingUpdatedStops.clear();
            originVehEdge = noEdge;
            originPsgEdge = noEdge;
            destinationVehEdge = noEdge;
            destinationPsgEdge = noEdge;
            sourceStop = noStop;
            targetStop = StopId(data.numberOfStops());
            sourceDepartureTime = never;
            bestCostAtTarget = INFTY;
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

        inline void initializeEdgeToStationMappings() noexcept {
            for (const auto &station: stations) {
                if (station.psgEdgeId == INVALID_EDGE)
                    continue;
                psgEdgeToStation[station.psgEdgeId] = station.stationId;
            }
        }

        inline void initialize(const int originPsgEdge_, const int originVehEdge_,
                               const int destPsgEdge_, const int destVehEdge_, const int departureTime,
                               const int ptOnlyUpperBoundCost,
                               karri::stats::PtPerformanceStats &stats) noexcept {
            originPsgEdge = originPsgEdge_;
            originVehEdge = originVehEdge_;
            destinationPsgEdge = destPsgEdge_;
            destinationVehEdge = destVehEdge_;

            targetStop = StopId(data.numberOfStops());
            if (destPsgEdge_ >= 0 && destPsgEdge_ < psgEdgeToStation.size()) {
                int stationId = psgEdgeToStation[destPsgEdge_];
                if (stationId != INVALID_ID) {
                    targetStop = StopId(stationId);
                }
            }

            sourceDepartureTime = departureTime;
            bestCostAtTarget = ptOnlyUpperBoundCost;
            startNewRound();

            // Initialize source stop if the origin edge maps to a station
            sourceStop = originSentinelStop;
            if (originPsgEdge_ >= 0 && originPsgEdge_ < psgEdgeToStation.size()) {
                int stationId = psgEdgeToStation[originPsgEdge_];
                if (stationId != INVALID_ID && data.isStop(StopId(stationId))) {
                    sourceStop = StopId(stationId);
                    Label initialLabel(sourceDepartureTime, sourceStop);
                    arrivalByRoute(sourceStop, initialLabel, stats);
                }
            }
            startNewRound();
        }

        inline void collectRoutesServingUpdatedStops() noexcept {
            for (const StopId stop: stopsUpdatedByTransfer) {
                KASSERT(data.isStop(stop), "Stop " << stop << " is out of range!");
                for (const RouteSegment &route: data.routesContainingStop(stop)) {
                    KASSERT(data.isRoute(route.routeId), "Route " << route.routeId << " is out of range!");
                    KASSERT(data.stopIds[data.firstStopIdOfRoute[route.routeId] + route.stopIndex] == stop,
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
            using Calc = karri::CostCalculator;
            stopsUpdatedByRoute.clear();
            for (const RouteId route: routesServingUpdatedStops.getKeys()) {
                profiler.countMetric(METRIC_ROUTES);
                ++stats.numRoutesScanned;
                StopIndex stopIndex = routesServingUpdatedStops[route];
                const size_t tripSize = data.numberOfStopsInRoute(route);
                KASSERT(stopIndex < tripSize - 1,
                       "Cannot scan a route starting at/after the last stop (Route: " << route << ", StopIndex: " <<
                       stopIndex << ", TripSize: " << tripSize << ")!");

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

                        // RouteLabels are Pareto-optimal with respect to trip index (earlier trips are better) and
                        // cost without trip costs. If a RouteLabel l1 has an earlier trip and a smaller cost without trip
                        // costs than another label l2, then l1 will arrive at every stop on the route earlier and have
                        // smaller cost without trip. The total cost of l1 is then also smaller since a smaller arrival
                        // time implies a smaller trip cost. Thus, this allows us to generate labels at every stop
                        // that are Pareto-optimal with respect to arrival time and total cost.
                        RouteLabel newLabel;
                        newLabel.trip = trip;
                        newLabel.costWithoutTripCost = label.cost - Calc::calcTripCost(
                                                           parrot::ultraToKarriTime(
                                                               label.arrivalTime - sourceDepartureTime));
                        // newLabel.cost = label.cost - computeOnlyTripCost(label.arrivalTime - requestEarliestDeparture);
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
                        newLabel.cost = label.costWithoutTripCost + Calc::calcTripCost(
                                            parrot::ultraToKarriTime(newLabel.arrivalTime - sourceDepartureTime));
                        newLabel.parentStop = stops[label.parentStop];
                        newLabel.parentIndex = label.parentIndex;
                        newLabel.parentDepartureTime = label.trip[label.parentStop].departureTime;
                        newLabel.routeId = route;
                        newLabel.usesRoute = true;
                        arrivalByRoute(stop, newLabel, stats);
                    }
                }
            }
        }

        inline void relaxInitialTransfers(karri::stats::PtPerformanceStats &stats) noexcept {
            using Calc = karri::CostCalculator;
            // If the origin edge maps to a station, it is considered reached by initial transfer, too
            if (sourceStop != originSentinelStop && previousRound()[sourceStop].size() > 0) {
                KASSERT(psgEdgeToStation[originPsgEdge] == sourceStop, "Source stop does not match mapping from origin edge!");
                stopsUpdatedByTransfer.insert(sourceStop);
                currentRound()[sourceStop].resize(1);
                currentRound()[sourceStop][0] = Label(previousRound()[sourceStop][0], sourceStop, 0);
            }
            initialTransfers.template run<true>(originPsgEdge, destinationPsgEdge);
            for (const Vertex stop: initialTransfers.getForwardPOIs()) {
                if (stop == targetStop) continue;
                KASSERT(data.isStop(stop), "Reached POI " << stop << " is not a stop!");
                KASSERT(initialTransfers.getForwardDistance(stop) != INFTY, "Vertex " << stop << " was not reached!");
                Label newLabel;
                newLabel.arrivalTime = sourceDepartureTime + parrot::karriToULTRATime(
                                           initialTransfers.getForwardDistance(stop));
                newLabel.cost = Calc::calcCostForAddedTransferTime(
                                    initialTransfers.getForwardDistance(stop)) + Calc::penaltyForNewTransfer();
                newLabel.parentStop = sourceStop;
                newLabel.parentIndex = 0;
                newLabel.parentDepartureTime = sourceDepartureTime;
                newLabel.transferId = noEdge;
                arrivalByTransfer(StopId(stop), newLabel, stats);
            }
        }

        void relaxInitialTransfersByTaxi(const karri::FirstTaxiLegResult &firstTaxiLeg, karri::stats::PtPerformanceStats &stats) noexcept {
            using Calc = karri::CostCalculator;

            // Extension for first taxi leg
            for (const auto &stationId: firstTaxiLeg.getStationsWithResults()) {
                const Vertex stationVertex = Vertex(stationId);

                // Check if this is the destination station
                bool isDestStation = (destinationPsgEdge >= 0 && destinationPsgEdge < psgEdgeToStation.size() &&
                                      psgEdgeToStation[destinationPsgEdge] == stationId);
                // Check if this is the origin station
                bool isOriginStation = (originPsgEdge >= 0 && originPsgEdge < psgEdgeToStation.size() &&
                                        psgEdgeToStation[originPsgEdge] == stationId);

                if (isOriginStation || isDestStation)
                    continue;

                const auto &resultsForStation = firstTaxiLeg.getResultsForStation(stationId);
                for (int i = 0; i < resultsForStation.size(); i++) {
                    const auto &taxiResult = resultsForStation[i];
                    KASSERT(data.isStop(stationVertex), "Taxi station " << stationVertex << " is not a stop!");
                    Label newLabel;
                    newLabel.arrivalTime = parrot::karriToULTRATime(taxiResult.arrivalTime);
                    newLabel.cost = taxiResult.bestCost + Calc::penaltyForNewTransfer();
                    newLabel.parentStop = sourceStop;
                    newLabel.parentIndex = i;
                    // Store index of the taxi result among pareto-optimal results at the station to be able to reconstruct the access taxi trip later
                    newLabel.parentDepartureTime = sourceDepartureTime;
                    newLabel.transferId = noEdge;
                    newLabel.usesTaxi = true;
                    arrivalByTransfer(StopId(stationVertex), newLabel, stats);
                }
            }
        }


        inline void relaxIntermediateTransfers(karri::stats::PtPerformanceStats &stats) noexcept {
            using Calc = karri::CostCalculator;
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
                    KASSERT(data.isStop(toStop), "Graph contains edges to non-stop vertices!");
                    const int travelTime = data.transferGraph.get(TravelTime, edge);
                    for (size_t i = 0; i < bag.size(); i++) {
                        Label newLabel;
                        newLabel.arrivalTime = bag[i].arrivalTime + travelTime;
                        newLabel.cost = bag[i].cost + Calc::calcCostForAddedTransferTime(
                                            parrot::ultraToKarriTime(travelTime)) + Calc::penaltyForNewTransfer();
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
                        newLabel.cost = bag[i].cost + Calc::calcCostForAddedTransferTime(travelTime) + Calc::penaltyForNewTransfer();
                        newLabel.parentStop = stop;
                        newLabel.parentIndex = i;
                        newLabel.parentDepartureTime = bag[i].arrivalTime;
                        newLabel.transferId = noEdge;
                        arrivalByTransfer(targetStop, newLabel, stats);
                    }
                }
            }
        }

        void relaxFinalTransfersByTaxi(const std::vector<int> &distFromStations,
                                       karri::stats::PtPerformanceStats &stats) noexcept {
            using Calc = karri::CostCalculator;

            for (const StopId stop: stopsUpdatedByRoute) {
                const BagType &bag = previousRound()[stop];
                // Extension for second taxi leg
                const int stationId = stop.value();
                const auto &station = stations[stationId];
                KASSERT(distFromStations.size() == stations.size(),
                       "Size of distFromStations (" << distFromStations.size() <<
                       ") does not match number of stations (" << stations.size() << ")!");
                // ensure that no second taxi leg is used if station edge id == destination edge id
                if (stationId != INVALID_ID && station.vehEdgeId != destinationVehEdge && distFromStations[stationId] <
                    INFTY) {
                    const int taxiTravelDistance = parrot::karriToULTRATime(distFromStations[stationId]);
                    for (size_t i = 0; i < bag.size(); i++) {
                        Label newLabel;
                        newLabel.arrivalTime = bag[i].arrivalTime + taxiTravelDistance;
                        newLabel.cost = bag[i].cost + Calc::calcHeuristicCostForFinalTransferTimeByRP(
                                            parrot::ultraToKarriTime(taxiTravelDistance)) +
                                        Calc::penaltyForNewTransfer();
                        newLabel.parentStop = stop;
                        newLabel.parentIndex = i;
                        newLabel.parentDepartureTime = bag[i].arrivalTime;
                        newLabel.transferId = noEdge;
                        newLabel.usesTaxi = true;
                        arrivalByTransfer(targetStop, newLabel, stats);
                    }
                }
            }
        }

        inline Round &currentRound() noexcept {
            KASSERT(!rounds.empty(), "Cannot return current round, because no round exists!");
            return rounds.back();
        }

        inline Round &previousRound() noexcept {
            KASSERT(rounds.size() >= 2, "Cannot return previous round, because less than two rounds exist!");
            return rounds[rounds.size() - 2];
        }

        inline void startNewRound() noexcept {
            rounds.emplace_back(data.numberOfStops() + 1);
        }

        inline void arrival(const StopId stop, const Label &label, IndexedSet<false, StopId> &updatedStops,
                            Metric metric, int64_t &numImproved) noexcept {
            KASSERT(data.isStop(stop) || stop == targetStop, "Stop " << stop << " is out of range!");
            // if (bestLabels[targetStop].dominates(label)) return;

            // Target pruning based only on cost since we're looking for the label with best cost at the target
            if (label.cost >= bestCostAtTarget) return;

            if (!bestLabels[stop].merge(BestLabel(label))) return;
            ++numImproved;
            profiler.countMetric(metric);
            currentRound()[stop].mergeUndominated(label);
            KASSERT(bestLabels[stop].dominates(currentRound()[stop]), "Best bag does not dominate current bag!");
            if (data.isStop(stop)) updatedStops.insert(stop);

            if (stop == targetStop) {
                KASSERT(label.cost < bestCostAtTarget,
                       "New label at target is not better than current best cost at target!");
                bestCostAtTarget = label.cost;
            }
        }

        inline void arrivalByTransfer(const StopId stop, const Label &label,
                                      karri::stats::PtPerformanceStats &stats) noexcept {
            arrival(stop, label, stopsUpdatedByTransfer, METRIC_STOPS_BY_TRANSFER, stats.numStopsImprovedByTransfer);
        }

        inline void arrivalByRoute(const StopId stop, const Label &label,
                                   karri::stats::PtPerformanceStats &stats) noexcept {
            arrival(stop, label, stopsUpdatedByRoute, METRIC_STOPS_BY_TRIP, stats.numStopsImprovedByTrip);
        }

        inline Journey getJourney(size_t round, StopId stop, size_t &index) const noexcept {
            Journey journey;
            do {
                KASSERT(round != size_t(-1), "Backtracking parent pointers did not pass through the source stop! Partial journey = " << Vector::reverse(journey));
                const Label &label = rounds[round][stop][index];
                journey.emplace_back(label.parentStop, stop, label.parentDepartureTime, label.arrivalTime,
                                     label.usesRoute, label.routeId, label.usesTaxi);
                stop = label.parentStop;
                index = label.parentIndex;
                round--;
            } while (journey.back().from != sourceStop);
            return Vector::reverse(journey);
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

        int originPsgEdge; // Origin edge ID in passenger graph
        int originVehEdge; // Origin edge ID in vehicle graph
        int destinationPsgEdge; // Destination edge ID in passenger graph
        int destinationVehEdge; // Destination edge ID in vehicle graph
        StopId sourceStop; // Stop if origin edge maps to a station, originSentinelStop otherwise
        StopId targetStop;
        int sourceDepartureTime;
        int bestCostAtTarget;


        Profiler profiler;
    };
}
