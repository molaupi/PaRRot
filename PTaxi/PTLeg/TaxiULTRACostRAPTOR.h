#pragma once

#include <vector>

#include <Common/TimeConversion.h>
#include <ULTRA/DataStructures/Container/IndexedSet.h>
#include <ULTRA/DataStructures/Container/Map.h>
#include <ULTRA/DataStructures/RAPTOR/Data.h>
#include <ULTRA/DataStructures/RAPTOR/Entities/ArrivalLabel.h>
#include <ULTRA/DataStructures/RAPTOR/Entities/Bags.h>

namespace parrot {
    // TODO: result struct
}

namespace RAPTOR {
    // ULTRA-RAPTOR specialization that uses initial labels based on access ride-pooling trips and finds the
    // journey with smallest cost according to a cost function.
    // Works like multi-criteria ULTRA-RAPTOR to deal with the fact that a ride-pooling trip with a later arrival time
    // might have a smaller cost than a ride-pooling trip with an earlier arrival time.
    // Uses target pruning based on only the cost though.
    template<typename InitialTransfersT, typename PROFILER = NoProfiler>
    class TaxiULTRACostRAPTOR {
    public:
        using Profiler = PROFILER;
        using Type = TaxiULTRACostRAPTOR<InitialTransfersT, Profiler>;
        using SourceType = Vertex;

    private:
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
        TaxiULTRACostRAPTOR(const Data &data,
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
            Assert(data.hasImplicitBufferTimes(), "Departure buffer times have to be implicit!");
            profiler.registerExtraRounds({EXTRA_ROUND_CLEAR, EXTRA_ROUND_INITIALIZATION});
            profiler.registerPhases({PHASE_INITIALIZATION, PHASE_COLLECT, PHASE_SCAN, PHASE_TRANSFERS});
            profiler.registerMetrics({
                METRIC_ROUTES, METRIC_ROUTE_SEGMENTS, METRIC_EDGES, METRIC_STOPS_BY_TRIP, METRIC_STOPS_BY_TRANSFER
            });
            profiler.initialize();
            initializeEdgeToStationMappings();
        }

        inline void run(const int origPsgEdge, const int origVehEdge,
                        const int destPsgEdge, const int destVehEdge,
                        const int departureTime,
                        const karri::FirstTaxiLegResult &firstTaxiLeg, const std::vector<int> &distFromStations,
                        karri::stats::PtPerformanceStats &stats, const size_t maxRounds = INFTY) noexcept {
            KaRRiTimer timer;
            profiler.start();
            profiler.startExtraRound(EXTRA_ROUND_CLEAR);
            clear();
            profiler.doneRound();

            profiler.startExtraRound(EXTRA_ROUND_INITIALIZATION);
            profiler.startPhase();
            initialize(origPsgEdge, origVehEdge, destPsgEdge, destVehEdge, departureTime, stats);
            profiler.donePhase(PHASE_INITIALIZATION);

            stats.roundInitializationTime += timer.elapsed<std::chrono::nanoseconds>();
            timer.restart();

            profiler.startPhase();
            relaxInitialTransfers(firstTaxiLeg, stats);
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
                relaxIntermediateTransfers(distFromStations, stats);
                profiler.donePhase(PHASE_TRANSFERS);
                stats.relaxIntermediateTransfersTime += timer.elapsed<std::chrono::nanoseconds>();
                profiler.doneRound();
            }
            profiler.done();
        }

        bool hasFoundJourney() const noexcept {
            return bestCostAtTarget < INFTY;
        }

        std::pair<Journey, int> getJourneyWithBestCost() const noexcept {
            // Assert(bestLabels[targetStop].size() == 1, "Target has more than one best label.");
            int smallestCost = INFTY;
            int bestRound = rounds.size();
            int bestParentIndex = -1;
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
                    return {{}, INFTY};
            // Assert(smallestCost < INFTY, "No label found for target stop!");
            const auto journey = getJourney(bestRound, targetStop, bestParentIndex);
            Assert(!journey.empty(), "Backtracking the journey for the target stop did not yield a valid journey!");

            // TODO: This is a quick fix to filter out journeys that use no taxi at all. We should ideally not generate
            //  such journeys at all. Instead, we should run the algorithm twice, once allowing only taxi initial
            //  transfers (and both taxi and walking final transfers), and once allowing only walking initial transfers
            //  and only taxi final transfers.
            if (!journey.front().usesTaxi && !journey.back().usesTaxi)
                return {{}, INFTY};

            return {journey, smallestCost};
        }

        // inline std::vector<Journey> getJourneys() const noexcept {
        //     return getJourneys(targetStop);
        // }
        //
        // inline std::vector<Journey> getJourneys(const Vertex vertex) const noexcept {
        //     const StopId target = (vertex == targetVertex) ? (targetStop) : (StopId(vertex));
        //     std::vector<Journey> journeys;
        //     for (size_t round = 0; round < rounds.size(); round += 2) {
        //         const size_t trueRound = std::min(round + 1, rounds.size() - 1);
        //         for (size_t i = 0; i < rounds[trueRound][target].size(); i++) {
        //             getJourney(journeys, trueRound, target, i);
        //         }
        //     }
        //     return journeys;
        // }
        //
        // inline std::vector<WalkingParetoLabel> getResults() const noexcept {
        //     return getResults(targetStop);
        // }
        //
        // inline std::vector<WalkingParetoLabel> getResults(const Vertex vertex) const noexcept {
        //     const StopId target = (vertex == targetVertex) ? (targetStop) : (StopId(vertex));
        //     std::vector<WalkingParetoLabel> result;
        //     for (size_t round = 0; round < rounds.size(); round += 2) {
        //         const size_t trueRound = std::min(round + 1, rounds.size() - 1);
        //         for (const Label& label : rounds[trueRound][target].labels) {
        //             result.emplace_back(label, round / 2);
        //         }
        //     }
        //     return result;
        // }

        template<bool RESET_CAPACITIES = false>
        inline void clear() noexcept {
            stopsUpdatedByRoute.clear();
            stopsUpdatedByTransfer.clear();
            routesServingUpdatedStops.clear();
            originVehEdge = noEdge;
            originPsgEdge = noEdge;
            destinationVehEdge = noEdge;
            destinationPsgEdge = noEdge;
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

        inline void initialize(const int originPsgEdge_, const int originVehEdge_,
                               const int destPsgEdge_, const int destVehEdge_, const int departureTime,
                               karri::stats::PtPerformanceStats &stats) noexcept {
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
                    Label initialLabel(sourceDepartureTime, noStop);
                    // Sentinel noStop since origin (which is an edge) is the parent
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
                        newLabel.costWithoutTripCost = label.cost - karri::CostCalculator::calcTripCost(parrot::ultraToKarriTime(
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
                        newLabel.cost = label.costWithoutTripCost + karri::CostCalculator::calcTripCost(parrot::ultraToKarriTime(
                                            newLabel.arrivalTime - sourceDepartureTime));
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

        inline void relaxInitialTransfers(const karri::FirstTaxiLegResult & firstTaxiLeg,
                                          karri::stats::PtPerformanceStats &stats) noexcept {
            // If the origin edge maps to a station, it is considered reached by initial transfer, too
            if (originPsgEdge >= 0 && originPsgEdge < psgEdgeToStation.size()) {
                int stationId = psgEdgeToStation[originPsgEdge];
                if (stationId != INVALID_ID && data.isStop(StopId(stationId))) {
                    StopId sourceStop = StopId(stationId);
                    stopsUpdatedByTransfer.insert(sourceStop);
                    currentRound()[stationId].resize(1);
                    currentRound()[stationId][0] = Label(previousRound()[stationId][0], sourceStop, 0);
                }
            }
            initialTransfers.template run<true>(originPsgEdge, destinationPsgEdge);
            for (const Vertex stop: initialTransfers.getForwardPOIs()) {
                if (stop == targetStop) continue;
                Assert(data.isStop(stop), "Reached POI " << stop << " is not a stop!");
                Assert(initialTransfers.getForwardDistance(stop) != INFTY, "Vertex " << stop << " was not reached!");
                Label newLabel;
                newLabel.arrivalTime = sourceDepartureTime + parrot::karriToULTRATime(
                                           initialTransfers.getForwardDistance(stop));
                newLabel.cost = karri::CostCalculator::calcCostForAddedTransferTime(initialTransfers.getForwardDistance(stop), false);
                newLabel.parentStop = noStop; // Sentinel noStop since origin (which is an edge) is the parent
                newLabel.parentIndex = 0;
                newLabel.parentDepartureTime = sourceDepartureTime;
                newLabel.transferId = noEdge;
                arrivalByTransfer(StopId(stop), newLabel, stats);
            }

            // Extension for first taxi leg
            if (firstTaxiLeg.getWorstCostForAllStations() == INFTY)
                return; // no stations reached by taxi
            for (const auto &station: stations) {
                const int stationId = station.stationId;
                const Vertex stationVertex = Vertex(stationId);
                const auto taxiResult = firstTaxiLeg.getResultForStation(stationId);

                // Check if this is the destination station
                bool isDestStation = (destinationPsgEdge >= 0 && destinationPsgEdge < psgEdgeToStation.size() &&
                                      psgEdgeToStation[destinationPsgEdge] == stationId);
                // Check if this is the origin station
                bool isOriginStation = (originPsgEdge >= 0 && originPsgEdge < psgEdgeToStation.size() &&
                                        psgEdgeToStation[originPsgEdge] == stationId);

                if (isOriginStation || isDestStation || taxiResult.bestCost == INFTY)
                    continue;
                Assert(data.isStop(stationVertex), "Taxi station " << stationVertex << " is not a stop!");
                Label newLabel;
                newLabel.arrivalTime = parrot::karriToULTRATime(taxiResult.arrivalTime);
                newLabel.cost = taxiResult.bestCost;
                newLabel.parentStop = noStop; // Sentinel: this stop was reached from origin via taxi
                newLabel.parentIndex = 0;
                newLabel.parentDepartureTime = sourceDepartureTime;
                newLabel.transferId = noEdge;
                newLabel.usesTaxi = true;
                arrivalByTransfer(StopId(stationVertex), newLabel, stats);
            }
        }

        inline void relaxIntermediateTransfers(const std::vector<int> &distFromStations,
                                               karri::stats::PtPerformanceStats &stats) noexcept {
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
                        newLabel.cost = bag[i].cost + karri::CostCalculator::calcCostForAddedTransferTime(
                                            parrot::ultraToKarriTime(travelTime), true);
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
                        newLabel.arrivalTime = bag[i].arrivalTime + travelTime;
                        newLabel.cost = bag[i].cost + karri::CostCalculator::calcCostForAddedTransferTime(
                                            parrot::ultraToKarriTime(travelTime), false);
                        newLabel.parentStop = stop;
                        newLabel.parentIndex = i;
                        newLabel.parentDepartureTime = bag[i].arrivalTime;
                        newLabel.transferId = noEdge;
                        arrivalByTransfer(targetStop, newLabel, stats);
                    }
                }

                // Extension for second taxi leg
                const int stationId = stop.value();
                const auto &station = stations[stationId];
                Assert(distFromStations.size() == stations.size(),
                       "Size of distFromStations (" << distFromStations.size() <<
                       ") does not match number of stations (" << stations.size() << ")!");
                // ensure that no second taxi leg is used if station edge id == destination edge id
                if (stationId != INVALID_ID && station.vehEdgeId != destinationVehEdge && distFromStations[stationId] <
                    INFTY) {
                    const int taxiTravelDistance = parrot::karriToULTRATime(distFromStations[stationId]);
                    for (size_t i = 0; i < bag.size(); i++) {
                        Label newLabel;
                        newLabel.arrivalTime = bag[i].arrivalTime + taxiTravelDistance;
                        newLabel.cost = bag[i].cost + karri::CostCalculator::calcHeuristicCostForFinalTransferTimeByRP(parrot::ultraToKarriTime(
                                            taxiTravelDistance));
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
            // if (bestLabels[targetStop].dominates(label)) return;

            // Target pruning based only on cost since we're looking for the label with best cost at the target
            if (label.cost >= bestCostAtTarget) return;

            if (!bestLabels[stop].merge(BestLabel(label))) return;
            ++numImproved;
            profiler.countMetric(metric);
            currentRound()[stop].mergeUndominated(label);
            Assert(bestLabels[stop].dominates(currentRound()[stop]), "Best bag does not dominate current bag!");
            if (data.isStop(stop)) updatedStops.insert(stop);

            if (stop == targetStop) {
                Assert(label.cost < bestCostAtTarget,
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

        inline Journey getJourney(size_t round, StopId stop, size_t index) const noexcept {
            Journey journey;
            do {
                Assert(round != size_t(-1), "Backtracking parent pointers did not pass through the source stop!");
                const Label &label = rounds[round][stop][index];
                journey.emplace_back(label.parentStop, stop, label.parentDepartureTime, label.arrivalTime,
                                     label.usesRoute, label.routeId, label.usesTaxi);
                stop = label.parentStop;
                index = label.parentIndex;
                round--;
                // Assert(
                // round != size_t(-1),
                // "Backtracking parent pointers did not pass through the source stop!");
                // const EarliestArrivalLabel& label = rounds[round][stop];
                //
                // // Check if this is an initial transfer from origin (parent == noVertex)
                // if (label.parent == noVertex) {
                //     // This leg starts from the origin - use noVertex as 'from' to signal origin
                //     journey.emplace_back(noVertex, stop, label.parentDepartureTime,
                //         label.arrivalTime, label.usesRoute, label.routeId, label.usesTaxi);
                //     break; // Reached origin, journey reconstruction complete
                // }
                //
                // // Regular intermediate leg: both parent and stop are valid stops
                // journey.emplace_back(label.parent, stop, label.parentDepartureTime,
                //     label.arrivalTime, label.usesRoute, label.routeId, label.usesTaxi);
                //
                // Assert(data.isStop(label.parent),
                //     "Backtracking parent pointers reached a non-stop vertex ("
                //         << label.parent << ")!");
                //
                // stop = StopId(label.parent);
                // if constexpr (SeparateRouteAndTransferEntries) {
                //     round--;
                // } else {
                //     if (label.usesRoute)
                //         round--;
                // }
            } while (journey.back().from != noStop);
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
        StopId targetStop;
        int sourceDepartureTime;
        int bestCostAtTarget;


        Profiler profiler;
    };
}
