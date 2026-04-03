/// ******************************************************************************
/// MIT License
///
/// Copyright (c) 2023 Moritz Laupichler <moritz.laupichler@kit.edu>
///
/// Permission is hereby granted, free of charge, to any person obtaining a copy
/// of this software and associated documentation files (the "Software"), to deal
/// in the Software without restriction, including without limitation the rights
/// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
/// copies of the Software, and to permit persons to whom the Software is
/// furnished to do so, subject to the following conditions:
///
/// The above copyright notice and this permission notice shall be included in all
/// copies or substantial portions of the Software.
///
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
/// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
/// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
/// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
/// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
/// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
/// SOFTWARE.
/// ******************************************************************************


#pragma once

#include <KARRI/Tools/Timer.h>
#include <KARRI/Algorithms/KaRRi/RequestState/RequestState.h>
#include <KARRI/Algorithms/KaRRi/LastStopSearches/LastStopBCHQuery.h>
#include <KARRI/Algorithms/KaRRi/CostCalculator.h>

namespace karri {
    template<typename InputGraphT, typename CHEnvT, typename LastStopBucketsEnvT, typename LastStopsAtVerticesT,
        typename
        StationDistancesT, typename
        LabelSetT>
    class PALSToStations {
        static constexpr int K = LabelSetT::K;
        using LabelMask = typename LabelSetT::LabelMask;
        using DistanceLabel = typename LabelSetT::DistanceLabel;

        struct PickupAfterLastStopPruner {
            static constexpr bool INCLUDE_IDLE_VEHICLES = true;

            PickupAfterLastStopPruner(PALSToStations &strat, CostCalculator calc)
                : strat(strat), calc(calc) {
            }

            // Returns whether a given distance from a vehicle's last stop to the pickup cannot lead to a better
            // assignment than the best known. Uses vehicle-independent lower bounds s.t. if this returns true, then
            // any vehicle with a last stop distance greater than the given one can also never lead to a better
            // assignment than the best known.
            LabelMask doesDistanceNotAdmitBestAsgn(const DistanceLabel &distancesToPickups,
                                                   const bool considerPickupWalkingDists = false) const {
                assert(strat.minDistanceToAnyStation < INFTY);

                if (strat.upperBoundCost >= INFTY) {
                    // If current best is INFTY, only indices i with distancesToPickups[i] >= INFTY or
                    // minDirectDistances[i] >= INFTY are worse than the current best.
                    return ~(distancesToPickups < INFTY);
                }

                const auto &walkingDists = considerPickupWalkingDists ? strat.currentPickupWalkingDists : 0;

                const DistanceLabel directDist = strat.minDistanceToAnyStation;
                KASSERT(strat.curReqState->now() == strat.curReqState->earliestDeparture());
                const auto detourTillDepAtPickup = distancesToPickups + DistanceLabel(
                                                       InputConfig::getInstance().stopTime);
                auto tripTimeTillDepAtPickup = detourTillDepAtPickup;
                tripTimeTillDepAtPickup.max(walkingDists);
                DistanceLabel costLowerBound = calc.template calcLowerBoundCostForKPairedAssignmentsAfterLastStop<
                    LabelSetT>(
                    detourTillDepAtPickup, tripTimeTillDepAtPickup, directDist, walkingDists, *strat.curReqState);

                costLowerBound.setIf(DistanceLabel(INFTY), ~(distancesToPickups < INFTY));

                return strat.upperBoundCost < costLowerBound;
            }

            // Returns whether a given arrival time and minimum distance from a vehicle's last stop to the pickup cannot
            // lead to a better assignment than the best known. Uses vehicle-independent lower bounds s.t. if this
            // returns true, then any vehicle with an arrival time later than the given one can also never lead to a
            // better assignment than the best known.
            // minDistancesToPickups needs to be a vehicle-independent lower bound on the last stop distance.
            LabelMask doesArrTimeNotAdmitBestAsgn(const DistanceLabel &arrTimesAtPickups,
                                                  const DistanceLabel &minDistancesToPickups) const {
                assert(strat.minDistanceToAnyStation < INFTY);

                if (strat.upperBoundCost >= INFTY) {
                    // If current best is INFTY, only indices i with arrTimesAtPickups[i] >= INFTY or
                    // minDistancesToPickups[i] >= INFTY are worse than the current best.
                    return ~((arrTimesAtPickups < INFTY) & (minDistancesToPickups < INFTY));
                }

                const DistanceLabel directDist = strat.minDistanceToAnyStation;
                const auto detourTillDepAtPickup = minDistancesToPickups + DistanceLabel(
                                                       InputConfig::getInstance().stopTime);
                auto depTimeAtPickup = arrTimesAtPickups + DistanceLabel(InputConfig::getInstance().stopTime);
                const auto reqTime = DistanceLabel(strat.curReqState->originalRequest.requestTime);
                KASSERT(strat.curReqState->now() == strat.curReqState->earliestDeparture());
                const auto tripTimeTillDepAtPickup = depTimeAtPickup - reqTime;
                DistanceLabel costLowerBound = calc.template calcLowerBoundCostForKPairedAssignmentsAfterLastStop<
                    LabelSetT>(
                    detourTillDepAtPickup, tripTimeTillDepAtPickup, directDist, strat.currentPickupWalkingDists,
                    *strat.curReqState);

                costLowerBound.setIf(DistanceLabel(INFTY),
                                     ~((arrTimesAtPickups < INFTY) & (minDistancesToPickups < INFTY)));
                return strat.upperBoundCost < costLowerBound;
            }

            LabelMask isWorseThanBestKnownVehicleDependent(const int vehId,
                                                           const DistanceLabel &distancesToPickups) {
                if (strat.upperBoundCost >= INFTY) {
                    // If current best is INFTY, only indices i with distancesToDropoffs[i] >= INFTY are worse than
                    // the current best.
                    return ~(distancesToPickups < INFTY);
                }

                const DistanceLabel directDist = strat.minDistanceToAnyStation;
                const auto detourTillDepAtPickup = distancesToPickups + InputConfig::getInstance().stopTime;
                const auto &stopIdx = strat.routeState.numStopsOf(vehId) - 1;
                const int vehDepTimeAtLastStop = time_utils::getVehDepTimeAtStopForRequest(vehId, stopIdx,
                    strat.curReqState->now(),
                    strat.routeState);
                auto depTimeAtPickups = vehDepTimeAtLastStop + distancesToPickups + InputConfig::getInstance().stopTime;
                depTimeAtPickups.max(strat.curPassengerArrTimesAtPickups);
                const auto tripTimeTillDepAtPickup = depTimeAtPickups - strat.curReqState->originalRequest.requestTime;
                DistanceLabel costLowerBound = calc.template calcLowerBoundCostForKPairedAssignmentsAfterLastStop<
                    LabelSetT>(
                    detourTillDepAtPickup, tripTimeTillDepAtPickup, directDist, strat.currentPickupWalkingDists,
                    *strat.curReqState);

                costLowerBound.setIf(INFTY, ~(distancesToPickups < INFTY));
                return strat.upperBoundCost < costLowerBound;
            }

            void updateUpperBoundCost(const int vehId, const DistanceLabel &distancesToPickups) {
                KASSERT(allSet(distancesToPickups >= 0));
                const DistanceLabel cost = calc.template calcUpperBoundCostForKPairedAssignmentsAfterLastStop<
                    LabelSetT>(
                    strat.fleet[vehId], distancesToPickups, strat.curPassengerArrTimesAtPickups,
                    strat.curMinDistancesToAnyStation,
                    strat.currentPickupWalkingDists, *strat.curReqState);

                strat.upperBoundCost = std::min(strat.upperBoundCost, cost.horizontalMin());
            }

            bool isVehicleEligible(const int &) const {
                // All vehicles can perform PALS assignments.
                return true;
            }

        private:
            PALSToStations &strat;
            CostCalculator calc;
        };

        using PickupBCHQuery = LastStopBCHQuery<CHEnvT, LastStopBucketsEnvT, PickupAfterLastStopPruner, LabelSetT>;

    public:
        PALSToStations(const InputGraphT &inputGraph,
                       const Fleet &fleet,
                       const CHEnvT &chEnv,
                       const LastStopBucketsEnvT &lastStopBucketsEnv,
                       const LastStopsAtVerticesT &lastStopsAtVertices,
                       const RouteState &routeState)
            : inputGraph(inputGraph),
              fleet(fleet),
              calculator(routeState),
              routeState(routeState),
              lastStopsAtVertices(lastStopsAtVertices),
              externalUpperBoundCost(INFTY),
              distances(fleet.size()),
              search(lastStopBucketsEnv, distances, chEnv, routeState, vehiclesSeenForPickups,
                     PickupAfterLastStopPruner(*this, CostCalculator(routeState))),
              vehiclesSeenForPickups(fleet.size()) {
        }

        void tryPickupAfterLastStop(const RequestState &requestState, const PDLocs &pdLocs,
                                    StationDistancesT &stationDistances,
                                    LightweightSubset &stationsSeen,
                                    const PTStations &stations,
                                    stats::PalsAssignmentsPerformanceStats &stats,
                                    FirstTaxiLegResult &firstTaxiLegResult) {
            minDistanceToAnyStation = stationDistances.getMinDistanceToAnyStation();
            enumerateAssignmentsWherePickupCoincidesWithLastStop(requestState, pdLocs, stationDistances, stationsSeen,
                                                                 stations, stats, firstTaxiLegResult);

            runBchSearches(requestState, pdLocs, stationDistances, stats, firstTaxiLegResult);
            enumerateAssignments(requestState, pdLocs, stationDistances, stationsSeen, stations, stats,
                                 firstTaxiLegResult);
        }

        void setExternalCostUpperBound(const int bestCost, const int worstCostForAllStations) {
            externalUpperBoundCost = bestCost;
            upperBoundCost = std::min(worstCostForAllStations, externalUpperBoundCost);
        }

    private:
        // Run BCH searches that find distances from last stops to pickups
        void runBchSearches(const RequestState &requestState, const PDLocs &pdLocs,
                            StationDistancesT &stationDistances,
                            stats::PalsAssignmentsPerformanceStats &stats,
                            FirstTaxiLegResult &firstTaxiLegResult) {
            KaRRiTimer timer;

            initPickupSearches(requestState, pdLocs);
            for (int i = 0; i < pdLocs.numPickups(); i += K)
                runSearchesForPickupBatch(i, requestState, stationDistances, pdLocs);

            const auto searchTime = timer.elapsed<std::chrono::nanoseconds>();
            stats.searchTime += searchTime;
            stats.numEdgeRelaxationsInSearchGraph += totalNumEdgeRelaxations;
            stats.numVerticesOrLabelsSettled += totalNumVerticesSettled;
            stats.numEntriesOrLastStopsScanned += totalNumEntriesScanned;
            stats.numCandidateVehicles += vehiclesSeenForPickups.size();
        }

        // Enumerate assignments with pickup after last stop
        void enumerateAssignments(const RequestState &requestState, const PDLocs &pdLocs,
                                  StationDistancesT &stationDistances,
                                  LightweightSubset &stationsSeen,
                                  const PTStations &stations,
                                  stats::PalsAssignmentsPerformanceStats &stats,
                                  FirstTaxiLegResult &firstTaxiLegResult) {
            using namespace time_utils;


            int numAssignmentsTried = 0;
            KaRRiTimer timer;

            Assignment asgn;
            // 1 vehicle with best cost until the pickup
            for (const auto &vehId: vehiclesSeenForPickups) {
                const int numStops = routeState.numStopsOf(vehId);
                if (numStops == 0)
                    continue;

                asgn.vehicle = &fleet[vehId];
                asgn.pickupStopIdx = numStops - 1;
                asgn.dropoffStopIdx = numStops - 1;

                for (auto &p: pdLocs.pickups) {
                    asgn.pickup = p;
                    asgn.distToPickup = getDistanceToPickup(vehId, asgn.pickup.id);
                    if (asgn.distToPickup >= INFTY)
                        continue;

                    // Compute cost lower bound for this pickup specifically
                    const auto depTimeAtThisPickup = getActualDepTimeAtPickup(asgn, requestState, routeState);
                    const auto vehTimeTillDepAtThisPickup = depTimeAtThisPickup -
                                                            getVehDepTimeAtStopForRequest(vehId, numStops - 1,
                                                                requestState.now(), routeState);
                    const auto psgTimeTillDepAtThisPickup =
                            depTimeAtThisPickup - requestState.earliestDeparture();
                    const auto minDirectDistForThisPickup = stationDistances.getMinDistanceForPDLoc(asgn.pickup.id);
                    const auto minCost = calculator.calcCostForPairedAssignmentAfterLastStop(vehTimeTillDepAtThisPickup,
                        psgTimeTillDepAtThisPickup,
                        minDirectDistForThisPickup,
                        asgn.pickup.walkingDist,
                        0,
                        requestState);
                    if (minCost > upperBoundCost)
                        continue;

                    // Consider only stations with feasible distances from StationBCH
                    for (const auto &stationId: stationsSeen) {
                        const auto &station = stations[stationId];
                        KASSERT(station.stationId == stationId);
                        asgn.dropoff = {
                            station.stationId, // PDLoc ID
                            station.vehEdgeId, // Location in road network
                            station.psgEdgeId, // Location in passenger road network
                            0, // Walking time from this dropoff to destination
                            0, // Vehicle driving time from this dropoff to the destination
                            0 // Vehicle driving time from destination to this dropoff
                        };

                        // Try inserting pair with pickup after last stop:
                        ++numAssignmentsTried;
                        asgn.distToDropoff = stationDistances.getDistance(station.stationId, asgn.pickup.id);
                        // requestState.tryAssignmentWithKnownCost(asgn, calculator.calc(asgn, requestState));
                        firstTaxiLegResult.tryAssignmentWithKnownCostForStation(
                            station.stationId, asgn, calculator.calc(asgn, requestState), InsertionType::PALS);
                    }
                }
            }

            const int64_t tryAssignmentsTime = timer.elapsed<std::chrono::nanoseconds>();
            stats.numAssignmentsTried += numAssignmentsTried;
            stats.tryAssignmentsTime += tryAssignmentsTime;
        }


        inline int getDistanceToPickup(const int vehId, const unsigned int pickupId) {
            return distances.getDistance(vehId, pickupId);
        }

        void initPickupSearches(const RequestState &requestState, const PDLocs &pdLocs) {
            totalNumEdgeRelaxations = 0;
            totalNumVerticesSettled = 0;
            totalNumEntriesScanned = 0;

            // Set request state to allow callbacks from within Dijkstra searches.
            curReqState = &requestState;

            vehiclesSeenForPickups.clear();
            const int numPickupBatches = pdLocs.numPickups() / K + (pdLocs.numPickups() % K != 0);
            distances.init(numPickupBatches);
        }

        void runSearchesForPickupBatch(const int firstPickupId, const RequestState &requestState,
                                       StationDistancesT &stationDistances, const PDLocs &pdLocs) {
            KASSERT(firstPickupId % K == 0 && firstPickupId < pdLocs.numPickups());

            std::array<int, K> pickupTails;
            std::array<int, K> travelTimes;
            for (int i = 0; i < K; ++i) {
                const auto &pickup =
                        firstPickupId + i < pdLocs.numPickups()
                            ? pdLocs.pickups[firstPickupId + i]
                            : pdLocs.pickups[firstPickupId];
                pickupTails[i] = inputGraph.edgeTail(pickup.loc);
                travelTimes[i] = inputGraph.travelTime(pickup.loc);
                currentPickupWalkingDists[i] = pickup.walkingDist;
                curPassengerArrTimesAtPickups[i] = requestState.getPassengerArrAtPickup(pickup);
                curMinDistancesToAnyStation[i] = stationDistances.getMinDistanceForPDLoc(pickup.id);
            }

            distances.setCurBatchIdx(firstPickupId / K);
            search.run(pickupTails, travelTimes);

            totalNumEdgeRelaxations += search.getNumEdgeRelaxations();
            totalNumVerticesSettled += search.getNumVerticesSettled();
            totalNumEntriesScanned += search.getNumEntriesScanned();
        }

        // Simple case for pickups that coincide with last stops of vehicles.
        void enumerateAssignmentsWherePickupCoincidesWithLastStop(const RequestState &requestState,
                                                                  const PDLocs &pdLocs,
                                                                  StationDistancesT &stationDistances,
                                                                  const LightweightSubset &stationsSeen,
                                                                  const PTStations &stations,
                                                                  stats::PalsAssignmentsPerformanceStats &stats,
                                                                  FirstTaxiLegResult &firstTaxiLegResult) {
            int numInsertionsForCoinciding = 0;
            int numCandidateVehiclesForCoinciding = 0;
            KaRRiTimer timer;

            Assignment asgn;
            asgn.distToPickup = 0;
            for (const auto &p: pdLocs.pickups) {
                asgn.pickup = p;

                const int head = inputGraph.edgeHead(asgn.pickup.loc);
                for (const auto &vehId: lastStopsAtVertices.vehiclesWithLastStopAt(head)) {
                    ++numCandidateVehiclesForCoinciding;
                    const auto numStops = routeState.numStopsOf(vehId);
                    if (routeState.stopLocationsFor(vehId)[numStops - 1] != asgn.pickup.loc)
                        continue;

                    // Calculate lower bound on insertion cost with this pickup and vehicle
                    const auto lowerBoundCost = calculator.calcCostLowerBoundForPickupAfterLastStop(
                        fleet[vehId], asgn.pickup, 0, minDistanceToAnyStation, requestState);
                    if (lowerBoundCost > upperBoundCost)
                        continue;

                    // If necessary, check paired insertion with each station
                    asgn.vehicle = &fleet[vehId];
                    asgn.pickupStopIdx = numStops - 1;
                    asgn.dropoffStopIdx = numStops - 1;

                    // Consider only stations with feasible distances from StationBCH
                    for (const auto &stationId: stationsSeen) {
                        const auto station = stations[stationId];
                        KASSERT(station.stationId == stationId);
                        asgn.dropoff = {
                            station.stationId, // PDLoc ID
                            station.vehEdgeId, // Location in road network
                            station.psgEdgeId, // Location in passenger road network
                            0, // Walking time from this dropoff to destination
                            0, // Vehicle driving time from this dropoff to the destination
                            0 // Vehicle driving time from destination to this dropoff
                        };

                        // Try inserting pair with pickup after last stop:
                        ++numInsertionsForCoinciding;
                        asgn.distToDropoff = stationDistances.getDistance(station.stationId, asgn.pickup.id);
                        // requestState.tryAssignmentWithKnownCost(asgn, calculator.calc(asgn, requestState));
                        firstTaxiLegResult.tryAssignmentWithKnownCostForStation(
                            station.stationId, asgn, calculator.calc(asgn, requestState), InsertionType::PALS);
                    }
                }
            }

            const auto time = timer.elapsed<std::chrono::nanoseconds>();
            stats.pickupAtLastStop_tryAssignmentsTime += time;
            stats.pickupAtLastStop_numCandidateVehicles += numCandidateVehiclesForCoinciding;
            stats.pickupAtLastStop_numAssignmentsTried += numInsertionsForCoinciding;
        }

        const InputGraphT &inputGraph;
        const Fleet &fleet;
        CostCalculator calculator;
        const RouteState &routeState;
        const LastStopsAtVerticesT &lastStopsAtVertices;

        int externalUpperBoundCost;
        int upperBoundCost;
        RequestState const *curReqState;

        TentativeLastStopDistances<LabelSetT> distances;
        PickupBCHQuery search;

        // Vehicles seen by any last stop pickup search
        LightweightSubset vehiclesSeenForPickups;
        DistanceLabel currentPickupWalkingDists;
        DistanceLabel curPassengerArrTimesAtPickups;
        DistanceLabel curMinDistancesToAnyStation;

        int minDistanceToAnyStation;

        int totalNumEdgeRelaxations;
        int totalNumVerticesSettled;
        int totalNumEntriesScanned;
    };
}
