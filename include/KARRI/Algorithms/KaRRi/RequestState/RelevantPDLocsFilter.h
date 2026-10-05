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

#include "RelevantPDLocs.h"
#include "../../../DataStructures/Containers/FastResetFlagArray.h"
#include "../../../DataStructures/Containers/LightweightSubset.h"

namespace karri {
    // Filters information about feasible distances found by elliptic BCH searches to pickups/dropoffs that are relevant
    // for certain stops by considering the leeway and the current best known assignment cost.
    template<typename InputGraphT, typename CHEnvT>
    class RelevantPDLocsFilter {
    public:
        RelevantPDLocsFilter(const Fleet &fleet, const InputGraphT &inputGraph, const CHEnvT &chEnv,
                             const RouteState &routeState)
            : fleet(fleet),
              inputGraph(inputGraph),
              ch(chEnv.getCH()),
              chQuery(chEnv.template getFullCHQuery<>()),
              calculator(routeState),
              routeState(routeState),
              vehiclesWithFeasibleDistances(fleet.size()) {
        }

        RelevantPDLocs
        getRelevantOrdinaryPickups(const FeasiblePDLocs &feasiblePickups, const RequestState &requestState,
                                   const PDLocs &pdLocs,
                                   stats::FilterRelevantPdLocsPerformanceStats &stats) {
            KaRRiTimer timer;
            int numStopsRelevant = 0;

            const auto rel = filterOrdinaryPareto<false>(feasiblePickups, numStopsRelevant, requestState,
                                                         pdLocs.pickups);

            const int64_t time = timer.elapsed<std::chrono::nanoseconds>();
            stats.filterRelevantPDLocsTime += time;
            stats.numRelevantStopsForPickups += numStopsRelevant;

            return rel;
        }

        RelevantPDLocs
        getRelevantOrdinaryDropoffs(const FeasiblePDLocs &feasibleDropoffs, const RequestState &requestState,
                                   const PDLocs &pdLocs,
                                   stats::FilterRelevantPdLocsPerformanceStats &stats) {
            KaRRiTimer timer;
            int numStopsRelevant = 0;

            const auto rel = filterOrdinaryPareto<true>(feasibleDropoffs, numStopsRelevant, requestState,
                                                         pdLocs.dropoffs);

            const int64_t time = timer.elapsed<std::chrono::nanoseconds>();
            stats.filterRelevantPDLocsTime += time;
            stats.numRelevantStopsForPickups += numStopsRelevant;

            return rel;
        }

        RelevantPDLocs
        getRelevantPickupsBeforeNextStop(const FeasiblePDLocs &feasiblePickups, const RequestState &requestState,
                                   const PDLocs &pdLocs,
                                   stats::FilterRelevantPdLocsPerformanceStats &stats) {
            KaRRiTimer timer;
            int numStopsRelevant = 0;

            const auto rel = filterPickupsBeforeNextStop(feasiblePickups, numStopsRelevant, requestState,
                                                         pdLocs.pickups);

            const int64_t time = timer.elapsed<std::chrono::nanoseconds>();
            stats.filterRelevantPDLocsTime += time;
            stats.numRelevantStopsForPickups += numStopsRelevant;

            return rel;
        }

    private:

        template<bool isDropoff>
        RelevantPDLocs filterOrdinaryPareto(const FeasiblePDLocs &feasible,
                                            int &numStopsRelevant,
                                            const RequestState &requestState,
                                            const std::vector<PDLoc> &pdLocs) {
            // For each stop s, prune the pickups and dropoffs deemed relevant for an ordinary assignment after s by
            // checking them against constraints and lower bounds.
            using namespace time_utils;

            numStopsRelevant = 0;

            RelevantPDLocs rel(fleet.size());

            vehiclesWithFeasibleDistances.clear();
            for (const auto &stopId: feasible.getStopIdsWithRelevantPDLocs()) {
                const auto vehId = routeState.vehicleIdOf(stopId);
                if (vehiclesWithFeasibleDistances.contains(vehId))
                    continue;
                const auto stopPos = routeState.stopPositionOf(stopId);
                if (stopPos == 0 || (!isDropoff && stopPos == routeState.numStopsOf(vehId) - 1))
                    continue;
                vehiclesWithFeasibleDistances.insert(vehId);
            }

            struct RelevantPDLocWithCriteria {
                RelevantPDLocs::RelevantPDLoc relPdLoc;
                // for pickups: fixed cost of pickup; for dropoffs: fixed cost (sum of trip cost and walking cost)
                int firstCriterionValue;
                // for pickups: arrival time at next stop; for dropoffs: detour for dropoff
                int secondCriterionValue;

                static bool dominates(const RelevantPDLocWithCriteria &a, const RelevantPDLocWithCriteria &b) {
                    return a.firstCriterionValue <= b.firstCriterionValue && a.secondCriterionValue <= b.
                           secondCriterionValue &&
                           (a.firstCriterionValue < b.firstCriterionValue || a.secondCriterionValue < b.
                            secondCriterionValue);
                }
            };

            ParetoBag<RelevantPDLocWithCriteria> relevantSpotsForStop;
            for (const auto &vehId: vehiclesWithFeasibleDistances) {
                const auto &veh = fleet[vehId];
                const auto &numStops = routeState.numStopsOf(vehId);
                const auto &stopIds = routeState.stopIdsFor(vehId);
                const auto &occupancies = routeState.occupanciesFor(vehId);
                KASSERT(numStops > 1);

                const int totalNumRelPdLocsBefore = static_cast<int>(rel.relevantSpots.size());


                // Track relevant PD locs for each stop in the relevant PD locs data structure.
                // Entries are ordered by vehicle and by stop.
                constexpr int beginStopIdx = 1;
                const int endStopIdx = isDropoff ? numStops : numStops - 1;
                for (int i = beginStopIdx; i < endStopIdx; ++i) {
                    if (!isDropoff && occupancies[i] + requestState.originalRequest.numRiders > veh.capacity)
                        continue;

                    const auto &stopId = stopIds[i];
                    if (!feasible.hasFeasiblePDLocs(stopId))
                        continue;

                    // Insert entries at this stop.

                    ++numStopsRelevant;
                    relevantSpotsForStop.clear();
                    for (const auto &[id, distToPDLoc, distFromPDLoc] : feasible.feasiblePdLocsFor(stopId)) {
                        int first, second;
                        if constexpr (isDropoff) {
                            std::tie(first, second) = getFixedCostAndDropoffDetourForDropoff(
                                veh, i, pdLocs[id], distToPDLoc, distFromPDLoc, requestState);
                        } else {
                            std::tie(first, second) = getFixedCostAndArrTimeAtNextStopForPickup(
                                veh, i, pdLocs[id], distToPDLoc, distFromPDLoc, requestState);
                        }
                        if (first >= INFTY || second >= INFTY)
                            continue;

                        const RelevantPDLocWithCriteria newSpot({i, id, distToPDLoc, distFromPDLoc}, first,
                                                                second);
                        relevantSpotsForStop.addCandidate(newSpot);
                    }
                    for (const auto &spotWithCriteria: relevantSpotsForStop) {
                        rel.relevantSpots.push_back(spotWithCriteria.relPdLoc);
                    }
                }

                // If vehicle has at least one stop with relevant PD loc, add the vehicle
                if (rel.relevantSpots.size() > totalNumRelPdLocsBefore) {
                    rel.vehiclesWithRelevantSpots.push_back(vehId);
                    rel.vehicleToPdLocs[vehId] = {totalNumRelPdLocsBefore, static_cast<int>(rel.relevantSpots.size())};
                }
            }

            KASSERT(std::all_of(rel.relevantSpots.begin(), rel.relevantSpots.end(),
                [&](const auto &h) {
                return h.distToPDLoc < INFTY && h.distFromPDLocToNextStop < INFTY;
                }));

            return rel;
        }

        std::pair<int, int> getFixedCostAndArrTimeAtNextStopForPickup(const Vehicle &veh, const int stopIndex,
                                                                      const PDLoc &pickup,
                                                                      const int distFromStopToPickup,
                                                                      const int distFromPickupToNextStop,
                                                                      const RequestState &requestState) const {
            using namespace time_utils;

            const int &vehId = veh.vehicleId;

            KASSERT(routeState.occupanciesFor(vehId)[stopIndex] + requestState.originalRequest.numRiders <=
                veh.capacity);
            if (distFromStopToPickup >= INFTY || distFromPickupToNextStop >= INFTY)
                return {INFTY, INFTY};

            KASSERT(distFromStopToPickup + distFromPickupToNextStop >=
                calcLengthOfLegStartingAt(stopIndex, vehId, routeState));

            KASSERT(stopIndex < routeState.numStopsOf(vehId) - 1);
            // If the pickup coincides with the location of the next stop, we will consider it at the next stop. Skip here
            if (pickup.loc == routeState.stopLocationsFor(vehId)[stopIndex + 1])
                return {INFTY, INFTY};

            const auto depTimeAtPickup = getActualDepTimeAtPickup(vehId, stopIndex, distFromStopToPickup, pickup,
                                                                  requestState, routeState);
            const int arrTimeAtNextStop = depTimeAtPickup + distFromPickupToNextStop;
            const int initialPickupDetour = arrTimeAtNextStop - routeState.schedArrTimesFor(vehId)[stopIndex + 1];
            KASSERT(initialPickupDetour >= 0);

            if (doesPickupDetourViolateHardConstraints(veh, requestState, stopIndex, initialPickupDetour, routeState))
                return {INFTY, INFTY};

            const int fixedCost = CostCalculator::CostFunction::calcWalkingCost(pickup.walkingDist) +
                                  CostCalculator::CostFunction::calcWaitViolationCost(depTimeAtPickup, requestState);

            return {fixedCost, arrTimeAtNextStop};
        }

        std::pair<int, int> getFixedCostAndDropoffDetourForDropoff(const Vehicle &veh, const int stopIndex,
                                                                   const PDLoc &dropoff,
                                                                   const int distFromStopToDropoff,
                                                                   const int distFromDropoffToNextStop,
                                                                   const RequestState &requestState) const {
            using namespace time_utils;

            const int &vehId = veh.vehicleId;

            // If this is the last stop in the route, we only consider this dropoff for ordinary assignments if it is at the
            // last stop. Similarly, if the vehicle is full after this stop, we can't perform the dropoff here unless the
            // dropoff coincides with the stop. A dropoff at an existing stop causes no detour, so it is always relevant.
            const auto &numStops = routeState.numStopsOf(vehId);
            const auto &occupancy = routeState.occupanciesFor(vehId)[stopIndex];
            const auto &stopLocations = routeState.stopLocationsFor(vehId);
            KASSERT(dropoff.loc != stopLocations[stopIndex] || distFromStopToDropoff == 0);
            if (stopIndex == numStops - 1 || occupancy + requestState.originalRequest.numRiders > veh.capacity) {
                if (dropoff.loc != stopLocations[stopIndex])
                    return {INFTY, INFTY};
                const int walkingCost = CostCalculator::CostFunction::calcWalkingCost(dropoff.walkingDist);
                return {walkingCost, 0};
            }

            if (stopLocations[stopIndex + 1] == dropoff.loc)
                return {INFTY, INFTY};

            if (distFromStopToDropoff >= INFTY || distFromDropoffToNextStop >= INFTY)
                return {INFTY, INFTY};

            const bool isDropoffAtExistingStop = dropoff.loc == stopLocations[stopIndex];
            const int initialDropoffDetour = calcInitialDropoffDetour(vehId, stopIndex, distFromStopToDropoff,
                                                                      distFromDropoffToNextStop,
                                                                      isDropoffAtExistingStop,
                                                                      routeState);
            KASSERT(initialDropoffDetour >= 0);
            if (doesDropoffDetourViolateHardConstraints(veh, requestState, stopIndex, initialDropoffDetour, routeState))
                return {INFTY, INFTY};

            const int tripTimeUntilDest = distFromStopToDropoff + dropoff.walkingDist;
            const int fixedCost = CostCalculator::CostFunction::calcTripCost(tripTimeUntilDest) +
                                  CostCalculator::CostFunction::calcWalkingCost(dropoff.walkingDist);
            return {fixedCost, initialDropoffDetour};
        }

        RelevantPDLocs filterPickupsBeforeNextStop(const FeasiblePDLocs &feasible,
                                                   int &numStopsRelevant,
                                                   const RequestState &requestState,
                                                   const std::vector<PDLoc> &pickups) {
            // For each stop s, prune the pickups and dropoffs deemed relevant for an ordinary assignment after s by
            // checking them against constraints and lower bounds.
            using namespace time_utils;

            numStopsRelevant = 0;

            RelevantPDLocs rel(fleet.size());

            vehiclesWithFeasibleDistances.clear();
            for (const auto &stopId: feasible.getStopIdsWithRelevantPDLocs()) {
                const auto vehId = routeState.vehicleIdOf(stopId);
                if (vehiclesWithFeasibleDistances.contains(vehId))
                    continue;
                const auto stopPos = routeState.stopPositionOf(stopId);
                if (stopPos > 0)
                    continue;
                // Skip vehicles that are already on edge that represents next stop for PBNS
                if (routeState.schedArrTimesFor(vehId)[1] - inputGraph.travelTime(routeState.stopLocationsFor(vehId)[1])
                    <= requestState.dispatchingTime)
                    continue;
                vehiclesWithFeasibleDistances.insert(vehId);
            }

            for (const auto &vehId: vehiclesWithFeasibleDistances) {
                const auto &veh = fleet[vehId];
                const auto &numStops = routeState.numStopsOf(vehId);
                const auto &stopIds = routeState.stopIdsFor(vehId);
                const auto &occupancies = routeState.occupanciesFor(vehId);
                KASSERT(numStops > 1);

                const int totalNumRelPdLocsBefore = static_cast<int>(rel.relevantSpots.size());

                // Track relevant PD locs for each stop in the relevant PD locs data structure.
                // Entries are ordered by vehicle and by stop.

                if (occupancies[0] + requestState.originalRequest.numRiders > veh.capacity)
                    continue;

                const auto &stopId = stopIds[0];

                // Insert entries at this stop.

                ++numStopsRelevant;
                // Check each PD loc
                for (const auto &[id, distToPDLoc, distFromPDLoc] : feasible.feasiblePdLocsFor(stopId)) {

                    const bool isRelevant = isPickupBeforeNextStopRelevant(
                        veh, 0, pickups[id], distToPDLoc, distFromPDLoc, requestState);
                    if (isRelevant) {
                        rel.relevantSpots.push_back({0, id, distToPDLoc, distFromPDLoc});
                    }
                }

                // If vehicle has at least one stop with relevant PD loc, add the vehicle
                if (rel.relevantSpots.size() > totalNumRelPdLocsBefore) {
                    rel.vehiclesWithRelevantSpots.push_back(vehId);
                    rel.vehicleToPdLocs[vehId] = {totalNumRelPdLocsBefore, static_cast<int>(rel.relevantSpots.size())};
                }
            }

            KASSERT(std::all_of(rel.relevantSpots.begin(), rel.relevantSpots.end(),
                [&](const auto &h) {
                return h.distToPDLoc < INFTY && h.distFromPDLocToNextStop < INFTY;
                }));

            return rel;
        }

        bool isPickupBeforeNextStopRelevant(const Vehicle &veh, const int stopIndex, const PDLoc &pickup,
                                            const int distFromStopToPickup,
                                            const int distFromPickupToNextStop,
                                            const RequestState &requestState) const {
            using namespace time_utils;

            const int &vehId = veh.vehicleId;

            KASSERT(routeState.occupanciesFor(vehId)[stopIndex] + requestState.originalRequest.numRiders <=
                veh.capacity);
            if (distFromStopToPickup >= INFTY || distFromPickupToNextStop >= INFTY)
                return false;

            KASSERT(distFromStopToPickup + distFromPickupToNextStop >=
                calcLengthOfLegStartingAt(stopIndex, vehId, routeState));

            KASSERT(stopIndex < routeState.numStopsOf(vehId) - 1);
            // If the pickup coincides with the location of the next stop, we will consider it at the next stop. Skip here
            if (pickup.loc == routeState.stopLocationsFor(vehId)[stopIndex + 1])
                return false;

            const auto depTimeAtPickup = getActualDepTimeAtPickup(vehId, stopIndex, distFromStopToPickup, pickup,
                                                                  requestState, routeState);
            const auto initialPickupDetour = calcInitialPickupDetour(vehId, stopIndex, INVALID_INDEX, depTimeAtPickup,
                                                                     distFromPickupToNextStop, requestState,
                                                                     routeState);

            if (doesPickupDetourViolateHardConstraints(veh, requestState, stopIndex, initialPickupDetour, routeState))
                return false;

            return true;
        }

        int recomputeDistToPDLocDirectly(const int vehId, const int stopIdxBefore, const int pdLocLocation) {
            auto src = ch.rank(inputGraph.edgeHead(routeState.stopLocationsFor(vehId)[stopIdxBefore]));
            auto tar = ch.rank(inputGraph.edgeTail(pdLocLocation));
            auto offset = inputGraph.travelTime(pdLocLocation);

            chQuery.run(src, tar);
            return chQuery.getDistance() + offset;
        }

        int recomputeDistFromPDLocDirectly(const int vehId, const int stopIdxAfter, const int pdLocLocation) {
            auto src = ch.rank(inputGraph.edgeHead(pdLocLocation));
            auto tar = ch.rank(inputGraph.edgeTail(routeState.stopLocationsFor(vehId)[stopIdxAfter]));
            auto offset = inputGraph.travelTime(routeState.stopLocationsFor(vehId)[stopIdxAfter]);

            chQuery.run(src, tar);
            return chQuery.getDistance() + offset;
        }


        const Fleet &fleet;
        const InputGraphT &inputGraph;
        const CH &ch;
        typename CHEnvT::template FullCHQuery<> chQuery;
        CostCalculator calculator;
        const RouteState &routeState;

        LightweightSubset vehiclesWithFeasibleDistances;
    };
}
