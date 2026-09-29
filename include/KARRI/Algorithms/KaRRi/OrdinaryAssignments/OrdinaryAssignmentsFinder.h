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

#include "../../../Tools/Timer.h"
#include "../BaseObjects/Assignment.h"
#include "../RequestState/RelevantPDLocs.h"
#include "../PDDistanceQueries/PDDistances.h"

namespace karri {
    // Finds ordinary assignments, i.e. those assignments where pickup and dropoff are both inserted after the vehicle's
    // next stop but before the vehicle's last stop. This includes ordinary paired assignments where the pickup and dropoff
    // are inserted between the same pair of existing stops.
    //
    // Works based on filtered relevant PD locs.
    class OrdinaryAssignmentsFinder {
        static constexpr int MIN_PD_PAIRS_FOR_PREPROCESSING = 30 * 30;

    public:
        OrdinaryAssignmentsFinder(const Fleet &fleet, const RouteState &routeState)
            : fleet(fleet),
              calculator(routeState),
              routeState(routeState) {
        }

        void findAssignments(const RelevantPDLocs &relPickups, const RelevantPDLocs &relDropoffs,
                             const RequestState &requestState,
                             const PDDistances &pdDistances,
                             const PDLocs &pdLocs,
                             InternalTaxiResult &result,
                             stats::OrdAssignmentsPerformanceStats &stats) {
            if (pdLocs.numPickups() * pdLocs.numDropoffs() >= MIN_PD_PAIRS_FOR_PREPROCESSING) {
                stopPairsToProcessPaired.clear();
                stopPairsToProcessNonPaired.clear();
                findOrderedStopPairsToProcess(relPickups, relDropoffs, requestState, pdLocs, pdDistances, result,
                                              stopPairsToProcessNonPaired,
                                              stopPairsToProcessPaired, stats);
                findOrdinaryAssignmentsWithPreprocessing(relPickups, relDropoffs, requestState, pdLocs,
                                                         stopPairsToProcessNonPaired, result, stats);
                findOrdinaryPairedAssignmentsWithPreprocessing(pdDistances, relPickups, relDropoffs, requestState,
                                                               pdLocs, stopPairsToProcessPaired, result, stats);
            } else {
                findOrdinaryAssignmentsWithoutPreprocessing(relPickups, relDropoffs, requestState, pdLocs, result,
                                                            stats);
                findOrdinaryPairedAssignmentsWithoutPreprocessing(pdDistances, relPickups, relDropoffs, requestState,
                                                                  pdLocs, result, stats);
            }
        }

        void init(const RequestState &, const PDLocs &, stats::OrdAssignmentsPerformanceStats &) {
            // no op
        }

    private:
        struct StopPairToProcess {
            int vehId = INVALID_ID;
            int pickupStopIdx = INVALID_INDEX;
            int dropoffStopIdx = INVALID_INDEX;
            int costLowerBound = INFTY;
        };

        // For each pair of stops (i, j) with i < j where the pickup can be inserted at or after stop i and the dropoff
        // can be inserted at or after stop j, this method computes a lower bound for the cost of an assignment using
        // i and j. It then returns a vector of all such stop pairs with their lower bounds, ordered by increasing lower bound.
        void findOrderedStopPairsToProcess(const RelevantPDLocs &relPickups, const RelevantPDLocs &relDropoffs,
                                           const RequestState &requestState, const PDLocs &pdLocs,
                                           const PDDistances &pdDistances,
                                           const InternalTaxiResult &result,
                                           std::vector<StopPairToProcess> &nonPaired,
                                           std::vector<StopPairToProcess> &paired,
                                           stats::OrdAssignmentsPerformanceStats &stats) const {
            KaRRiTimer timer;
            std::vector<int> minDepTimeAtPickup;
            std::vector<int> minDistFromPickup;
            std::vector<int> minDistToDropoff;
            std::vector<int> minDistFromDropoff;
            int64_t numCandidateVehicles = 0;
            using namespace time_utils;
            for (const auto &vehId: relPickups.getVehiclesWithRelevantPDLocs()) {
                if (!relDropoffs.hasRelevantSpotsFor(vehId))
                    continue;
                ++numCandidateVehicles;
                const int numStops = routeState.numStopsOf(vehId);
                std::ranges::fill(minDepTimeAtPickup, INFTY);
                std::ranges::fill(minDistFromPickup, INFTY);
                std::ranges::fill(minDistToDropoff, INFTY);
                std::ranges::fill(minDistFromDropoff, INFTY);
                if (minDepTimeAtPickup.size() < numStops) {
                    minDepTimeAtPickup.resize(numStops, INFTY);
                    minDistFromPickup.resize(numStops, INFTY);
                    minDistToDropoff.resize(numStops, INFTY);
                    minDistFromDropoff.resize(numStops, INFTY);
                }
                const auto &schedArrTimes = routeState.schedArrTimesFor(vehId);
                const auto &occs = routeState.occupanciesFor(vehId);
                for (const auto &pickupEntry: relPickups.relevantSpotsFor(vehId)) {
                    const int stopIdx = pickupEntry.stopIndex;
                    const int depTimeAtPickup = getActualDepTimeAtPickup(
                        vehId, stopIdx, pickupEntry.distToPDLoc, pdLocs.pickups[pickupEntry.pdId], requestState,
                        routeState);
                    minDepTimeAtPickup[stopIdx] = std::min(minDepTimeAtPickup[stopIdx], depTimeAtPickup);
                    minDistFromPickup[stopIdx] = std::min(minDistFromPickup[stopIdx],
                                                          pickupEntry.distFromPDLocToNextStop);
                }
                for (const auto &dropoffEntry: relDropoffs.relevantSpotsFor(vehId)) {
                    const int stopIdx = dropoffEntry.stopIndex;
                    minDistToDropoff[stopIdx] = std::min(minDistToDropoff[stopIdx], dropoffEntry.distToPDLoc);
                    minDistFromDropoff[stopIdx] = std::min(minDistFromDropoff[stopIdx],
                                                           dropoffEntry.distFromPDLocToNextStop);
                }

                for (int i = 0; i < numStops - 1; ++i) {
                    if (minDepTimeAtPickup[i] == INFTY)
                        continue;

                    // Paired
                    if (minDistToDropoff[i] != INFTY) {
                        const int minDetour = std::max(
                            minDepTimeAtPickup[i] + pdDistances.getMinDirectDistance() +
                            InputConfig::getInstance().stopTime + minDistFromDropoff[i] - schedArrTimes[i + 1], 0);
                        const auto residualDetourAtEnd = calcResidualTotalDetourForStopAfterDropoff(
                            vehId, i, numStops - 1, minDetour, routeState);
                        if (isAnyHardConstraintViolated(fleet[vehId], i, i, requestState, INFTY, minDetour,
                                                        residualDetourAtEnd, false, routeState))
                            continue;

                        const int addedTripTime = calcAddedTripTimeAffectedByPickupAndDropoff(
                            vehId, i, minDetour, routeState);

                        const auto arrTimeAtDropoff = minDepTimeAtPickup[i] + pdDistances.getMinDirectDistance();
                        const int tripTime = arrTimeAtDropoff - requestState.earliestDeparture();

                        const auto tripCost = CostCalculator::CostFunction::calcTripCost(tripTime);
                        const auto waitTimeViolationCost = CostCalculator::CostFunction::calcWaitViolationCost(
                            minDepTimeAtPickup[i], requestState);
                        const auto changeInTripCostsOfOthers =
                                CostCalculator::CostFunction::calcChangeInTripCostsOfExistingPassengers(addedTripTime);
                        const auto vehCost = CostCalculator::CostFunction::calcVehicleCost(residualDetourAtEnd);

                        const int minCost = vehCost + tripCost + waitTimeViolationCost + changeInTripCostsOfOthers;
                        if (minCost > result.bestCost)
                            continue;

                        paired.push_back({vehId, i, i, minCost});
                    }

                    // Non-paired
                    const int minPickupDetour = minDepTimeAtPickup[i] + minDistFromPickup[i] - schedArrTimes[i + 1];
                    // Compute smallest stop index after pickup at which capacity of vehicle would be broken (end of route
                    // if never broken). Dropoff has to be made before this index.
                    int capacityBrokenIndex = i;
                    const int cap = fleet[vehId].capacity;
                    while (capacityBrokenIndex < numStops && occs[capacityBrokenIndex] + requestState.originalRequest.
                           numRiders <= cap) {
                        ++capacityBrokenIndex;
                    }
                    for (int j = i + 1; j < numStops; ++j) {
                        if (j >= capacityBrokenIndex)
                            break;
                        if (minDistToDropoff[j] == INFTY)
                            continue;

                        int addedTripTime = calcAddedTripTimeInInterval(vehId, i, j, minPickupDetour, routeState);
                        const int minDropoffDetour =
                                minDistToDropoff[j] + minDistFromDropoff[j] + (minDistToDropoff[j] == 0
                                                                                   ? 0
                                                                                   : InputConfig::getInstance().
                                                                                   stopTime) -
                                calcLengthOfLegStartingAt(j, vehId, routeState);
                        const auto detourRightAfterDropoff = calcDetourRightAfterDropoff(
                            vehId, i, j, minPickupDetour, minDropoffDetour, routeState);
                        const auto residualDetourAtEnd = calcResidualTotalDetourForStopAfterDropoff(vehId,
                            j, numStops - 1, detourRightAfterDropoff, routeState);

                        if (isAnyHardConstraintViolated(fleet[vehId], i, j, requestState, minPickupDetour,
                                                        detourRightAfterDropoff, residualDetourAtEnd, true, routeState))
                            continue;

                        addedTripTime += calcAddedTripTimeAffectedByPickupAndDropoff(
                            vehId, j, detourRightAfterDropoff, routeState);

                        const auto arrTimeAtDropoff = getArrTimeAtDropoff(
                            vehId, i, j, minDepTimeAtPickup[i], minDistToDropoff[j], minPickupDetour,
                            minDistToDropoff[j] == 0, routeState);
                        const int tripTime = arrTimeAtDropoff - requestState.earliestDeparture();

                        const auto tripCost = CostCalculator::CostFunction::calcTripCost(tripTime);
                        const auto waitTimeViolationCost = CostCalculator::CostFunction::calcWaitViolationCost(
                            minDepTimeAtPickup[i], requestState);
                        const auto changeInTripCostsOfOthers =
                                CostCalculator::CostFunction::calcChangeInTripCostsOfExistingPassengers(addedTripTime);
                        const auto vehCost = CostCalculator::CostFunction::calcVehicleCost(residualDetourAtEnd);

                        const int minCost = vehCost + tripCost + waitTimeViolationCost + changeInTripCostsOfOthers;

                        if (requestState.originalRequest.requestId == 3269 && vehId == 182 && i == 1 && j == 2) {
                            std::cout << "OrdinaryAssignmentsFinder::findOrderedStopPairsToProcess: "
                                            "vehId=" << vehId << ", i=" << i << ", j=" << j
                                            << ", minDepTimeAtPickup[i]=" << minDepTimeAtPickup[i]
                                            << ", minDistFromPickup[i]=" << minDistFromPickup[i]
                                            << ", minDistToDropoff[j]=" << minDistToDropoff[j]
                                            << ", minDistFromDropoff[j]=" << minDistFromDropoff[j]
                                            << ", minPickupDetour=" << minPickupDetour
                                            << ", minDropoffDetour=" << minDropoffDetour
                                            << ", detourRightAfterDropoff=" << detourRightAfterDropoff
                                            << ", residualDetourAtEnd=" << residualDetourAtEnd
                                            << ", addedTripTime=" << addedTripTime
                                            << ", tripTime=" << tripTime
                                            << ", tripCost=" << tripCost
                                            << ", waitTimeViolationCost=" << waitTimeViolationCost
                                            << ", changeInTripCostsOfOthers=" << changeInTripCostsOfOthers
                                            << ", vehCost=" << vehCost
                                            << ", minCost=" << minCost
                                            << ", result.bestCost=" << result.bestCost << std::endl;
                        }

                        if (minCost > result.bestCost)
                            continue;

                        nonPaired.push_back({vehId, i, j, minCost});
                    }
                }
            }


            std::ranges::sort(paired, [](const StopPairToProcess &a, const StopPairToProcess &b) {
                return a.costLowerBound < b.costLowerBound;
            });
            std::ranges::sort(nonPaired, [](const StopPairToProcess &a, const StopPairToProcess &b) {
                return a.costLowerBound < b.costLowerBound;
            });

            stats.preFilteringTime += timer.elapsed<std::chrono::nanoseconds>();
            stats.numCandidateVehicles += numCandidateVehicles;
        }


        // Try assignments where pickup is inserted at or just after stop i and dropoff is inserted at or just after stop j
        // with j > i. Does not deal with inserting the pickup at or after a last stop. Does not deal with inserting the
        // dropoff after a last stop.
        void findOrdinaryAssignmentsWithoutPreprocessing(const RelevantPDLocs &relPickups,
                                                         const RelevantPDLocs &relDropoffs,
                                                         const RequestState &requestState, const PDLocs &pdLocs,
                                                         InternalTaxiResult &result,
                                                         stats::OrdAssignmentsPerformanceStats &stats) const {
            KaRRiTimer timer;
            int numCandidateVehicles = 0;
            int numAssignmentsTried = 0;

            for (const auto &vehId: relPickups.getVehiclesWithRelevantPDLocs()) {
                if (!relDropoffs.hasRelevantSpotsFor(vehId))
                    continue;

                KASSERT(relPickups.hasRelevantSpotsFor(vehId));

                ++numCandidateVehicles;
                Assignment asgn(&fleet[vehId]);

                const auto relevantDropoffs = relDropoffs.relevantSpotsFor(vehId);
                auto curFirstDropoffIt = relevantDropoffs.begin();

                for (const auto &pickupEntry: relPickups.relevantSpotsFor(vehId)) {
                    // Find first stop position after the pickup's stop position that has relevant dropoffs.
                    const auto &stopPos = pickupEntry.stopIndex;
                    while (curFirstDropoffIt < relevantDropoffs.end() && curFirstDropoffIt->stopIndex <= stopPos) {
                        ++curFirstDropoffIt;
                    }
                    if (curFirstDropoffIt == relevantDropoffs.end())
                        break; // No dropoffs later in route than current (or subsequent) pickup(s)

                    asgn.pickup = pdLocs.pickups[pickupEntry.pdId];
                    asgn.pickupStopIdx = pickupEntry.stopIndex;
                    asgn.distToPickup = pickupEntry.distToPDLoc;
                    asgn.distFromPickup = pickupEntry.distFromPDLocToNextStop;

                    numAssignmentsTried += tryDropoffsLaterThanPickup(asgn, curFirstDropoffIt, relevantDropoffs.end(),
                                                                      requestState,
                                                                      pdLocs, result);
                }
            }

            const auto time = timer.elapsed<std::chrono::nanoseconds>();
            stats.tryNonPairedAssignmentsTime += time;
            stats.numNonPairedAssignmentsTried += numAssignmentsTried;
            stats.numCandidateVehicles += numCandidateVehicles;
        }

        // Try assignments where pickup is inserted at or just after stop i and dropoff is inserted at or just after stop j
        // with j > i. Does not deal with inserting the pickup at or after a last stop. Does not deal with inserting the
        // dropoff after a last stop.
        // Uses preprocessed lower bounds on cost to order the stop pairs to process and to stop when remaining stop
        // pairs cannot yield a better assignment than the best found so far.
        void findOrdinaryAssignmentsWithPreprocessing(const RelevantPDLocs &relPickups,
                                                      const RelevantPDLocs &relDropoffs,
                                                      const RequestState &requestState, const PDLocs &pdLocs,
                                                      const std::vector<StopPairToProcess> &stopPairs,
                                                      InternalTaxiResult &result,
                                                      stats::OrdAssignmentsPerformanceStats &stats) const {
            KaRRiTimer timer;
            int numAssignmentsTried = 0;

            for (const auto &stopPair: stopPairs) {
                if (stopPair.costLowerBound > result.bestCost)
                    break;

                const auto &vehId = stopPair.vehId;
                KASSERT(relPickups.hasRelevantSpotsFor(vehId));
                KASSERT(relDropoffs.hasRelevantSpotsFor(vehId));

                Assignment asgn(&fleet[vehId]);
                asgn.pickupStopIdx = stopPair.pickupStopIdx;
                asgn.dropoffStopIdx = stopPair.dropoffStopIdx;

                // Find ranges of pickup entries and dropoff entries for the given stop pair.
                const auto &relPickupEntries = relPickups.relevantSpotsFor(vehId);
                const auto &relDropoffEntries = relDropoffs.relevantSpotsFor(vehId);
                constexpr struct {
                    bool operator()(const RelevantPDLocs::RelevantPDLoc &entry, const int stopIdx) const {
                        return entry.stopIndex < stopIdx;
                    }

                    bool operator()(const int stopIdx, const RelevantPDLocs::RelevantPDLoc &entry) const {
                        return stopIdx < entry.stopIndex;
                    }
                } comp;
                const auto [startPickups, endPickups] = std::equal_range(
                    relPickupEntries.begin(), relPickupEntries.end(), asgn.pickupStopIdx, comp);
                const auto [startDropoffs, endDropoffs] = std::equal_range(
                    relDropoffEntries.begin(), relDropoffEntries.end(), asgn.dropoffStopIdx, comp);

                // Try all pickup entries for the given pickup stop index and all dropoff entries for the given dropoff stop index.
                for (auto pickupIt = startPickups; pickupIt < endPickups; ++pickupIt) {
                    const auto &pickupEntry = *pickupIt;
                    asgn.pickup = pdLocs.pickups[pickupEntry.pdId];
                    asgn.distToPickup = pickupEntry.distToPDLoc;
                    asgn.distFromPickup = pickupEntry.distFromPDLocToNextStop;
                    numAssignmentsTried += tryDropoffsLaterThanPickup(asgn, startDropoffs, endDropoffs, requestState,
                                                                      pdLocs, result);
                }
            }

            const auto time = timer.elapsed<std::chrono::nanoseconds>();
            stats.tryNonPairedAssignmentsTime += time;
            stats.numNonPairedAssignmentsTried += numAssignmentsTried;
        }

        // Given a partial assignment for a pickup and a starting index in the relevant PD locs
        // startIdxInRegularSpots, this method scans all relevant regular dropoffs that come after startIdxInRegularSpots,
        // completes the assignment with those dropoffs, and tries the resulting assignments.
        // Note that startIdxInRegularStops has to be an absolute index in relevantRegularHaltingSpots.
        template<bool checkCapacity = true>
        int tryDropoffsLaterThanPickup(Assignment &asgn,
                                       const RelevantPDLocs::It &startDropoffs,
                                       const RelevantPDLocs::It &endDropoffs,
                                       const RequestState &requestState,
                                       const PDLocs &pdLocs,
                                       InternalTaxiResult &result) const {
            KASSERT(asgn.vehicle && asgn.pickup.id != INVALID_ID);
            const auto &vehId = asgn.vehicle->vehicleId;

            auto numAssignmentsTried = 0;

            const auto &numStops = routeState.numStopsOf(vehId);
            const auto &stopLocations = routeState.stopLocationsFor(vehId);

            // Compute smallest stop index after pickup at which capacity of vehicle would be broken (end of route
            // if never broken). Dropoff has to be made before this index.
            int capacityBrokenIndex = numStops;
            if constexpr (checkCapacity) {
                capacityBrokenIndex = asgn.pickupStopIdx;
                const auto occs = routeState.occupanciesFor(vehId);
                const int cap = asgn.vehicle->capacity;
                while (capacityBrokenIndex < numStops && occs[capacityBrokenIndex] + requestState.originalRequest.
                       numRiders <= cap) {
                    ++capacityBrokenIndex;
                }
            }

            for (auto dropoffIt = startDropoffs; dropoffIt < endDropoffs; ++dropoffIt) {
                const auto &dropoffEntry = *dropoffIt;

                if constexpr (checkCapacity) {
                    if (dropoffEntry.stopIndex > capacityBrokenIndex) {
                        // All remaining dropoffs would be after the stop where capacity is broken. Need to allow dropoff
                        // at capacityBrokenIndex, since dropoff may be made at stop.
                        break;
                    }
                }

                asgn.dropoff = pdLocs.dropoffs[dropoffEntry.pdId];

                if (dropoffEntry.stopIndex + 1 < numStops &&
                    stopLocations[dropoffEntry.stopIndex + 1] == asgn.dropoff.loc) {
                    // If the dropoff is at the location of the following stop, do not try an assignment here as it would
                    // introduce a new stop after dropoffIndex that is at the same location as dropoffIndex + 1.
                    // Instead, this will be dealt with as an assignment at dropoffIndex + 1 afterwards.
                    continue;
                }

                if (asgn.dropoff.loc == asgn.pickup.loc) {
                    // In this case, this spot is the best spot at or after pickupIndex and the best spot at or after
                    // dropoffIndex. We ignore this case here since inserting them paired into the same leg will be better.
                    continue;
                }

                asgn.dropoffStopIdx = dropoffEntry.stopIndex;
                asgn.distToDropoff = dropoffEntry.distToPDLoc;
                asgn.distFromDropoff = dropoffEntry.distFromPDLocToNextStop;
                result.tryAssignmentWithKnownCost(asgn, calculator.calc(asgn, requestState));
                ++numAssignmentsTried;
            }

            return numAssignmentsTried;
        }


        void findOrdinaryPairedAssignmentsWithoutPreprocessing(
            const PDDistances &pdDistances, const RelevantPDLocs &relPickups,
            const RelevantPDLocs &relDropoffs,
            const RequestState &requestState, const PDLocs &pdLocs,
            InternalTaxiResult &result,
            stats::OrdAssignmentsPerformanceStats &stats) const {
            KaRRiTimer timer;
            int numAssignmentsTried = 0;

            Assignment asgn;
            const auto minDirectDistance = pdDistances.getMinDirectDistance();

            unsigned int minPickupId = INVALID_ID, minDropoffId = INVALID_ID;
            for (const auto &vehId: relPickups.getVehiclesWithRelevantPDLocs()) {
                if (!relDropoffs.hasRelevantSpotsFor(vehId))
                    continue;

                const auto &veh = fleet[vehId];
                const auto &stopLocations = routeState.stopLocationsFor(vehId);

                asgn.vehicle = &veh;

                const auto relevantPickups = relPickups.relevantSpotsFor(vehId);
                const auto relevantDropoffs = relDropoffs.relevantSpotsFor(vehId);

                auto pickupIt = relevantPickups.begin();
                auto dropoffIt = relevantDropoffs.begin();
                while (pickupIt < relevantPickups.end() && dropoffIt < relevantDropoffs.end()) {
                    // Alternating sweep over pickups and dropoffs which pause once they meet or pass the other sweep.
                    while (pickupIt < relevantPickups.end() && pickupIt->stopIndex < dropoffIt->stopIndex)
                        ++pickupIt;
                    if (pickupIt == relevantPickups.end())
                        break;
                    while (dropoffIt < relevantDropoffs.end() && dropoffIt->stopIndex < pickupIt->stopIndex)
                        ++dropoffIt;
                    if (dropoffIt == relevantDropoffs.end())
                        break;

                    // If both sweeps paused at the same stopIndex, there are pickups and dropoffs at this stop.
                    // We attempt a paired assignment.
                    if (pickupIt->stopIndex == dropoffIt->stopIndex) {
                        const auto stopPos = pickupIt->stopIndex;

                        if (routeState.occupanciesFor(vehId)[stopPos] + requestState.originalRequest.numRiders > veh.
                            capacity) {
                            continue;
                        }

                        const auto beginOfStopInPickups = pickupIt;
                        const auto beginOfStopInDropoffs = dropoffIt;

                        // Iterate over all pickups/dropoffs at this stop once to find a lower bound on the cost of any
                        // paired assignment here
                        int minDistToPickup = INFTY;
                        int minDistFromDropoff = INFTY;

                        while (pickupIt < relevantPickups.end() && pickupIt->stopIndex == stopPos) {
                            const auto &entry = *pickupIt;
                            if (entry.distToPDLoc < minDistToPickup) {
                                minDistToPickup = entry.distToPDLoc;
                                minPickupId = entry.pdId;
                            }
                            ++pickupIt;
                        }

                        while (dropoffIt < relevantDropoffs.end() && dropoffIt->stopIndex == stopPos) {
                            const auto &entry = *dropoffIt;
                            if (entry.distFromPDLocToNextStop < minDistFromDropoff) {
                                minDistFromDropoff = entry.distFromPDLocToNextStop;
                                minDropoffId = entry.pdId;
                            }
                            ++dropoffIt;
                        }

                        if (minDistToPickup == INFTY || minDistFromDropoff == INFTY)
                            continue;

                        const auto endOfStopInPickups = pickupIt;
                        const auto endOfStopInDropoffs = dropoffIt;

                        // With collected lower bounds, we check whether an assignment better than the best known is possible with this vehicle
                        asgn.pickup = pdLocs.pickups[minPickupId];
                        asgn.dropoff = pdLocs.dropoffs[minDropoffId];
                        asgn.pickupStopIdx = stopPos;
                        asgn.dropoffStopIdx = stopPos;
                        asgn.distToPickup = minDistToPickup;
                        asgn.distToDropoff = minDirectDistance;
                        asgn.distFromDropoff = minDistFromDropoff;
                        const auto lowerBoundCost =
                                calculator.calcCostLowerBoundForOrdinaryPairedAssignment(asgn, requestState);
                        if (lowerBoundCost > result.getBestCost())
                            continue;

                        // Try paired assignment for every combination of relevant pickup and dropoff
                        numAssignmentsTried += tryPairingPickupsAndDropoffs(
                            asgn, beginOfStopInPickups, endOfStopInPickups,
                            beginOfStopInDropoffs, endOfStopInDropoffs, pdDistances,
                            requestState, pdLocs, result);
                    }
                }
            }

            const auto pairedTime = timer.elapsed<std::chrono::nanoseconds>();
            stats.tryPairedAssignmentsTime += pairedTime;
            stats.numPairedAssignmentsTried += numAssignmentsTried;
        }

        // Tries all paired assignments for the given stop pairs, ordered by increasing lower bound on cost.
        // Stops trying once the lower bound exceeds the best known cost.
        void findOrdinaryPairedAssignmentsWithPreprocessing(
            const PDDistances &pdDistances, const RelevantPDLocs &relPickups,
            const RelevantPDLocs &relDropoffs,
            const RequestState &requestState, const PDLocs &pdLocs,
            const std::vector<StopPairToProcess> &stopPairs,
            InternalTaxiResult &result,
            stats::OrdAssignmentsPerformanceStats &stats) const {
            KaRRiTimer timer;
            int numAssignmentsTried = 0;

            Assignment asgn;
            for (const auto &stopPair: stopPairs) {
                if (stopPair.costLowerBound > result.bestCost)
                    break;

                const auto &vehId = stopPair.vehId;
                KASSERT(relPickups.hasRelevantSpotsFor(vehId));
                KASSERT(relDropoffs.hasRelevantSpotsFor(vehId));

                const auto &veh = fleet[vehId];

                asgn.vehicle = &veh;
                asgn.pickupStopIdx = stopPair.pickupStopIdx;
                asgn.dropoffStopIdx = stopPair.dropoffStopIdx;

                // Find ranges of pickup entries and dropoff entries for the given stop pair.
                const auto &relPickupEntries = relPickups.relevantSpotsFor(vehId);
                const auto &relDropoffEntries = relDropoffs.relevantSpotsFor(vehId);
                constexpr struct {
                    bool operator()(const RelevantPDLocs::RelevantPDLoc &entry, const int stopIdx) const {
                        return entry.stopIndex < stopIdx;
                    }

                    bool operator()(const int stopIdx, const RelevantPDLocs::RelevantPDLoc &entry) const {
                        return stopIdx < entry.stopIndex;
                    }
                } comp;
                const auto [startPickups, endPickups] = std::equal_range(
                    relPickupEntries.begin(), relPickupEntries.end(), asgn.pickupStopIdx, comp);
                const auto [startDropoffs, endDropoffs] = std::equal_range(
                    relDropoffEntries.begin(), relDropoffEntries.end(), asgn.dropoffStopIdx, comp);

                // Try paired assignment for every combination of relevant pickup and dropoff
                numAssignmentsTried += tryPairingPickupsAndDropoffs(asgn, startPickups, endPickups,
                                                                    startDropoffs, endDropoffs, pdDistances,
                                                                    requestState, pdLocs, result);
            }

            const auto pairedTime = timer.elapsed<std::chrono::nanoseconds>();
            stats.tryPairedAssignmentsTime += pairedTime;
            stats.numPairedAssignmentsTried += numAssignmentsTried;
        }

        // Given a range of pickups and dropoffs between the same consecutive vehicle stops, this method tries all
        // paired assignments.
        // The given partial assignment needs to already specify the vehicle and stop index.
        int tryPairingPickupsAndDropoffs(Assignment &asgn,
                                         const RelevantPDLocs::It &startPickups,
                                         const RelevantPDLocs::It &endPickups,
                                         const RelevantPDLocs::It &startDropoffs,
                                         const RelevantPDLocs::It &endDropoffs,
                                         const PDDistances &pdDistances,
                                         const RequestState &requestState,
                                         const PDLocs &pdLocs,
                                         InternalTaxiResult &result) const {
            KASSERT(asgn.vehicle);
            const auto &vehId = asgn.vehicle->vehicleId;
            const int stopPos = asgn.pickupStopIdx;
            KASSERT(stopPos == asgn.dropoffStopIdx);

            auto numAssignmentsTried = 0;

            const auto &stopLocations = routeState.stopLocationsFor(vehId);

            // Try paired assignment for every combination of relevant pickup and dropoff
            for (auto dropoffIt2 = startDropoffs; dropoffIt2 < endDropoffs; ++dropoffIt2) {
                const auto &dropoffEntry = *dropoffIt2;
                asgn.dropoff = pdLocs.dropoffs[dropoffEntry.pdId];

                // if dropoff coincides with the following stop, an ordinary non-paired assignment with dropoffIndex = pickupIndex + 1 will cover this case
                if (stopLocations[stopPos + 1] == asgn.dropoff.loc)
                    continue;

                asgn.distFromDropoff = dropoffEntry.distFromPDLocToNextStop;
                for (auto pickupIt2 = startPickups; pickupIt2 < endPickups; ++pickupIt2) {
                    const auto &pickupEntry = *pickupIt2;
                    asgn.pickup = pdLocs.pickups[pickupEntry.pdId];
                    if (asgn.pickup.loc == asgn.dropoff.loc)
                        continue;

                    asgn.distToPickup = pickupEntry.distToPDLoc;

                    KASSERT(asgn.distToPickup < INFTY && asgn.distFromDropoff < INFTY);
                    asgn.distToDropoff = pdDistances.getDirectDistance(asgn.pickup, asgn.dropoff);
                    result.tryAssignmentWithKnownCost(asgn, calculator.calc(asgn, requestState));
                    ++numAssignmentsTried;
                }
            }

            return numAssignmentsTried;
        }

        const Fleet &fleet;
        CostCalculator calculator;
        const RouteState &routeState;

        std::vector<StopPairToProcess> stopPairsToProcessNonPaired;
        std::vector<StopPairToProcess> stopPairsToProcessPaired;
    };
}
