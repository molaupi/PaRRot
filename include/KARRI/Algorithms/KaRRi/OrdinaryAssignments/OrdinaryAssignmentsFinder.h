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
    public:
        OrdinaryAssignmentsFinder(const Fleet &fleet, const RouteState &routeState)
            : fleet(fleet),
              calculator(routeState),
              routeState(routeState) {
        }

        void findAssignments(const RelevantPDLocs &relPickups,
                             const RelevantPDLocs &relDropoffs,
                             const FeasiblePDLocs &feasiblePickups,
                             const FeasiblePDLocs &feasibleDropoffs,
                             const RequestState &requestState,
                             const PDDistances &pdDistances,
                             const PDLocs &pdLocs,
                             InternalTaxiResult &result,
                             stats::OrdAssignmentsPerformanceStats &stats) const {
            findOrdinaryAssignments(relPickups, relDropoffs, requestState, pdLocs, result, stats);
            findOrdinaryPairedAssignments(feasiblePickups, feasibleDropoffs, pdDistances, requestState, pdLocs, result, stats);
        }

        void init(const RequestState &, const PDLocs &, stats::OrdAssignmentsPerformanceStats &) {
            // no op
        }

    private:
        // Try assignments where pickup is inserted at or just after stop i and dropoff is inserted at or just after stop j
        // with j > i. Does not deal with inserting the pickup at or after a last stop. Does not deal with inserting the
        // dropoff after a last stop.
        void findOrdinaryAssignments(const RelevantPDLocs &relPickups, const RelevantPDLocs &relDropoffs,
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
                //                if (!relPickups.hasRelevantSpotsFor(vehId) ||
                //                    !relDropoffs.hasRelevantSpotsFor(vehId))
                //                    continue;

                ++numCandidateVehicles;
                Assignment asgn(&fleet[vehId]);

                const auto relevantDropoffs = relDropoffs.relevantSpotsFor(vehId);
                auto curFirstDropoffIt = relevantDropoffs.begin();

                for (const auto &pickupEntry: relPickups.relevantSpotsFor(vehId)) {
                    // Find first stop position after the pickup's stop position that has relevant dropoffs.
                    const auto &stopPos = pickupEntry.stopIndex;
                    while (curFirstDropoffIt < relevantDropoffs.end() &&
                           curFirstDropoffIt->stopIndex <= stopPos) {
                        ++curFirstDropoffIt;
                    }
                    if (curFirstDropoffIt == relevantDropoffs.end())
                        break; // No dropoffs later in route than current (or subsequent) pickup(s)

                    asgn.pickup = pdLocs.pickups[pickupEntry.pdId];
                    asgn.pickupStopIdx = pickupEntry.stopIndex;
                    asgn.distToPickup = pickupEntry.distToPDLoc;
                    asgn.distFromPickup = pickupEntry.distFromPDLocToNextStop;

                    numAssignmentsTried += tryDropoffLaterThanPickup(asgn, curFirstDropoffIt, relDropoffs, requestState,
                                                                     pdLocs, result);
                }
            }

            const auto time = timer.elapsed<std::chrono::nanoseconds>();
            stats.tryNonPairedAssignmentsTime += time;
            stats.numNonPairedAssignmentsTried += numAssignmentsTried;
            stats.numCandidateVehicles += numCandidateVehicles;
        }

        // Given a partial assignment for a pickup and a starting index in the relevant PD locs
        // startIdxInRegularSpots, this method scans all relevant regular dropoffs that come after startIdxInRegularSpots,
        // completes the assignment with those dropoffs, and tries the resulting assignments.
        // Note that startIdxInRegularStops has to be an absolute index in relevantRegularHaltingSpots.
        int tryDropoffLaterThanPickup(Assignment &asgn,
                                      const RelevantPDLocs::It &startItInRegularDropoffs,
                                      const RelevantPDLocs &relDropoffs,
                                      const RequestState &requestState,
                                      const PDLocs &pdLocs,
                                      InternalTaxiResult &result) const {
            assert(asgn.vehicle && asgn.pickup.id != INVALID_ID);
            const auto &vehId = asgn.vehicle->vehicleId;
            const auto occs = routeState.occupanciesFor(vehId);

            const auto relevantDropoffs = relDropoffs.relevantSpotsFor(vehId);
            assert(startItInRegularDropoffs >= relevantDropoffs.begin() &&
                startItInRegularDropoffs <= relevantDropoffs.end());

            if (!relDropoffs.hasRelevantSpotsFor(vehId))
                return 0;

            auto numAssignmentsTriedWithOrdinaryDropoff = 0;

            const auto &numStops = routeState.numStopsOf(vehId);
            const auto &stopLocations = routeState.stopLocationsFor(vehId);

            // Compute smallest stop index after pickup at which capacity of vehicle would be broken (end of route
            // if never broken). Dropoff has to be made before this index.
            int capacityBrokenIndex = asgn.pickupStopIdx;
            const int cap = asgn.vehicle->capacity;
            while (capacityBrokenIndex < numStops && occs[capacityBrokenIndex] + requestState.originalRequest.numRiders
                   <= cap) {
                ++capacityBrokenIndex;
            }

            for (auto dropoffIt = startItInRegularDropoffs; dropoffIt < relevantDropoffs.end(); ++dropoffIt) {
                const auto &dropoffEntry = *dropoffIt;

                if (dropoffEntry.stopIndex > capacityBrokenIndex) {
                    // All remaining dropoffs would be after the stop where capacity is broken. Need to allow dropoff
                    // at capacityBrokenIndex, since dropoff may be made at stop.
                    break;
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
                ++numAssignmentsTriedWithOrdinaryDropoff;
            }

            return numAssignmentsTriedWithOrdinaryDropoff;
        }


        void findOrdinaryPairedAssignments(const FeasiblePDLocs &feasiblePickups,
                                           const FeasiblePDLocs &feasibleDropoffs,
                                           const PDDistances &pdDistances,
                                           const RequestState &requestState, const PDLocs &pdLocs,
                                           InternalTaxiResult &result,
                                           stats::OrdAssignmentsPerformanceStats &stats) const {
            KaRRiTimer timer;
            int numAssignmentsTried = 0;

            // Try pairs with pickup at existing stop
            Assignment asgn;

            for (const auto &stopId: feasiblePickups.getStopIdsWithRelevantPDLocs()) {
                if (!feasibleDropoffs.hasFeasiblePDLocs(stopId))
                    continue;
                const int vehId = routeState.vehicleIdOf(stopId);
                const int stopIdx = routeState.stopPositionOf(stopId);

                if (stopIdx == 0 || stopIdx == routeState.numStopsOf(vehId) - 1)
                    continue;

                const auto nextStopLoc = routeState.stopLocationsFor(vehId)[stopIdx + 1];
                asgn.vehicle = &fleet[vehId];
                asgn.pickupStopIdx = stopIdx;
                asgn.dropoffStopIdx = stopIdx;

                // Compute lower bound on cost and check against best known cost.
                asgn.distToPickup = feasiblePickups.minDistToPDLocFor(stopId);
                asgn.distToDropoff = pdDistances.getMinDirectDistance();
                asgn.distFromDropoff = feasibleDropoffs.minDistFromPDLocToNextStopOf(stopId);
                const int minCost = calculator.calcCostLowerBoundForOrdinaryPairedAssignment(asgn, requestState);
                if (minCost >= result.getBestCost())
                    continue;

                // Try paired assignment for every combination of relevant pickup and dropoff
                for (const auto &[dropoffId, _, distFromDropoff]: feasibleDropoffs.feasiblePdLocsFor(stopId)) {
                    asgn.dropoff = pdLocs.dropoffs[dropoffId];

                    // if dropoff coincides with the following stop, an ordinary non-paired assignment with dropoffIndex = pickupIndex + 1 will cover this case
                    if (nextStopLoc == asgn.dropoff.loc)
                        continue;

                    asgn.distFromDropoff = distFromDropoff;
                    for (const auto &[pickupId, distToPickup, _]: feasiblePickups.feasiblePdLocsFor(stopId)) {
                        asgn.pickup = pdLocs.pickups[pickupId];
                        if (asgn.pickup.loc == asgn.dropoff.loc)
                            continue;

                        asgn.distToPickup = distToPickup;

                        KASSERT(asgn.distToPickup < INFTY && asgn.distFromDropoff < INFTY);
                        asgn.distToDropoff = pdDistances.getDirectDistance(asgn.pickup, asgn.dropoff);
                        result.tryAssignmentWithKnownCost(asgn, calculator.calc(asgn, requestState));
                        ++numAssignmentsTried;
                    }
                }
            }

            const auto pairedTime = timer.elapsed<std::chrono::nanoseconds>();
            stats.tryPairedAssignmentsTime += pairedTime;
            stats.numPairedAssignmentsTried += numAssignmentsTried;
        }

        const Fleet &fleet;
        CostCalculator calculator;
        const RouteState &routeState;
    };
}
