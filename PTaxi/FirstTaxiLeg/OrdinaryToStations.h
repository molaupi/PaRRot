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

namespace parrot {
    using namespace karri;

    template<typename StationsInEllipseT, typename StationDistancesT>
    class OrdinaryToStations {
    public:
        OrdinaryToStations(const Fleet &fleet, const RouteState &routeState)
            : fleet(fleet),
              calculator(routeState),
              routeState(routeState) {
        }

        void enumerateAssignments(const RequestState &requestState, const PDLocs &pdLocs,
                                  const RelevantPDLocs &relPickups,
                                  const PTStations &stations, StationsInEllipseT &stationsInEllipse,
                                  StationDistancesT &stationDistances,
                                  stats::OrdAssignmentsPerformanceStats &stats,
                                  FirstTaxiLegResult &firstTaxiLegResult,
                                  const int externalUpperBoundCost) {
            for (const auto &vehId: relPickups.getVehiclesWithRelevantPDLocs()) {
                KASSERT(relPickups.hasRelevantSpotsFor(vehId));
                ++stats.numCandidateVehicles;

                enumerateOrdinaryAssignments(vehId, requestState, pdLocs, relPickups, stations, stationsInEllipse,
                                             stationDistances, stats, firstTaxiLegResult, externalUpperBoundCost);
                enumeratePairedAssignments(vehId, requestState, pdLocs, relPickups, stations, stationsInEllipse,
                                           stationDistances, stats, firstTaxiLegResult, externalUpperBoundCost);
            }
        }

    private:
        void enumerateOrdinaryAssignments(const int vehId, const RequestState &requestState, const PDLocs &pdLocs,
                                          const RelevantPDLocs &relPickups,
                                          const PTStations &stations, StationsInEllipseT &stationsInEllipse,
                                          StationDistancesT &stationDistances,
                                          stats::OrdAssignmentsPerformanceStats &stats,
                                          FirstTaxiLegResult &firstTaxiLegResult,
                                          const int externalUpperBoundCost) {
            using namespace time_utils;
            static int stopTime = InputConfig::getInstance().stopTime;

            const int reqTime = requestState.originalRequest.requestTime;
            KaRRiTimer timer;

            KASSERT(relPickups.hasRelevantSpotsFor(vehId));
            const auto numStops = routeState.numStopsOf(vehId);
            const auto stopLocations = routeState.stopLocationsFor(vehId);
            const auto schedArrTimes = routeState.schedArrTimesFor(vehId);
            const auto schedDepTimes = routeState.schedDepTimesFor(vehId);
            const auto maxArrTimes = routeState.maxArrTimesFor(vehId);
            const auto occs = routeState.occupanciesFor(vehId);

            Assignment asgn(&fleet[vehId]);

            for (const auto &pickupEntry: relPickups.relevantSpotsFor(vehId)) {
                const int i = pickupEntry.stopIndex;
                asgn.pickup = pdLocs.pickups[pickupEntry.pdId];
                asgn.pickupStopIdx = i;
                asgn.distToPickup = pickupEntry.distToPDLoc;
                asgn.distFromPickup = pickupEntry.distFromPDLocToNextStop;

                using namespace time_utils;
                const int depTimeAtPickup = getActualDepTimeAtPickup(
                    vehId, i, pickupEntry.distToPDLoc, asgn.pickup, requestState, routeState);
                const int initialPickupDetour = calcInitialPickupDetour(
                    vehId, i, INVALID_INDEX, depTimeAtPickup,
                    pickupEntry.distFromPDLocToNextStop, requestState, routeState);

                // Compute smallest stop index after pickup at which capacity of vehicle would be broken (end of route
                // if never broken). Dropoff has to be made before this index.
                int capacityBrokenIndex = asgn.pickupStopIdx;
                const int cap = asgn.vehicle->capacity;
                while (capacityBrokenIndex < numStops - 1 && occs[capacityBrokenIndex] + requestState.originalRequest.numRiders <= cap) {
                    ++capacityBrokenIndex;
                }
                // Need to allow dropoff at capacityBrokenIndex, since dropoff may be made at stop.
                const int endDropoffIndex = std::min(capacityBrokenIndex + 1, numStops - 1);

                // Iterates through stops (pickup's stop index; last stop) and try to find a station as a dropoff.
                for (int j = i + 1; j < endDropoffIndex; ++j) {
                    asgn.dropoffStopIdx = j;
                    const auto curStopId = routeState.stopIdsFor(vehId)[j];
                    const auto curStopLoc = stopLocations[j];
                    const auto nextStopLoc = stopLocations[j + 1];
                    const int maxDetourAtJ = maxArrTimes[j + 1] - schedArrTimes[j + 1];
                    const int lengthOfLegJ = calcLengthOfLegStartingAt(j, vehId, routeState);

                    const int detourUntilArrAtJ =
                            calcResidualPickupDetour(vehId, i, j, initialPickupDetour, routeState);
                    KASSERT(schedArrTimes[j] >= reqTime);
                    const int arrTimeAtJ = schedArrTimes[j] + detourUntilArrAtJ;
                    const int minTripTime = arrTimeAtJ - reqTime;

                    const int detourUntilDepAtJ =
                            calcResidualPickupDetour(vehId, i, j + 1, initialPickupDetour, routeState);
                    const int depTimeAtJ = std::max(schedDepTimes[j], schedArrTimes[j] + detourUntilArrAtJ + stopTime);
                    KASSERT(depTimeAtJ == schedDepTimes[j] + detourUntilDepAtJ);
                    const int addedTripTimeUntilJ = calcAddedTripTimeInInterval(vehId, i, j,
                        initialPickupDetour, routeState);

                    // for each station in ellipse, try assignment
                    for (const auto &entry: stationsInEllipse.getStationsInEllipse(curStopId)) {
                        const auto &station = stations[entry.targetId];

                        if (nextStopLoc == station.vehEdgeId) {
                            // If the station is at the location of the following stop, do not try an assignment here as it would
                            // introduce a new stop after dropoffIndex that is at the same location as dropoffIndex + 1.
                            // Instead, this will be dealt with as an assignment at dropoffIndex + 1 afterwards.
                            continue;
                        }

                        if (asgn.pickup.loc == station.vehEdgeId)
                            continue;

                        // Stations in ellipse are sorted by detour so that we can break after the first station
                        // that has a detour that is large enough to lead to a total cost that is above the external upper bound.
                        // (We use a lower bound that does not consider the trip time from stop i to the station as
                        // this is not respected in the order of stations).
                        const bool stationAtExistingStop = curStopLoc == station.vehEdgeId;
                        int detourRightAfterStation = detourUntilDepAtJ + (stationAtExistingStop
                                                                               ? 0
                                                                               : entry.distFromStopToStation + stopTime
                                                                                   +
                                                                                   entry.distFromStationToStop -
                                                                                   lengthOfLegJ);
                        if (detourRightAfterStation > maxDetourAtJ)
                            break;
                        const int totalResDetour = calcResidualTotalDetourForStopAfterDropoff(
                            vehId, j, numStops - 1, detourRightAfterStation, routeState);
                        const int addedTripTime = addedTripTimeUntilJ + calcAddedTripTimeAffectedByPickupAndDropoff(
                                                      vehId, j, detourRightAfterStation, routeState);
                        const int minCost = calculator.calcMinCostForOrdinaryToStations(
                            totalResDetour, minTripTime, addedTripTime);
                        if (minCost >= externalUpperBoundCost)
                            break;

                        asgn.dropoff = {
                            station.stationId, // PDLoc ID
                            station.vehEdgeId, // Location in road network
                            station.psgEdgeId, // Location in passenger road network
                            station.walkingTimeFromVehEdge, // Walking time from vehEdge to station
                            0, // Dummy vehicle driving time from this dropoff to the destination
                            0, // Dummy vehicle driving time from destination to this dropoff,
                            true
                        };

                        ++stats.numNonPairedAssignmentsTried;
                        asgn.distToDropoff = entry.distFromStopToStation;
                        asgn.distFromDropoff = entry.distFromStationToStop;

                        // requestState.tryAssignmentWithKnownCost(asgn, calculator.calc(asgn, requestState));
                        const int arrivalTime = (stationAtExistingStop ? arrTimeAtJ : depTimeAtJ + asgn.distToDropoff) + asgn.dropoff.walkingDist;
                        KASSERT(arrivalTime == calcArrivalTime(asgn, requestState, routeState));
                        firstTaxiLegResult.tryAssignmentForStation(
                            station.stationId, asgn, calculator.calc(asgn, requestState),
                            arrivalTime, ORDINARY);
                    }
                }
            }

            stats.tryNonPairedAssignmentsTime += timer.elapsed<std::chrono::nanoseconds>();
        }

        void enumeratePairedAssignments(const int vehId, const RequestState &requestState, const PDLocs &pdLocs,
                                        const RelevantPDLocs &relPickups,
                                        const PTStations &stations, StationsInEllipseT &stationsInEllipse,
                                        StationDistancesT &stationDistances,
                                        stats::OrdAssignmentsPerformanceStats &stats,
                                        FirstTaxiLegResult &firstTaxiLegResult,
                                        const int externalUpperBoundCost) {
            KASSERT(relPickups.hasRelevantSpotsFor(vehId));
            static int stopTime = InputConfig::getInstance().stopTime;
            KaRRiTimer timer;
            const int reqTime = requestState.originalRequest.requestTime;
            const auto numStops = routeState.numStopsOf(vehId);
            const auto stopLocations = routeState.stopLocationsFor(vehId);
            const auto schedArrTimes = routeState.schedArrTimesFor(vehId);
            const auto schedDepTimes = routeState.schedDepTimesFor(vehId);
            const auto maxArrTimes = routeState.maxArrTimesFor(vehId);

            Assignment asgn(&fleet[vehId]);
            asgn.distFromPickup = 0;

            const auto &entries = relPickups.relevantSpotsFor(vehId);
            if (entries.empty())
                return;
            int curStopIdx = entries[0].stopIndex;
            int beginEntryIdx = 0;
            using namespace time_utils;
            for (int entryIdx = 0; entryIdx <= entries.size(); ++entryIdx) {
                if (entryIdx < entries.size() && entries[entryIdx].stopIndex == curStopIdx)
                    continue;
                const int endEntryIdx = entryIdx;

                // All entries at indices [beginEntryIdx, endEntryIdx) have the same stop index curStopIdx.
                // Consider stations in ellipse for curStopIdx and the paired assignments with the entries.
                const auto curStopId = routeState.stopIdsFor(vehId)[curStopIdx];
                const int nextStopLoc = stopLocations[curStopIdx + 1];

                asgn.pickupStopIdx = curStopIdx;
                asgn.dropoffStopIdx = curStopIdx;
                for (const auto &entry: stationsInEllipse.getStationsInEllipse(curStopId)) {
                    const auto &station = stations[entry.targetId];

                    if (nextStopLoc == station.vehEdgeId) {
                        // If the station is at the location of the following stop, do not try an assignment here as it would
                        // introduce a new stop after dropoffIndex that is at the same location as dropoffIndex + 1.
                        // Instead, this will be dealt with as an assignment at dropoffIndex + 1 afterwards.
                        continue;
                    }
                    if (!stationDistances.hasValidDistances(station.stationId))
                        continue;

                    // The pickup with the smallest arrival time at the station has the smallest arrival time and
                    // non-trip cost, so its label Pareto-dominates all other pickups. Find it and only test the
                    // assignment with this pickup.
                    int bestPickupIdx = INVALID_INDEX;
                    int bestArrivalTime = INFTY;
                    for (int i = beginEntryIdx; i < endEntryIdx; ++i) {
                        const auto &pickupEntry = entries[i];
                        const auto &p = pdLocs.pickups[pickupEntry.pdId];

                        const int depTimeAtPickup = getActualDepTimeAtPickup(
                            vehId, curStopIdx, pickupEntry.distToPDLoc, p, requestState, routeState);
                        const int distPickupToStation = stationDistances.getDistance(station.stationId, p.id);
                        const int arrivalTime = depTimeAtPickup + distPickupToStation + station.walkingTimeFromVehEdge;
                        if (arrivalTime < bestArrivalTime) {
                            bestArrivalTime = arrivalTime;
                            bestPickupIdx = i;
                        }
                    }
                    if (bestPickupIdx == INVALID_INDEX)
                        continue;

                    // Try the best pickup entry for this station.
                    const auto &pickupEntry = entries[bestPickupIdx];
                    asgn.pickup = pdLocs.pickups[pickupEntry.pdId];
                    asgn.dropoff = {
                        station.stationId, // PDLoc ID
                        station.vehEdgeId, // Location in road network
                        station.psgEdgeId, // Location in passenger road network
                        station.walkingTimeFromVehEdge, // Walking time from vehEdge to station
                        0, // Dummy vehicle driving time from this dropoff to the destination
                        0, // Dummy vehicle driving time from destination to this dropoff,
                        true
                    };
                    asgn.distToPickup = pickupEntry.distToPDLoc;
                    asgn.distToDropoff = stationDistances.getDistance(station.stationId, asgn.pickup.id);
                    asgn.distFromDropoff = entry.distFromStationToStop;

                    ++stats.numPairedAssignmentsTried;
                    KASSERT(bestArrivalTime == calcArrivalTime(asgn, requestState, routeState));
                    firstTaxiLegResult.tryAssignmentForStation(
                        station.stationId, asgn, calculator.calc(asgn, requestState), bestArrivalTime, ORDINARY);
                }

                // Following entries are at new stop index
                if (entryIdx < entries.size()) {
                    beginEntryIdx = entryIdx;
                    curStopIdx = entries[entryIdx].stopIndex;
                }
            }

            // const auto &entries = relPickups.relevantSpotsFor(vehId);
            // if (entries.empty())
            //     return;
            // int curStopIdx = entries[0].stopIndex;
            // int beginEntryIdx = 0;
            // using namespace time_utils;
            // std::vector<int> depTimesAtPickups(entries.size(), INFTY);
            // for (int entryIdx = 0; entryIdx <= entries.size(); ++entryIdx) {
            //     // Compute the actual departure time at pickup for the next entry (if any) and store it in depTimesAtPickups.
            //     if (entryIdx < entries.size()) {
            //         depTimesAtPickups[entryIdx] = getActualDepTimeAtPickup(
            //             vehId, curStopIdx, entries[entryIdx].distToPDLoc, pdLocs.pickups[entries[entryIdx].pdId], requestState, routeState);
            //         if (entries[entryIdx].stopIndex == curStopIdx)
            //             continue;
            //     }
            //     const int endEntryIdx = entryIdx;
            //
            //     // All entries at indices [beginEntryIdx, endEntryIdx) have the same stop index curStopIdx.
            //     // Consider stations in ellipse for curStopIdx and the paired assignments with the entries.
            //
            //     const auto lengthOfLeg = calcLengthOfLegStartingAt(curStopIdx, vehId, routeState);
            //
            //     int minDistToPickup = INFTY;
            //     int minDepTimeAtPickup = INFTY;
            //     for (int i = beginEntryIdx; i < endEntryIdx; ++i) {
            //         const auto &pickupEntry = entries[i];
            //         minDistToPickup = std::min(minDistToPickup, pickupEntry.distToPDLoc);
            //         minDepTimeAtPickup = std::min(minDepTimeAtPickup, depTimesAtPickups[i]);
            //     }
            //
            //     const auto curStopId = routeState.stopIdsFor(vehId)[curStopIdx];
            //     const int nextStopLoc = stopLocations[curStopIdx + 1];
            //     const int remainingLeeway = routeState.leewayOfLegStartingAt(curStopId) - minDistToPickup -
            //                                 InputConfig::getInstance().stopTime;
            //
            //     for (const auto &entry: stationsInEllipse.getStationsInEllipse(curStopId)) {
            //         const auto &station = stations[entry.targetId];
            //
            //         if (nextStopLoc == station.vehEdgeId) {
            //             // If the station is at the location of the following stop, do not try an assignment here as it would
            //             // introduce a new stop after dropoffIndex that is at the same location as dropoffIndex + 1.
            //             // Instead, this will be dealt with as an assignment at dropoffIndex + 1 afterwards.
            //             continue;
            //         }
            //
            //         const int minPickupToStationDist = stationDistances.getMinDistanceForStation(station.stationId);
            //
            //         if (minPickupToStationDist + entry.distFromStationToStop > remainingLeeway)
            //             continue;
            //
            //         // Compute a lower bound on the arrival time and non-trip cost to this station that can be reached
            //         // with an ordinary paired assignment with any of the pickup entries.
            //         // If these lower bounds are dominated by an existing label at the station, we skip all pickup
            //         // entries for this station.
            //         const int minArrTimeAtStation = minDepTimeAtPickup + minPickupToStationDist;
            //         const int minDetourRightAfterStation = minArrTimeAtStation + stopTime + entry.distFromStationToStop - schedDepTimes[curStopIdx] - lengthOfLeg;
            //         const int totalResDetour = calcResidualTotalDetourForStopAfterDropoff(
            //             vehId, curStopIdx, numStops - 1, minDetourRightAfterStation, routeState);
            //         const int addedTripTime = calcAddedTripTimeAffectedByPickupAndDropoff(
            //             vehId, curStopIdx, minDetourRightAfterStation, routeState);
            //         const int minNonTripCost = calculator.calcMinCostForOrdinaryToStations(
            //             totalResDetour, 0, addedTripTime);
            //         if (firstTaxiLegResult.isLabelDominated(station.stationId, minArrTimeAtStation, minNonTripCost))
            //             continue;
            //
            //         // Try all pickup entries for this station.
            //         asgn.dropoff = {
            //             station.stationId, // PDLoc ID
            //             station.vehEdgeId, // Location in road network
            //             station.psgEdgeId, // Location in passenger road network
            //             station.walkingTimeFromVehEdge, // Walking time from vehEdge to station
            //             0, // Dummy vehicle driving time from this dropoff to the destination
            //             0, // Dummy vehicle driving time from destination to this dropoff,
            //             true
            //         };
            //
            //         asgn.distFromDropoff = entry.distFromStationToStop;
            //
            //         for (int i = beginEntryIdx; i < endEntryIdx; ++i) {
            //             const auto &pickupEntry = entries[i];
            //             asgn.pickup = pdLocs.pickups[pickupEntry.pdId];
            //             asgn.pickupStopIdx = curStopIdx;
            //             asgn.distToPickup = pickupEntry.distToPDLoc;
            //             asgn.distToDropoff = stationDistances.getDistance(station.stationId, asgn.pickup.id);
            //
            //             ++stats.numPairedAssignmentsTried;
            //             const int depTimeAtPickup = depTimesAtPickups[i];
            //             const int arrivalTime = depTimeAtPickup + asgn.distToDropoff + asgn.dropoff.walkingDist;
            //             KASSERT(arrivalTime == calcArrivalTime(asgn, requestState, routeState));
            //             firstTaxiLegResult.tryAssignmentForStation(
            //                 station.stationId, asgn, calculator.calc(asgn, requestState), arrivalTime, ORDINARY);
            //         }
            //     }
            //
            //     // Following entries are at new stop index
            //     if (entryIdx < entries.size()) {
            //         beginEntryIdx = entryIdx;
            //         curStopIdx = entries[entryIdx].stopIndex;
            //     }
            // }

            stats.tryPairedAssignmentsTime += timer.elapsed<std::chrono::nanoseconds>();
        }

        const Fleet &fleet;
        CostCalculator calculator;
        const RouteState &routeState;
    };
}
