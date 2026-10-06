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

#include "../../include/KARRI/DataStructures/Pareto/ParetoBag.h"

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
                                  const FeasiblePDLocs &feasiblePickups,
                                  const PTStations &stations, StationsInEllipseT &stationsInEllipse,
                                  StationDistancesT &stationDistances,
                                  stats::OrdAssignmentsPerformanceStats &stats,
                                  FirstTaxiLegResult &firstTaxiLegResult,
                                  const int externalUpperBoundCost) {
            for (const auto &vehId: relPickups.getVehiclesWithRelevantPDLocs()) {
                KASSERT(relPickups.hasRelevantSpotsFor(vehId));
                ++stats.numCandidateVehicles;

                enumerateOrdinaryAssignments(vehId, requestState, pdLocs, relPickups, stations, stationsInEllipse,
                                             stats, firstTaxiLegResult, externalUpperBoundCost);
            }

            for (const auto &stopId: feasiblePickups.getStopIdsWithRelevantPDLocs()) {
                enumeratePairedAssignments(stopId, requestState, pdLocs, feasiblePickups,
                                           stations, stationsInEllipse, stationDistances, stats, firstTaxiLegResult, externalUpperBoundCost);
            }
        }

    private:
        void enumerateOrdinaryAssignments(const int vehId, const RequestState &requestState, const PDLocs &pdLocs,
                                          const RelevantPDLocs &relPickups,
                                          const PTStations &stations, StationsInEllipseT &stationsInEllipse,
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
                while (capacityBrokenIndex < numStops - 1 && occs[capacityBrokenIndex] + requestState.originalRequest.
                       numRiders <= cap) {
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
                        const int arrivalTime = (stationAtExistingStop ? arrTimeAtJ : depTimeAtJ + asgn.distToDropoff) +
                                                asgn.dropoff.walkingDist;
                        KASSERT(arrivalTime == calcArrivalTime(asgn, requestState, routeState));
                        firstTaxiLegResult.tryAssignmentForStation(
                            station.stationId, asgn, calculator.calc(asgn, requestState),
                            arrivalTime, ORDINARY);
                    }
                }
            }

            stats.tryNonPairedAssignmentsTime += timer.elapsed<std::chrono::nanoseconds>();
        }

        void enumeratePairedAssignments(const int stopId, const RequestState &requestState, const PDLocs &pdLocs,
                                        const FeasiblePDLocs &feasiblePickups,
                                        const PTStations &stations, StationsInEllipseT &stationsInEllipse,
                                        StationDistancesT &stationDistances,
                                        stats::OrdAssignmentsPerformanceStats &stats,
                                        FirstTaxiLegResult &firstTaxiLegResult,
                                        const int externalUpperBoundCost) {
            KaRRiTimer timer;
            const int vehId = routeState.vehicleIdOf(stopId);
            const int stopIdx = routeState.stopPositionOf(stopId);
            if (stopIdx == 0 || stopIdx == routeState.numStopsOf(vehId) - 1)
                return;
            const int nextStopLoc = routeState.stopLocationsFor(vehId)[stopIdx + 1];

            Assignment asgn(&fleet[vehId]);
            asgn.pickupStopIdx = stopIdx;
            asgn.dropoffStopIdx = stopIdx;
            asgn.distFromPickup = 0;

            struct ParetoPickup {
                int idx; // index in feasiblePickupsForStop
                int fixedCost; // walking cost + wait violation cost
                int arrivalTimeAtStation;

                static bool dominates(const ParetoPickup &a, const ParetoPickup &b) {
                    return a.fixedCost <= b.fixedCost && a.arrivalTimeAtStation <= b.arrivalTimeAtStation &&
                           (a.fixedCost < b.fixedCost || a.arrivalTimeAtStation < b.arrivalTimeAtStation);
                }
            };
            ParetoBag<ParetoPickup> paretoPickups;

            const int minDistToPickup = feasiblePickups.minDistToPDLocFor(stopId);
            const auto &feasiblePickupsForStop = feasiblePickups.feasiblePdLocsFor(stopId);
            const int reqTime = requestState.originalRequest.requestTime;

            for (const auto &entry: stationsInEllipse.getStationsInEllipse(stopId)) {
                const auto &station = stations[entry.targetId];

                if (nextStopLoc == station.vehEdgeId) {
                    // If the station is at the location of the following stop, do not try an assignment here as it would
                    // introduce a new stop after dropoffIndex that is at the same location as dropoffIndex + 1.
                    // Instead, this will be dealt with as an assignment at dropoffIndex + 1 afterwards.
                    continue;
                }
                if (!stationDistances.hasValidDistances(station.stationId))
                    continue;

                asgn.dropoff = {
                    station.stationId, // PDLoc ID
                    station.vehEdgeId, // Location in road network
                    station.psgEdgeId, // Location in passenger road network
                    station.walkingTimeFromVehEdge, // Walking time from vehEdge to station
                    0, // Dummy vehicle driving time from this dropoff to the destination
                    0, // Dummy vehicle driving time from destination to this dropoff,
                    true
                };
                asgn.distFromDropoff = entry.distFromStationToStop;

                // Compute lower bound on cost and arrival time at station with ordinary paired assignment
                asgn.distToPickup = minDistToPickup;
                asgn.distToDropoff = stationDistances.getMinDistanceForStation(station.stationId);
                const auto [minArrTime, minNonTripCost] = calculator.calcArrivalTimeAndNonTripCostLowerBoundForPairedToStationAssignment(
                    asgn, requestState);
                if (minArrTime == INFTY || minNonTripCost == INFTY)
                    continue;
                const int minFullCost = minNonTripCost + CostCalculator::CostFunction::calcTripCost(minArrTime - reqTime);
                if (minFullCost > externalUpperBoundCost)
                    continue;
                if (firstTaxiLegResult.isLabelDominated(station.stationId, minNonTripCost, minArrTime))
                    continue;

                // We only have to consider pickups that are Pareto-optimal with respect to fixed cost at the pickup
                // (walking cost and wait violation cost) and arrival time at the station.
                paretoPickups.clear();
                using namespace time_utils;
                for (int i = 0; i < feasiblePickupsForStop.size(); ++i) {
                    const auto &pickupEntry = feasiblePickupsForStop[i];
                    const auto &p = pdLocs.pickups[pickupEntry.pdId];

                    const int depTimeAtPickup = getActualDepTimeAtPickup(
                        vehId, stopIdx, pickupEntry.distToPDLoc, p, requestState, routeState);
                    const int distPickupToStation = stationDistances.getDistance(station.stationId, p.id);
                    const int arrivalTime = depTimeAtPickup + distPickupToStation + station.walkingTimeFromVehEdge;
                    const int fixedCost = CostCalculator::CostFunction::calcWalkingCost(p.walkingDist) +
                                          CostCalculator::CostFunction::calcWaitViolationCost(
                                              depTimeAtPickup, requestState);

                    paretoPickups.addCandidate({i, fixedCost, arrivalTime});
                }

                for (const auto &[idx, _, arrivalTime]: paretoPickups) {
                    // Try the best pickup entry for this station.
                    const auto &pickupEntry = feasiblePickupsForStop[idx];
                    asgn.pickup = pdLocs.pickups[pickupEntry.pdId];

                    asgn.distToPickup = pickupEntry.distToPDLoc;
                    asgn.distToDropoff = stationDistances.getDistance(station.stationId, asgn.pickup.id);

                    ++stats.numPairedAssignmentsTried;
                    KASSERT(arrivalTime == calcArrivalTime(asgn, requestState, routeState));
                    const int cost = calculator.calc(asgn, requestState);
                    if (cost > externalUpperBoundCost)
                        continue;
                    firstTaxiLegResult.tryAssignmentForStation(
                        station.stationId, asgn, cost, arrivalTime, ORDINARY);
                }
            }

            stats.tryPairedAssignmentsTime += timer.elapsed<std::chrono::nanoseconds>();
        }

        const Fleet &fleet;
        CostCalculator calculator;
        const RouteState &routeState;
    };
}
