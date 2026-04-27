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

                // Iterates through stops (pickup's stop index; last stop) and try to find a station as a dropoff.
                for (int j = i + 1; j < routeState.numStopsOf(vehId) - 1; ++j) {
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
                            0, // Walking time from this dropoff to destination
                            0, // Vehicle driving time from this dropoff to the destination
                            0, // Vehicle driving time from destination to this dropoff
                            true
                        };

                        ++stats.numNonPairedAssignmentsTried;
                        asgn.distToDropoff = entry.distFromStopToStation;
                        asgn.distFromDropoff = entry.distFromStationToStop;

                        // requestState.tryAssignmentWithKnownCost(asgn, calculator.calc(asgn, requestState));
                        const int arrivalTime = stationAtExistingStop ? arrTimeAtJ : depTimeAtJ + asgn.distToDropoff;
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

            for (const auto &pickupEntry: relPickups.relevantSpotsFor(vehId)) {
                const int i = pickupEntry.stopIndex;
                const int j = i;
                const int nextStopLoc = stopLocations[j + 1];
                asgn.pickup = pdLocs.pickups[pickupEntry.pdId];
                asgn.pickupStopIdx = i;
                asgn.dropoffStopIdx = j;
                asgn.distToPickup = pickupEntry.distToPDLoc;

                using namespace time_utils;
                const auto lengthOfLegJ = calcLengthOfLegStartingAt(j, vehId, routeState);
                const int depTimeAtPickup = getActualDepTimeAtPickup(
                    vehId, i, pickupEntry.distToPDLoc, asgn.pickup, requestState, routeState);

                const int minPickupToStationDist = stationDistances.getMinDistanceForPDLoc(asgn.pickup.id);
                const auto timeUntilDep = depTimeAtPickup - schedDepTimes[j];
                const int minDetour = timeUntilDep + std::max(pickupEntry.distFromPDLocToNextStop,
                                                              minPickupToStationDist) + stopTime - lengthOfLegJ;

                const int minTripTime = depTimeAtPickup + minPickupToStationDist - reqTime;

                const auto curStopId = routeState.stopIdsFor(vehId)[j];
                const int remainingLeeway = routeState.leewayOfLegStartingAt(curStopId) - asgn.distToPickup -
                                            InputConfig::getInstance().stopTime;

                // for each station in ellipse, try assignment
                for (const auto &entry: stationsInEllipse.getStationsInEllipse(curStopId)) {
                    const auto &station = stations[entry.targetId];

                    const int pickupToStationDist = stationDistances.getDistance(station.stationId, asgn.pickup.id);
                    if (pickupToStationDist + entry.distFromStationToStop > remainingLeeway)
                        continue;

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
                    int detourRightAfterStation = std::max(minDetour, entry.distFromStopToStation + stopTime +
                                                                      entry.distFromStationToStop - lengthOfLegJ);
                    const int totalResDetour = calcResidualTotalDetourForStopAfterDropoff(
                        vehId, j, numStops - 1, detourRightAfterStation, routeState);
                    const int addedTripTime = calcAddedTripTimeAffectedByPickupAndDropoff(
                        vehId, j, detourRightAfterStation, routeState);
                    const int minCost = calculator.calcMinCostForOrdinaryToStations(
                        totalResDetour, minTripTime, addedTripTime);
                    if (minCost >= externalUpperBoundCost)
                        break;

                    asgn.dropoff = {
                        station.stationId, // PDLoc ID
                        station.vehEdgeId, // Location in road network
                        station.psgEdgeId, // Location in passenger road network
                        0, // Walking time from this dropoff to destination
                        0, // Vehicle driving time from this dropoff to the destination
                        0, // Vehicle driving time from destination to this dropoff
                        true
                    };

                    ++stats.numPairedAssignmentsTried;
                    asgn.distToDropoff = pickupToStationDist;
                    asgn.distFromDropoff = entry.distFromStationToStop;

                    // requestState.tryAssignmentWithKnownCost(asgn, calculator.calc(asgn, requestState));
                    const int arrivalTime = depTimeAtPickup + pickupToStationDist;
                    KASSERT(arrivalTime == calcArrivalTime(asgn, requestState, routeState));
                    firstTaxiLegResult.tryAssignmentForStation(
                        station.stationId, asgn, calculator.calc(asgn, requestState), arrivalTime, ORDINARY);
                }
            }
            stats.tryPairedAssignmentsTime += timer.elapsed<std::chrono::nanoseconds>();
        }

        const Fleet &fleet;
        CostCalculator calculator;
        const RouteState &routeState;
    };
}
