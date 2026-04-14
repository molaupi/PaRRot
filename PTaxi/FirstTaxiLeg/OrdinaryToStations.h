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
            using namespace time_utils;

            int numNonPairedAssignmentsTried = 0;
            int numPairedAssignmentsTried = 0;
            int numCandidateVehicles = 0;
            KaRRiTimer pairedTimer;
            int64_t pairedTime = 0;
            const int reqTime = requestState.originalRequest.requestTime;
            KaRRiTimer timer;

            for (const auto &vehId: relPickups.getVehiclesWithRelevantPDLocs()) {
                KASSERT(relPickups.hasRelevantSpotsFor(vehId));
                ++numCandidateVehicles;
                const auto numStops = routeState.numStopsOf(vehId);
                const auto stopLocations = routeState.stopLocationsFor(vehId);
                const auto schedArrTimes = routeState.schedArrTimesFor(vehId);

                Assignment asgn(&fleet[vehId]);

                for (const auto &pickupEntry: relPickups.relevantSpotsFor(vehId)) {
                    const int i = pickupEntry.stopIndex;
                    asgn.pickup = pdLocs.pickups[pickupEntry.pdId];
                    asgn.pickupStopIdx = i;
                    asgn.distToPickup = pickupEntry.distToPDLoc;

                    using namespace time_utils;
                    const int depTimeAtPickup = getActualDepTimeAtPickup(
                        vehId, i, pickupEntry.distToPDLoc, asgn.pickup, requestState, routeState);
                    const int initialPickupDetour = calcInitialPickupDetour(
                        vehId, i, INVALID_INDEX, depTimeAtPickup,
                        pickupEntry.distFromPDLocToNextStop, requestState, routeState);

                    // Iterates through stops [pickup's stop index; last stop) and try to find a station as a dropoff.
                    pairedTimer.restart();
                    for (int j = i; j < routeState.numStopsOf(vehId) - 1; ++j) {
                        const auto curStopId = routeState.stopIdsFor(vehId)[j];
                        const int curLeeway = routeState.leewayOfLegStartingAt(curStopId);


                        const int detourUntilArrAtJ =
                                j == i ? 0 : calcResidualPickupDetour(vehId, i, j, initialPickupDetour, routeState);
                        KASSERT(j == i || schedArrTimes[j] >= reqTime);
                        const int minTripTime
                                = (j == i
                                       ? depTimeAtPickup + stationDistances.getMinDistanceForPDLoc(asgn.pickup.id)
                                       : schedArrTimes[j] + detourUntilArrAtJ)
                                  - reqTime;

                        const int detourUntilDepAtJ =
                                j == i ? 0 : calcResidualPickupDetour(vehId, i, j + 1, initialPickupDetour, routeState);
                        const int addedTripTimeUntilJ = calcAddedTripTimeInInterval(vehId, i, j,
                            initialPickupDetour, routeState);

                        // for each station in ellipse, try assignment
                        for (const auto &stationEntry: stationsInEllipse.getStationsInEllipse(curStopId)) {
                            const auto &station = stations[stationEntry.targetId];

                            if (j + 1 < numStops && stopLocations[j + 1] == station.vehEdgeId) {
                                // If the station is at the location of the following stop, do not try an assignment here as it would
                                // introduce a new stop after dropoffIndex that is at the same location as dropoffIndex + 1.
                                // Instead, this will be dealt with as an assignment at dropoffIndex + 1 afterwards.
                                continue;
                            }

                            // Stations in ellipse are sorted by detour so that we can break after the first station
                            // that has a detour that is large enough to lead to a total cost that is above the external upper bound.
                            // (We use a lower bound that does not consider the trip time from stop i to the station as
                            // this is not respected in the order of stations).
                            const bool stationAtExistingStop = j != i && stopLocations[j] == station.vehEdgeId;
                            int detourRightAfterStation = detourUntilDepAtJ + calcInitialDropoffDetour(
                                                              vehId, j, stationEntry.distFromStopToStation,
                                                              stationEntry.distFromStationToStop,
                                                              stationAtExistingStop, routeState);
                            if (detourRightAfterStation > curLeeway)
                                break;
                            if (i == j)
                                detourRightAfterStation = std::max(detourRightAfterStation, initialPickupDetour);
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
                                0 // Vehicle driving time from destination to this dropoff
                            };

                            asgn.dropoffStopIdx = j;

                            asgn.distFromDropoff = stationEntry.distFromStationToStop;

                            // In case of paired assignment:
                            if (asgn.pickupStopIdx == asgn.dropoffStopIdx) {
                                ++numPairedAssignmentsTried;
                                asgn.distFromPickup = 0;
                                asgn.distToDropoff = stationDistances.getDistance(asgn.dropoff.id, asgn.pickup.id);
                            } else {
                                ++numNonPairedAssignmentsTried;
                                asgn.distFromPickup = pickupEntry.distFromPDLocToNextStop;
                                asgn.distToDropoff = stationEntry.distFromStopToStation;
                            }

                            // requestState.tryAssignmentWithKnownCost(asgn, calculator.calc(asgn, requestState));
                            firstTaxiLegResult.tryAssignmentWithKnownCostForStation(
                                station.stationId, asgn, calculator.calc(asgn, requestState), InsertionType::ORDINARY);
                        }
                        if (i == j)
                            pairedTime += pairedTimer.elapsed<std::chrono::nanoseconds>();
                    }
                }
            }

            const int64_t tryAssignmentsTime = timer.elapsed<std::chrono::nanoseconds>();
            stats.numCandidateVehicles += numCandidateVehicles;
            stats.numNonPairedAssignmentsTried += numNonPairedAssignmentsTried;
            stats.numPairedAssignmentsTried += numPairedAssignmentsTried;
            const auto nonPairedTime = tryAssignmentsTime - pairedTime;
            stats.tryPairedAssignmentsTime += pairedTime;
            stats.tryNonPairedAssignmentsTime += nonPairedTime;
        }

    private:
        const Fleet &fleet;
        CostCalculator calculator;
        const RouteState &routeState;
    };
}
