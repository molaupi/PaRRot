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

#include <KARRI/Algorithms/KaRRi/RequestState/RelevantPDLocs.h>
#include <KARRI/Algorithms/KaRRi/PbnsAssignments/CurVehLocToPickupSearches.h>
#include <Station/StationEntry.h>

namespace parrot {

    using namespace karri;

    // Finds pickup before next stop assignments, i.e. assignments where the pickup is inserted before the vehicle's next
    // stop, which means the vehicle is rerouted at its current location. Dropoff station may be inserted before the next stop or
    // at ordinary locations, i.e. after the next stop but before the last stop.
    // (The case of pickup before next stop and dropoff station after last stop is considered by the DALSToStation.)
    //
    // Works based on filtered relevant pickups before next stop as well as stations in ellipse and station distances to pickups.
    template<typename CurVehLocToPickupSearchesT, typename StationsInEllipseT, typename StationDistancesT>
    class PBNSToStations {
        // Algorithm iterates through combinations of pickups and dropoffs and filters based on lower bound cost.
        // If exact distance from vehicle's current location to a pickup is ever needed, we delay the computation of that
        // distance, so it can be bundled with other such computations. In this case, a continuation marks the combination
        // of pickup and a stop index in the vehicle's route where the exact distance was first needed for this pickup,
        // so the iteration of combinations can continue after the bundled computation of exact distances.
        struct Continuation {
            int pickupId = INVALID_ID;
            int distFromPickup = INFTY;
            int continueStopIndex = INVALID_INDEX;
        };

    public:
        PBNSToStations(CurVehLocToPickupSearchesT &curVehLocToPickupSearches,
                       const Fleet &fleet, const RouteState &routeState)
            : curVehLocToPickupSearches(curVehLocToPickupSearches),
              fleet(fleet),
              calculator(routeState),
              routeState(routeState) {
        }

        void findAssignments(const RequestState &requestState, const PDLocs &pdLocs,
                             const RelevantPDLocs &relPickupsBns,
                             const PTStations &stations, StationsInEllipseT &stationsInEllipse,
                             StationDistancesT &stationDistances,
                             stats::PbnsAssignmentsPerformanceStats &stats,
                             FirstTaxiLegResult &firstTaxiLegResult) {
            numAssignmentsTriedWithPickupBeforeNextStop = 0;

            KaRRiTimer timer;

            int numCandidateVehicles = 0;
            for (const auto &vehId: relPickupsBns.getVehiclesWithRelevantPDLocs()) {
                ++numCandidateVehicles;

                ordinaryContinuations.clear();
                pairedContinuations.clear();

                KASSERT(
                    routeState.occupanciesFor(vehId)[0] + requestState.originalRequest.numRiders <= fleet[vehId].
                    capacity);

                determineNecessaryExactDistances(fleet[vehId], relPickupsBns, stations, stationsInEllipse,
                                                 stationDistances, requestState, pdLocs, firstTaxiLegResult);

                curVehLocToPickupSearches.computeExactDistancesVia(fleet[vehId], pdLocs);

                finishContinuations(fleet[vehId], stations, stationsInEllipse, stationDistances, requestState, pdLocs,
                                    firstTaxiLegResult);
            }

            const auto time = timer.elapsed<std::chrono::nanoseconds>() -
                              curVehLocToPickupSearches.getTotalLocatingVehiclesTimeForRequest();
            stats.tryAssignmentsTime += time;
            stats.numCandidateVehicles += numCandidateVehicles;
            stats.numAssignmentsTried += numAssignmentsTriedWithPickupBeforeNextStop;

            stats.locatingVehiclesTime += curVehLocToPickupSearches.getTotalLocatingVehiclesTimeForRequest();
            stats.numCHSearches += curVehLocToPickupSearches.getTotalNumCHSearchesRunForRequest();
            stats.directCHSearchTime += curVehLocToPickupSearches.getTotalVehicleToPickupSearchTimeForRequest();
        }

        void setExternalCostUpperBound(const int bestCost) {
            externalUpperBoundCost = bestCost;
            upperBoundCost = bestCost;
        }

    private:
        // Filters combinations of pickups and dropoffs using a cost lower bound.
        // If a combination is found for a pickup that cannot be filtered, we need the exact distance from the vehicle
        // location to the pickup.
        // These pickups are added to the queue of curVehLocToPickupSearches and continuations are stored to restart
        // the iteration of combinations for that pickup after the computation of exact distances.
        void determineNecessaryExactDistances(const Vehicle &veh, const RelevantPDLocs &relPickupsBns,
                                              const PTStations &stations, StationsInEllipseT &stationsInEllipse,
                                              StationDistancesT &stationDistances,
                                              const RequestState &requestState, const PDLocs &pdLocs,
                                              const FirstTaxiLegResult &firstTaxiLegResult) {
            Assignment asgn(&veh);

            for (const auto &entry: relPickupsBns.relevantSpotsFor(veh.vehicleId)) {
                asgn.pickup = pdLocs.pickups[entry.pdId];
                asgn.distFromPickup = 0;

                // Distance from stop 0 to pickup is actually a lower bound on the distance from stop 0 via the
                // vehicle's current location to the pickup => we get lower bound costs.
                asgn.distToPickup = entry.distToPDLoc;
                const int distFromPickup = entry.distFromPDLocToNextStop;

                using namespace time_utils;
                const int minDepTimeAtPickup = getActualDepTimeAtPickup(
                    veh.vehicleId, 0, asgn.distToPickup, asgn.pickup, requestState, routeState);

                // For paired assignments before next stop, first try a lower bound with the smallest distance to a station
                const auto lowerBoundCostPairedAssignment = calculator.
                        calcCostLowerBoundForPairedAssignmentBeforeNextStop(
                            veh, asgn.pickup, asgn.distToPickup,
                            stationDistances.getMinDistanceForPDLoc(asgn.pickup.id),
                            distFromPickup, requestState);
                if (lowerBoundCostPairedAssignment < upperBoundCost) {
                    const auto requireExactDistance = tryLowerBoundsForPaired(
                        asgn, minDepTimeAtPickup, stations, stationsInEllipse, stationDistances, requestState, pdLocs,
                        firstTaxiLegResult);
                    if (requireExactDistance) {
                        // In this case some paired assignment before the next stop needs the exact distance to pickup via
                        // the vehicle. Postpone computation of the yet unknown exact distance and the rest of the paired
                        // assignments as well as all assignments with later dropoffs. That way, the exact distances can be
                        // computed in a bundled fashion and the postponed assignments can use exact distances afterward.
                        curVehLocToPickupSearches.addPickupForProcessing(asgn.pickup.id, asgn.distToPickup);
                        pairedContinuations.push_back({asgn.pickup.id, 0, INVALID_INDEX});
                        ++numAssignmentsTriedWithPickupBeforeNextStop; // Count first ordinary continuation
                        ordinaryContinuations.push_back({asgn.pickup.id, distFromPickup, 1});
                        continue;
                        // Continue with next pickup, rest of assignments for this pickup later with exact distance
                    }
                }

                asgn.distFromPickup = distFromPickup;
                const int minPickupDetour = calcInitialPickupDetour(
                    veh.vehicleId, 0, INVALID_INDEX, minDepTimeAtPickup, asgn.distFromPickup, requestState, routeState);
                const auto scannedUntilIndex = tryLowerBoundsForOrdinary(
                    asgn, minPickupDetour, stations, stationsInEllipse, requestState, pdLocs, firstTaxiLegResult);

                if (scannedUntilIndex < routeState.numStopsOf(veh.vehicleId)) {
                    // In this case some assignment with the pickup before the next stop and an ordinary dropoff
                    // needs the exact distance to pickup via the vehicle. Postpone computation
                    // of the yet unknown exact distance and the rest of the assignments with later dropoffs. That way,
                    // the exact distances can be computed in a bundled fashion and the postponed assignments can use
                    // exact distances afterward.
                    curVehLocToPickupSearches.addPickupForProcessing(asgn.pickup.id, asgn.distToPickup);
                    ordinaryContinuations.push_back({asgn.pickup.id, distFromPickup, scannedUntilIndex});
                }
            }
        }


        // Examines combinations of a given pickup and all stations before the next stop of a given vehicle until a
        // paired assignment needs the exact distance to the pickup via the vehicle. Returns true if the exact distance is needed
        // or false if all combinations could be filtered.
        bool tryLowerBoundsForPaired(Assignment &asgn,
                                     const int depTimeAtPickup,
                                     const PTStations &stations,
                                     StationsInEllipseT &stationsInEllipse,
                                     StationDistancesT &stationDistances,
                                     const RequestState &requestState,
                                     const PDLocs &pdLocs,
                                     const FirstTaxiLegResult &firstTaxiLegResult) {
            using namespace time_utils;
            assert(asgn.vehicle && asgn.pickup.id != INVALID_ID);
            const int reqTime = requestState.originalRequest.requestTime;
            const auto vehId = asgn.vehicle->vehicleId;
            const int numStops = routeState.numStopsOf(vehId);
            const auto schedArrTimes = routeState.schedArrTimesFor(vehId);
            const auto maxArrTimes = routeState.maxArrTimesFor(vehId);

            const auto firstStopId = routeState.stopIdsFor(vehId)[0];
            const int maxDetourAtJ = maxArrTimes[1] - schedArrTimes[1];
            const auto lengthOfLegJ = calcLengthOfLegStartingAt(0, vehId, routeState);

            const int minTripTime = depTimeAtPickup + stationDistances.getMinDistanceForPDLoc(asgn.pickup.id) - reqTime;

            const auto &relevantStations = stationsInEllipse.getStationsInEllipse(firstStopId);

            if (relevantStations.empty())
                return false;

            const auto stopLocations = routeState.stopLocationsFor(vehId);

            asgn.distFromPickup = 0;
            asgn.dropoffStopIdx = 0;

            for (auto &entry: relevantStations) {
                const auto &station = stations[entry.targetId];

                if (stopLocations[1] == station.vehEdgeId)
                    continue;
                if (asgn.pickup.loc == station.vehEdgeId)
                    continue;

                // Stations in ellipse are sorted by detour so that we can break after the first station
                // that has a detour that is large enough to lead to a total cost that is above the external upper bound.
                // (We use a lower bound that does not consider the trip time from stop i to the station as
                // this is not respected in the order of stations).
                static int stopTime = InputConfig::getInstance().stopTime;
                int detourRightAfterStation = entry.distFromStopToStation + stopTime +
                                              entry.distFromStationToStop - lengthOfLegJ;
                if (detourRightAfterStation > maxDetourAtJ)
                    break;
                const int totalResDetour = calcResidualTotalDetourForStopAfterDropoff(
                    vehId, 0, numStops - 1, detourRightAfterStation, routeState);
                const int addedTripTime = calcAddedTripTimeAffectedByPickupAndDropoff(
                    vehId, 0, detourRightAfterStation, routeState);
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

                ++numAssignmentsTriedWithPickupBeforeNextStop;
                asgn.distToDropoff = stationDistances.getDistance(asgn.dropoff.id, asgn.pickup.id);
                asgn.distFromDropoff = entry.distFromStationToStop;
                const auto cost = calculator.calc(asgn, requestState);
                if (cost < upperBoundCost) {
                    // Lower bound is better than best known cost => We need the exact distance to pickup.
                    // Return and postpone remaining combinations.

                    return true;
                }
            }

            return false;
        }

        // Examines combinations of a given pickup before the next stop and all relevant stations after later stops of a given
        // vehicle until an assignment requires the exact distance to the pickup via the vehicle. Returns a stop index in the vehicle's route
        // at which the exact distance is first needed or the vehicle's number of stops if all combinations could be filtered.
        int tryLowerBoundsForOrdinary(Assignment &asgn,
                                      const int initialPickupDetour,
                                      const PTStations &stations,
                                      StationsInEllipseT &stationsInEllipse,
                                      const RequestState &requestState,
                                      const PDLocs &pdLocs,
                                      const FirstTaxiLegResult &firstTaxiLegResult) {
            using namespace time_utils;
            assert(asgn.vehicle && asgn.pickup.id != INVALID_ID);
            const int reqTime = requestState.originalRequest.requestTime;
            const auto vehId = asgn.vehicle->vehicleId;

            const auto numStops = routeState.numStopsOf(vehId);
            const auto stopLocations = routeState.stopLocationsFor(vehId);
            const auto stopIds = routeState.stopIdsFor(vehId);
            const auto schedArrTimes = routeState.schedArrTimesFor(vehId);
            const auto maxArrTimes = routeState.maxArrTimesFor(vehId);

            for (int j = 1; j < numStops - 1; ++j) {
                const auto curStopId = stopIds[j];
                const int maxDetourAtJ = maxArrTimes[j + 1] - schedArrTimes[j + 1];
                const auto lengthOfLegJ = calcLengthOfLegStartingAt(j, vehId, routeState);

                const int detourUntilArrAtJ = calcResidualPickupDetour(vehId, 0, j, initialPickupDetour, routeState);
                KASSERT(schedArrTimes[j] >= reqTime);
                const int minTripTime = schedArrTimes[j] + detourUntilArrAtJ - reqTime;

                const int detourUntilDepAtJ =
                        calcResidualPickupDetour(vehId, 0, j + 1, initialPickupDetour, routeState);
                const int addedTripTimeUntilJ = calcAddedTripTimeInInterval(vehId, 0, j,
                                                                            initialPickupDetour, routeState);

                const auto &relevantStations = stationsInEllipse.getStationsInEllipse(curStopId);

                for (auto &entry: relevantStations) {
                    const auto &station = stations[entry.targetId];

                    if (stopLocations[j + 1] == station.vehEdgeId) {
                        // If the station is at the location of the following stop, do not try an assignment here as it would
                        // introduce a new stop after j that is at the same location as j + 1.
                        // Instead, this will be dealt with as an assignment at j + 1 afterwards.
                        continue;
                    }

                    if (asgn.pickup.loc == station.vehEdgeId)
                        continue;

                    // Stations in ellipse are sorted by detour so that we can break after the first station
                    // that has a detour that is large enough to lead to a total cost that is above the external upper bound.
                    // (We use a lower bound that does not consider the trip time from stop i to the station as
                    // this is not respected in the order of stations).
                    static int stopTime = InputConfig::getInstance().stopTime;
                    const bool stationAtExistingStop = stopLocations[j] == station.vehEdgeId;
                    const int detourRightAfterStation = detourUntilDepAtJ +
                                                        (stationAtExistingStop
                                                             ? 0
                                                             : entry.distFromStopToStation + stopTime +
                                                               entry.distFromStationToStop - lengthOfLegJ);
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

                    asgn.dropoffStopIdx = j;
                    asgn.distToDropoff = entry.distFromStopToStation;
                    asgn.distFromDropoff = entry.distFromStationToStop;

                    ++numAssignmentsTriedWithPickupBeforeNextStop;
                    const auto cost = calculator.calc(asgn, requestState);
                    if (cost < upperBoundCost) {
                        // Lower bound is better than best known cost => We need the exact distance to pickup.
                        // Return and postpone remaining combinations.

                        return j;
                    }
                }
            }

            return numStops;
        }

        void finishContinuations(const Vehicle &veh,
                                 const PTStations &stations,
                                 StationsInEllipseT &stationsInEllipse,
                                 StationDistancesT &stationDistances,
                                 const RequestState &requestState,
                                 const PDLocs &pdLocs,
                                 FirstTaxiLegResult &firstTaxiLegResult) {
            using namespace time_utils;
            static int stopTime = InputConfig::getInstance().stopTime;
            const int reqTime = requestState.originalRequest.requestTime;
            const int vehId = veh.vehicleId;
            const auto numStops = routeState.numStopsOf(veh.vehicleId);
            const auto stopLocations = routeState.stopLocationsFor(veh.vehicleId);
            const auto stopIds = routeState.stopIdsFor(veh.vehicleId);
            const auto schedArrTimes = routeState.schedArrTimesFor(veh.vehicleId);
            const auto schedDepTimes = routeState.schedDepTimesFor(veh.vehicleId);
            const auto maxArrTimes = routeState.maxArrTimesFor(veh.vehicleId);
            Assignment asgn(&veh);

            // Finish all postponed assignments where station is at stop >= 1.

            for (const auto &continuation: ordinaryContinuations) {
                asgn.pickup = pdLocs.pickups[continuation.pickupId];

                asgn.distToPickup = curVehLocToPickupSearches.getDistance(veh.vehicleId, continuation.pickupId);
                if (asgn.distToPickup >= INFTY)
                    continue;

                asgn.distFromPickup = continuation.distFromPickup;

                const int depTimeAtPickup = getActualDepTimeAtPickup(
                    veh.vehicleId, 0, asgn.distToPickup, asgn.pickup, requestState, routeState);
                const int initialPickupDetour = calcInitialPickupDetour(
                    veh.vehicleId, 0, INVALID_INDEX, depTimeAtPickup, asgn.distFromPickup, requestState, routeState);

                for (auto j = continuation.continueStopIndex; j < numStops - 1; ++j) {
                    asgn.dropoffStopIdx = j;

                    const auto curStopId = stopIds[j];
                    const int maxDetourAtJ = maxArrTimes[j + 1] - schedArrTimes[j + 1];
                    const auto lengthOfLegJ = calcLengthOfLegStartingAt(j, vehId, routeState);

                    const int detourUntilArrAtJ =
                            calcResidualPickupDetour(vehId, 0, j, initialPickupDetour, routeState);
                    KASSERT(schedArrTimes[j] >= reqTime);
                    const int arrTimeAtJ = schedArrTimes[j] + detourUntilArrAtJ;
                    const int minTripTime = arrTimeAtJ - reqTime;

                    const int detourUntilDepAtJ =
                            calcResidualPickupDetour(vehId, 0, j + 1, initialPickupDetour, routeState);
                    const int depTimeAtJ = std::max(schedDepTimes[j], schedArrTimes[j] + detourUntilArrAtJ + stopTime);
                    KASSERT(depTimeAtJ == schedDepTimes[j] + detourUntilDepAtJ);
                    const int addedTripTimeUntilJ = calcAddedTripTimeInInterval(vehId, 0, j,
                        initialPickupDetour, routeState);

                    const auto &relevantOrdinaryStations = stationsInEllipse.getStationsInEllipse(curStopId);

                    for (auto &entry: relevantOrdinaryStations) {
                        const auto &station = stations[entry.targetId];

                        if (stopLocations[j + 1] == station.vehEdgeId)
                            continue;

                        if (asgn.pickup.loc == station.vehEdgeId)
                            continue;

                        // Stations in ellipse are sorted by detour so that we can break after the first station
                        // that has a detour that is large enough to lead to a total cost that is above the external upper bound.
                        // (We use a lower bound that does not consider the trip time from stop i to the station as
                        // this is not respected in the order of stations).
                        const bool stationAtExistingStop = stopLocations[j] == station.vehEdgeId;
                        const int detourRightAfterStation = detourUntilDepAtJ +
                                                            (stationAtExistingStop
                                                                 ? 0
                                                                 : entry.distFromStopToStation + stopTime +
                                                                   entry.distFromStationToStop - lengthOfLegJ);
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

                        asgn.dropoffStopIdx = j;
                        asgn.distToDropoff = entry.distFromStopToStation;
                        asgn.distFromDropoff = entry.distFromStationToStop;

                        const auto cost = calculator.calc(asgn, requestState);
                        if (cost < upperBoundCost) {
                            // Cost is better than best known cost => Update best known cost and assignment.

                            // requestState.tryAssignmentWithKnownCost(asgn, cost);
                            const int arrivalTime =
                                    stationAtExistingStop ? arrTimeAtJ : depTimeAtJ + asgn.distToDropoff;
                            KASSERT(arrivalTime == calcArrivalTime(asgn, requestState, routeState));
                            firstTaxiLegResult.tryAssignmentForStation(
                                station.stationId, asgn, cost, arrivalTime, PBNS);
                        }


                        if (j > continuation.continueStopIndex) {
                            // Do not count assignment at continuation twice
                            ++numAssignmentsTriedWithPickupBeforeNextStop;
                        }
                    }
                }
            }

            // Finish all paired assignments.
            asgn.distFromPickup = 0;
            for (const auto &continuation: pairedContinuations) {
                asgn.pickup = pdLocs.pickups[continuation.pickupId];

                asgn.dropoffStopIdx = 0;
                asgn.distToPickup = curVehLocToPickupSearches.getDistance(veh.vehicleId, continuation.pickupId);
                if (asgn.distToPickup >= INFTY)
                    continue;

                asgn.distFromPickup = 0;

                const int depTimeAtPickup = getActualDepTimeAtPickup(
                    veh.vehicleId, 0, asgn.distToPickup, asgn.pickup, requestState, routeState);

                const auto firstStopId = routeState.stopIdsFor(veh.vehicleId)[0];
                const int maxDetourAtJ = maxArrTimes[1] - schedArrTimes[1];
                const auto lengthOfLegJ = calcLengthOfLegStartingAt(0, vehId, routeState);
                const int minTripTime = depTimeAtPickup + stationDistances.getMinDistanceForPDLoc(asgn.pickup.id) -
                                        reqTime;

                const auto &relevantPairedStations = stationsInEllipse.getStationsInEllipse(firstStopId);

                for (auto &entry: relevantPairedStations) {
                    const auto &station = stations[entry.targetId];

                    if (stopLocations[1] == station.vehEdgeId)
                        continue;
                    if (asgn.pickup.loc == station.vehEdgeId)
                        continue;

                    // Stations in ellipse are sorted by detour so that we can break after the first station
                    // that has a detour that is large enough to lead to a total cost that is above the external upper bound.
                    // (We use a lower bound that does not consider the trip time from stop i to the station as
                    // this is not respected in the order of stations).
                    int detourRightAfterStation = entry.distFromStopToStation + stopTime +
                                                  entry.distFromStationToStop - lengthOfLegJ;
                    if (detourRightAfterStation > maxDetourAtJ)
                        break;
                    const int totalResDetour = calcResidualTotalDetourForStopAfterDropoff(
                        vehId, 0, numStops - 1, detourRightAfterStation, routeState);
                    const int addedTripTime = calcAddedTripTimeAffectedByPickupAndDropoff(
                        vehId, 0, detourRightAfterStation, routeState);
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

                    asgn.distFromDropoff = entry.distFromStationToStop;
                    if (asgn.distFromDropoff >= INFTY)
                        continue;

                    asgn.distToDropoff = stationDistances.getDistance(asgn.dropoff.id, asgn.pickup.id);

                    const auto cost = calculator.calc(asgn, requestState);
                    if (cost < upperBoundCost) {
                        // Cost is better than best known cost => Update best known cost and assignment

                        // requestState.tryAssignmentWithKnownCost(asgn, calculator.calc(asgn, requestState));
                        const int arrivalTime = depTimeAtPickup + asgn.distToDropoff;
                        KASSERT(arrivalTime == calcArrivalTime(asgn, requestState, routeState));
                        firstTaxiLegResult.tryAssignmentForStation(
                            station.stationId, asgn, calculator.calc(asgn, requestState), arrivalTime, PBNS);
                    }
                }
            }
        }

        int upperBoundCost;
        int externalUpperBoundCost;

        CurVehLocToPickupSearchesT &curVehLocToPickupSearches;
        const Fleet &fleet;
        CostCalculator calculator;
        const RouteState &routeState;

        int numAssignmentsTriedWithPickupBeforeNextStop;
        std::vector<Continuation> ordinaryContinuations;
        std::vector<Continuation> pairedContinuations;
    };
}
