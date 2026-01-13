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

namespace karri {


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
                  routeState(routeState) {}

        void findAssignments(RequestState& requestState, const PDLocs& pdLocs, 
                             const RelevantPDLocs &relPickupsBns, 
                             const PTStations& stations, StationsInEllipseT &stationsInEllipse, StationDistancesT &stationDistances,
                             stats::PbnsAssignmentsPerformanceStats& stats,
                             FirstTaxiLegResult &firstTaxiLegResult) {
            
            numAssignmentsTriedWithPickupBeforeNextStop = 0;
            init(requestState, pdLocs, stats);
            
            KaRRiTimer timer;

            int numCandidateVehicles = 0;
            for (const auto &vehId: relPickupsBns.getVehiclesWithRelevantPDLocs()) {

                ++numCandidateVehicles;

                ordinaryContinuations.clear();
                pairedContinuations.clear();

                assert(routeState.occupanciesFor(vehId)[0]  + requestState.originalRequest.numRiders <= fleet[vehId].capacity);

                determineNecessaryExactDistances(fleet[vehId], relPickupsBns, stations, stationsInEllipse, stationDistances, requestState, pdLocs, firstTaxiLegResult);

                curVehLocToPickupSearches.computeExactDistancesVia(fleet[vehId], pdLocs);

                finishContinuations(fleet[vehId], stations, stationsInEllipse, stationDistances, requestState, pdLocs, firstTaxiLegResult);
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

        void setExternalCostUpperBound(const int bestCost, const int worstCostForAllStations) {
            externalUpperBoundCost = bestCost;
            upperBoundCost = std::min(worstCostForAllStations, externalUpperBoundCost);
        }

    private:

        // Initialize for new request.
        void init(const RequestState& requestState, const PDLocs& pdLocs, stats::PbnsAssignmentsPerformanceStats& stats) {
            KaRRiTimer timer;
            curVehLocToPickupSearches.initialize(requestState.now(), pdLocs);
            const auto time = timer.elapsed<std::chrono::nanoseconds>();
            stats.initializationTime += time;
        }

        // Filters combinations of pickups and dropoffs using a cost lower bound.
        // If a combination is found for a pickup that cannot be filtered, we need the exact distance from the vehicle
        // location to the pickup.
        // These pickups are added to the queue of curVehLocToPickupSearches and continuations are stored to restart
        // the iteration of combinations for that pickup after the computation of exact distances.
        void determineNecessaryExactDistances(const Vehicle &veh, const RelevantPDLocs &relPickupsBns,
                                              const PTStations& stations, StationsInEllipseT &stationsInEllipse , StationDistancesT &stationDistances,
                                              RequestState& requestState, const PDLocs& pdLocs, FirstTaxiLegResult &firstTaxiLegResult) {

            Assignment asgn(&veh);

            for (const auto &entry: relPickupsBns.relevantSpotsFor(veh.vehicleId)) {
                asgn.pickup = pdLocs.pickups[entry.pdId];

                // Distance from stop 0 to pickup is actually a lower bound on the distance from stop 0 via the
                // vehicle's current location to the pickup => we get lower bound costs.
                asgn.distToPickup = entry.distToPDLoc;
                const int distFromPickup = entry.distFromPDLocToNextStop;

                // For paired assignments before next stop, first try a lower bound with the smallest distance to a station
                const auto lowerBoundCostPairedAssignment = calculator.calcCostLowerBoundForPairedAssignmentBeforeNextStop(
                        veh, asgn.pickup, asgn.distToPickup, stationDistances.getMinDistanceForPDLoc(asgn.pickup.id),
                        distFromPickup, requestState);
                if (lowerBoundCostPairedAssignment < upperBoundCost) {
                    const auto requireExactDistance = tryLowerBoundsForPaired(asgn, stations, stationsInEllipse, stationDistances, requestState, pdLocs, firstTaxiLegResult);
                    if (requireExactDistance) {
                        // In this case some paired assignment before the next stop needs the exact distance to pickup via
                        // the vehicle. Postpone computation of the yet unknown exact distance and the rest of the paired
                        // assignments as well as all assignments with later dropoffs. That way, the exact distances can be
                        // computed in a bundled fashion and the postponed assignments can use exact distances afterward.
                        curVehLocToPickupSearches.addPickupForProcessing(asgn.pickup.id, asgn.distToPickup);
                        pairedContinuations.push_back({asgn.pickup.id, 0, 0});
                        ++numAssignmentsTriedWithPickupBeforeNextStop; // Count first ordinary continuation
                        ordinaryContinuations.push_back(
                                {asgn.pickup.id, distFromPickup, 0});
                        continue; // Continue with next pickup, rest of assignments for this pickup later with exact distance
                    }
                }

                asgn.distFromPickup = distFromPickup;
                const auto scannedUntilIndex = tryLowerBoundsForOrdinary(asgn, stations, stationsInEllipse, requestState, pdLocs, firstTaxiLegResult);

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
                                     const PTStations& stations, 
                                     StationsInEllipseT &stationsInEllipse, 
                                     StationDistancesT &stationDistances, 
                                     RequestState& requestState,
                                     const PDLocs& pdLocs,
                                     FirstTaxiLegResult &firstTaxiLegResult) {
            assert(asgn.vehicle && asgn.pickup.id != INVALID_ID);
            const auto vehId = asgn.vehicle->vehicleId;

            const auto firstStopId = routeState.stopIdsFor(vehId)[0];
            const auto &relevantStations = stationsInEllipse.getStationsInEllipse(firstStopId);

            if (relevantStations.empty())
                return false;

            const auto stopLocations = routeState.stopLocationsFor(vehId);

            asgn.distFromPickup = 0;
            asgn.dropoffStopIdx = 0;

            for (auto &entry: relevantStations) {
                const auto &station = stations[entry.targetId];
                asgn.dropoff = {
                    station.stationId, // PDLoc ID
                    station.vehEdgeId, // Location in road network
                    station.psgEdgeId, // Location in passenger road network
                    0, // Walking time from this dropoff to destination
                    0, // Vehicle driving time from this dropoff to the destination
                    0 // Vehicle driving time from destination to this dropoff
                };

                if (stopLocations[1] == asgn.dropoff.loc)
                    continue;

                ++numAssignmentsTriedWithPickupBeforeNextStop;
                asgn.distToDropoff = stationDistances.getDistance(asgn.dropoff.id, asgn.pickup.id);
                asgn.distFromDropoff = entry.distFromStationToStop;
                const auto cost = calculator.calc(asgn, requestState);
                if (cost < upperBoundCost || (cost == firstTaxiLegResult.getWorstCostForAllStations() &&
                                                          breakCostTie(asgn, firstTaxiLegResult.getWorstAssignmentForAllStations()))) {
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
                                      const PTStations& stations, 
                                      StationsInEllipseT &stationsInEllipse, 
                                      RequestState& requestState, 
                                      const PDLocs& pdLocs,
                                      FirstTaxiLegResult &firstTaxiLegResult) {
            using namespace time_utils;
            assert(asgn.vehicle && asgn.pickup.id != INVALID_ID);
            const auto vehId = asgn.vehicle->vehicleId;

            const auto numStops = routeState.numStopsOf(vehId);
            const auto stopLocations = routeState.stopLocationsFor(vehId);
            const auto stopIds = routeState.stopIdsFor(vehId);

            for (int i = 1; i < numStops - 1; ++i) {
                const auto curStopId = stopIds[i];
                const auto &relevantStations = stationsInEllipse.getStationsInEllipse(curStopId);

                for (auto &entry: relevantStations) {
                    const auto &station = stations[entry.targetId];
                    asgn.dropoff = {
                        station.stationId, // PDLoc ID
                        station.vehEdgeId, // Location in road network
                        station.psgEdgeId, // Location in passenger road network
                        0, // Walking time from this dropoff to destination
                        0, // Vehicle driving time from this dropoff to the destination
                        0 // Vehicle driving time from destination to this dropoff
                    };

                    if (stopLocations[i + 1] == asgn.dropoff.loc)
                        continue;
                    if (asgn.dropoff.loc == asgn.pickup.loc)
                        continue;

                    asgn.dropoffStopIdx = i;
                    asgn.distToDropoff = entry.distFromStopToStation;
                    asgn.distFromDropoff = entry.distFromStationToStop;

                    ++numAssignmentsTriedWithPickupBeforeNextStop;
                    const auto cost = calculator.calc(asgn, requestState);
                    if (cost < upperBoundCost || (cost == firstTaxiLegResult.getWorstCostForAllStations() &&
                                                              breakCostTie(asgn, firstTaxiLegResult.getWorstAssignmentForAllStations()))) {
                        // Lower bound is better than best known cost => We need the exact distance to pickup.
                        // Return and postpone remaining combinations.

                        return i;
                    }
                }
            }

            return numStops;
        }

        void finishContinuations(const Vehicle &veh, 
                                 const PTStations& stations, 
                                 StationsInEllipseT &stationsInEllipse, 
                                 StationDistancesT &stationDistances, 
                                 RequestState& requestState, 
                                 const PDLocs& pdLocs,
                                 FirstTaxiLegResult &firstTaxiLegResult) {
            
            const auto stopLocations = routeState.stopLocationsFor(veh.vehicleId);
            const auto numStops = routeState.numStopsOf(veh.vehicleId);
            const auto stopIds = routeState.stopIdsFor(veh.vehicleId);
            Assignment asgn(&veh);

            // Finish all postponed assignments where station is at stop >= 1.

            for (const auto &continuation: ordinaryContinuations) {
                asgn.pickup = pdLocs.pickups[continuation.pickupId];

                asgn.distToPickup = curVehLocToPickupSearches.getDistance(veh.vehicleId, continuation.pickupId);
                if (asgn.distToPickup >= INFTY)
                    continue;

                asgn.distFromPickup = continuation.distFromPickup;

                for (auto stopIndex = continuation.continueStopIndex;
                     stopIndex < numStops - 1; ++stopIndex) {
                    asgn.dropoffStopIdx = stopIndex;
                
                    const auto curStopId = stopIds[stopIndex];
                    const auto &relevantOrdinaryStations = stationsInEllipse.getStationsInEllipse(curStopId);

                    for (auto &entry: relevantOrdinaryStations) {
                        const auto &station = stations[entry.targetId];
                        asgn.dropoff = {
                            station.stationId, // PDLoc ID
                            station.vehEdgeId, // Location in road network
                            station.psgEdgeId, // Location in passenger road network
                            0, // Walking time from this dropoff to destination
                            0, // Vehicle driving time from this dropoff to the destination
                            0 // Vehicle driving time from destination to this dropoff
                        };

                        if (stopLocations[stopIndex + 1] == asgn.dropoff.loc)
                            continue;

                        if (asgn.dropoff.loc == asgn.pickup.loc)
                            continue;
                        
                        asgn.dropoffStopIdx = stopIndex;
                        asgn.distToDropoff = entry.distFromStopToStation;
                        asgn.distFromDropoff = entry.distFromStationToStop;

                        // requestState.tryAssignmentWithKnownCost(asgn, calculator.calc(asgn, requestState));
                        firstTaxiLegResult.tryAssignmentWithKnownCostForStation(station.stationId, asgn, calculator.calc(asgn, requestState), InsertionType::PBNS);

                        if (stopIndex > continuation.continueStopIndex) { // Do not count assignment at continuation twice
                            ++numAssignmentsTriedWithPickupBeforeNextStop;
                        }
                    }
                }
            }
            
            // Finish all paired assignments.

            for (const auto &continuation: pairedContinuations) {
                asgn.pickup = pdLocs.pickups[continuation.pickupId];

                asgn.dropoffStopIdx = 0;
                asgn.distToPickup = curVehLocToPickupSearches.getDistance(veh.vehicleId, continuation.pickupId);
                if (asgn.distToPickup >= INFTY)
                    continue;

                asgn.distFromPickup = 0;

                const auto firstStopId = routeState.stopIdsFor(veh.vehicleId)[0];
                const auto &relevantPairedStations = stationsInEllipse.getStationsInEllipse(firstStopId);

                for (auto &entry: relevantPairedStations) {
                    const auto &station = stations[entry.targetId];
                    asgn.dropoff = {
                        station.stationId, // PDLoc ID
                        station.vehEdgeId, // Location in road network
                        station.psgEdgeId, // Location in passenger road network
                        0, // Walking time from this dropoff to destination
                        0, // Vehicle driving time from this dropoff to the destination
                        0 // Vehicle driving time from destination to this dropoff
                    };

                    if (stopLocations[1] == asgn.dropoff.loc)
                        continue;

                    asgn.distFromDropoff = entry.distFromStationToStop;
                    if (asgn.distFromDropoff >= INFTY)
                        continue;

                    asgn.distToDropoff = stationDistances.getDistance(asgn.dropoff.id, asgn.pickup.id);
                    // requestState.tryAssignmentWithKnownCost(asgn, calculator.calc(asgn, requestState));
                    firstTaxiLegResult.tryAssignmentWithKnownCostForStation(station.stationId, asgn, calculator.calc(asgn, requestState), InsertionType::PBNS);
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