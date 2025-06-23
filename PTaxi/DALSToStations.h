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
#include <KARRI/DataStructures/Containers/FastResetFlagArray.h>

namespace karri {

    template<typename InputGraphT,
            typename CHEnvT,
            typename StationDistancesT,
            typename CurVehLocToPickupSearchesT,
            typename LabelSet>
    class DALSToStations {
    private:


        static constexpr int K = LabelSet::K;
        using LabelMask = typename LabelSet::LabelMask;
        using DistanceLabel = typename LabelSet::DistanceLabel;

    public:

        DALSToStations(const InputGraphT &inputGraph,
                              const Fleet &fleet,
                              const CHEnvT &chEnv,
                              CurVehLocToPickupSearchesT &curVehLocToPickupSearchesT,
                              const RouteState &routeState,
                              PTStations &ptStations)
                : inputGraph(inputGraph),
                  fleet(fleet),
                  calculator(routeState),
                  curVehLocToPickupSearches(curVehLocToPickupSearchesT),
                  routeState(routeState),
                  checkPBNSForVehicle(fleet.size()),
                  ptStations(ptStations),
                  lastStopDistances(fleet.size())  {}

        void tryDropoffAfterLastStop(const RelevantPDLocs &relevantOrdinaryPickups,
                                     const RelevantPDLocs &relevantPickupsBeforeNextStop,
                                     RequestState& requestState,
                                     const PDLocs& pdLocs, stats::DalsAssignmentsPerformanceStats& stats) {
            curReqState = &requestState;
            curRelOrdinaryPickups = &relevantOrdinaryPickups;
            curRelPickupsBns = &relevantPickupsBeforeNextStop;

            enumerateAssignments(relevantOrdinaryPickups, relevantPickupsBeforeNextStop, requestState, pdLocs, stats);
        }

    private:

        // Enumerate DALS assignments
        void enumerateAssignments(const RelevantPDLocs &relevantOrdinaryPickups,
                                  const RelevantPDLocs &relevantPickupsBeforeNextStop,
                                  RequestState& requestState,
                                  const PDLocs& pdLocs, stats::DalsAssignmentsPerformanceStats& stats) {
            int numAssignmentsTried = 0;
            const int64_t pbnsTimeBefore = curVehLocToPickupSearches.getTotalLocatingVehiclesTimeForRequest() +
                                           curVehLocToPickupSearches.getTotalVehicleToPickupSearchTimeForRequest();
            KaRRiTimer timer;

            enumerateAssignmentsWithOrdinaryPickup(numAssignmentsTried, relevantOrdinaryPickups, requestState, pdLocs);
            enumerateAssignmentsWithPBNS(numAssignmentsTried, relevantPickupsBeforeNextStop, requestState, pdLocs);

            // Time spent to locate vehicles and compute distances from current vehicle locations to pickups is counted
            // into PBNS time so subtract it here.
            const int64_t pbnsTime = curVehLocToPickupSearches.getTotalLocatingVehiclesTimeForRequest() +
                                     curVehLocToPickupSearches.getTotalVehicleToPickupSearchTimeForRequest() -
                                     pbnsTimeBefore;

            const int64_t tryAssignmentsTime = timer.elapsed<std::chrono::nanoseconds>() - pbnsTime;
            stats.tryAssignmentsTime += tryAssignmentsTime;
            stats.numAssignmentsTried += numAssignmentsTried;

            // Find total number of candidate dropoffs for statistics
            int totalNumberOfCandidateDropoffs = 0;
            for (int vehId = 0; vehId < fleet.size(); ++vehId)
                for (const auto &station: ptStations)
                    totalNumberOfCandidateDropoffs += (getDistanceToDropoff(vehId, station.stationId) < INFTY);
            stats.numCandidateDropoffsAcrossAllVehicles += totalNumberOfCandidateDropoffs;
        }

        // Enumerate assignments where pickup is after next stop (ordinary pickup):
        void enumerateAssignmentsWithOrdinaryPickup(int &numAssignmentsTried,
                                                    const RelevantPDLocs &relevantOrdinaryPickups,
                                                    RequestState& requestState,
                                                    const PDLocs& pdLocs) {
            Assignment asgn;

            checkPBNSForVehicle.reset();
            for (int vehId = 0; vehId < fleet.size(); ++vehId) {
                if (!relevantOrdinaryPickups.hasRelevantSpotsFor(vehId)) {
                    // vehicle may still have relevant assignment with pickup before next stop
                    checkPBNSForVehicle.set(vehId);
                    continue;
                }

                const auto &numStops = routeState.numStopsOf(vehId);
                const auto &occupancies = routeState.occupanciesFor(vehId);
                const auto relevantPickupsInRevOrder = relevantOrdinaryPickups.relevantSpotsForInReverseOrder(vehId);
                asgn.vehicle = &fleet[vehId];
                asgn.dropoffStopIdx = numStops - 1;

                for (const auto &station: ptStations) {
                    asgn.dropoff = {
                            station.stationId, // PDLoc ID
                            station.vehEdgeId, // Location in road network
                            station.psgEdgeId, // Location in passenger road network
                            0, // Walking time from this dropoff to destination
                            0, // Vehicle driving time from this dropoff to the destination
                            0 // Vehicle driving time from destination to this dropoff
                        };

                    asgn.distToDropoff = getDistanceToDropoff(vehId, asgn.dropoff.id);
                    if (asgn.distToDropoff >= INFTY)
                        continue; // no need to check pickup before next stop

                    assert(asgn.distToDropoff >= 0 && asgn.distToDropoff < INFTY);
                    int curPickupIndex = numStops - 1;
                    auto pickupIt = relevantPickupsInRevOrder.begin();
                    for (; pickupIt < relevantPickupsInRevOrder.end(); ++pickupIt) {
                        const auto &entry = *pickupIt;

                        if (entry.stopIndex < curPickupIndex) {
                            // New smaller pickup index reached: Check if seating capacity and cost lower bound admit
                            // any valid assignments at this or earlier indices.
                            if (occupancies[entry.stopIndex] + requestState.originalRequest.numRiders > asgn.vehicle->capacity)
                                break;

                            assert(entry.stopIndex < numStops - 1);
                            const auto minTripTimeToLastStop = routeState.schedDepTimesFor(vehId)[numStops - 1] -
                                                               routeState.schedArrTimesFor(vehId)[entry.stopIndex + 1];

                            const auto minCostFromHere = calculator.calcVehicleIndependentCostLowerBoundForDALSWithKnownMinDistToDropoff(
                                    asgn.dropoff.walkingDist, asgn.distToDropoff, minTripTimeToLastStop, requestState);
                            if (minCostFromHere > requestState.getBestCost())
                                break;

                            curPickupIndex = entry.stopIndex;
                        }

                        asgn.pickup = pdLocs.pickups[entry.pdId];
                        if (asgn.pickup.loc == asgn.dropoff.loc)
                            continue;
                        ++numAssignmentsTried;
                        asgn.pickupStopIdx = entry.stopIndex;
                        asgn.distToPickup = entry.distToPDLoc;
                        asgn.distFromPickup = entry.distFromPDLocToNextStop;
                        requestState.tryAssignmentWithKnownCost(asgn, calculator.calc(asgn, requestState));
                    }

                    if (pickupIt == relevantPickupsInRevOrder.end()) {
                        // If the reverse scan of the vehicle route did not break early at a later stop, then we also
                        // need to consider the pickup before next stop case.
                        checkPBNSForVehicle.set(vehId);
                    }
                }
            }
        }

        // Enumerate assignments where the pickup is before the next stop (PBNS + DALS):
        void enumerateAssignmentsWithPBNS(int &numAssignmentsTried,
                                          const RelevantPDLocs &relevantPickupsBeforeNextStop,
                                          RequestState& requestState,
                                          const PDLocs& pdLocs) {
            Assignment asgn;
            asgn.pickupStopIdx = 0;

            for (const auto &vehId: relevantPickupsBeforeNextStop.getVehiclesWithRelevantPDLocs()) {

                if (!checkPBNSForVehicle.isSet(vehId))
                    continue;

                if (routeState.numStopsOf(vehId) == 0 ||
                    routeState.occupanciesFor(vehId)[0] + requestState.originalRequest.numRiders > fleet[vehId].capacity)
                    continue;

                pbnsContinuations.clear();

                const auto numStops = routeState.numStopsOf(vehId);
                asgn.vehicle = &fleet[vehId];
                asgn.dropoffStopIdx = numStops - 1;


                for (auto &entry: relevantPickupsBeforeNextStop.relevantSpotsFor(vehId)) {
                    asgn.pickup = pdLocs.pickups[entry.pdId];
                    asgn.distFromPickup = entry.distFromPDLocToNextStop;
                    for (const auto &station: ptStations) {
                        asgn.dropoff = {
                            station.stationId, // PDLoc ID
                            station.vehEdgeId, // Location in road network
                            station.psgEdgeId, // Location in passenger road network
                            0, // Walking time from this dropoff to destination
                            0, // Vehicle driving time from this dropoff to the destination
                            0 // Vehicle driving time from destination to this dropoff
                        };

                        if (asgn.pickup.loc == asgn.dropoff.loc)
                            continue;

                        asgn.distToDropoff = getDistanceToDropoff(vehId, asgn.dropoff.id);
                        if (asgn.distToDropoff >= INFTY)
                            continue;

                        if (curVehLocToPickupSearches.knowsDistance(vehId, asgn.pickup.id)) {
                            asgn.distToPickup = curVehLocToPickupSearches.getDistance(vehId, asgn.pickup.id);
                            requestState.tryAssignmentWithKnownCost(asgn, calculator.calc(asgn, requestState));
                            ++numAssignmentsTried;
                        } else {
                            asgn.distToPickup = entry.distToPDLoc;
                            const auto lowerBoundCost = calculator.calc(asgn, requestState);
                            if (lowerBoundCost < requestState.getBestCost() ||
                                (lowerBoundCost == requestState.getBestCost() &&
                                 breakCostTie(asgn, requestState.getBestAssignment()))) {
                                // In this case, we need the exact distance to the pickup via the current location of the
                                // vehicle. We postpone computation of that distance to be able to bundle it with the
                                // computation of distances to other pickups via the vehicle location. Then all remaining
                                // assignments with this pickup can be tried with the exact distance later.
                                curVehLocToPickupSearches.addPickupForProcessing(asgn.pickup.id, asgn.distToPickup);
                                pbnsContinuations.push_back({asgn.pickup.id, asgn.distFromPickup, asgn.dropoff.id});
                                break;
                            }
                        }
                    }
                }

                // Continue with assignments for pickups where exact distance via vehicle location is needed
                curVehLocToPickupSearches.computeExactDistancesVia(fleet[vehId], pdLocs);
                for (const auto &continuation: pbnsContinuations) {
                    assert(continuation.pickupID >= 0 && continuation.pickupID < pdLocs.numPickups());
                    assert(continuation.fromStationID >= 0 && continuation.fromStationID < ptStations.size());
                    asgn.pickup = pdLocs.pickups[continuation.pickupID];

                    asgn.distToPickup = curVehLocToPickupSearches.getDistance(vehId,
                                                                              continuation.pickupID);
                    if (asgn.distToPickup >= INFTY)
                        continue;

                    asgn.distFromPickup = continuation.distFromPickup;
                    for (int stationId = continuation.fromStationID;
                         stationId < ptStations.size(); ++stationId) {
                        asgn.dropoff = {
                            stationId, // PDLoc ID
                            ptStations[stationId].vehEdgeId, // Location in road network
                            ptStations[stationId].psgEdgeId, // Location in passenger road network
                            0, // Walking time from this dropoff to destination
                            0, // Vehicle driving time from this dropoff to the destination
                            0 // Vehicle driving time from destination to this dropoff
                        };

                        if (asgn.pickup.loc == asgn.dropoff.loc)
                            continue;

                        asgn.distToDropoff = getDistanceToDropoff(vehId, asgn.dropoff.id);
                        if (asgn.distToDropoff >= INFTY)
                            continue;

                        ++numAssignmentsTried;
                        asgn.dropoffStopIdx = numStops - 1;
                        requestState.tryAssignmentWithKnownCost(asgn, calculator.calc(asgn, requestState));
                    }
                }
            }
        }

        inline int getDistanceToDropoff(const int vehId, const int stationId) {
            return lastStopDistances.getDistance(vehId, stationId);
        }


        const InputGraphT &inputGraph;
        const Fleet &fleet;
        CostCalculator calculator;
        CurVehLocToPickupSearchesT &curVehLocToPickupSearches;
        const RouteState &routeState;

        // Stations
        PTStations &ptStations;

        // Flag per vehicle that tells us if we still have to consider a pickup before the next stop of the vehicle.
        FastResetFlagArray<> checkPBNSForVehicle;

        // Records for postponing and bundling computations of distances from current locations of vehicles to
        // pickups needed in the PBNS+DALS case.
        struct PickupBeforeNextStopContinuation {
            int pickupID;
            int distFromPickup;
            int fromStationID;
        };
        std::vector<PickupBeforeNextStopContinuation> pbnsContinuations;

        int upperBoundCost;

        // Vehicles seen by any last stop search
        DistanceLabel currentDropoffWalkingDists;
        TentativeLastStopDistances<LabelSet> lastStopDistances;

        // Pointers to request state and relevant PD locs so Dijkstra search callback has access to them
        RequestState const *curReqState;
        RelevantPDLocs const *curRelOrdinaryPickups;
        RelevantPDLocs const *curRelPickupsBns;

    };
    
}