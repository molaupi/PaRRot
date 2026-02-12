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
                  routeState(routeState) {}

        void enumerateAssignments(const RequestState& requestState, const PDLocs& pdLocs, const RelevantPDLocs& relPickups,
                                  const PTStations& stations, StationsInEllipseT &stationsInEllipse , StationDistancesT &stationDistances,
                                  stats::OrdAssignmentsPerformanceStats &stats, FirstTaxiLegResult &firstTaxiLegResult) {

            using namespace time_utils;

            int numAssignmentsTried = 0;
            int numCandidateVehicles = 0;
            KaRRiTimer timer;

            for (const auto &vehId: relPickups.getVehiclesWithRelevantPDLocs()) {
                KASSERT(relPickups.hasRelevantSpotsFor(vehId));
                ++numCandidateVehicles;

                Assignment asgn(&fleet[vehId]);

                for (const auto &pickupEntry: relPickups.relevantSpotsFor(vehId)) {

                    asgn.pickup = pdLocs.pickups[pickupEntry.pdId];
                    asgn.pickupStopIdx = pickupEntry.stopIndex;
                    asgn.distToPickup = pickupEntry.distToPDLoc;

                    // Iterates through stops [pickup's stop index; last stop) and try to find a station as a dropoff.
                    for (int i = pickupEntry.stopIndex; i < routeState.numStopsOf(vehId) - 1; ++i) {

                        const auto curStopId = routeState.stopIdsFor(vehId)[i];

                        // for each station in ellipse, try assignment
                        for (const auto &stationEntry: stationsInEllipse.getStationsInEllipse(curStopId)) {
                            const auto &station = stations[stationEntry.targetId];

                            asgn.dropoff = {
                                station.stationId, // PDLoc ID
                                station.vehEdgeId, // Location in road network
                                station.psgEdgeId, // Location in passenger road network
                                0, // Walking time from this dropoff to destination
                                0, // Vehicle driving time from this dropoff to the destination
                                0 // Vehicle driving time from destination to this dropoff
                            };

                            asgn.dropoffStopIdx = i;
                            asgn.distFromDropoff = stationEntry.distFromStationToStop;

                            // In case of paired assignment:
                            if (asgn.pickupStopIdx == asgn.dropoffStopIdx) {
                                asgn.distFromPickup = 0;
                                asgn.distToDropoff = stationDistances.getDistance(asgn.dropoff.id, asgn.pickup.id);
                            } else {
                                asgn.distFromPickup = pickupEntry.distFromPDLocToNextStop;
                                asgn.distToDropoff = stationEntry.distFromStopToStation;
                            }

                            ++numAssignmentsTried;
                            // requestState.tryAssignmentWithKnownCost(asgn, calculator.calc(asgn, requestState));
                            firstTaxiLegResult.tryAssignmentWithKnownCostForStation(station.stationId, asgn, calculator.calc(asgn, requestState), InsertionType::ORDINARY);
                        }
                    }
                }        
            }

            const int64_t tryAssignmentsTime = timer.elapsed<std::chrono::nanoseconds>();
            stats.numCandidateVehicles += numCandidateVehicles;
            stats.numAssignmentsTried += numAssignmentsTried;
            stats.tryNonPairedAssignmentsTime += tryAssignmentsTime; // no clear separation between paired and non-paired assignments here
        }

    private:

        const Fleet &fleet;
        CostCalculator calculator;
        const RouteState &routeState;
    };

}
