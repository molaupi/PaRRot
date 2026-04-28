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
#include "../RequestState/RequestState.h"
#include "../LastStopSearches/OnlyLastStopsAtVerticesBucketSubstitute.h"

namespace karri {

// Finds pickup-after-last-stop (PALS) insertions using the encapsulated strategy.
    template<typename InputGraphT, typename StrategyT, typename LastStopsAtVerticesT>
    class PALSAssignmentsFinder {

    public:

        PALSAssignmentsFinder(StrategyT &strategy, const InputGraphT &inputGraph, const Fleet &fleet,
                              const LastStopsAtVerticesT &lastStopsAtVertices,
                              const RouteState &routeState)
                : strategy(strategy),
                  inputGraph(inputGraph),
                  fleet(fleet),
                  calculator(routeState),
                  lastStopsAtVertices(lastStopsAtVertices),
                  routeState(routeState) {}

        void findAssignments(const RequestState& requestState, const PDDistances& pdDistances, const PDLocs& pdLocs,
            TaxiResult &result,
            stats::PalsAssignmentsPerformanceStats& stats) {
            findAssignmentsWherePickupCoincidesWithLastStop(requestState, pdDistances, pdLocs, result, stats);
            strategy.tryPickupAfterLastStop(requestState, pdDistances, pdLocs, result, stats);
        }

        void init(const RequestState&, const PDLocs&, stats::PalsAssignmentsPerformanceStats&) {
            // no op
        }

    private:

        // Simple case for pickups that coincide with last stops of vehicles is the same regardless of strategy, so it
        // is treated here.
        void findAssignmentsWherePickupCoincidesWithLastStop(const RequestState& requestState,
            const PDDistances& pdDistances,
            const PDLocs& pdLocs,
            TaxiResult &result,
            stats::PalsAssignmentsPerformanceStats& stats) {
            int numInsertionsForCoinciding = 0;
            int numCandidateVehiclesForCoinciding = 0;
            KaRRiTimer timer;

            Assignment asgn;
            asgn.distToPickup = 0;
            for (const auto &p: pdLocs.pickups) {
                asgn.pickup = p;

                const int head = inputGraph.edgeHead(asgn.pickup.loc);
                for (const auto &vehId: lastStopsAtVertices.vehiclesWithLastStopAt(head)) {
                    ++numCandidateVehiclesForCoinciding;
                    const auto numStops = routeState.numStopsOf(vehId);
                    if (routeState.stopLocationsFor(vehId)[numStops - 1] != asgn.pickup.loc)
                        continue;

                    // Calculate lower bound on insertion cost with this pickup and vehicle
                    const auto lowerBoundCost = calculator.calcCostLowerBoundForPickupAfterLastStop(
                            fleet[vehId], asgn.pickup, 0, pdDistances.getMinDirectDistance(), requestState);
                    if (lowerBoundCost > result.getBestCost())
                        continue;

                    // If necessary, check paired insertion with each dropoff
                    asgn.vehicle = &fleet[vehId];
                    asgn.pickupStopIdx = numStops - 1;
                    asgn.dropoffStopIdx = numStops - 1;

                    for (const auto &d: pdLocs.dropoffs) {
                        if (d.loc == asgn.pickup.loc)
                            continue;
                        asgn.dropoff = d;
                        asgn.distToDropoff = pdDistances.getDirectDistance(asgn.pickup, asgn.dropoff);
                        ++numInsertionsForCoinciding;
                        result.tryAssignmentWithKnownCost(asgn, calculator.calc(asgn, requestState));
                    }
                }
            }

            const auto time = timer.elapsed<std::chrono::nanoseconds>();
            stats.pickupAtLastStop_tryAssignmentsTime += time;
            stats.pickupAtLastStop_numCandidateVehicles += numCandidateVehiclesForCoinciding;
            stats.pickupAtLastStop_numAssignmentsTried += numInsertionsForCoinciding;
        }

        StrategyT &strategy;

        const InputGraphT &inputGraph;
        const Fleet &fleet;
        CostCalculator calculator;
        const LastStopsAtVerticesT &lastStopsAtVertices;
        const RouteState &routeState;

    };
}