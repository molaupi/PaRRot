#pragma once

#include <vector>
#include "../TaxiResult.h"
#include <KARRI/Algorithms/KaRRi/BaseObjects/Assignment.h>

#include "KARRI/DataStructures/Utilities/DynamicRagged2DArrays.h"

namespace karri {
    class FirstTaxiLegResult {
        static bool dominates(const TaxiResult &r1, const TaxiResult &r2) {
            return r1.bestCost < r2.bestCost
                   || (r1.bestCost == r2.bestCost && r1.arrivalTime < r2.arrivalTime)
                   || (r1.bestCost == r2.bestCost && r1.arrivalTime == r2.arrivalTime &&
                       breakCostTie(r1.bestAssignment, r2.bestAssignment)
                   );
        }

    public:
        explicit FirstTaxiLegResult(const RouteState &routeState, const int numStations)
            : routeState(routeState),
        // requestState(requestState),
              calculator(routeState), externalUpperBoundCost(INFTY),
              // worstCostForAllStations(INFTY), worstAssignmentForAllStations(),
              pos(numStations),
              paretoResults(numStations, TaxiResult()) {
            assert(numStations >= 0);
            for (int i = 0; i < numStations; ++i) {
                pos[i] = {i, i};
            }
        }

        void reset(const int newExternalUpperBoundCost, const int pCurRequestId) {
            KASSERT(externalUpperBoundCost <= INFTY);
            externalUpperBoundCost = newExternalUpperBoundCost;
            for (int stationId: stationsWithResults) {
                pos[stationId] = {pos[stationId].start, pos[stationId].start};
            }
            stationsWithResults.clear();
            paretoResults.clear();
            curRequestId = pCurRequestId;
        }

        bool tryAssignmentWithForStation(const int stationId, const Assignment &asgn,
            const int cost, const int arrivalTime,
                                                  InsertionType insertionType) {
            if (cost >= externalUpperBoundCost) return false;
            if (stationId < 0 || stationId >= pos.size()) return false;

            TaxiResult newResult(cost, arrivalTime, asgn, insertionType);

            // Check if new result is dominated
            const auto [start, end] = pos[stationId];
            for (int i = start; i < end; ++i) {
                const TaxiResult &existingResult = paretoResults[i];
                if (dominates(existingResult, newResult)) {
                    return false; // new result is dominated, discard it
                }
            }

            // If station has no results yet, mark that it is now active
            if (pos[stationId].end == pos[stationId].start)
                stationsWithResults.push_back(stationId);

            // Remove all that are dominated by new result
            removalIf(stationId, [&](const TaxiResult &existingResult) {
                return dominates(newResult, existingResult);
            }, pos, paretoResults);

            // Add new result to pareto results
            insertion(stationId, newResult, pos, paretoResults);
            return true;

            // TaxiResult &taxiResult = results[stationId];
            //
            // if (cost < INFTY && (cost < taxiResult.bestCost || (cost == taxiResult.bestCost &&
            //                                                     breakCostTie(asgn, taxiResult.bestAssignment)))) {
            //     taxiResult = TaxiResult(cost, calcArrivalTime(asgn), asgn, insertionType);
            //     // if (worstCostForAllStations == INFTY || taxiResult.bestCost > worstCostForAllStations) {
            //     //     worstCostForAllStations = taxiResult.bestCost;
            //     //     worstAssignmentForAllStations = taxiResult.bestAssignment;
            //     // }
            //     return true;
            // }
            // return false;
        }

        // const Assignment &getWorstAssignmentForAllStations() const {
        //     return worstAssignmentForAllStations;
        // }
        //
        // const int &getWorstCostForAllStations() const {
        //     return worstCostForAllStations;
        // }

        const std::vector<int> &getStationsWithResults() const {
            return stationsWithResults;
        }

        ConstantVectorRange<TaxiResult> getResultsForStation(const int stationId) const {
            KASSERT(stationId >= 0 && stationId < pos.size());
            return {paretoResults.begin() + pos[stationId].start, paretoResults.begin() + pos[stationId].end};
        }

        // const int getCostWithoutTripTimeForStation(const int stationId) const {
        //     KASSERT(stationId >= 0 && stationId < pos.size());
        //     auto result = results[stationId];
        //     return calculator.calc(result.bestAssignment, requestState, true);
        // }

        std::vector<TaxiResult> getValidResults() const {
            std::vector<TaxiResult> validResults;
            for (const auto &stationId: stationsWithResults) {
                const auto [start, end] = pos[stationId];
                for (int i = start; i < end; ++i) {
                    validResults.push_back(paretoResults[i]);
                }
            }
            return validResults;
        }

    private:
        // int calcArrivalTime(const Assignment &asgn) {
        //     using namespace time_utils;
        //     const int actualDepTimeAtPickup = getActualDepTimeAtPickup(asgn, requestState, routeState);
        //     const int initialPickupDetour = calcInitialPickupDetour(asgn, actualDepTimeAtPickup, requestState,
        //                                                             routeState);
        //     const bool dropoffAtExistingStop = isDropoffAtExistingStop(asgn, routeState);
        //     return getArrTimeAtDropoff(actualDepTimeAtPickup, asgn, initialPickupDetour, dropoffAtExistingStop,
        //                                routeState);
        // }

        const RouteState &routeState;
        // const RequestState &requestState;
        CostCalculator calculator;
        int externalUpperBoundCost;
        int curRequestId;
        // std::vector<TaxiResult> results;
        // int worstCostForAllStations;
        // Assignment worstAssignmentForAllStations;

        // One bucket per station. Bucket for station S contains all taxi results for S that are pareto-optimal with
        // respect to cost and arrival time.
        std::vector<ValueBlockPosition> pos;
        std::vector<TaxiResult> paretoResults;

        std::vector<int> stationsWithResults;
    };
} // namespace karri
