#pragma once

#include <vector>
#include "../TaxiResult.h"
#include <KARRI/Algorithms/KaRRi/BaseObjects/Assignment.h>

namespace karri {
    class FirstTaxiLegResult {
    public:
        explicit FirstTaxiLegResult(const RouteState &routeState, const RequestState &requestState,
                                    const int numStations, const int &externalUpperBoundCost = INFTY)
            : results(numStations), routeState(routeState), requestState(requestState),
              externalUpperBoundCost(externalUpperBoundCost), calculator(routeState),
              worstCostForAllStations(INFTY), worstAssignmentForAllStations() {
            assert(numStations >= 0);
        }

        bool tryAssignmentWithKnownCostForStation(const int stationId, const Assignment &asgn, const int cost,
                                                  InsertionType insertionType) {
            if (cost >= externalUpperBoundCost) return false;
            if (stationId < 0 || stationId >= results.size()) return false;
            TaxiResult &taxiResult = results[stationId];

            if (cost < INFTY && (cost < taxiResult.bestCost || (cost == taxiResult.bestCost &&
                                                                breakCostTie(asgn, taxiResult.bestAssignment)))) {
                taxiResult = TaxiResult(cost, calcArrivalTime(asgn), asgn, insertionType);
                if (worstCostForAllStations == INFTY || taxiResult.bestCost > worstCostForAllStations) {
                    worstCostForAllStations = taxiResult.bestCost;
                    worstAssignmentForAllStations = taxiResult.bestAssignment;
                }
                return true;
            }
            return false;
        }

        const Assignment &getWorstAssignmentForAllStations() const {
            return worstAssignmentForAllStations;
        }

        const int &getWorstCostForAllStations() const {
            return worstCostForAllStations;
        }

        const TaxiResult &getResultForStation(const int stationId) const {
            assert(stationId >= 0 && stationId < results.size());
            return results[stationId];
        }

        const int getCostWithoutTripTimeForStation(const int stationId) const {
            assert(stationId >= 0 && stationId < results.size());
            auto result = results[stationId];
            return calculator.calc(result.bestAssignment, requestState, true);
        }

        std::vector<TaxiResult> getValidResults() const {
            std::vector<TaxiResult> validResults;
            for (const auto &result: results) {
                if (result.bestCost != INFTY) {
                    validResults.push_back(result);
                }
            }
            return validResults;
        }

    private:
        int calcArrivalTime(const Assignment &asgn) {
            using namespace time_utils;
            const int actualDepTimeAtPickup = getActualDepTimeAtPickup(asgn, requestState, routeState);
            const int initialPickupDetour = calcInitialPickupDetour(asgn, actualDepTimeAtPickup, requestState,
                                                                    routeState);
            const bool dropoffAtExistingStop = isDropoffAtExistingStop(asgn, routeState);
            return getArrTimeAtDropoff(actualDepTimeAtPickup, asgn, initialPickupDetour, dropoffAtExistingStop,
                                       routeState);
        }

        const RouteState &routeState;
        const RequestState &requestState;
        const int &externalUpperBoundCost;
        CostCalculator calculator;
        std::vector<TaxiResult> results;
        int worstCostForAllStations;
        Assignment worstAssignmentForAllStations;
    };
} // namespace karri
