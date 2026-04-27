#pragma once

#include <vector>
#include "../TaxiResult.h"
#include <KARRI/Algorithms/KaRRi/BaseObjects/Assignment.h>

#include "AccessRPTrip.h"
#include "KARRI/DataStructures/Utilities/DynamicRagged2DArrays.h"

namespace parrot {

    using karri::RouteState, karri::Request, karri::CostCalculator;

    class FirstTaxiLegResult {


    public:
        explicit FirstTaxiLegResult(const RouteState &routeState, const int numStations)
            : routeState(routeState),
              calculator(routeState), externalUpperBoundCost(INFTY),
              pos(numStations),
              paretoResults(numStations) {
            assert(numStations >= 0);
            for (int i = 0; i < numStations; ++i) {
                pos[i] = {i, i};
            }
        }

        void reset(const int newExternalUpperBoundCost, const Request pCurRequest) {
            KASSERT(externalUpperBoundCost <= INFTY);
            externalUpperBoundCost = newExternalUpperBoundCost;
            for (int stationId: stationsWithResults) {
                removalOfAllCols(stationId, pos, paretoResults);
            }
            stationsWithResults.clear();
            curRequest = pCurRequest;
        }

        bool tryAssignmentForStation(const int stationId, const Assignment &asgn,
                                     const int cost, const int arrivalTime, InsertionType insertionType) {
            if (cost >= externalUpperBoundCost) return false;
            if (stationId < 0 || stationId >= pos.size()) return false;

            const int costWithoutTrip = cost - calculator.calcTripCost(arrivalTime - curRequest.requestTime);
            const AccessRPTrip newResult(costWithoutTrip, arrivalTime, asgn, insertionType);

            // Check if new result is dominated
            const auto [start, end] = pos[stationId];
            for (int i = start; i < end; ++i) {
                const AccessRPTrip &existingResult = paretoResults[i];
                if (dominates(existingResult, newResult)) {
                    return false; // new result is dominated, discard it
                }
            }

            // If station has no results yet, mark that it is now active
            if (pos[stationId].end == pos[stationId].start)
                stationsWithResults.push_back(stationId);

            // Remove all that are dominated by new result
            removalIf(stationId, [&](const AccessRPTrip &existingResult) {
                return dominates(newResult, existingResult);
            }, pos, paretoResults);

            // Add new result to pareto results
            insertion(stationId, newResult, pos, paretoResults);
            return true;
        }

        const std::vector<int> &getStationsWithResults() const {
            return stationsWithResults;
        }

        ConstantVectorRange<AccessRPTrip> getResultsForStation(const int stationId) const {
            KASSERT(stationId >= 0 && stationId < pos.size());
            return {paretoResults.begin() + pos[stationId].start, paretoResults.begin() + pos[stationId].end};
        }

        std::vector<AccessRPTrip> getValidResults() const {
            std::vector<AccessRPTrip> validResults;
            for (const auto &stationId: stationsWithResults) {
                const auto [start, end] = pos[stationId];
                for (int i = start; i < end; ++i) {
                    validResults.push_back(paretoResults[i]);
                }
            }
            return validResults;
        }

    private:
        const RouteState &routeState;
        CostCalculator calculator;
        int externalUpperBoundCost;
        Request curRequest;

        // One bucket per station. Bucket for station S contains all taxi results for S that are pareto-optimal with
        // respect to cost and arrival time.
        std::vector<ValueBlockPosition> pos;
        std::vector<AccessRPTrip> paretoResults;

        std::vector<int> stationsWithResults;
    };
} // namespace karri
