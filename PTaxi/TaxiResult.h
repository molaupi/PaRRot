#pragma once
#include <KARRI/Algorithms/KaRRi/BaseObjects/Assignment.h>
#include "InsertionType.h"
#include "Common/Constants.h"

namespace karri {
    struct TaxiResult {
        TaxiResult() noexcept = default;

        TaxiResult(const int cost, const int arrivalTime, const Assignment &asgn, InsertionType insertionType) noexcept
            : bestCost(cost), arrivalTime(arrivalTime), bestAssignment(asgn), insertionType(insertionType) {
        }

        int bestCost = INFTY;
        int arrivalTime = INFTY;
        Assignment bestAssignment;
        InsertionType insertionType = UNDEFINED;

        bool operator==(const TaxiResult &other) const {
            return bestCost == other.bestCost && arrivalTime == other.arrivalTime;
        }

        bool tryAssignmentWithKnownCost(const Assignment &asgn, const int cost) {
            if (asgn.pickup.loc == asgn.dropoff.loc)
                return false;
            if (cost < INFTY && (cost < bestCost || (cost == bestCost &&
                                                     breakCostTie(asgn, bestAssignment)))) {
                bestAssignment = asgn;
                bestCost = cost;
                return true;
            }
            return false;
        }

        const int &getBestCost() const {
            return bestCost;
        }

        const Assignment &getBestAssignment() const {
            return bestAssignment;
        }

        bool isValid() const {
            return bestAssignment.vehicle && bestAssignment.pickup.id != INVALID_ID && bestAssignment.dropoff.id !=
                   INVALID_ID;
        }
    };
}
