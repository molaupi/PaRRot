#pragma once
#include "Assignment.h"
#include "Common/Constants.h"

namespace karri {
    struct InternalTaxiResult {
        InternalTaxiResult() noexcept = default;

        InternalTaxiResult(const int cost, const Assignment &asgn) noexcept
            : bestCost(cost), bestAssignment(asgn) {
        }

        int bestCost = INFTY;
        Assignment bestAssignment;

        bool operator==(const InternalTaxiResult &other) const {
            return bestCost == other.bestCost;
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
