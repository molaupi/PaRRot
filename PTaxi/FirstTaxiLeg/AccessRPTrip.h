#pragma once
#include <KARRI/Algorithms/KaRRi/BaseObjects/Assignment.h>
#include "InsertionType.h"
#include "Common/Constants.h"

namespace parrot {

    using karri::Assignment, karri::InsertionType;

    struct AccessRPTrip {
        AccessRPTrip() noexcept = default;

        AccessRPTrip(const int costWithoutTrip, const int arrivalTime, const int nextDepartureIndex, const Assignment &asgn, const InsertionType insertionType) noexcept
            : costWithoutTrip(costWithoutTrip), arrivalTime(arrivalTime), nextDepartureIndex(nextDepartureIndex), bestAssignment(asgn), insertionType(insertionType) {
        }

        int costWithoutTrip = INFTY;
        int arrivalTime = INFTY;
        int nextDepartureIndex = INFTY;
        Assignment bestAssignment;
        InsertionType insertionType = InsertionType::UNDEFINED;

        bool operator==(const AccessRPTrip &other) const {
            return costWithoutTrip == other.costWithoutTrip && arrivalTime == other.arrivalTime;
        }

        const int &getBestCostWithoutTrip() const {
            return costWithoutTrip;
        }

        const Assignment &getBestAssignment() const {
            return bestAssignment;
        }

        bool isValid() const {
            return bestAssignment.vehicle && bestAssignment.pickup.id != INVALID_ID && bestAssignment.dropoff.id !=
                   INVALID_ID;
        }

        friend bool dominates(const AccessRPTrip &r1, const AccessRPTrip &r2) {
            if (r1.costWithoutTrip > r2.costWithoutTrip || r1.nextDepartureIndex > r2.nextDepartureIndex)
                return false;
            const bool departureIndexStronglyBetter = r1.nextDepartureIndex < r2.nextDepartureIndex;
            const bool costStronglyBetter = r1.costWithoutTrip < r2.costWithoutTrip || (r1.costWithoutTrip == r2.costWithoutTrip && breakCostTie(r1.bestAssignment, r2.bestAssignment));
            return departureIndexStronglyBetter || costStronglyBetter;
        }
    };
}
