#pragma once
#include "KARRI/Algorithms/KaRRi/BaseObjects/InternalTaxiResult.h"

namespace parrot {

    struct TaxiResult {

        TaxiResult() : internal(), arrivalTime(INFTY) {}

        TaxiResult(const karri::InternalTaxiResult &internal, const int arrivalTime) : internal(internal), arrivalTime(arrivalTime) {}

        karri::InternalTaxiResult internal;
        int arrivalTime;

        int getArrivalTime() const {
            return arrivalTime;
        }

        const karri::Assignment &getBestAssignment() const {
            return internal.getBestAssignment();
        }

        const int &getBestCost() const {
            return internal.getBestCost();
        }

        bool isValid() const {
            return internal.isValid() && arrivalTime < INFTY;
        }
    };

}
