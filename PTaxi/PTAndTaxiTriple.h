#pragma once

#include <utility>
#include <limits>
#include <KARRI/Algorithms/KaRRi/RequestState/RequestState.h>
#include "ApproximateCombinedTripResult.h"

namespace karri {

class PTAndTaxiTriple {
public:
    PTAndTaxiTriple() = default;
    
    PTAndTaxiTriple(RequestState firstTaxiLeg,
                    PTResult ptLeg, 
                    bool hasSecondTaxiLeg,
                    int firstTaxiLegCost,
                    int ptLegCost,
                    int secondTaxiLegApproxCost
                )
        : firstTaxiLeg(firstTaxiLeg),
          ptLeg(std::move(ptLeg)), 
          hasSecondTaxiLeg(hasSecondTaxiLeg),
          firstTaxiLegCost(firstTaxiLegCost),
          ptLegCost(ptLegCost),
          secondTaxiLegApproxCost(secondTaxiLegApproxCost) {}
    
    const RequestState& getFirstTaxiLeg() const { return firstTaxiLeg; }
    const PTResult& getPTLeg() const { return ptLeg; }
    
    bool hasValidFirstTaxiLeg() const { 
        return firstTaxiLeg.getBestAssignment().vehicle != nullptr && !firstTaxiLeg.isNotUsingVehicleBest();
    }
    
    bool hasValidPTLeg() const { 
        return ptLeg.isValid(); 
    }
    
    // Check if this is a valid taxi-only trip
    bool isValidTaxiOnlyTrip() const {
        return hasValidFirstTaxiLeg() && !hasValidPTLeg() && !hasSecondTaxiLeg;
    }
    
    // Check if this is a valid PT-only trip
    bool isValidPTOnlyTrip() const {
        return !hasValidFirstTaxiLeg() && hasValidPTLeg() && !hasSecondTaxiLeg;
    }

    // Cost related
    int firstTaxiLegCost;         // Approximated cost of the combined trip
    int ptLegCost;                // Cost of the PT leg
    int secondTaxiLegApproxCost;  // Approximated cost of the second taxi leg

private:
    RequestState firstTaxiLeg;  // First taxi leg result
    PTResult ptLeg;               // Public transit leg result
    bool hasSecondTaxiLeg;        // Flag indicating if there is a second taxi leg
};

} // namespace karri
