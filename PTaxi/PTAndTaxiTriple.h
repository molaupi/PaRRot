#pragma once

#include <utility>
#include <limits>
#include <KARRI/Algorithms/KaRRi/RequestState/RequestState.h>
#include "Results.h"

namespace karri {

class PTAndTaxiTriple {
public:
    PTAndTaxiTriple() = default;
    
    PTAndTaxiTriple(RequestState &firstTaxiLeg, PTResult &ptLeg, RequestState &secondTaxiLeg)
        : firstTaxiLeg(firstTaxiLeg), 
          ptLeg(ptLeg), 
          secondTaxiLeg(secondTaxiLeg) {}
    
    RequestState& getFirstTaxiLeg() { return firstTaxiLeg; }
    PTResult& getPTLeg() { return ptLeg; }
    RequestState& getSecondTaxiLeg() { return secondTaxiLeg; }

    const int getTotalCost() const {
        return firstTaxiLeg.getBestCost() + ptLeg.getBestCost() + secondTaxiLeg.getBestCost();
    }
    
    bool hasValidFirstTaxiLeg() const { 
        // check whether vehicle is set
        return firstTaxiLeg.getBestAssignment().vehicle != nullptr || firstTaxiLeg.isNotUsingVehicleBest(); 
    }
    
    bool hasValidPTLeg() const { 
        return ptLeg.isValid(); 
    }
    
    bool hasValidSecondTaxiLeg() const { 
        // check whether vehicle is set
        return secondTaxiLeg.getBestAssignment().vehicle != nullptr || secondTaxiLeg.isNotUsingVehicleBest(); 
    }
    
    // Check if this is a valid combined trip (taxi + PT + taxi)
    bool isValidCombinedTrip() const {
        return hasValidFirstTaxiLeg() && hasValidPTLeg() && hasValidSecondTaxiLeg();
    }
    
    // Check if this is a valid taxi-only trip
    bool isValidTaxiOnlyTrip() const {
        return hasValidFirstTaxiLeg() && !hasValidPTLeg() && !hasValidSecondTaxiLeg();
    }
    
    // Check if this is a valid PT-only trip
    bool isValidPTOnlyTrip() const {
        return !hasValidFirstTaxiLeg() && hasValidPTLeg() && !hasValidSecondTaxiLeg();
    }

private:
    RequestState firstTaxiLeg;  // First taxi leg result
    PTResult ptLeg;             // Public transit leg result
    RequestState secondTaxiLeg; // Second taxi leg result
};

} // namespace karri
