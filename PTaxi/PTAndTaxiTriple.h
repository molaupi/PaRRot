#pragma once

#include <utility>
#include <limits>
#include <KARRI/Algorithms/KaRRi/RequestState/RequestState.h>
#include "Results.h"

namespace karri {

class PTAndTaxiTriple {
public:
    PTAndTaxiTriple() = default;
    
    PTAndTaxiTriple(std::pair<RequestState, stats::DispatchingPerformanceStats> firstTaxiLeg, PTResult ptLeg, bool hasSecondTaxiLeg)
        : firstTaxiLeg(std::move(firstTaxiLeg)), 
          ptLeg(std::move(ptLeg)), 
          hasSecondTaxiLeg(hasSecondTaxiLeg) {}
    
    std::pair<RequestState, stats::DispatchingPerformanceStats>& getFirstTaxiLeg() { return firstTaxiLeg; }
    PTResult& getPTLeg() { return ptLeg; }
    
    bool hasValidFirstTaxiLeg() const { 
        // check whether vehicle is set
        return firstTaxiLeg.first.getBestAssignment().vehicle != nullptr || firstTaxiLeg.first.isNotUsingVehicleBest(); 
    }
    
    bool hasValidPTLeg() const { 
        return ptLeg.isValid(); 
    }
    
    // Check if this is a valid combined trip (taxi + PT + taxi)
    bool isValidCombinedTrip() const {
        return hasValidFirstTaxiLeg() && hasValidPTLeg() && hasSecondTaxiLeg;
    }
    
    // Check if this is a valid taxi-only trip
    bool isValidTaxiOnlyTrip() const {
        return hasValidFirstTaxiLeg() && !hasValidPTLeg() && !hasSecondTaxiLeg;
    }
    
    // Check if this is a valid PT-only trip
    bool isValidPTOnlyTrip() const {
        return !hasValidFirstTaxiLeg() && hasValidPTLeg() && !hasSecondTaxiLeg;
    }

private:
    std::pair<RequestState, stats::DispatchingPerformanceStats> firstTaxiLeg;  // First taxi leg result
    PTResult ptLeg;             // Public transit leg result
    bool hasSecondTaxiLeg;      // Flag indicating if there is a second taxi leg
};

} // namespace karri
