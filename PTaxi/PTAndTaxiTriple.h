#pragma once

#include <utility>
#include <limits>
#include <KARRI/Algorithms/KaRRi/RequestState/RequestState.h>
#include "Results.h"

namespace karri {

class PTAndTaxiTriple {
public:
    PTAndTaxiTriple() = default;
    
    PTAndTaxiTriple(std::pair<RequestState, stats::DispatchingPerformanceStats> firstTaxiLeg, 
                    PTResult ptLeg, 
                    bool hasSecondTaxiLeg,
                    int firstTaxiLegCost,
                    int ptLegCost,
                    int secondTaxiLegApproxCost
                )
        : firstTaxiLeg(std::move(firstTaxiLeg)), 
          ptLeg(std::move(ptLeg)), 
          hasSecondTaxiLeg(hasSecondTaxiLeg),
          firstTaxiLegCost(firstTaxiLegCost),
          ptLegCost(ptLegCost),
          secondTaxiLegApproxCost(secondTaxiLegApproxCost) {}
    
    std::pair<RequestState, stats::DispatchingPerformanceStats>& getFirstTaxiLeg() { return firstTaxiLeg; }
    PTResult& getPTLeg() { return ptLeg; }
    
    bool hasValidFirstTaxiLeg() const { 
        return firstTaxiLeg.first.getBestAssignment().vehicle != nullptr || firstTaxiLeg.first.isNotUsingVehicleBest(); 
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
    std::pair<RequestState, stats::DispatchingPerformanceStats> firstTaxiLeg;  // First taxi leg result
    PTResult ptLeg;               // Public transit leg result
    bool hasSecondTaxiLeg;        // Flag indicating if there is a second taxi leg
};

} // namespace karri
