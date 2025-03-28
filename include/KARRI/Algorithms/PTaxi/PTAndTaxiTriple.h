#pragma once

#include "../KaRRi/RequestState/RequestState.h"
#include <utility>
#include <limits>

namespace karri {

// TODO: Define the proper PT result type based ULTRARAPTOR result
class PTResult {
public:
    PTResult() = default;

    PTResult(bool valid) : valid(valid) {}
    
    // Flag to indicate if this PT result is valid or not
    bool isValid() const { return valid; }
    void setValid(bool isValid) { valid = isValid; }
    
private:
    bool valid = false;
};

class PTAndTaxiTriple {
public:
    PTAndTaxiTriple() = default;
    
    PTAndTaxiTriple(RequestState &firstTaxiLeg, PTResult &ptLeg, RequestState &secondTaxiLeg)
        : firstTaxiLeg(firstTaxiLeg), 
          ptLeg(ptLeg), 
          secondTaxiLeg(secondTaxiLeg) {}
    
    const RequestState& getFirstTaxiLeg() const { return firstTaxiLeg; }
    const PTResult& getPTLeg() const { return ptLeg; }
    const RequestState& getSecondTaxiLeg() const { return secondTaxiLeg; }
    
    bool hasValidFirstTaxiLeg() const { 
        // check whether vehicle is set
        return firstTaxiLeg.getBestAssignment().vehicle != nullptr; 
    }
    
    bool hasValidPTLeg() const { 
        return ptLeg.isValid(); 
    }
    
    bool hasValidSecondTaxiLeg() const { 
        // check whether vehicle is set
        return secondTaxiLeg.getBestAssignment().vehicle != nullptr; 
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

    RequestState& getFirstTaxiLeg() { return firstTaxiLeg; }
    PTResult& getPTLeg() { return ptLeg; }
    RequestState& getSecondTaxiLeg() { return secondTaxiLeg; }

private:
    RequestState &firstTaxiLeg;  // First taxi leg result
    PTResult &ptLeg;             // Public transit leg result
    RequestState &secondTaxiLeg; // Second taxi leg result
};

} // namespace karri
