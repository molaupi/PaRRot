#pragma once

#include <utility>
#include <limits>
#include <KARRI/Algorithms/KaRRi/RequestState/RequestState.h>
#include <ULTRA/DataStructures/RAPTOR/Entities/Journey.h>

namespace karri {

// TODO: Define the proper PT result type based ULTRARAPTOR result
class PTResult {
    using Journey = RAPTOR::Journey;
public:
    PTResult() : valid(false), bestCost(INFTY) {}

    PTResult(std::vector<Journey> &journeyParetoFront, RequestState &firstTaxiLeg) 
        : bestCost(INFTY), valid(true) {
            int cost;
            for (Journey &journey: journeyParetoFront) {
                cost = CostCalculator::calcPTJourneyCost(
                                                getTotalTripTime(journey), 
                                                getTotalTransferTime(journey), 
                                                getNumberOfTransfers(journey),
                                                firstTaxiLeg);
                if (cost < bestCost) {
                    bestCost = cost;
                    bestJourney = journey;
                };
            }
        }
    
    // Flag to indicate if this PT result is valid or not
    bool isValid() const { return valid; }
    void setValid(bool isValid) { valid = isValid; }

    const int &getBestCost() const { return bestCost;}

    // Best cost from the pareto front
    const Journey &getBestJourney() const {
        return bestJourney;
    }

    inline const int getTotalTransferTime(Journey journey) const {
        return valid ? RAPTOR::totalTransferTime(journey) : 0;
    }

    inline const int getTotalTripTime(Journey journey) const {
        return valid ? journey.back().arrivalTime - journey.front().departureTime : 0;
    }

    inline const int getNumberOfTransfers(Journey journey) const {
        return valid ? RAPTOR::countTrips(journey) - 1 : 0;
    }
    
private:
    bool valid;
    int bestCost;
    Journey bestJourney;
};

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
