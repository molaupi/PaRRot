#pragma once

#include <vector>
#include <ULTRA/DataStructures/RAPTOR/Entities/Journey.h>

namespace karri {

class PTResult {
    using Journey = RAPTOR::Journey;
public:
    PTResult() : valid(false), bestCost(INFTY) {}

    PTResult(std::vector<Journey> &journeyParetoFront, RequestState &curReqState) 
        : bestCost(INFTY), valid(false) {
            int cost;
            for (Journey &journey: journeyParetoFront) {
                cost = CostCalculator::calcPTJourneyCost(
                                                getTotalTripTime(journey), 
                                                getTotalTransferTime(journey), 
                                                getNumberOfTransfers(journey),
                                                curReqState);
                if (cost < bestCost) {
                    valid = true;
                    bestCost = cost;
                    bestJourney = journey;
                };
            }
        }
    
    // Flag to indicate if this PT result is valid or not
    bool isValid() const { return valid; }
    void setValid(bool isValid) { valid = isValid; }

    const int &getBestCost() const { return bestCost; }

    const int getCostWithoutTripTime() const { 
        return valid && !bestJourney.empty() ? 
        CostCalculator::calcPTJourneyCostWithoutTripTime(
            getTotalTransferTime(bestJourney), 
            getNumberOfTransfers(bestJourney)
        ) : 0;
    }

    const Journey &getBestJourney() const {
        return bestJourney;
    }

    // at Destination
    const int getArrivalTime() const {
        return valid && !bestJourney.empty() ? 
            convertToKaRRiTime(bestJourney.back().arrivalTime) : 
            INFTY;
    }

    const int getArrivalTimeAtLastStation() const {
        return valid && bestJourney.size() >= 2 ? 
            convertToKaRRiTime(bestJourney[bestJourney.size() - 2].arrivalTime) : 
            INFTY;
    }

    const int getFirstStation() const {
        return bestJourney.empty() ? INVALID_ID : bestJourney.front().to.value();
    }

    const int getLastStation() const {
        return bestJourney.empty() ? INVALID_ID : bestJourney.back().from.value();
    }

    const bool isInitialTransferByTaxi() const {
        return bestJourney.empty() ? false : bestJourney.front().usesTaxi;
    }

    const bool isFinalTransferByTaxi() const {
        return bestJourney.empty() ? false : bestJourney.back().usesTaxi;
    }

    inline const int getTotalTransferTime(Journey journey) const {
        return convertToKaRRiTime(RAPTOR::totalTransferTime(journey));
    }

    inline const int getTotalTripTime(Journey journey) const {
        return convertToKaRRiTime(journey.back().arrivalTime - journey.front().departureTime);
    }

    inline const int getNumberOfTransfers(Journey journey) const {
        const int numTrips = RAPTOR::countTrips(journey);
        return numTrips == 0 ? 0 : numTrips - 1;
    }
    
private:
    const int convertToKaRRiTime(const int timeInSeconds) const {
        return timeInSeconds * 10;
    }

    bool valid;
    int bestCost;
    Journey bestJourney;
};
} // namespace karri