#pragma once

#include <vector>
#include <ULTRA/DataStructures/RAPTOR/Entities/Journey.h>

namespace karri {

class PTResult {
    using Journey = RAPTOR::Journey;
public:
    PTResult() : valid(false), bestCost(INFTY) {}

    PTResult(std::vector<Journey> &journeyParetoFront, const int &maxTripTime)
        : bestCost(INFTY), valid(false) {
            int cost;
            for (Journey &journey: journeyParetoFront) {
                cost = CostCalculator::calcPTJourneyCost(
                                                getTotalTripTime(journey), 
                                                getTotalTransferTime(journey), 
                                                getNumberOfTransfers(journey),
                                                maxTripTime);
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

    const bool isJourneyWalking() const { 
        return bestJourney.size() == 1 && 
               !bestJourney.front().usesRoute && 
               !bestJourney.front().usesTaxi; 
    }

    // at Destination
    const int getArrivalTime() const {
        return valid && !bestJourney.empty() ? 
            convertToKaRRiTime(bestJourney.back().arrivalTime) : 
            INFTY;
    }

    const int getArrivalTimeAtLastStation() const {
        if (!valid)
            return INFTY;
        for (auto it = bestJourney.rbegin(); it != bestJourney.rend(); ++it) {
            if (it->usesRoute) {
                return convertToKaRRiTime(it->arrivalTime);
            }
        }
        return INFTY;
    }

    // at Origin
    const int getDepartureTime() const {
        return valid && !bestJourney.empty() ? 
            convertToKaRRiTime(bestJourney.front().departureTime) : 
            INFTY;
    }

    const int getDepartureTimeAtFirstStation() const {
        for (const auto &leg : bestJourney) {
            if (leg.usesRoute) {
                return convertToKaRRiTime(leg.departureTime);
            }
        }
        return INFTY;
    }

    const int getWalkingTimeToFirstStation() const {
        return valid && !bestJourney.empty() ? 
            convertToKaRRiTime(bestJourney.front().arrivalTime - bestJourney.front().departureTime) : 
            INFTY;
    }

    const int getWalkingTimeFromLastStation() const {
        return valid && !bestJourney.empty() ? 
            convertToKaRRiTime(bestJourney.back().arrivalTime - bestJourney.back().departureTime) : 
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