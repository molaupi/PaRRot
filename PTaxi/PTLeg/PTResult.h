#pragma once

#include <vector>
#include <ULTRA/DataStructures/RAPTOR/Entities/Journey.h>
#include "../util.h"

namespace karri {

class PTResult {
    using Journey = RAPTOR::Journey;

    static int computeCost(const Journey &journey) {
        if (journey.empty())
            return INFTY;
        return CostCalculator::calcPTJourneyCost(getTotalTripTime(journey),
                                            getTotalTransferTime(journey),
                                            getNumberOfTransfers(journey));
    }

public:
    PTResult() : valid(false), cost(INFTY) {}

    PTResult(const Journey &journey)
    : journey(journey), cost(computeCost(journey)), valid(!journey.empty()) {
        for (const auto& leg :journey) {
            KASSERT(!leg.usesTaxi);
        }
    }
    
    // Flag to indicate if this PT result is valid or not
    bool isValid() const { return valid; }
    void setValid(bool isValid) { valid = isValid; }

    const int &getCost() const { return cost; }

    const Journey &getBestJourney() const {
        return journey;
    }

    // at Destination
    int getArrivalTime() const {
        return valid && !journey.empty() ?
            convertToKaRRiTime(journey.back().arrivalTime) :
            INFTY;
    }

    int getArrivalTimeAtLastStation() const {
        if (!valid)
            return INFTY;
        for (auto it = journey.rbegin(); it != journey.rend(); ++it) {
            if (it->usesRoute) {
                return convertToKaRRiTime(it->arrivalTime);
            }
        }
        return INFTY;
    }

    // at Origin
    int getDepartureTime() const {
        return valid && !journey.empty() ?
            convertToKaRRiTime(journey.front().departureTime) :
            INFTY;
    }

    int getDepartureTimeAtFirstStation() const {
        for (const auto &leg : journey) {
            if (leg.usesRoute) {
                return convertToKaRRiTime(leg.departureTime);
            }
        }
        return INFTY;
    }

    int getWalkingTimeToFirstStation() const {
        return valid && !journey.empty() ?
            convertToKaRRiTime(journey.front().arrivalTime - journey.front().departureTime) :
            INFTY;
    }

    int getWalkingTimeFromLastStation() const {
        return valid && !journey.empty() ?
            convertToKaRRiTime(journey.back().arrivalTime - journey.back().departureTime) :
            INFTY;
    }

    int getFirstStation() const {
        return journey.empty() ? INVALID_ID : journey.front().usesRoute? journey.front().from.value() : journey.front().to.value();
    }

    int getLastStation() const {
        return journey.empty() ? INVALID_ID : journey.back().usesRoute? journey.back().to.value() : journey.back().from.value();
    }

    int getAccessEgressTransferTime() const {
        return convertToKaRRiTime(RAPTOR::initialTransferTime(journey));
    }

    int getIntermediateTransferTime() const {
        return convertToKaRRiTime(RAPTOR::intermediateTransferTime(journey));
    }

    int getTotalInVehicleTime() const {
        int totalSeconds = 0;
        for (const auto &leg: journey) {
            if (leg.usesRoute) {
                totalSeconds += leg.arrivalTime - leg.departureTime;
            }
        }
        return convertToKaRRiTime(totalSeconds);
    }

    bool valid;
    int cost;
    Journey journey;
};
} // namespace karri