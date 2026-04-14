#pragma once
#include "Common/Constants.h"
#include "KARRI/Algorithms/KaRRi/CostCalculator.h"
#include "ULTRA/DataStructures/RAPTOR/Entities/Journey.h"
#include "util.h"

namespace karri {

    template<typename Journeys>
    RAPTOR::Journey chooseBestJourney(const Journeys &journeys) {
        int bestCost = INFTY;
        RAPTOR::Journey bestJourney;
        for (const auto &journey: journeys) {
            const int cost = CostCalculator::calcPTJourneyCost(
                                            getTotalTripTime(journey),
                                            getTotalTransferTime(journey),
                                            getNumberOfTransfers(journey));
            if (cost < bestCost) {
                bestCost = cost;
                bestJourney = journey;
            };
        }
        return bestJourney;
    }
}