#pragma once
#include "Common/Constants.h"
#include "KARRI/Algorithms/KaRRi/CostCalculator.h"
#include "ULTRA/DataStructures/RAPTOR/Entities/Journey.h"
#include "util.h"

namespace karri {

    template<typename Journeys>
    std::pair<RAPTOR::Journey, int> chooseBestJourney(const Journeys &journeys, const int reqTime) {
        int bestCost = INFTY;
        RAPTOR::Journey bestJourney;
        for (const auto &journey: journeys) {
            const int cost = CostCalculator::calcPTJourneyCost(
                                            parrot::ultraToKarriTime(journey.back().arrivalTime) - reqTime,
                                            getTotalTransferTime(journey),
                                            getNumberOfTransfers(journey));
            if (cost < bestCost) {
                bestCost = cost;
                bestJourney = journey;
            };
        }
        return {bestJourney, bestCost};
    }
}