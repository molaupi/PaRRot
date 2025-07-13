#pragma once

#include <vector>
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

class FirstTaxiLegResult {
    struct StationCost {
        StationCost() noexcept = default;
        StationCost(const int stationId, const int cost, const Assignment &asgn) noexcept
            : bestCost(cost), stationId(stationId), bestAssignment(asgn) {}

        int stationId = INVALID_ID;
        int bestCost = INFTY;
        Assignment bestAssignment;
    };


public:
    explicit FirstTaxiLegResult(const int numStations): results(numStations) {
            assert(numStations >= 0);
        }

    bool tryAssignmentWithKnownCostForStation(const int stationId, const Assignment &asgn, const int cost) {
        if (stationId < 0 || stationId >= results.size()) return false;
        StationCost &stationCost = results[stationId];

        if (cost < INFTY && (cost < stationCost.bestCost || (cost == stationCost.bestCost &&
                                breakCostTie(asgn, stationCost.bestAssignment)))) {

            stationCost.bestAssignment = asgn;
            stationCost.bestCost = cost;
            return true;
        }
        return false;
    }

private:
    std::vector<StationCost> results;
};

} // namespace karri
