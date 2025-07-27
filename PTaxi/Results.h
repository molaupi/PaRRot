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
        StationCost(const int stationId, const int cost, const int arrivalTime, const Assignment &asgn) noexcept
            : stationId(stationId), bestCost(cost), arrivalTime(arrivalTime), bestAssignment(asgn) {}

        int stationId = INVALID_ID;
        int bestCost = INFTY;
        int arrivalTime = INFTY;
        Assignment bestAssignment;
    };


public:
    explicit FirstTaxiLegResult(const RouteState &routeState, const RequestState &requestState, const int numStations)
            : results(numStations), routeState(routeState), requestState(requestState), worstCostForAllStations(INFTY), worstAssignmentForAllStations() {
                assert(numStations >= 0);
            }

    bool tryAssignmentWithKnownCostForStation(const int stationId, const Assignment &asgn, const int cost) {
        if (stationId < 0 || stationId >= results.size()) return false;
        StationCost &stationCost = results[stationId];

        if (cost < INFTY && (cost < stationCost.bestCost || (cost == stationCost.bestCost &&
                                breakCostTie(asgn, stationCost.bestAssignment)))) {

            stationCost.bestAssignment = asgn;
            stationCost.bestCost = cost;
            stationCost.arrivalTime = calcArrivalTime(asgn);
            if (stationCost.bestCost > worstCostForAllStations) {
                worstCostForAllStations = stationCost.bestCost;
                worstAssignmentForAllStations = stationCost.bestAssignment;
            }
            return true;
        }
        return false;
    }

    const Assignment &getWorstAssignmentForAllStations() const {
        return worstAssignmentForAllStations;
    }

    const int &getWorstCostForAllStations() const {
        return worstCostForAllStations;
    }

    const StationCost &getStationCost(const int stationId) const {
        assert(stationId >= 0 && stationId < results.size());
        return results[stationId];
    }

private:
    int calcArrivalTime(const Assignment &asgn) {
        using namespace time_utils;
        const int actualDepTimeAtPickup = getActualDepTimeAtPickup(asgn, requestState, routeState);
        const int initialPickupDetour = calcInitialPickupDetour(asgn, actualDepTimeAtPickup, requestState, routeState);
        const bool dropoffAtExistingStop = isDropoffAtExistingStop(asgn, routeState);
        return getArrTimeAtDropoff(actualDepTimeAtPickup, asgn, initialPickupDetour, dropoffAtExistingStop, routeState);
    }

    const RouteState &routeState;
    const RequestState &requestState;
    std::vector<StationCost> results;
    int worstCostForAllStations;
    Assignment worstAssignmentForAllStations;
};

} // namespace karri
