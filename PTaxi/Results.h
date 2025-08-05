#pragma once

#include <vector>
#include <ULTRA/DataStructures/RAPTOR/Entities/Journey.h>

namespace karri {

// TODO: Define the proper PT result type based ULTRARAPTOR result
class PTResult {
    using Journey = RAPTOR::Journey;
public:
    PTResult() : valid(false), bestCost(INFTY) {}

    PTResult(std::vector<Journey> &journeyParetoFront, RequestState &curReqState) 
        : bestCost(INFTY), valid(true) {
            int cost;
            for (Journey &journey: journeyParetoFront) {
                cost = CostCalculator::calcPTJourneyCost(
                                                getTotalTripTime(journey), 
                                                getTotalTransferTime(journey), 
                                                getNumberOfTransfers(journey),
                                                curReqState);
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

    const int getFirstStation() const {
        return bestJourney.front().to.value();
    }

    const int getLastStation() const {
        return bestJourney.back().from.value();
    }

    const bool isInitialTransferByTaxi() const {
        return bestJourney.front().usesTaxi;
    }

    const bool isFinalTransferByTaxi() const {
        return bestJourney.back().usesTaxi;
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
public:
    struct StationCost {
        StationCost() noexcept = default;
        StationCost(const int cost, const int arrivalTime, const Assignment &asgn) noexcept
            : bestCost(cost), arrivalTime(arrivalTime), bestAssignment(asgn) {}

        int bestCost = INFTY;
        int arrivalTime = INFTY;
        Assignment bestAssignment;
    };

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
            if (worstCostForAllStations == INFTY || stationCost.bestCost > worstCostForAllStations) {
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

class PreliminaryResult {
    using FirstTaxiLeg = FirstTaxiLegResult::StationCost;
public:
    PreliminaryResult(FirstTaxiLegResult &firstTaxiLegResult, PTResult &ptLeg)
        : ptLeg(ptLeg), firstStationId(ptLeg.getFirstStation()), lastStationId(ptLeg.getLastStation()), isFinalTransferByTaxi(ptLeg.isFinalTransferByTaxi()) {
            if (ptLeg.isInitialTransferByTaxi()) {
                firstTaxiLegCost = firstTaxiLegResult.getStationCost(ptLeg.getFirstStation());
            } else {
                firstTaxiLegCost = FirstTaxiLeg();
            }
        }

private:
    FirstTaxiLeg firstTaxiLegCost;
    PTResult ptLeg;
    int firstStationId;
    int lastStationId;
    bool isFinalTransferByTaxi;
};

} // namespace karri
