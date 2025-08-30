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

    const int &getBestCost() const { return bestCost; }

    const int getCostWithoutTripTime() const { 
        return CostCalculator::calcPTJourneyCostWithoutTripTime(getTotalTransferTime(bestJourney), getNumberOfTransfers(bestJourney)); 
    }

    // Best cost from the pareto front
    const Journey &getBestJourney() const {
        return bestJourney;
    }

    // at Destination
    const int getArrivalTime() const {
        return bestJourney.back().arrivalTime / 10;
    }

    const int getArrivalTimeAtLastStation() const {
        return bestJourney[bestJourney.size() - 2].arrivalTime / 10;
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
        return valid ? RAPTOR::totalTransferTime(journey) / 10 : 0;
    }

    inline const int getTotalTripTime(Journey journey) const {
        return valid ? (journey.back().arrivalTime - journey.front().departureTime) / 10 : 0;
    }

    inline const int getNumberOfTransfers(Journey journey) const {
        return valid ? RAPTOR::countTrips(journey) - 1 : 0;
    }
    
private:
    bool valid;
    int bestCost;
    Journey bestJourney;
};

enum InsertionType {
    PALS,
    DALS,
    DALS_PBNS,
    ORDINARY,
    PBNS,
    UNDEFINED
};

struct TaxiResult {    
    TaxiResult() noexcept = default;
    TaxiResult(const int cost, const int arrivalTime, const Assignment &asgn, InsertionType insertionType) noexcept
        : bestCost(cost), arrivalTime(arrivalTime), bestAssignment(asgn), insertionType(insertionType) {}

    int bestCost = INFTY;
    int arrivalTime = INFTY;
    Assignment bestAssignment;
    InsertionType insertionType = UNDEFINED;
};

class FirstTaxiLegResult {
public:

    explicit FirstTaxiLegResult(const RouteState &routeState, const RequestState &requestState, const int numStations)
            : results(numStations), routeState(routeState), requestState(requestState), calculator(routeState), 
              worstCostForAllStations(INFTY), worstAssignmentForAllStations() {
                assert(numStations >= 0);
            }

    bool tryAssignmentWithKnownCostForStation(const int stationId, const Assignment &asgn, const int cost, InsertionType insertionType) {
        if (stationId < 0 || stationId >= results.size()) return false;
        TaxiResult &taxiResult = results[stationId];

        if (cost < INFTY && (cost < taxiResult.bestCost || (cost == taxiResult.bestCost &&
                                breakCostTie(asgn, taxiResult.bestAssignment)))) {

            taxiResult = TaxiResult(cost, calcArrivalTime(asgn), asgn, insertionType);
            if (worstCostForAllStations == INFTY || taxiResult.bestCost > worstCostForAllStations) {
                worstCostForAllStations = taxiResult.bestCost;
                worstAssignmentForAllStations = taxiResult.bestAssignment;
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

    const TaxiResult &getResultForStation(const int stationId) const {
        assert(stationId >= 0 && stationId < results.size());
        return results[stationId];
    }

    const int getCostWithoutTripTimeForStation(const int stationId) const {
        assert(stationId >= 0 && stationId < results.size());
        auto result = results[stationId];
        return calculator.calc(result.bestAssignment, requestState, true);
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
    CostCalculator calculator;
    std::vector<TaxiResult> results;
    int worstCostForAllStations;
    Assignment worstAssignmentForAllStations;
};

template<typename TaxiLegApproximationT>
class IntermediateResult {
public:
    IntermediateResult(const int requestTime, const int maxTripTime, const FirstTaxiLegResult &firstTaxiLegResult, 
                       const PTResult &ptLeg, const TaxiLegApproximationT &taxiLegApproximation)
                        : requestTime(requestTime),
                        maxTripTime(maxTripTime),
                        firstTaxiLeg(), 
                        ptLeg(ptLeg), 
                        firstStationId(ptLeg.getFirstStation()), 
                        lastStationId(ptLeg.getLastStation()), 
                        firstTaxiLegCost(),
                        secondTaxiLegCost(),
                        ptLegCost(ptLeg.getBestCost()),
                        arrivalTime(ptLeg.getArrivalTime()),
                        bestCost() {

        if (isInitialTransferByTaxi()) {
            firstTaxiLeg = firstTaxiLegResult.getResultForStation(ptLeg.getFirstStation());
            firstTaxiLegCost = firstTaxiLeg.bestCost;
            bestCost += firstTaxiLegResult.getCostWithoutTripTimeForStation(ptLeg.getFirstStation()); 
        }

        bestCost += ptLeg.getCostWithoutTripTime();

        if (isFinalTransferByTaxi()) {
            secondTaxiLegCost = taxiLegApproximation.getCostForStation(lastStationId);
            arrivalTime += taxiLegApproximation.getDistanceFromStation(lastStationId);
        }

        int totalTripCost = CostCalculator::calcTripCost(arrivalTime - requestTime, maxTripTime);
        bestCost += totalTripCost;
    }

    const bool isInitialTransferByTaxi() const {
        return ptLeg.isInitialTransferByTaxi();
    }
        
    const bool isFinalTransferByTaxi() const {
        return ptLeg.isFinalTransferByTaxi();
    }

    const InsertionType getFirstTaxiLegInsertionType() const {
        return firstTaxiLeg.insertionType;
    }

    const int &getFirstTaxiLegCost() const { return firstTaxiLegCost; }

    const int &getPTLegCost() const { return ptLegCost; }

    const int &getSecondTaxiLegCost() const { return secondTaxiLegCost; }

    const int &getBestCost() const { return bestCost; }

    const int &getArrivalTime() const {
        return arrivalTime;
    }

private:
    const int requestTime;
    const int maxTripTime;

    TaxiResult firstTaxiLeg;
    PTResult ptLeg;
    int firstStationId;
    int lastStationId;
    int firstTaxiLegCost;
    int secondTaxiLegCost;
    int ptLegCost;
    int bestCost;
    int arrivalTime;
};

} // namespace karri
