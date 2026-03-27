#pragma once

#include "FirstTaxiLeg/FirstTaxiLegResult.h"
#include "PTLeg/PTResult.h"

namespace karri {

class ApproximateCombinedTripResult {
public:
    ApproximateCombinedTripResult(const int requestTime,
        const int maxTripTime,
        const bool firstLegByTaxi,
        const bool lastLegByTaxi,
        const FirstTaxiLegResult &firstTaxiLegResult,
                       const PTResult &ptLeg,
                       const int secondLegApproximationCost,
                       const int secondLegApproximationTravelTime)
                        : requestTime(requestTime),
                        maxTripTime(maxTripTime),
                        firstTaxiLeg(), 
                        ptLeg(ptLeg), 
                        firstStationId(ptLeg.getFirstStation()), 
                        lastStationId(ptLeg.getLastStation()), 
                        firstTaxiLegCost(),
                        secondTaxiLegCost(INFTY),
                        ptLegCost(ptLeg.getCost()),
                        bestCost(),
                        arrivalTime(ptLeg.getArrivalTime()) {

        if (!ptLeg.isValid()) {
            bestCost = INFTY;
            return;
        }

        if (firstLegByTaxi) {
            firstTaxiLeg = firstTaxiLegResult.getResultForStation(firstStationId);
            firstTaxiLegCost = firstTaxiLeg.bestCost;
            bestCost += firstTaxiLegResult.getCostWithoutTripTimeForStation(firstStationId); 
        }

        bestCost += ptLeg.getCost();

        if (lastLegByTaxi) {
            secondTaxiLegCost = secondLegApproximationCost;
            arrivalTime += secondLegApproximationTravelTime;
        }

        int totalTripCost = CostCalculator::calcTripCost(arrivalTime - requestTime, maxTripTime);
        bestCost += totalTripCost;
    }

    bool isInitialTransferByTaxi() const {
        return firstTaxiLeg.isValid();
    }
        
    bool isFinalTransferByTaxi() const {
        return secondTaxiLegCost != INFTY;
    }

    InsertionType getFirstTaxiLegInsertionType() const {
        return firstTaxiLeg.insertionType;
    }

    const TaxiResult &getFirstTaxiLeg() const {
        return firstTaxiLeg;
    }

    const PTResult &getPTLeg() const {
        return ptLeg;
    }

    const int &getFirstTaxiLegCost() const { return firstTaxiLegCost; }

    const int &getPTLegCost() const { return ptLegCost; }

    const int &getSecondTaxiLegCost() const { return secondTaxiLegCost; }

    const int &getBestCost() const { return bestCost; }

    const int &getArrivalTime() const { return arrivalTime; }

    bool isValid() const {
        return ptLeg.isValid() && firstTaxiLeg.isValid();
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
