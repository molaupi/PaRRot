#pragma once

#include <vector>
#include <ULTRA/DataStructures/RAPTOR/Entities/Journey.h>
#include "FirstTaxiLeg/FirstTaxiLegResult.h"
#include "PTLeg/PTResult.h"

namespace karri {

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

        if (!ptLeg.isValid()) {
            bestCost = INFTY;
            return;
        }

        if (isInitialTransferByTaxi()) {
            firstTaxiLeg = firstTaxiLegResult.getResultForStation(firstStationId);
            firstTaxiLegCost = firstTaxiLeg.bestCost;
            bestCost += firstTaxiLegResult.getCostWithoutTripTimeForStation(firstStationId); 
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

    const int &getArrivalTime() const { return arrivalTime; }

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
