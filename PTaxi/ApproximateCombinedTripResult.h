#pragma once

#include "FirstTaxiLeg/FirstTaxiLegResult.h"
#include "PTLeg/PTResult.h"
#include "ULTRA/DataStructures/RAPTOR/Entities/Journey.h"
#include "ULTRA/DataStructures/RAPTOR/Entities/Journey.h"

namespace karri {

class ApproximateCombinedTripResult {
public:
    template<typename SecondTaxiLegApproximationT>
    ApproximateCombinedTripResult(const int requestTime,
        TaxiResult accessRpTrip,
        RAPTOR::Journey journey,
        const SecondTaxiLegApproximationT &secondTaxiLegApproximation)
                        : requestTime(requestTime),
                        firstTaxiLeg(), 
                        ptLeg(),
                        firstStationId(INVALID_ID),
                        lastStationId(INVALID_ID),
                        firstTaxiLegCost(INFTY),
                        secondTaxiLegCost(INFTY),
                        ptLegCost(INFTY),
                        bestCost(0),
                        arrivalTime(never) {

        bool journeyHasRouteLeg = false;
        for (const auto &leg: journey) {
            if (leg.usesRoute) {
                journeyHasRouteLeg = true;
                break;
            }
        }

        const bool firstLegByTaxi = !journey.empty() && journey.front().usesTaxi;
        const bool lastLegByTaxi = !journey.empty() && journey.back().usesTaxi;
        if (!(journeyHasRouteLeg && (firstLegByTaxi || lastLegByTaxi))) {
            bestCost = INFTY;
            return;
        }
        bestCost = 0;
        arrivalTime = 0;
        if (lastLegByTaxi) {
            // remove taxi leg in end
            journey.pop_back();

            const int secondTaxiLegApproximationTravelTime = secondTaxiLegApproximation.getDistanceFromStation(journey.back().to);
            secondTaxiLegCost = CostCalculator::calcTripCost(secondTaxiLegApproximationTravelTime);
            bestCost += secondTaxiLegCost;
            arrivalTime += secondTaxiLegApproximationTravelTime;
        }
        if (firstLegByTaxi) {
            // remove taxi leg in beginning
            journey.erase(journey.begin());

            firstTaxiLeg = accessRpTrip;
            firstTaxiLegCost = firstTaxiLeg.bestCost;
            bestCost += firstTaxiLeg.bestCost;
        }
        KASSERT(!journey.empty());

        const int ptEarliestDep = firstLegByTaxi ? firstTaxiLeg.arrivalTime : requestTime;
        int numTransfers = RAPTOR::countTrips(journey);
        if (numTransfers > 0) --numTransfers; // number of transfers is number of trips - 1 but at least 0
        ptLegCost = CostCalculator::calcPTJourneyCost(
                                      parrot::ultraToKarriTime(journey.back().arrivalTime) - ptEarliestDep,
                                      parrot::ultraToKarriTime(RAPTOR::totalTransferTime(journey)),
                                      numTransfers);
        bestCost += ptLegCost;
        ptLeg = {journey, ptLegCost};
        firstStationId = ptLeg.getFirstStation();
        lastStationId = ptLeg.getLastStation();
        arrivalTime += ptLeg.getArrivalTime();

        // int totalTripCost = CostCalculator::CostFunction::calcTripCost(arrivalTime - requestTime);
        // bestCost += totalTripCost;
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
        return ptLeg.isValid() && (firstTaxiLeg.isValid() || secondTaxiLegCost != INFTY);
    }

private:
    const int requestTime;

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
