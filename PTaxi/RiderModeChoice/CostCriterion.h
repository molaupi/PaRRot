#pragma once

#include <kassert/kassert.hpp>

#include "TransportMode.h"
#include "ModeChoiceInputStats.h"
#include "KARRI/Algorithms/KaRRi/RequestState/RequestState.h"

namespace parrot::mode_choice {
    // Mode choice based on cost function only.
    class CostCriterion {
    public:
        explicit CostCriterion(const RouteState &routeState)
            : routeState(routeState),
              calculator(routeState),
              walkingCost(INFTY),
              taxiCost(INFTY),
              ptCost(INFTY),
              combinedCost(INFTY) {
        }

        void init() {
            walkingCost = INFTY;
            taxiCost = INFTY;
            ptCost = INFTY;
            combinedCost = INFTY;
            stats.reset();
        }

        void registerPed(const WalkingResult &walkOnlyResult, const RequestState &) {
            walkingCost = walkOnlyResult.cost;
            stats.walkTravelTime = walkOnlyResult.walkingDist;
        }

        void registerCar(const CarResult &, const RequestState &) {
            // Car cannot be considered, since cost function is not suited for car trips.
        }

        void registerPublicTransport(const PTResult &ptOnlyResult, const RequestState &requestState) {
            ptCost = ptOnlyResult.cost;
            const int accEgrTransferTime = ptOnlyResult.getAccessEgressTransferTime();
            const int intermediateTransferTime = ptOnlyResult.getIntermediateTransferTime();
            const int ptOnlyTravelTime = ptOnlyResult.getTotalInVehicleTime();
            const int ptOnlyWaitTime = ptOnlyResult.getArrivalTime() - requestState.originalRequest.requestTime -
                                       ptOnlyTravelTime -
                                       accEgrTransferTime - intermediateTransferTime;
            const int ptOnlyAccessEgressTime = accEgrTransferTime + intermediateTransferTime;
            stats.ptTravelTime = ptOnlyTravelTime;
            stats.ptWaitTime = ptOnlyWaitTime;
            stats.ptAccEgrTime = ptOnlyAccessEgressTime;
        }

        void registerTaxi(const TaxiResult &taxiOnlyResult, const RequestState &requestState) {
            using namespace time_utils;
            const auto &bestAsgn = taxiOnlyResult.getBestAssignment();
            const auto depTimeAtPickup = getActualDepTimeAtPickup(bestAsgn, requestState, routeState);
            const auto initialPickupDetour = calcInitialPickupDetour(
                bestAsgn, depTimeAtPickup, requestState, routeState);
            const auto dropoffAtExistingStop = isDropoffAtExistingStop(bestAsgn, routeState);
            const auto arrTimeAtDropoff = getArrTimeAtDropoff(depTimeAtPickup, bestAsgn, initialPickupDetour,
                                                              dropoffAtExistingStop, routeState);
            const int taxiOnlyTravelTime = arrTimeAtDropoff - depTimeAtPickup;
            const int taxiOnlyWaitTime = depTimeAtPickup - requestState.originalRequest.requestTime - bestAsgn.pickup.
                                         walkingDist;
            const int taxiOnlyAccessEgressTime = bestAsgn.pickup.walkingDist + bestAsgn.dropoff.walkingDist;

            // Taxi trips with an excessive wait time do not take part in mode choice
            if (taxiOnlyWaitTime <= InputConfig::getInstance().modeChoiceMaxTaxiWaitTime) {
                taxiCost = taxiOnlyResult.bestCost;
            }
            stats.taxiTravelTime = taxiOnlyTravelTime;
            stats.taxiWaitTime = taxiOnlyWaitTime;
            stats.taxiAccEgrTime = taxiOnlyAccessEgressTime;
        }

        void registerCombined(const ApproximateCombinedTripResult &approxCombinedResult,
                              const RequestState &requestState) {
            using namespace time_utils;

            int combinedTravelTime = 0;
            int combinedWaitTime = 0;
            int combinedAccessEgressTime = 0;

            // First taxi leg
            int ptLegStartTime = requestState.originalRequest.requestTime;
            if (approxCombinedResult.isInitialTransferByTaxi()) {
                const auto &flBestAsgn = approxCombinedResult.getFirstTaxiLeg().getBestAssignment();
                const auto flDepTimeAtPickup = getActualDepTimeAtPickup(flBestAsgn, requestState, routeState);
                const auto flInitialPickupDetour = calcInitialPickupDetour(
                    flBestAsgn, flDepTimeAtPickup, requestState, routeState);
                const auto flDropoffAtExistingStop = isDropoffAtExistingStop(flBestAsgn, routeState);
                const auto flArrTimeAtDropoff = getArrTimeAtDropoff(flDepTimeAtPickup, flBestAsgn,
                                                                    flInitialPickupDetour,
                                                                    flDropoffAtExistingStop, routeState);
                combinedTravelTime += flArrTimeAtDropoff - flDepTimeAtPickup;
                combinedWaitTime += flDepTimeAtPickup - requestState.originalRequest.requestTime -
                        flBestAsgn.pickup.walkingDist;
                combinedAccessEgressTime += flBestAsgn.pickup.walkingDist + flBestAsgn.dropoff.walkingDist;

                // If there is a first taxi leg, then the PT leg starts at the dropoff time of the first taxi leg
                ptLegStartTime = flArrTimeAtDropoff;
            }

            // Combined journeys that use an access RP trip with excessive wait time do not take part in mode choice
            if (combinedWaitTime > InputConfig::getInstance().modeChoiceMaxTaxiWaitTime)
                return;

            // PT leg
            const auto &approxPtLeg = approxCombinedResult.getPTLeg();
            const int ptAccEgrTransferTime = approxPtLeg.getAccessEgressTransferTime();
            const int intermediateTransferTime = approxPtLeg.getIntermediateTransferTime();
            const int ptInVehicleTime = approxPtLeg.getTotalInVehicleTime();
            combinedTravelTime += ptInVehicleTime;
            combinedWaitTime += approxPtLeg.getArrivalTime() - ptLegStartTime - ptInVehicleTime -
                    ptAccEgrTransferTime - intermediateTransferTime;
            combinedAccessEgressTime += ptAccEgrTransferTime + intermediateTransferTime;

            // Second taxi leg
            if (approxCombinedResult.isFinalTransferByTaxi()) {
                // Real travel time approximated by linear function with parameters based on known KaRRi data.
                static const double &m = InputConfig::getInstance().parrotEgressTravelTimeHeuristicSlope;
                static const int &b = InputConfig::getInstance().parrotEgressTravelTimeHeuristicIntercept;
                const int directDist = approxCombinedResult.getArrivalTime() - approxPtLeg.getArrivalTime();
                const int heuristicTravelTime = std::max(
                    static_cast<int>(std::round(m * static_cast<double>(directDist))) + b, 0);
                stats.combinedSecondLegHeuristicTravelTime = heuristicTravelTime;
                combinedTravelTime += std::max(heuristicTravelTime, directDist);
            }

            combinedCost = approxCombinedResult.getBestCost();
            stats.combinedTravelTime = combinedTravelTime;
            stats.combinedWaitTime = combinedWaitTime;
            stats.combinedAccEgrTime = combinedAccessEgressTime;
        }

        // Executes mode choice with previously registered modes.
        TransportMode apply() const {
            // Return mode with smallest cost
            if (walkingCost < INFTY && isMinimum(walkingCost))
                return TransportMode::Ped;
            if (ptCost < INFTY && isMinimum(ptCost))
                return TransportMode::PublicTransport;
            if (combinedCost < INFTY && isMinimum(combinedCost))
                return TransportMode::TaxiAndPT;
            if (taxiCost < INFTY && isMinimum(taxiCost))
                return TransportMode::Taxi;
            KASSERT(false);
            return TransportMode::None;
        }

        const ModeChoiceInputStats &getStats() const {
            return stats;
        }

    private:
        bool isMinimum(const int cost) const {
            return cost <= walkingCost && cost <= taxiCost && cost <= ptCost && cost <= combinedCost;
        }

        const RouteState &routeState;
        CostCalculator calculator;

        int walkingCost;
        int taxiCost;
        int ptCost;
        int combinedCost;

        ModeChoiceInputStats stats;
    };
} // karri
