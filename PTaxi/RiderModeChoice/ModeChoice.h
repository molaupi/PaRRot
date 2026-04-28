#pragma once

#include "KARRI/Tools/Logging/NullLogger.h"
#include "KARRI/Tools/Logging/LogManager.h"
#include "KARRI/Tools/ThreadSafeRandom.h"
#include "KARRI/Algorithms/KaRRi/BaseObjects/Request.h"
#include "KARRI/Algorithms/KaRRi/TimeUtils.h"
#include "KARRI/Algorithms/KaRRi/RouteState.h"
#include "TransportMode.h"
#include "ModeChoiceInput.h"
#include "UtilityLogit/types.h"
#include "PTJourneyData.h"
#include "../WalkingResult.h"
#include "../TaxiResult.h"
#include "../PTLeg/PTResult.h"
#include "../FirstTaxiLeg/FirstTaxiLegResult.h"
#include "../ApproximateCombinedTripResult.h"
#include "../CarResult.h"
#include "Common/Constants.h"
#include "KARRI/Algorithms/KaRRi/RequestState/RequestState.h"

namespace parrot::mode_choice {
    template<typename CriterionT, typename LoggerT = NullLogger>
    class ModeChoice {

        static double tenthsOfSecondsToMinutes(int tenthsOfSeconds) {
            return static_cast<double>(tenthsOfSeconds) / 600.0;
        }

    public:
        ModeChoice(const RouteState &routeState) : criterion(),
                                                   routeState(routeState),
                                                   logger(LogManager<LoggerT>::getLogger("modechoice.csv",
                                                       "request_id,"
                                                       "walk_travel_time,"
                                                       "car_travel_time,"
                                                       "pt_travel_time,"
                                                       "pt_wait_time,"
                                                       "pt_accegr_time,"
                                                       "taxi_travel_time,"
                                                       "taxi_wait_time,"
                                                       "taxi_accegr_time,"
                                                       "combined_travel_time,"
                                                       "combined_wait_time,"
                                                       "combined_accegr_time,"
                                                       "mode\n")) {
        }

        TransportMode chooseMode(const RequestState &requestState,
                                 const WalkingResult &walkOnlyResult,
                                 const CarResult &carOnlyResult,
                                 const TaxiResult &taxiOnlyResult,
                                 const PTResult &ptOnlyResult,
                                 const ApproximateCombinedTripResult &approxCombinedResult) const {
            using namespace time_utils;
            using namespace utility_logit;

            std::vector<Alternative<TransportMode> > entries;

            int walkTravelTime = INFTY;
            if (walkOnlyResult.isValid()) {
                walkTravelTime = walkOnlyResult.walkingDist;
                entries.push_back({TransportMode::Ped, constructAttributesForTimesInTenthsOfSeconds(walkTravelTime, 0, 0)});
            }

            int carTravelTime = INFTY;
            if (carOnlyResult.isValid()) {
                carTravelTime = carOnlyResult.carDist;
                entries.push_back({TransportMode::Car, constructAttributesForTimesInTenthsOfSeconds(carTravelTime, 0, 0)});
            }

            int ptOnlyTravelTime = INFTY;
            int ptOnlyWaitTime = INFTY;
            int ptOnlyAccessEgressTime = INFTY;
            if (ptOnlyResult.isValid()) {
                const int accEgrTransferTime = ptOnlyResult.getAccessEgressTransferTime();
                const int intermediateTransferTime = ptOnlyResult.getIntermediateTransferTime();
                ptOnlyTravelTime = ptOnlyResult.getTotalInVehicleTime();
                ptOnlyWaitTime = ptOnlyResult.getArrivalTime() - requestState.originalRequest.requestTime -
                                 ptOnlyTravelTime -
                                 accEgrTransferTime - intermediateTransferTime;
                ptOnlyAccessEgressTime = accEgrTransferTime + intermediateTransferTime;

                entries.push_back({
                    TransportMode::PublicTransport,
                    constructAttributesForTimesInTenthsOfSeconds(ptOnlyTravelTime, ptOnlyWaitTime,
                                                                 ptOnlyAccessEgressTime)
                });
            }

            int taxiOnlyTravelTime = INFTY;
            int taxiOnlyWaitTime = INFTY;
            int taxiOnlyAccessEgressTime = INFTY;
            if (taxiOnlyResult.isValid()) {
                const auto &bestAsgn = taxiOnlyResult.getBestAssignment();
                const auto depTimeAtPickup = getActualDepTimeAtPickup(bestAsgn, requestState, routeState);
                const auto initialPickupDetour = calcInitialPickupDetour(
                    bestAsgn, depTimeAtPickup, requestState, routeState);
                const auto dropoffAtExistingStop = isDropoffAtExistingStop(bestAsgn, routeState);
                const auto arrTimeAtDropoff = getArrTimeAtDropoff(depTimeAtPickup, bestAsgn, initialPickupDetour,
                                                                  dropoffAtExistingStop, routeState);
                taxiOnlyTravelTime = arrTimeAtDropoff - depTimeAtPickup;
                taxiOnlyWaitTime = depTimeAtPickup - requestState.originalRequest.requestTime - bestAsgn.pickup.
                                   walkingDist;
                taxiOnlyAccessEgressTime = bestAsgn.pickup.walkingDist + bestAsgn.dropoff.walkingDist;
                entries.push_back({
                    TransportMode::Taxi,
                    constructAttributesForTimesInTenthsOfSeconds(taxiOnlyTravelTime, taxiOnlyWaitTime,
                                                                 taxiOnlyAccessEgressTime)
                });
            }

            int combinedTravelTime = INFTY;
            int combinedWaitTime = INFTY;
            int combinedAccessEgressTime = INFTY;
            if (approxCombinedResult.isValid()) {
                combinedTravelTime = 0;
                combinedWaitTime = 0;
                combinedAccessEgressTime = 0;

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

                // PT leg
                const auto &approxPtLeg = approxCombinedResult.getPTLeg();
                const int ptAccEgrTransferTime = approxPtLeg.getAccessEgressTransferTime();
                const int intermediateTransferTime = approxPtLeg.getIntermediateTransferTime();
                const int ptInVehicleTime = approxPtLeg.getTotalInVehicleTime();
                combinedTravelTime += ptInVehicleTime;
                combinedWaitTime += approxPtLeg.getArrivalTime() - ptLegStartTime - ptInVehicleTime - ptAccEgrTransferTime - intermediateTransferTime;
                combinedAccessEgressTime += ptAccEgrTransferTime + intermediateTransferTime;

                // Second taxi leg
                if (approxCombinedResult.isFinalTransferByTaxi()) {
                    // Real travel time approximated by linear function with parameters based on known KaRRi data.
                    static const double &m = InputConfig::getInstance().parrotEgressTravelTimeHeuristicSlope;
                    static const int &b = InputConfig::getInstance().parrotEgressTravelTimeHeuristicIntercept;
                    const int directDist = approxCombinedResult.getArrivalTime() - approxPtLeg.getArrivalTime();
                    const int heuristicTravelTime = static_cast<int>(std::round(m * static_cast<double>(directDist))) + b;
                    combinedTravelTime += std::max(heuristicTravelTime, directDist);
                }

                entries.push_back({
                    TransportMode::TaxiAndPT,
                    constructAttributesForTimesInTenthsOfSeconds(combinedTravelTime, combinedWaitTime,
                                                                 combinedAccessEgressTime)
                });
            }

            const auto choice = criterion.apply(entries);

            logger << requestState.originalRequest.requestId << ","
                    << walkTravelTime << ","
                    << carTravelTime << ","
                    << ptOnlyTravelTime << ","
                    << ptOnlyWaitTime << ","
                    << ptOnlyAccessEgressTime << ","
                    << taxiOnlyTravelTime << ","
                    << taxiOnlyWaitTime << ","
                    << taxiOnlyAccessEgressTime << ","
                    << combinedTravelTime << ","
                    << combinedWaitTime << ","
                    << combinedAccessEgressTime << ","
                    << transportModeToString(choice) << "\n";

            return choice;
        }

    private:
        CriterionT criterion;
        const RouteState &routeState;
        LoggerT &logger;
    };
} // namespace karri::mode_choice
