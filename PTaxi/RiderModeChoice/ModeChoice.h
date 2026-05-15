#pragma once

#include "KARRI/Tools/Logging/NullLogger.h"
#include "KARRI/Tools/Logging/LogManager.h"
#include "KARRI/Tools/ThreadSafeRandom.h"
#include "KARRI/Algorithms/KaRRi/BaseObjects/Request.h"
#include "KARRI/Algorithms/KaRRi/TimeUtils.h"
#include "KARRI/Algorithms/KaRRi/RouteState.h"
#include "TransportMode.h"
#include "ModeChoiceInput.h"
#include "ModeChoiceInputStats.h"
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

    public:
        ModeChoice(const RouteState &routeState) : criterion(routeState),
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
                                                       "combined_2nd_leg_heuristic_travel_time,"
                                                       "mode\n")) {
        }

        TransportMode chooseMode(const RequestState &requestState,
                                 const WalkingResult &walkOnlyResult,
                                 const CarResult &carOnlyResult,
                                 const TaxiResult &taxiOnlyResult,
                                 const PTResult &ptOnlyResult,
                                 const ApproximateCombinedTripResult &approxCombinedResult) {
            using namespace time_utils;
            using namespace utility_logit;

            criterion.init();

            if (walkOnlyResult.isValid()) {
                criterion.registerPed(walkOnlyResult, requestState);
            }

            if (carOnlyResult.isValid()) {
                criterion.registerCar(carOnlyResult, requestState);
            }

            if (ptOnlyResult.isValid()) {
                criterion.registerPublicTransport(ptOnlyResult, requestState);
            }

            if (taxiOnlyResult.isValid()) {
                criterion.registerTaxi(taxiOnlyResult, requestState);
            }

            if (approxCombinedResult.isValid()) {
                criterion.registerCombined(approxCombinedResult, requestState);
            }

            const auto choice = criterion.apply();

            const ModeChoiceInputStats &stats = criterion.getStats();
            logger << requestState.originalRequest.requestId << ","
                    << stats.walkTravelTime << ","
                    << stats.privateCarTravelTime << ","
                    << stats.ptTravelTime << ","
                    << stats.ptWaitTime << ","
                    << stats.ptAccEgrTime << ","
                    << stats.taxiTravelTime << ","
                    << stats.taxiWaitTime << ","
                    << stats.taxiAccEgrTime << ","
                    << stats.combinedTravelTime << ","
                    << stats.combinedWaitTime << ","
                    << stats.combinedAccEgrTime << ","
                    << stats.combinedSecondLegHeuristicTravelTime << ","
                    << transportModeToString(choice) << "\n";

            return choice;
        }

    private:
        CriterionT criterion;
        const RouteState &routeState;
        LoggerT &logger;
    };
} // namespace karri::mode_choice
