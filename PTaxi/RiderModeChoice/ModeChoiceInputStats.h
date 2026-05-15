#pragma once

#include "Common/Constants.h"

namespace parrot::mode_choice {
    struct ModeChoiceInputStats {

        int walkTravelTime = INFTY;

        int privateCarTravelTime = INFTY;

        int taxiTravelTime = INFTY;
        int taxiWaitTime = INFTY;
        int taxiAccEgrTime = INFTY;

        int ptTravelTime = INFTY;
        int ptWaitTime = INFTY;
        int ptAccEgrTime = INFTY;

        int combinedTravelTime = INFTY;
        int combinedWaitTime = INFTY;
        int combinedAccEgrTime = INFTY;
        int combinedSecondLegHeuristicTravelTime = INFTY;


        // Set all values back to default
        void reset() {
            walkTravelTime = INFTY;
            privateCarTravelTime = INFTY;
            taxiTravelTime = INFTY;
            taxiWaitTime = INFTY;
            taxiAccEgrTime = INFTY;
            ptTravelTime = INFTY;
            ptWaitTime = INFTY;
            ptAccEgrTime = INFTY;
            combinedTravelTime = INFTY;
            combinedWaitTime = INFTY;
            combinedAccEgrTime = INFTY;
            combinedSecondLegHeuristicTravelTime = INFTY;
        }

        // bool isWalkValid() const {
        //     return walkTravelTime != INFTY;
        // }
        //
        // bool isPrivateCarValid() const {
        //     return privateCarTravelTime != INFTY;
        // }
        //
        // bool isTaxiValid() const {
        //     return taxiTravelTime != INFTY && taxiWaitTime != INFTY && taxiAccEgrTime != INFTY;
        // }
        //
        // bool isPublicTransportValid() const {
        //     return ptTravelTime != INFTY && ptWaitTime != INFTY && ptAccEgrTime != INFTY;
        // }
        //
        // void disableWalk() {
        //     walkTravelTime = INFTY;
        // }
        //
        // void disablePrivateCar() {
        //     privateCarTravelTime = INFTY;
        // }
        //
        // void disableTaxi() {
        //     taxiTravelTime = INFTY;
        //     taxiWaitTime = INFTY;
        //     taxiAccEgrTime = INFTY;
        // }
        //
        // void disablePublicTransport() {
        //     ptTravelTime = INFTY;
        //     ptWaitTime = INFTY;
        //     ptAccEgrTime = INFTY;
        // }
    };
}