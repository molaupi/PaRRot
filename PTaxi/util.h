#pragma once

#include <Common/TimeConversion.h>

namespace karri {

    static int getTotalTransferTime(const RAPTOR::Journey &journey) {
        return parrot::ultraToKarriTime(RAPTOR::totalTransferTime(journey));
    }

    static int getTotalTripTime(const RAPTOR::Journey &journey) {
        return parrot::ultraToKarriTime(journey.back().arrivalTime - journey.front().departureTime);
    }

    static int getNumberOfTransfers(const RAPTOR::Journey &journey) {
        const int numTrips = RAPTOR::countTrips(journey);
        return numTrips == 0 ? 0 : numTrips - 1;
    }
}