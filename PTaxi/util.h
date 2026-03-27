#pragma once

namespace karri {
    static constexpr int convertToKaRRiTime(const int timeInSeconds) {
        return timeInSeconds * 10;
    }

    static int getTotalTransferTime(const RAPTOR::Journey &journey) {
        return convertToKaRRiTime(RAPTOR::totalTransferTime(journey));
    }

    static int getTotalTripTime(const RAPTOR::Journey &journey) {
        return convertToKaRRiTime(journey.back().arrivalTime - journey.front().departureTime);
    }

    static int getNumberOfTransfers(const RAPTOR::Journey &journey) {
        const int numTrips = RAPTOR::countTrips(journey);
        return numTrips == 0 ? 0 : numTrips - 1;
    }
}