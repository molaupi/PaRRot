#pragma once

#include <vector>
#include <Common/TimeConversion.h>
#include <ULTRA/DataStructures/RAPTOR/Entities/Journey.h>

namespace parrot {
    class PTResult {
        using Journey = RAPTOR::Journey;

    public:
        PTResult() : cost(INFTY) {
        }

        PTResult(const Journey &journey, const int cost)
            : journey(journey), cost(cost) {
            for (const auto &leg: journey) {
                KASSERT(!leg.usesTaxi);
            }
        }

        // Flag to indicate if this PT result is valid or not
        bool isValid() const { return !journey.empty(); }

        const int &getCost() const { return cost; }

        const Journey &getBestJourney() const {
            return journey;
        }

        // at Destination
        int getArrivalTime() const {
            return journey.empty() ? INFTY : parrot::ultraToKarriTime(journey.back().arrivalTime);
        }

        int getArrivalTimeAtLastStation() const {
            if (journey.empty())
                return INFTY;
            for (auto it = journey.rbegin(); it != journey.rend(); ++it) {
                if (it->usesRoute) {
                    return parrot::ultraToKarriTime(it->arrivalTime);
                }
            }
            return INFTY;
        }

        // at Origin
        int getDepartureTime() const {
            return journey.empty() ? INFTY : parrot::ultraToKarriTime(journey.front().departureTime);
        }

        int getDepartureTimeAtFirstStation() const {
            for (const auto &leg: journey) {
                if (leg.usesRoute) {
                    return parrot::ultraToKarriTime(leg.departureTime);
                }
            }
            return INFTY;
        }

        int getWalkingTimeToFirstStation() const {
            return journey.empty()
                       ? INFTY
                       : parrot::ultraToKarriTime(journey.front().arrivalTime - journey.front().departureTime);
        }

        int getWalkingTimeFromLastStation() const {
            return journey.empty()
                       ? INFTY
                       : parrot::ultraToKarriTime(journey.back().arrivalTime - journey.back().departureTime);
        }

        int getFirstStation() const {
            return journey.empty()
                       ? INVALID_ID
                       : journey.front().usesRoute
                             ? journey.front().from.value()
                             : journey.front().to.value();
        }

        int getLastStation() const {
            return journey.empty()
                       ? INVALID_ID
                       : journey.back().usesRoute
                             ? journey.back().to.value()
                             : journey.back().from.value();
        }

        int getAccessEgressTransferTime() const {
            return parrot::ultraToKarriTime(RAPTOR::initialTransferTime(journey));
        }

        int getIntermediateTransferTime() const {
            return parrot::ultraToKarriTime(RAPTOR::intermediateTransferTime(journey));
        }

        int getTotalInVehicleTime() const {
            int totalSeconds = 0;
            for (const auto &leg: journey) {
                if (leg.usesRoute) {
                    totalSeconds += leg.arrivalTime - leg.departureTime;
                }
            }
            return parrot::ultraToKarriTime(totalSeconds);
        }

        int cost;
        Journey journey;
    };
} // namespace karri
