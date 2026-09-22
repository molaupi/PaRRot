/// ******************************************************************************
/// MIT License
///
/// Copyright (c) 2023 Moritz Laupichler <moritz.laupichler@kit.edu>
///
/// Permission is hereby granted, free of charge, to any person obtaining a copy
/// of this software and associated documentation files (the "Software"), to deal
/// in the Software without restriction, including without limitation the rights
/// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
/// copies of the Software, and to permit persons to whom the Software is
/// furnished to do so, subject to the following conditions:
///
/// The above copyright notice and this permission notice shall be included in all
/// copies or substantial portions of the Software.
///
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
/// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
/// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
/// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
/// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
/// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
/// SOFTWARE.
/// ******************************************************************************


#pragma once

#include <algorithm>
#include <vector>

#include "../../include/ULTRA/DataStructures/RAPTOR/Data.h"
#include "../../include/Common/TimeConversion.h"
#include "KARRI/DataStructures/Utilities/IteratorRange.h"

namespace parrot {

    // Allows querying all departure times at a given station (PT stop) of a RAPTOR network. RAPTOR itself does not
    // need this information (it is organized by route and trip, not by stop), so this data structure builds and
    // caches it separately. Departure times are stored in KaRRi time format (tenths of seconds), converted from the
    // RAPTOR data's original format (full seconds).
    class DeparturesPerStation {

    public:
        explicit DeparturesPerStation(const RAPTOR::Data &raptorData)
            : firstDepartureOfStation(raptorData.numberOfStops() + 1, 0) {
            initializeDepartures(raptorData);
        }

        // Returns the departure times (in KaRRi time, i.e. tenths of seconds) at the given station in ascending order.
        ConstantVectorRange<int> getDepartures(const int stationId) const {
            const int startIndex = firstDepartureOfStation[stationId];
            const int endIndex = firstDepartureOfStation[stationId + 1];
            return {departures.begin() + startIndex, departures.begin() + endIndex};
        }

        // Returns the index into getDepartures(stationId) of the next departure at or after the given time (in
        // KaRRi time, i.e. tenths of seconds) at the given station, or the number of departures at the station if
        // there is no such departure.
        int getNextDepartureIndex(const int stationId, const int time) const {
            const auto departuresAtStation = getDepartures(stationId);
            const auto it = std::ranges::lower_bound(departuresAtStation, time);
            return static_cast<int>(it - departuresAtStation.begin());
        }

    private:
        void initializeDepartures(const RAPTOR::Data &raptorData) {
            // Count departures per station to get offsets. The last stop of a route is skipped since there is no
            // boarding (and thus no relevant departure) there.
            for (const StopId stop : raptorData.stops()) {
                for (const RAPTOR::RouteSegment &routeSegment : raptorData.routesContainingStop(stop)) {
                    if (routeSegment.stopIndex + 1 == raptorData.numberOfStopsInRoute(routeSegment.routeId))
                        continue;
                    firstDepartureOfStation[stop + 1] += raptorData.numberOfTripsInRoute(routeSegment.routeId);
                }
            }

            // Compute offsets as prefix sum of counts
            for (size_t i = 1; i < firstDepartureOfStation.size(); ++i) {
                firstDepartureOfStation[i] += firstDepartureOfStation[i - 1];
            }

            // Fill departures according to offsets
            departures.resize(firstDepartureOfStation.back());
            std::vector<int> nextInsertIndex(firstDepartureOfStation.begin(), firstDepartureOfStation.end() - 1);
            for (const StopId stop : raptorData.stops()) {
                for (const RAPTOR::RouteSegment &routeSegment : raptorData.routesContainingStop(stop)) {
                    const RouteId route = routeSegment.routeId;
                    const StopIndex stopIndex = routeSegment.stopIndex;
                    if (stopIndex + 1 == raptorData.numberOfStopsInRoute(route))
                        continue;
                    for (size_t tripNum = 0; tripNum < raptorData.numberOfTripsInRoute(route); ++tripNum) {
                        const int departureTimeInSeconds = raptorData.tripOfRoute(route, tripNum)[stopIndex].departureTime;
                        departures[nextInsertIndex[stop]++] = ultraToKarriTime(departureTimeInSeconds);
                    }
                }
            }

            // Departures were collected route by route, so they need to be sorted per station afterwards.
            for (const StopId stop : raptorData.stops()) {
                std::sort(departures.begin() + firstDepartureOfStation[stop], departures.begin() + firstDepartureOfStation[stop + 1]);
            }
        }

        // departures[firstDepartureOfStation[s]..firstDepartureOfStation[s + 1]) are the departure times (in KaRRi
        // time, i.e. tenths of seconds) at station s, sorted in ascending order.
        std::vector<int> firstDepartureOfStation;
        std::vector<int> departures;
    };
}
