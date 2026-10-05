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

#include <type_traits>
#include "../../../DataStructures/Labels/BasicLabelSet.h"
#include "../../../DataStructures/Labels/SimdLabelSet.h"
#include "../../../DataStructures/Containers/Subset.h"
#include "../../../Tools/Simd/AlignedVector.h"
#include "../../../Tools/Timer.h"

#include "../RequestState/RequestState.h"
#include "../RouteState.h"
#include "../TimeUtils.h"

namespace karri {
    class FeasiblePDLocs {
    public:
        struct FeasiblePDLoc {
            int pdId = INVALID_ID;
            int distToPDLoc = INFTY;
            int distFromPDLocToNextStop = INFTY;
        };

    private:
        using FeasibleVec = std::vector<FeasiblePDLoc>;

    public:
        using It = FeasibleVec::const_iterator;

        FeasiblePDLocs() = default;

        template<typename EllipticBCHSearchResultT>
        void construct(const EllipticBCHSearchResultT &searchResult, const std::vector<PDLoc> &pdLocs,
                       const RouteState &routeState, const RequestState &requestState,
                       stats::FilterRelevantPdLocsPerformanceStats &stats) {
            KaRRiTimer timer;

            stopIdsWithRelevantPDLocs.clear();
            stopInfo.clear();
            feasiblePdLocs.clear();

            static const int &stopTime = InputConfig::getInstance().stopTime;

            for (const int &stopId: searchResult.getStopIdsWithRelevantPDLocs()) {
                const int totalNumFeasibleBefore = feasiblePdLocs.size();
                const int vehId = routeState.vehicleIdOf(stopId);
                const int numStops = routeState.numStopsOf(vehId);
                const int stopIdx = routeState.stopPositionOf(stopId);
                const int stopLoc = routeState.stopLocationsFor(vehId)[stopIdx];
                const int depTimeAtStop = time_utils::getVehDepTimeAtStopForRequest(
                    vehId, stopIdx, requestState.now(), routeState);
                const int maxArrTimeAtNextStop = (stopIdx < numStops - 1
                                                      ? routeState.maxArrTimesFor(vehId)[stopIdx + 1]
                                                      : INFTY);

                const auto &distsToPDLocs = searchResult.distancesToRelevantPDLocsFor(stopId);
                const auto &distsFromPDLocs = searchResult.distancesFromRelevantPDLocsToNextStopOf(stopId);
                int minDistToPdLoc = INFTY;
                int minDistFromPdLocToNextStop = INFTY;
                for (int id = 0; id < pdLocs.size(); ++id) {
                    const auto &distToPDLoc = distsToPDLocs[id];
                    const auto &distFromPDLoc = distsFromPDLocs[id];
                    if (distToPDLoc >= INFTY || distFromPDLoc >= INFTY)
                        continue;
                    const bool atStop = stopLoc == pdLocs[id].loc;
                    if (stopIdx == numStops - 1 && !atStop)
                        continue;
                    if (atStop || depTimeAtStop + distToPDLoc + stopTime + distFromPDLoc <= maxArrTimeAtNextStop) {
                        feasiblePdLocs.push_back({id, distToPDLoc, distFromPDLoc});
                        minDistToPdLoc = std::min(minDistToPdLoc, distToPDLoc);
                        minDistFromPdLocToNextStop = std::min(minDistFromPdLocToNextStop, distFromPDLoc);
                    }
                }

                const int totalNumFeasibleAfter = feasiblePdLocs.size();
                if (totalNumFeasibleAfter == totalNumFeasibleBefore)
                    continue;

                stopInfo[stopId] = {
                    totalNumFeasibleBefore, totalNumFeasibleAfter, minDistToPdLoc, minDistFromPdLocToNextStop
                };
                stopIdsWithRelevantPDLocs.push_back(stopId);
            }

            const int64_t time = timer.elapsed<std::chrono::nanoseconds>();
            stats.filterFeasiblePDLocsTime += time;
        }

        bool hasFeasiblePDLocs(const int stopId) const {
            return stopInfo.contains(stopId);
        }

        IteratorRange<It> feasiblePdLocsFor(const int stopId) const {
            const auto it = stopInfo.find(stopId);
            if (it == stopInfo.end())
                return {feasiblePdLocs.cbegin(), feasiblePdLocs.cbegin()};
            const auto &stopInfoEntry = it->second;
            return {
                feasiblePdLocs.cbegin() + stopInfoEntry.startIdxInValueArray,
                feasiblePdLocs.cbegin() + stopInfoEntry.endIdxInValueArray
            };
        }

        int minDistToPDLocFor(const int stopId) const {
            const auto it = stopInfo.find(stopId);
            if (it == stopInfo.end())
                return INFTY;
            return it->second.minDistToPDLoc;
        }

        int minDistFromPDLocToNextStopOf(const int stopId) const {
            const auto it = stopInfo.find(stopId);
            if (it == stopInfo.end())
                return INFTY;
            return it->second.minDistFromPDLocToNextStop;
        }

        const std::vector<int> &getStopIdsWithRelevantPDLocs() const {
            return stopIdsWithRelevantPDLocs;
        }

        int numStopsWithRelevantPDLocs() const {
            return stopIdsWithRelevantPDLocs.size();
        }

    private:
        // Describes range of feasible PD locs and minimum distance values for a stop that has feasible PD locs.
        struct FeasibleForStop {
            int startIdxInValueArray = INVALID_INDEX;
            int endIdxInValueArray = INVALID_INDEX;
            int minDistToPDLoc = INFTY;
            int minDistFromPDLocToNextStop = INFTY;
        };

        // Iterable set of stop IDs that have relevant PD locs.
        std::vector<int> stopIdsWithRelevantPDLocs;

        // Points from a stop id to the start of the entries in the array of feasible PD locs for this stop.
        std::unordered_map<int, FeasibleForStop> stopInfo;

        // Value arrays.
        std::vector<FeasiblePDLoc> feasiblePdLocs;
    };
}
