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

#include <KARRI/Algorithms/CH/CH.h>
#include <KARRI/Algorithms/Buckets/DynamicBucketContainer.h>
#include <KARRI/DataStructures/Labels/BasicLabelSet.h>
#include <KARRI/DataStructures/Containers/LightweightSubset.h>
#include <KARRI/Tools/Constants.h>
#include "StationEntry.h"

namespace karri {
    template<typename InputGraphT, typename CHEnvT,
        typename StationBucketsEnvT>
    class StationsInEllipse {
    private:

        using LabelSetT = BasicLabelSet<0, ParentInfo::FULL_PARENT_INFO>;
        static constexpr int K = LabelSetT::K;
        using DistanceLabel = typename LabelSetT::DistanceLabel;
        using LabelMask = typename LabelSetT::LabelMask;

        using StationBucketContainer = typename StationBucketsEnvT::BucketContainer;
        using StopBucketContainer = DynamicBucketContainer<StationEntry>;

        struct ScanSourceBucket {
        public:
            explicit ScanSourceBucket(StationsInEllipse &search) : search(search) {
            }

            template<typename DistLabelT, typename DistLabelContainerT>
            bool operator()(const int v, DistLabelT &distFromV, const DistLabelContainerT & /*distLabels*/) {
                // Check if we can prune at this vertex based only on the distance from v to the pickup(s)
                if (allSet(search.exceedsLeewayForStop(distFromV)))
                    return true;

                int numEntriesScannedHere = 0;

                if constexpr (!StationBucketsEnvT::SORTED) {
                    auto bucket = search.stationSourceBucketContainer.getBucketOf(v);
                    for (const auto &entry: bucket) {
                        ++numEntriesScannedHere;

                        const int &stationId = entry.targetId;
                        const DistanceLabel distViaV = distFromV + DistanceLabel(entry.distToTarget);
                        tryUpdatingDistance(stationId, distViaV);
                    }
                } else {
                    auto bucket = search.stationSourceBucketContainer.getBucketOf(v);

                    for (const auto &entry: bucket) {
                        ++numEntriesScannedHere;

                        const int &stationId = entry.targetId;
                        const DistanceLabel distViaV = distFromV + DistanceLabel(entry.distToTarget);
                        const auto atLeastAsGoodAsCurBest = ~search.exceedsLeewayForStop(distViaV);
                        if (!anySet(atLeastAsGoodAsCurBest))
                            break;

                        tryUpdatingDistance(stationId, distViaV);
                    }
                }

                search.numEntriesVisited += numEntriesScannedHere;
                ++search.numVerticesSettled;

                return false;
            }

        private:
            void tryUpdatingDistance(const int stationId, const DistanceLabel &distFromStationToStop) {
                // Update tentative distances to v for any searches where distViaV admits a possible better assignment
                // than the current best and where distViaV is at least as good as the current tentative distance.
                LabelMask mask = ~(search.distFromStationsToStop[stationId] < distFromStationToStop);
                mask &= distFromStationToStop < INFTY;
                if (!anySet(mask))
                    return;

                if (anySet(mask)) {
                    // if any search requires updates, update the right ones according to mask
                    search.distFromStationsToStop[stationId].setIf(distFromStationToStop, mask);
                    search.stationsSeen.insert(stationId);
                }
            }


            StationsInEllipse &search;
        };

        struct ScanTargetBucket {
        public:
            explicit ScanTargetBucket(StationsInEllipse &search) : search(search) {
            }

            template<typename DistLabelT, typename DistLabelContainerT>
            bool operator()(const int v, DistLabelT &distToV, const DistLabelContainerT & /*distLabels*/) {
                // Check if we can prune at this vertex based only on the distance from v to the pickup(s)
                if (allSet(search.exceedsLeewayForStop(distToV)))
                    return true;

                int numEntriesScannedHere = 0;

                if constexpr (!StationBucketsEnvT::SORTED) {
                    auto bucket = search.stationTargetBucketContainer.getBucketOf(v);
                    for (const auto &entry: bucket) {
                        ++numEntriesScannedHere;

                        const int &stationId = entry.targetId;
                        const DistanceLabel distViaV = distToV + DistanceLabel(entry.distToTarget);
                        tryUpdatingDistance(stationId, distViaV);
                    }
                } else {
                    auto bucket = search.stationTargetBucketContainer.getBucketOf(v);

                    for (const auto &entry: bucket) {
                        ++numEntriesScannedHere;

                        const int &stationId = entry.targetId;
                        const DistanceLabel distViaV = distToV + DistanceLabel(entry.distToTarget);
                        const auto atLeastAsGoodAsCurBest = ~search.exceedsLeewayForStop(distViaV);
                        if (!anySet(atLeastAsGoodAsCurBest))
                            break;

                        tryUpdatingDistance(stationId, distViaV);
                    }
                }

                search.numEntriesVisited += numEntriesScannedHere;
                ++search.numVerticesSettled;

                return false;
            }

        private:
            void tryUpdatingDistance(const int stationId, const DistanceLabel &distFromStopToStation) {
                // Update tentative distances to v for any searches where distViaV admits a possible better assignment
                // than the current best and where distViaV is at least as good as the current tentative distance.
                LabelMask mask = ~(search.distFromStopToStations[stationId] < distFromStopToStation);
                mask &= distFromStopToStation < INFTY;
                if (!anySet(mask))
                    return;

                if (anySet(mask)) {
                    // if any search requires updates, update the right ones according to mask
                    search.distFromStopToStations[stationId].setIf(distFromStopToStation, mask);
                    search.stationsSeen.insert(stationId);
                }
            }


            StationsInEllipse &search;
        };


        struct StopEllipseSearch {
            explicit StopEllipseSearch(const StationsInEllipse &search) : search(search) {
            }

            template<typename DistLabelT, typename DistLabelContainerT>
            bool operator()(const int, DistLabelT &distToOrFromV, const DistLabelContainerT & /*distLabels*/) const {
                return allSet(search.exceedsLeewayForStop(distToOrFromV));
            }

        private:
            const StationsInEllipse &search;
        };

    public:
        StationsInEllipse(
            const InputGraphT &inputGraph,
            const CHEnvT &chEnv,
            const RouteState &routeState,
            const StationBucketsEnvT &stationBucketsEnv,
            const PTStations &stations)
            : inputGraph(inputGraph),
              upwardSearch(chEnv.template getForwardSearch<ScanTargetBucket, StopEllipseSearch, LabelSetT>(
                  ScanTargetBucket(*this), StopEllipseSearch(*this))),
              reverseUpwardSearch(chEnv.template getReverseSearch<ScanSourceBucket, StopEllipseSearch, LabelSetT>(
                  ScanSourceBucket(*this), StopEllipseSearch(*this))),
              ch(chEnv.getCH()),
              routeState(routeState),
              stations(stations),
              stationSourceBucketContainer(stationBucketsEnv.getSourceBuckets()),
              stationTargetBucketContainer(stationBucketsEnv.getTargetBuckets()),
              stopBucketContainer(routeState.getMaxStopId()),
              stationsSeen(stations.size()),
              distFromStopToStations(stations.size(), DistanceLabel(INFTY)),
              distFromStationsToStop(stations.size(), DistanceLabel(INFTY)),
              curLeeway(0),
              numVerticesSettled(0),
              numEntriesVisited(0) {
        }

        // TODO: ADD TO UPDATE STATS
        void computeNewStationsInEllipsesForStop(const int stopIndex, const int vehId) {
            const int stopId = routeState.stopIdsFor(vehId)[stopIndex];
            const int stopLoc = routeState.stopLocationsFor(vehId)[stopIndex];
            const int stopVertex = inputGraph.edgeHead(stopLoc);
            const int stopRank = ch.rank(stopVertex);

            assert(stopIndex < routeState.numStopsOf(vehId) - 1);

            const int nextStopLoc = routeState.stopLocationsFor(vehId)[stopIndex + 1];
            const int nextStopVertex = inputGraph.edgeTail(nextStopLoc);
            const int nextStopOffset = inputGraph.travelTime(nextStopLoc);
            const int nextStopRank = ch.rank(nextStopVertex);

            init(stopId);
            initStationsAtStop(stopLoc);

            // Run the upward search from the first stop vertex
            // This will compute the distances from the stop to all stations
            upwardSearch.run(stopRank);

            // Run the reverse upward search from the second stop vertex
            // This will compute the distances from all stations to the next stop
            reverseUpwardSearch.runWithOffset(nextStopRank, nextStopOffset);

            for (const auto &stationId: stationsSeen) {
                // If the station was seen, we can check the leeway
                const int distFromStopToStation = distFromStopToStations[stationId][0];
                const int distFromStationToStop = distFromStationsToStop[stationId][0];

                // Only add the entry to the container if the leeway is not exceeded
                if (!allSet(exceedsLeewayForStop(distFromStopToStation + distFromStationToStop))) {
                    StationEntry entry(stationId, distFromStopToStation, distFromStationToStop);
                    stopBucketContainer.insert(stopId, entry);
                }
            }
        }

        void recomputeStationsInEllipseForStop(const int stopIndex, const int vehId) {
            removeStationsForStop(stopIndex, vehId);
            computeNewStationsInEllipsesForStop(stopIndex, vehId);
        }

        void removeStationsForStop(const int stopIndex, const int vehId) {
            const int stopId = routeState.stopIdsFor(vehId)[stopIndex];
            stopBucketContainer.clearBucket(stopId);
        }

        LabelMask exceedsLeewayForStop(const DistanceLabel &distanceToStop) const {
            return curLeeway < distanceToStop;
        }

        ConstantVectorRange<StationEntry> getStationsInEllipse(const int stopId) const {
            assert(stopId >= 0);
            if (stopId >= stopBucketContainer.getBucketPositionsSize()) {
                return ConstantVectorRange<StationEntry>();
            }
            return stopBucketContainer.getBucketOf(stopId);
        }

    private:
        void init(const int stopId) {
            curLeeway = routeState.leewayOfLegStartingAt(stopId);
            stationsSeen.clear();
            numVerticesSettled = 0;
            numEntriesVisited = 0;

            // Initialize the distances from the stop to the stations
            std::fill(distFromStopToStations.begin(), distFromStopToStations.end(), INFTY);
            std::fill(distFromStationsToStop.begin(), distFromStationsToStop.end(), INFTY);
            stopBucketContainer.checkAndResize(routeState.getMaxStopId());
        }

        void initStationsAtStop(const int stopLoc) {
            for (const auto & station: stations) {
                if (station.vehEdgeId == stopLoc)
                    distFromStopToStations[station.stationId][0] = 0;
            }
        }

        int getNumEdgeRelaxations() const {
            return upwardSearch.getNumEdgeRelaxations();
        }

        int getNumVerticesSettled() const {
            return numVerticesSettled;
        }

        int getNumEntriesScanned() const {
            return numEntriesVisited;
        }

        typename CHEnvT::template UpwardSearch<ScanTargetBucket, StopEllipseSearch, LabelSetT> upwardSearch;
        typename CHEnvT::template UpwardSearch<ScanSourceBucket, StopEllipseSearch, LabelSetT> reverseUpwardSearch;

        const InputGraphT &inputGraph;
        const CH &ch;
        const RouteState &routeState;

        const PTStations &stations;
        const StationBucketContainer &stationSourceBucketContainer;
        const StationBucketContainer &stationTargetBucketContainer;
        StopBucketContainer stopBucketContainer;

        LightweightSubset stationsSeen;

        std::vector<DistanceLabel> distFromStopToStations;
        std::vector<DistanceLabel> distFromStationsToStop;

        int curLeeway;

        int numVerticesSettled;
        int numEntriesVisited;

        int totalNumEdgeRelaxations;
        int totalNumVerticesSettled;
        int totalNumEntriesScanned;
    };
}
