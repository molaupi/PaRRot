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
#include "KARRI/Algorithms/Buckets/SortedBucketContainer.h"

namespace karri {
    template<typename InputGraphT, typename CHEnvT,
        typename StationBucketsEnvT>
    class StationsInEllipse {
    private:
        using StationBucketContainer = typename StationBucketsEnvT::BucketContainer;

        struct IsDetourSmaller {
            bool operator()(const StationEntry &e1, const StationEntry &e2) const {
                return e1.distFromStopToStation + e1.distFromStationToStop < e2.distFromStopToStation + e2.distFromStationToStop;
            }
        };

        using StopBucketContainer = SortedBucketContainer<StationEntry, IsDetourSmaller>;

        struct ScanSourceBucket {
        public:
            explicit ScanSourceBucket(StationsInEllipse &search) : search(search) {
            }

            template<typename DistLabelT, typename DistLabelContainerT>
            bool operator()(const int v, DistLabelT &distFromV, const DistLabelContainerT & /*distLabels*/) {
                // Check if we can prune at this vertex based only on the distance from v to the pickup(s)
                if (search.exceedsLeewayForStop(distFromV[0]))
                    return true;

                int numEntriesScannedHere = 0;

                if constexpr (!StationBucketsEnvT::SORTED) {
                    auto bucket = search.stationSourceBucketContainer.getBucketOf(v);
                    for (const auto &entry: bucket) {
                        ++numEntriesScannedHere;

                        const int &stationId = entry.targetId;
                        const int distViaV = distFromV[0] + entry.distToTarget;
                        tryUpdatingDistance(stationId, distViaV);
                    }
                } else {
                    auto bucket = search.stationSourceBucketContainer.getBucketOf(v);

                    for (const auto &entry: bucket) {
                        ++numEntriesScannedHere;

                        const int &stationId = entry.targetId;
                        const int distViaV = distFromV[0] + entry.distToTarget;
                        const bool atLeastAsGoodAsCurBest = !search.exceedsLeewayForStop(distViaV);
                        if (!atLeastAsGoodAsCurBest)
                            break;

                        tryUpdatingDistance(stationId, distViaV);
                    }
                }

                search.numEntriesVisited += numEntriesScannedHere;
                ++search.numVerticesSettled;

                return false;
            }

        private:
            void tryUpdatingDistance(const int stationId, const int &distFromStationToStop) {
                // If station was not seen by previously run search for distances from stop to stations, in cannot be
                // in the ellipse so skip here.
                if (!search.stationsSeen.contains(stationId))
                    return;
                auto &cur = search.distFromStationsToStop[stationId];
                if (distFromStationToStop < cur) {
                    cur = distFromStationToStop;
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
                if (search.exceedsLeewayForStop(distToV[0]))
                    return true;

                int numEntriesScannedHere = 0;

                if constexpr (!StationBucketsEnvT::SORTED) {
                    auto bucket = search.stationTargetBucketContainer.getBucketOf(v);
                    for (const auto &entry: bucket) {
                        ++numEntriesScannedHere;

                        const int &stationId = entry.targetId;
                        const int distViaV = distToV[0] + entry.distToTarget;
                        tryUpdatingDistance(stationId, distViaV);
                    }
                } else {
                    auto bucket = search.stationTargetBucketContainer.getBucketOf(v);

                    for (const auto &entry: bucket) {
                        ++numEntriesScannedHere;

                        const int &stationId = entry.targetId;
                        const int distViaV = distToV[0] + entry.distToTarget;
                        const bool atLeastAsGoodAsCurBest = !search.exceedsLeewayForStop(distViaV);
                        if (!atLeastAsGoodAsCurBest)
                            break;

                        tryUpdatingDistance(stationId, distViaV);
                    }
                }

                search.numEntriesVisited += numEntriesScannedHere;
                ++search.numVerticesSettled;

                return false;
            }

        private:
            void tryUpdatingDistance(const int stationId, const int &distFromStopToStation) {
                search.stationsSeen.insert(stationId);
                auto &cur = search.distFromStopToStations[stationId];
                if (distFromStopToStation < cur) {
                    cur = distFromStopToStation;
                }
            }


            StationsInEllipse &search;
        };


        struct StopEllipseSearch {
            explicit StopEllipseSearch(const StationsInEllipse &search) : search(search) {
            }

            template<typename DistLabelT, typename DistLabelContainerT>
            bool operator()(const int, DistLabelT &distToOrFromV, const DistLabelContainerT & /*distLabels*/) const {
                return search.exceedsLeewayForStop(distToOrFromV[0]);
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
            const PTStations &stations,
            const StationsAtLocations &stationsAtLocations)
            : inputGraph(inputGraph),
              upwardSearch(chEnv.template getForwardSearch<ScanTargetBucket, StopEllipseSearch, LabelSetT>(
                  ScanTargetBucket(*this), StopEllipseSearch(*this))),
              reverseUpwardSearch(chEnv.template getReverseSearch<ScanSourceBucket, StopEllipseSearch, LabelSetT>(
                  ScanSourceBucket(*this), StopEllipseSearch(*this))),
              ch(chEnv.getCH()),
              routeState(routeState),
              stations(stations),
              stationsAtLocations(stationsAtLocations),
              stationSourceBucketContainer(stationBucketsEnv.getSourceBuckets()),
              stationTargetBucketContainer(stationBucketsEnv.getTargetBuckets()),
              stopBucketContainer(routeState.getMaxStopId()),
              stationsSeen(stations.size()),
              distFromStopToStations(stations.size(), INFTY),
              distFromStationsToStop(stations.size(), INFTY),
              curLeeway(0),
              numVerticesSettled(0),
              numEntriesVisited(0) {
        }

        // TODO: ADD TO UPDATE STATS
        void computeNewStationsInEllipsesForStop(const int stopIndex, const int vehId,
                                                 stats::UpdatePerformanceStats &stats) {
            KaRRiTimer timer;
            const int stopId = routeState.stopIdsFor(vehId)[stopIndex];
            const int stopLoc = routeState.stopLocationsFor(vehId)[stopIndex];
            const int stopVertex = inputGraph.edgeHead(stopLoc);
            const int stopRank = ch.rank(stopVertex);
            const int lengthOfLegStartingAtStop = time_utils::calcLengthOfLegStartingAt(stopIndex, vehId, routeState);

            assert(stopIndex < routeState.numStopsOf(vehId) - 1);

            const int nextStopLoc = routeState.stopLocationsFor(vehId)[stopIndex + 1];
            const int nextStopVertex = inputGraph.edgeTail(nextStopLoc);
            const int nextStopOffset = inputGraph.travelTime(nextStopLoc);
            const int nextStopRank = ch.rank(nextStopVertex);

            init(stopId);
            KASSERT(stopBucketContainer.getBucketOf(stopId).empty());
            initStationsAtStop(stopLoc, lengthOfLegStartingAtStop);

            // Run the upward search from the first stop vertex
            // This will compute the distances from the stop to all stations
            upwardSearch.run(stopRank);

            // Run the reverse upward search from the second stop vertex
            // This will compute the distances from all stations to the next stop
            reverseUpwardSearch.runWithOffset(nextStopRank, nextStopOffset);

            for (const auto &stationId: stationsSeen) {
                // If the station was seen, we can check the leeway
                const int distFromStopToStation = distFromStopToStations[stationId];
                const int distFromStationToStop = distFromStationsToStop[stationId];

                // Only add the entry to the container if the leeway is not exceeded
                if (!exceedsLeewayForStop(distFromStopToStation + distFromStationToStop)) {
                    StationEntry entry(stationId, distFromStopToStation, distFromStationToStop);
                    stopBucketContainer.insert(stopId, entry);
                }
            }
            stats.stationsInEllipse_generate_time += timer.elapsed<std::chrono::nanoseconds>();
        }

        void recomputeStationsInEllipseForStop(const int stopIndex, const int vehId,
                                               stats::UpdatePerformanceStats &stats) {
            removeStationsForStop(stopIndex, vehId, stats);
            computeNewStationsInEllipsesForStop(stopIndex, vehId, stats);
        }

        void removeStationsForStop(const int stopIndex, const int vehId, stats::UpdatePerformanceStats &stats) {
            KaRRiTimer timer;
            const int stopId = routeState.stopIdsFor(vehId)[stopIndex];
            stopBucketContainer.checkAndResize(stopId);
            stopBucketContainer.clearBucket(stopId);
            stats.stationsInEllipse_remove_time += timer.elapsed<std::chrono::nanoseconds>();
        }

        bool exceedsLeewayForStop(const int &distanceToStop) const {
            return curLeeway < distanceToStop;
        }

        ConstantVectorRange<StationEntry> getStationsInEllipse(const int stopId) const {
            assert(stopId >= 0);
            if (stopId >= stopBucketContainer.getBucketPositionsSize()) {
                return {};
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
            std::ranges::fill(distFromStopToStations, INFTY);
            std::ranges::fill(distFromStationsToStop, INFTY);
            stopBucketContainer.checkAndResize(routeState.getMaxStopId());
        }

        void initStationsAtStop(const int stopLoc, const int lengthOfLegStartingAtStop) {
            for (const auto &stationId: stationsAtLocations.getIdsOfStationsAtVehEdge(stopLoc)) {
                stationsSeen.insert(stationId);
                distFromStopToStations[stationId] = 0;
                distFromStationsToStop[stationId] = lengthOfLegStartingAtStop;
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

        using LabelSetT = BasicLabelSet<0, ParentInfo::NO_PARENT_INFO>;
        typename CHEnvT::template UpwardSearch<ScanTargetBucket, StopEllipseSearch, LabelSetT> upwardSearch;
        typename CHEnvT::template UpwardSearch<ScanSourceBucket, StopEllipseSearch, LabelSetT> reverseUpwardSearch;

        const InputGraphT &inputGraph;
        const CH &ch;
        const RouteState &routeState;

        const PTStations &stations;
        const StationsAtLocations &stationsAtLocations;
        const StationBucketContainer &stationSourceBucketContainer;
        const StationBucketContainer &stationTargetBucketContainer;
        StopBucketContainer stopBucketContainer;

        LightweightSubset stationsSeen;

        std::vector<int> distFromStopToStations;
        std::vector<int> distFromStationsToStop;

        int curLeeway;

        int numVerticesSettled;
        int numEntriesVisited;

        int totalNumEdgeRelaxations;
        int totalNumVerticesSettled;
        int totalNumEntriesScanned;
    };
}
