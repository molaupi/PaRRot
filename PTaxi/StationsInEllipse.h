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

#include "KARRI/Algorithms/CH/CH.h"
#include "KARRI/Algorithms/Buckets/DynamicBucketContainer.h"
#include "KARRI/DataStructures/Labels/BasicLabelSet.h"
#include "KARRI/DataStructures/Containers/LightweightSubset.h"
#include "KARRI/Tools/Constants.h"
#include "StationEntry.h"

namespace karri {


    template<typename InputGraphT, typename CHEnvT,
            typename StationBucketsEnvT,
            typename LabelSetT = BasicLabelSet<0, ParentInfo::FULL_PARENT_INFO>>
    class StationsInEllipse {

    private:

        static constexpr int K = LabelSetT::K;
        using DistanceLabel = typename LabelSetT::DistanceLabel;
        using LabelMask = typename LabelSetT::LabelMask;

        using StationBucketContainer = typename StationBucketsEnvT::BucketContainer;
        using StopBucketContainer = DynamicBucketContainer<StationEntry>;

        struct ScanSourceBucket {

        public:

            explicit ScanSourceBucket(StationsInEllipse &search) : search(search) {}

            template<typename DistLabelT, typename DistLabelContainerT>
            bool operator()(const int v, DistLabelT &distToV, const DistLabelContainerT & /*distLabels*/) {

                // Check if we can prune at this vertex based only on the distance from v to the pickup(s)
                if (allSet(search.exceedsLeewayForStop(distToV)))
                    return true;

                int numEntriesScannedHere = 0;

                if constexpr (!StationBucketsEnvT::SORTED) {
                    auto bucket = search.stationbucketContainer.getBucketOf(v);
                    for (const auto &entry: bucket) {
                        ++numEntriesScannedHere;

                        const int &stationId = entry.targetId;

                        const DistanceLabel distViaV = distToV + DistanceLabel(entry.distToTarget);
                        tryUpdatingDistance(stationId, distViaV);
                    }
                } else {

                    auto bucket = search.stationbucketContainer.getBucketOf(v);

                    for (const auto &entry: bucket) {
                        ++numEntriesScannedHere;

                        const int &stationId = entry.targetId;
                        const DistanceLabel distViaV = entry.distToTarget + distToV;
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

            void tryUpdatingDistance(const int stationId, const DistanceLabel& distToStop) {
                // Update tentative distances to v for any searches where distViaV admits a possible better assignment
                // than the current best and where distViaV is at least as good as the current tentative distance.
                LabelMask mask = ~(search.distFromStopToStations[stationId] < distToStop);
                mask &= distToStop < INFTY;
                if (!anySet(mask))
                    return;
                
                if (anySet(mask)) { // if any search requires updates, update the right ones according to mask
                    search.distFromStopToStations[stationId].setIf(distToStop, mask);
                    search.stationsSeen.insert(stationId);
                }
            }


            StationsInEllipse &search;
        };

        struct ScanTargetBucket {

        public:

            explicit ScanTargetBucket(StationsInEllipse &search) : search(search) {}

            template<typename DistLabelT, typename DistLabelContainerT>
            bool operator()(const int v, DistLabelT &distFromV, const DistLabelContainerT & /*distLabels*/) {

                // Check if we can prune at this vertex based only on the distance from v to the pickup(s)
                if (allSet(search.exceedsLeewayForStop(distFromV)))
                    return true;

                int numEntriesScannedHere = 0;

                if constexpr (!StationBucketsEnvT::SORTED) {
                    auto bucket = search.stationbucketContainer.getBucketOf(v);
                    for (const auto &entry: bucket) {
                        ++numEntriesScannedHere;

                        const int &stationId = entry.targetId;

                        const DistanceLabel distViaV = distFromV + DistanceLabel(entry.distToTarget);
                        tryUpdatingDistance(stationId, distViaV);
                    }
                } else {

                    auto bucket = search.stationbucketContainer.getBucketOf(v);

                    for (const auto &entry: bucket) {
                        ++numEntriesScannedHere;

                        const int &stationId = entry.targetId;
                        const DistanceLabel distViaV = entry.distToTarget + distFromV;
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

            void tryUpdatingDistance(const int stationId, const DistanceLabel& distFromV) {
                // Update tentative distances to v for any searches where distViaV admits a possible better assignment
                // than the current best and where distViaV is at least as good as the current tentative distance.
                LabelMask mask = ~(search.distFromStationsToStop[stationId] < distFromV);
                mask &= distFromV < INFTY;
                if (!anySet(mask))
                    return;
                
                if (anySet(mask)) { // if any search requires updates, update the right ones according to mask
                    search.distFromStationsToStop[stationId].setIf(distFromV, mask);
                    search.stationsSeen.insert(stationId);
                }
            }


            StationsInEllipse &search;
        };


        struct StopEllipseSearch {
            explicit StopEllipseSearch(const StationsInEllipse &search) : search(search) {}

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
                const int numberOfStations)
                : inputGraph(inputGraph),
                  upwardSearch(chEnv.template getForwardSearch<ScanSourceBucket, StopEllipseSearch, LabelSetT>(
                               ScanSourceBucket(*this), StopEllipseSearch(*this))),
                  reverseUpwardSearch(chEnv.template getReverseSearch<ScanTargetBucket, StopEllipseSearch, LabelSetT>(
                               ScanTargetBucket(*this), StopEllipseSearch(*this))),
                  ch(chEnv.getCH()),
                  routeState(routeState),
                  stationbucketContainer(stationBucketsEnv.getBuckets()),
                  stopBucketContainer(routeState.getMaxStopId()),
                  stationsSeen(numberOfStations),
                  distFromStopToStations(numberOfStations, DistanceLabel(INFTY)),
                  distFromStationsToStop(numberOfStations, DistanceLabel(INFTY)),
                  curStopId(INVALID_ID),
                  numVerticesSettled(0),
                  numEntriesVisited(0) {}

        void computeNewStationsInEllipsesForStop(const int stopIndex, const int vehId) {
            const int stopId = routeState.stopIdsFor(vehId)[stopIndex];
            const int stopVertex = inputGraph.edgeHead(stopId);
            const int stopRank = ch.rank(stopVertex);

            const int nextStopId = routeState.stopIdsFor(vehId)[stopIndex + 1];
            const int nextStopVertex = inputGraph.edgeHead(nextStopId);
            const int nextStopRank = ch.rank(nextStopVertex);
            
            init(stopId);
            
            // Run the upward search 
            upwardSearch.run(stopRank);

            // Run the reverse upward search from the second stop vertex
            reverseUpwardSearch.run(nextStopRank);

            for (int stationId = 0; stationId < stationsSeen.size(); ++stationId) {
                if (!stationsSeen.contains(stationId)) {
                    continue;
                }

                // If the station was seen, we can check the leeway
                const DistanceLabel distFromStopToStation = distFromStopToStations[stationId];
                const DistanceLabel distFromStationToStop = distFromStationsToStop[stationId];

                // only add the entry to the container if the leeway is not exceeded
                if (!allSet(exceedsLeewayForStop(distFromStopToStation + distFromStationToStop))) {
                    stopBucketContainer.insert(curStopId, StationEntry(stationId, distFromStopToStation.horizontalMin(), distFromStationToStop.horizontalMin()));
                }
            }
        }

        void recomputeStationsInEllipseForStop(const int stopIndex, const int vehId) {
            const int stopId = routeState.stopIdsFor(vehId)[stopIndex];
            init(stopId);

            // Clear the bucket for the current stop
            stopBucketContainer.clearBucket(curStopId);

            // Recompute the stations in the ellipse for the current stop
            computeNewStationsInEllipsesForStop(stopIndex, vehId);
        }

        LabelMask exceedsLeewayForStop(const DistanceLabel &distanceToStop) const {
            return routeState.leewayOfLegStartingAt(curStopId) < distanceToStop;
        }

        ConstantVectorRange<StationEntry> getStationsInEllipse(const int stopId) const {
            assert(stopId >= 0);
            return stopBucketContainer.getBucketOf(stopId);
        }

    private:

        void init(const int stopId) {
            curStopId = stopId;
            stationsSeen.clear();
            distFromStopToStations.clear();
            distFromStationsToStop.clear();
            numVerticesSettled = 0;
            numEntriesVisited = 0;

            // Initialize the distances from the stop to the stations
            distFromStopToStations.resize(stationsSeen.size(), DistanceLabel(INFTY));
            distFromStationsToStop.resize(stationsSeen.size(), DistanceLabel(INFTY));
            stopBucketContainer.checkAndResize(routeState.getMaxStopId());
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

        typename CHEnvT::template UpwardSearch<ScanSourceBucket, StopEllipseSearch, LabelSetT> upwardSearch;
        typename CHEnvT::template UpwardSearch<ScanTargetBucket, StopEllipseSearch, LabelSetT> reverseUpwardSearch;

        const InputGraphT &inputGraph;
        const CH &ch;
        const RouteState &routeState;
        
        const StationBucketContainer &stationbucketContainer;
        StopBucketContainer stopBucketContainer;

        LightweightSubset stationsSeen;

        std::vector<DistanceLabel> distFromStopToStations;
        std::vector<DistanceLabel> distFromStationsToStop;

        int curStopId;

        int numVerticesSettled;
        int numEntriesVisited;

        int totalNumEdgeRelaxations;
        int totalNumVerticesSettled;
        int totalNumEntriesScanned;
    };

}