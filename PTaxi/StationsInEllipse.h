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
            typename EllipticBucketsEnvT,
            typename StationBucketsEnvT,
            typename LabelSetT = BasicLabelSet<0, ParentInfo::FULL_PARENT_INFO>>
    class StationsInEllipse {

    private:

        static constexpr int K = LabelSetT::K;
        using DistanceLabel = typename LabelSetT::DistanceLabel;
        using LabelMask = typename LabelSetT::LabelMask;


        using EllipticBucketContainer = typename EllipticBucketsEnvT::BucketContainer;
        using StationBucketContainer = typename StationBucketsEnvT::BucketContainer;
        using BucketContainer = DynamicBucketContainer<StationEntry>;

        struct ScanSourceBucket {

        public:

            explicit ScanSourceBucket(StationsInEllipse &search) 
                : search(search) {}

            template<typename DistLabelT, typename DistLabelContainerT>
            bool operator()(const int v, DistLabelT &distToV, const DistLabelContainerT & /*distLabels*/) {
            
                int numEntriesScannedHere = 0;

                auto ellipticBucket = search.sourceEllipticBucketContainer.getBucketOf(v);
                for (const auto &entry: ellipticBucket) {
                    ++numEntriesScannedHere;

                    if (search.curStopId == entry.targetId) {
                        checkAllStationsInEllipse(entry);
                    }
                }
                
                // when station bucket is sorted -> check leeway and break if exceeded
                search.numEntriesVisited += numEntriesScannedHere;
                ++search.numVerticesSettled;

                return false;
            }

        private:
        
            template<typename BucketEntryT>
            void checkAllStationsInEllipse(const BucketEntryT &entry) {
                auto stationBucket = search.stationbucketContainer.getBucketOf(entry.targetId);
                for (const auto &stationEntry: stationBucket) {
                    const int distFromStopToStation = entry.distToTarget + stationEntry.distToTarget;

                    if (distFromStopToStation > entry.leeway) {
                        continue; // skip stations that are too far away
                    }

                    // If the station was seen, we can update the entry
                    search.stationAndStopBucketContainer.insert(search.curStopId, StationEntry(stationEntry.targetId, distFromStopToStation, INFTY));
                    search.stationsSeen.insert(stationEntry.targetId);
                }
            }

            StationsInEllipse &search;
        };

        struct ScanTargetBucket {

        public:

            explicit ScanTargetBucket(StationsInEllipse &search) 
                : search(search) {}

            template<typename DistLabelT, typename DistLabelContainerT>
            bool operator()(const int v, DistLabelT &distToV, const DistLabelContainerT & /*distLabels*/) {
            
                int numEntriesScannedHere = 0;

                auto ellipticBucket = search.targetEllipticBucketContainer.getBucketOf(v);
                for (const auto &entry: ellipticBucket) {
                    ++numEntriesScannedHere;

                    if (search.curNextStopId == entry.targetId) {
                        checkAllStationsInEllipse(entry);
                    }

                }
                
                search.numEntriesVisited += numEntriesScannedHere;
                ++search.numVerticesSettled;

                return false;
            }

        private:
        
            template<typename BucketEntryT>
            void checkAllStationsInEllipse(const BucketEntryT &entry) {
                auto stationBucket = search.stationbucketContainer.getBucketOf(entry.targetId);
                for (const auto &stationEntry: stationBucket) {
                    const int distFromStationToStop = entry.distToTarget + stationEntry.distToTarget;

                    if (distFromStationToStop > entry.leeway) {
                        continue; // skip stations that are too far away
                    }

                    // If the station was seen, we can update the entry
                    search.stationAndStopBucketContainer.insert(search.curStopId, StationEntry(stationEntry.targetId, INFTY, distFromStationToStop));
                    search.stationsSeen.insert(stationEntry.targetId);
                }
            }

            StationsInEllipse &search;
        };


        struct StopEllipseSearch {
            explicit StopEllipseSearch(const StationsInEllipse &search) : search(search) {}

            template<typename DistLabelT, typename DistLabelContainerT>
            bool operator()(const int, DistLabelT &distToV, const DistLabelContainerT & /*distLabels*/) const {
                // no stop criterium
                return false;
            }

        private:
            const StationsInEllipse &search;

        };


    public:
    
        StationsInEllipse(
                const InputGraphT &inputGraph,
                const CHEnvT &chEnv,
                RouteState &routeState, 
                const EllipticBucketsEnvT &ellipticBucketsEnv,
                const StationBucketsEnvT &stationBucketsEnv,
                const int numberOfStations)
                : inputGraph(inputGraph),
                  upwardSearch(chEnv.template getForwardSearch<ScanSourceBucket, StopEllipseSearch, LabelSetT>(
                               ScanSourceBucket(*this), StopEllipseSearch(*this))),
                  reverseUpwardSearch(chEnv.template getReverseSearch<ScanTargetBucket, StopEllipseSearch, LabelSetT>(
                               ScanTargetBucket(*this), StopEllipseSearch(*this))),
                  ch(chEnv.getCH()),
                  routeState(routeState),
                  sourceEllipticBucketContainer(ellipticBucketsEnv.getSourceBuckets()),
                  targetEllipticBucketContainer(ellipticBucketsEnv.getTargetBuckets()),
                  stationbucketContainer(stationBucketsEnv.getBuckets()),
                  stationAndStopBucketContainer(inputGraph.numVertices()),
                  stationsSeen(numberOfStations),
                  curStopId(INVALID_ID),
                  curNextStopId(INVALID_ID),
                  numVerticesSettled(0),
                  numEntriesVisited(0) {}

        void computeNewStationsInEllipsesForStop(const int stopIndex, const int vehId) {
            // Run the upward search from the first stop vertex
            const int stopId = routeState.stopIdsFor(vehId)[stopIndex];
            curStopId = stopId;
            const int stopVertex = inputGraph.edgeHead(stopId);
            const int stopRank = ch.rank(stopVertex);
            upwardSearch.runWithOffset({stopRank});

            // Run the reverse upward search from the second stop vertex
            const int nextStopId = routeState.stopIdsFor(vehId)[stopIndex + 1];
            curNextStopId = nextStopId;
            const int nextStopVertex = inputGraph.edgeHead(nextStopId);
            const int nextStopRank = ch.rank(nextStopVertex);
            reverseUpwardSearch.runWithOffset({nextStopRank});

            for (auto &stationEntry: stationAndStopBucketContainer.getBucketOf(curStopId)) {
                const int stationId = stationEntry.targetId;
                if (!stationsSeen.contains(stationId)) {
                    continue;
                }
                // If the station was seen, we can check the leeway
                const int distFromStopToStation = stationEntry.distFromStopToStation;
                const int distFromStationToStop = stationEntry.distFromStationToStop;
                const int leeway = routeState.leewayOfLegStartingAt(curStopId);

                // only add the entry to the container if the leeway is not exceeded
                if (distFromStopToStation + distFromStationToStop > leeway) {
                    stationAndStopBucketContainer.remove(curStopId, stationId);
                }
            }
        }

    private:

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
        RouteState &routeState;
        
        const EllipticBucketContainer &sourceEllipticBucketContainer;
        const EllipticBucketContainer &targetEllipticBucketContainer;
        const StationBucketContainer &stationbucketContainer;
        BucketContainer stationAndStopBucketContainer;

        LightweightSubset stationsSeen;

        int curStopId;
        int curNextStopId;

        int numVerticesSettled;
        int numEntriesVisited;

        int totalNumEdgeRelaxations;
        int totalNumVerticesSettled;
        int totalNumEntriesScanned;
    };

}