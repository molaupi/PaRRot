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
#include "KARRI/Algorithms/KaRRi/LastStopSearches/TentativeLastStopDistances.h"
#include "KARRI/DataStructures/Labels/BasicLabelSet.h"
#include "KARRI/DataStructures/Containers/LightweightSubset.h"
#include "KARRI/DataStructures/Utilities/IteratorRange.h"
#include "KARRI/Tools/Constants.h"
#include "TentativeStationDistances.h"

namespace karri {


    template<typename InputGraphT, typename CHEnvT,
            typename StationBucketsEnvT,
            typename LabelSetT = BasicLabelSet<0, ParentInfo::FULL_PARENT_INFO>>
    class LastStopToStationBCHQuery {

    private:

        static constexpr int K = LabelSetT::K;
        using DistanceLabel = typename LabelSetT::DistanceLabel;
        using LabelMask = typename LabelSetT::LabelMask;

        struct ScanBucket {

        public:

            explicit ScanBucket(LastStopToStationBCHQuery &search) : search(search){}

            template<typename DistLabelT, typename DistLabelContainerT>
            bool operator()(const int v, DistLabelT &distToV, const DistLabelContainerT & /*distLabels*/) {
            
                int numEntriesScannedHere = 0;

                auto bucket = search.bucketContainer.getBucketOf(v);
                for (const auto &entry: bucket) {
                    ++numEntriesScannedHere;

                    const int &stationId = entry.targetId;

                    const DistanceLabel distViaV = distToV + DistanceLabel(entry.distToTarget);
                    tryUpdatingDistance(stationId, distViaV);
                }
                
                search.numEntriesVisited += numEntriesScannedHere;
                ++search.numVerticesSettled;

                return false;
            }

        private:

            void tryUpdatingDistance(const int stationId, const DistanceLabel& distFromPDLoc) {
                // Update tentative distances to v for any searches where distViaV admits a possible better assignment
                // than the current best and where distViaV is at least as good as the current tentative distance.
                LabelMask mask = ~(search.tentativeDistances.getDistancesForCurBatch(stationId) < distFromPDLoc);
                mask &= distFromPDLoc < INFTY;
                if (!anySet(mask))
                    return;
                    
                search.tentativeDistances.setDistancesForCurBatchIf(stationId, distFromPDLoc, mask);
                search.stationsSeen.insert(stationId);
            }


            LastStopToStationBCHQuery &search;
        };


        struct StopStationBCH {
            explicit StopStationBCH(const LastStopToStationBCHQuery &search) : search(search) {}

            template<typename DistLabelT, typename DistLabelContainerT>
            bool operator()(const int, DistLabelT &distToV, const DistLabelContainerT & /*distLabels*/) const {
                return false; // No stop criterion, we scan all entries in the bucket.
            }

        private:
            const LastStopToStationBCHQuery &search;

        };


    public:

     using StationDistances = TentativeStationDistances<LabelSetT>;

        LastStopToStationBCHQuery(
                const InputGraphT &inputGraph,
                const CHEnvT &chEnv,
                const Fleet &fleet,
                const RouteState &routeState,
                const StationBucketsEnvT &stationBucketsEnv,
                const int numberOfStations)
                : inputGraph(inputGraph),
                  ch(chEnv.getCH()),
                  routeState(routeState),
                  calc(CostCalculator(routeState)),
                  upwardSearch(chEnv.template getForwardSearch<ScanBucket, StopStationBCH, LabelSetT>(
                               ScanBucket(*this), StopStationBCH(*this))),
                  bucketContainer(stationBucketsEnv.getBuckets()),
                  tentativeDistances(numberOfStations),
                  stationsSeen(numberOfStations),
                  fleetSize(fleet.size()),
                  numVerticesSettled(0),
                  numEntriesVisited(0),
                  externalUpperBoundCost(INFTY) {}

        // Run BCH queries that obtain distances from last stops to stations
        void runBchQueries(const int vehId) {
            initSearch();
            runSearchForVehId(vehId);
        }

        TentativeStationDistances<LabelSetT> &getTentativeDistances() {
            return tentativeDistances;
        }

         // Sets a known upper bound on the cost of a PALS insertion.
        void setExternalCostUpperBound(const int c) {
            externalUpperBoundCost = c;
        }


    private:

        void initSearch() {
            totalNumEdgeRelaxations = 0;
            totalNumVerticesSettled = 0;
            totalNumEntriesScanned = 0;
            
            upperBoundCost = externalUpperBoundCost;
            externalUpperBoundCost = INFTY;

            stationsSeen.clear();
            tentativeDistances.init(fleetSize);
        }

        void runSearchForVehId(const int vehId) {
            assert(vehId < fleetSize);

            const int lastStopIndex = routeState.numStopsOf(vehId) - 1;
            
            const int lastStopLocation = routeState.stopLocationsFor(vehId)[lastStopIndex];
            const int lastStopTail = inputGraph.edgeTail(lastStopLocation);
            const int lastStopTravelTime = inputGraph.travelTime(lastStopLocation);

            tentativeDistances.setCurBatchIdx(vehId);
            run({lastStopTail}, {lastStopTravelTime});

            totalNumEdgeRelaxations += getNumEdgeRelaxations();
            totalNumVerticesSettled += getNumVerticesSettled();
            totalNumEntriesScanned += getNumEntriesScanned();
        }

        void run(const std::array<int, K> &sources,
                 const std::array<int, K> offsets = {}) {
            numVerticesSettled = 0;
            numEntriesVisited = 0;
            std::array<int, K> sources_ranks = {};
            std::transform(sources.begin(), sources.end(), sources_ranks.begin(),
                           [&](const int v) { return ch.rank(v); });
            upwardSearch.runWithOffset(sources_ranks, offsets);
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

        typename CHEnvT::template UpwardSearch<ScanBucket, StopStationBCH, LabelSetT> upwardSearch;

        const int fleetSize;
        const InputGraphT &inputGraph;
        const CH &ch;
        const RouteState &routeState;
        const typename StationBucketsEnvT::BucketContainer &bucketContainer;
        CostCalculator calc;

        StationDistances tentativeDistances;

        int externalUpperBoundCost;
        int upperBoundCost;

        LightweightSubset stationsSeen;
        int numVerticesSettled;
        int numEntriesVisited;

        int totalNumEdgeRelaxations;
        int totalNumVerticesSettled;
        int totalNumEntriesScanned;
    };

}