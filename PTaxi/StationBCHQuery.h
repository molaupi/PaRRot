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
#include "KARRI/Tools/Constants.h"
#include "TentativeStationDistances.h"

namespace karri {


    template<typename InputGraphT, typename CHEnvT,
            typename StationBucketsEnvT,
            typename LabelSetT = BasicLabelSet<0, ParentInfo::FULL_PARENT_INFO>>
    class StationBCHQuery {

    private:

        static constexpr int K = LabelSetT::K;
        using DistanceLabel = typename LabelSetT::DistanceLabel;
        using LabelMask = typename LabelSetT::LabelMask;

        struct ScanBucket {

        public:

            explicit ScanBucket(StationBCHQuery &search) : search(search) {}

            template<typename DistLabelT, typename DistLabelContainerT>
            bool operator()(const int v, DistLabelT &distToV, const DistLabelContainerT & /*distLabels*/) {

                int numEntriesScannedHere = 0;
                
                // previous if block here
                auto bucket = search.bucketContainer.getBucketOf(v);
                for (const auto &entry: bucket) {
                    ++numEntriesScannedHere;

                    const int &stationId = entry.targetId;

                    const DistanceLabel distViaV = distToV + DistanceLabel(entry.distToTarget);
                    tryUpdatingDistance(stationId, distViaV);
                }

                // removed else branch
                
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

                if (anySet(mask)) { // if any search requires updates, update the right ones according to mask
                    search.tentativeDistances.setDistancesForCurBatchIf(stationId, distFromPDLoc, mask);
                    search.stationsSeen.insert(stationId);
                }
            }


            StationBCHQuery &search;
        };


        struct StopLastStopBCH {
            explicit StopLastStopBCH(const StationBCHQuery &search) : search(search) {}

            template<typename DistLabelT, typename DistLabelContainerT>
            bool operator()(const int, DistLabelT &distToV, const DistLabelContainerT & /*distLabels*/) const {
                // no stop criterium
                return false;
            }

        private:
            const StationBCHQuery &search;

        };


    public:
    
        // Pruning: 
        // 1. no
        // 2. from origin to v
        // 3. consider the routes of the taxi before origin

        StationBCHQuery(
                const InputGraphT &inputGraph,
                const CHEnvT &chEnv,
                const StationBucketsEnvT &stationBucketsEnv,
                const int numberOfStations)
                : inputGraph(inputGraph),
                  upwardSearch(chEnv.template getForwardSearch<ScanBucket, StopLastStopBCH, LabelSetT>(
                               ScanBucket(*this), StopLastStopBCH(*this))),
                  ch(chEnv.getCH()),
                  bucketContainer(stationBucketsEnv.getBuckets()),
                  tentativeDistances(numberOfStations),
                  stationsSeen(numberOfStations),
                  numVerticesSettled(0),
                  numEntriesVisited(0) {}

        // Run BCH queries that obtain distances from pickups to stations
        void runBchQueries(const PDLocs& pdLocs) {

            initPickupSearches(pdLocs);
            for (unsigned int i = 0; i < pdLocs.numPickups(); i += K)
                runSearchesForPickupBatch(i, pdLocs);
        }

    private:

        void initPickupSearches(const PDLocs& pdLocs) {
            totalNumEdgeRelaxations = 0;
            totalNumVerticesSettled = 0;
            totalNumEntriesScanned = 0;

            stationsSeen.clear();
            const int numPickupBatches = pdLocs.numPickups() / K + (pdLocs.numPickups() % K != 0);
            tentativeDistances.init(numPickupBatches);
        }

        void runSearchesForPickupBatch(const int firstPickupId, const PDLocs& pdLocs) {
            assert(firstPickupId % K == 0 && firstPickupId < pdLocs.numPickups());


            std::array<int, K> pickupTails;
            std::array<int, K> travelTimes;
            for (int i = 0; i < K; ++i) {
                const auto &pickup =
                        firstPickupId + i < pdLocs.numPickups() ? pdLocs.pickups[firstPickupId + i]
                                                                      : pdLocs.pickups[firstPickupId];
                pickupTails[i] = inputGraph.edgeTail(pickup.loc);
                travelTimes[i] = inputGraph.travelTime(pickup.loc);
            }

            tentativeDistances.setCurBatchIdx(firstPickupId / K);
            run(pickupTails, travelTimes);

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

        typename CHEnvT::template UpwardSearch<ScanBucket, StopLastStopBCH, LabelSetT> upwardSearch;

        const InputGraphT &inputGraph;
        const CH &ch;
        const typename StationBucketsEnvT::BucketContainer &bucketContainer;

        TentativeStationDistances<LabelSetT> tentativeDistances;

        LightweightSubset stationsSeen;
        int numVerticesSettled;
        int numEntriesVisited;

        int totalNumEdgeRelaxations;
        int totalNumVerticesSettled;
        int totalNumEntriesScanned;
    };

}