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
    class TaxiLegApproximation {

    private:

        static constexpr int K = LabelSetT::K;
        using DistanceLabel = typename LabelSetT::DistanceLabel;
        using LabelMask = typename LabelSetT::LabelMask;

        using StationBucketContainer = typename StationBucketsEnvT::BucketContainer;
        using StopBucketContainer = DynamicBucketContainer<StationEntry>;

        struct ScanSourceBucket {

        public:

            explicit ScanSourceBucket(TaxiLegApproximation &search) : search(search) {}

            template<typename DistLabelT, typename DistLabelContainerT>
            bool operator()(const int v, DistLabelT &distToV, const DistLabelContainerT & /*distLabels*/) {

                // Check if we can prune at this vertex based only on the distance from v to the pickup(s)
                if (allSet(search.exceedsGlobalBestCost(distToV)))
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
                        const DistanceLabel distViaV = distToV + DistanceLabel(entry.distToTarget);
                        const auto atLeastAsGoodAsCurBest = ~search.exceedsGlobalBestCost(distViaV);
                        if (!anySet(atLeastAsGoodAsCurBest))
                            break;

                        tryUpdatingDistance(stationId, distViaV);
                        
                    }
                }

                return false;
            }

        private:

            void tryUpdatingDistance(const int stationId, const DistanceLabel& distFromStopToStation) {
                // Update tentative distances to v for any searches where distViaV admits a possible better assignment
                // than the current best and where distViaV is at least as good as the current tentative distance.
                LabelMask mask = ~(search.distFromStations[stationId] < distFromStopToStation);
                mask &= distFromStopToStation < INFTY;
                if (!anySet(mask))
                    return;
                
                if (anySet(mask)) { // if any search requires updates, update the right ones according to mask
                    search.distFromStations[stationId].setIf(distFromStopToStation, mask);
                }
            }


            TaxiLegApproximation &search;
        };

        struct StopSearch {
            explicit StopSearch(const TaxiLegApproximation &search) : search(search) {}

            template<typename DistLabelT, typename DistLabelContainerT>
            bool operator()(const int, DistLabelT &distToOrFromV, const DistLabelContainerT & /*distLabels*/) const {
                return allSet(search.exceedsGlobalBestCost(distToOrFromV));
            }

        private:
            const TaxiLegApproximation &search;

        };

    public:
    
        // Pruning: Fahrzeit = trip time -> untere Schranke
        // > besten globalen Kosten -> abbrechen
        TaxiLegApproximation(
                const InputGraphT &inputGraph,
                const CHEnvT &chEnv,
                const StationBucketsEnvT &stationBucketsEnv,
                const int numberOfStations)
                : inputGraph(inputGraph),
                  reverseUpwardSearch(chEnv.template getReverseSearch<ScanSourceBucket, StopSearch, LabelSetT>(
                               ScanSourceBucket(*this), StopSearch(*this))),
                  ch(chEnv.getCH()),
                  stationbucketContainer(stationBucketsEnv.getSourceBuckets()),
                  distFromStations(numberOfStations, DistanceLabel(INFTY)),
                  upperBoundCost(INFTY) {}

        void findDistancesFromStationsToDest(const int destination, const int maxTripTime) {
            init(maxTripTime);

            const int destinationVertex = inputGraph.edgeTail(destination);
            const int rank = ch.rank(destinationVertex);
            const int offset = inputGraph.travelTime(destination);

            // Run the reverse upward search from the second stop vertex
            // This will compute the distances from all stations to the next stop
            reverseUpwardSearch.runWithOffset(rank, offset);

        }

        void setCostUpperBound(const int c) {
            upperBoundCost = c;
        }

        std::vector<DistanceLabel> getDistancesFromStations() const {
            return distFromStations;
        }

        const int getDistanceFromStation(const int stationId) const {
            assert(stationId >= 0 && stationId < distFromStations.size());
            return distFromStations[stationId][0];
        }

        // trip time to the station
        const int getCostForStation(const int stationId) const {
            assert(stationId >= 0 && stationId < distFromStations.size());
            const DistanceLabel &dist = distFromStations[stationId];
            return CostCalculator::calcTripCost(dist[0], curMaxTripTime);
        }

    private:

        void init(const int maxTripTime) {
            curMaxTripTime = maxTripTime;
            std::fill(distFromStations.begin(), distFromStations.end(), INFTY);
        }

        LabelMask exceedsGlobalBestCost(const DistanceLabel &dist) const {
            const auto tripCost = CostCalculator::calcTripCost(dist[0], curMaxTripTime);
            return tripCost > upperBoundCost;
        }

        typename CHEnvT::template UpwardSearch<ScanSourceBucket, StopSearch, LabelSetT> reverseUpwardSearch;

        const InputGraphT &inputGraph;
        const CH &ch;

        int upperBoundCost;
        int curMaxTripTime;
        
        const StationBucketContainer &stationbucketContainer;

        std::vector<DistanceLabel> distFromStations;
    };

}