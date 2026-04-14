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
#include "../Station/StationEntry.h"

namespace karri {
    template<typename InputGraphT, typename CHEnvT,
        typename StationBucketsEnvT>
    class TaxiLegApproximation {

        using StationBucketContainer = typename StationBucketsEnvT::BucketContainer;
        using StopBucketContainer = DynamicBucketContainer<StationEntry>;

        struct ScanSourceBucket {
        public:
            explicit ScanSourceBucket(TaxiLegApproximation &search) : search(search) {
            }

            template<typename DistLabelT, typename DistLabelContainerT>
            bool operator()(const int v, DistLabelT &distToV, const DistLabelContainerT & /*distLabels*/) {
                // Check if we can prune at this vertex based only on the distance from v to the pickup(s)
                if (search.exceedsGlobalBestCost(distToV[0]))
                    return true;

                int numEntriesScannedHere = 0;

                if constexpr (!StationBucketsEnvT::SORTED) {
                    auto bucket = search.stationBucketContainer.getBucketOf(v);
                    for (const auto &entry: bucket) {
                        ++numEntriesScannedHere;

                        const int &stationId = entry.targetId;
                        const int distViaV = distToV[0] + entry.distToTarget;
                        tryUpdatingDistance(stationId, distViaV);
                    }
                } else {
                    auto bucket = search.stationBucketContainer.getBucketOf(v);

                    for (const auto &entry: bucket) {
                        ++numEntriesScannedHere;

                        const int &stationId = entry.targetId;
                        const int distViaV = distToV[0] + entry.distToTarget;
                        if (search.exceedsGlobalBestCost(distViaV))
                            break;

                        tryUpdatingDistance(stationId, distViaV);
                    }
                }

                search.numEntriesVisited += numEntriesScannedHere;

                return false;
            }

        private:
            void tryUpdatingDistance(const int stationId, const int &distFromStopToStation) {
                // Update tentative distances to v for any searches where distViaV admits a possible better assignment
                // than the current best and where distViaV is at least as good as the current tentative distance.
                if (distFromStopToStation >= search.distFromStations[stationId])
                    return;

                // if any search requires updates, update the right ones according to mask
                search.distFromStations[stationId] = distFromStopToStation;
                search.stationsSeen.insert(stationId);
            }


            TaxiLegApproximation &search;
        };

        struct StopSearch {
            explicit StopSearch(const TaxiLegApproximation &search) : search(search) {
            }

            template<typename DistLabelT, typename DistLabelContainerT>
            bool operator()(const int, DistLabelT &distToOrFromV, const DistLabelContainerT & /*distLabels*/) const {
                return search.exceedsGlobalBestCost(distToOrFromV[0]);
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
              reverseUpwardSearch(chEnv.template getReverseSearch<ScanSourceBucket, StopSearch, BasicLabelSet<0, ParentInfo::NO_PARENT_INFO>>(
                  ScanSourceBucket(*this), StopSearch(*this))),
              ch(chEnv.getCH()),
              stationBucketContainer(stationBucketsEnv.getSourceBuckets()),
              distFromStations(numberOfStations, INFTY),
              upperBoundCost(INFTY),
              stationsSeen(numberOfStations) {
        }

        void findDistancesFromStationsToDest(const int destination,
                                             stats::StationBchPerformanceStats &stats) {
            KaRRiTimer timer;
            init();
            stats.destInitializationTime += timer.elapsed<std::chrono::nanoseconds>();

            timer.restart();
            const int destinationVertex = inputGraph.edgeTail(destination);
            const int rank = ch.rank(destinationVertex);
            const int offset = inputGraph.travelTime(destination);

            // Run the reverse upward search from the second stop vertex
            // This will compute the distances from all stations to the next stop
            reverseUpwardSearch.runWithOffset(rank, offset);
            stats.destBchSearchTime += timer.elapsed<std::chrono::nanoseconds>();

            stats.destNumEdgeRelaxations += reverseUpwardSearch.getNumEdgeRelaxations();
            stats.destNumVerticesSettled += reverseUpwardSearch.getNumVerticesSettled();
            stats.destNumEntriesScanned += numEntriesVisited;
            stats.destNumStationsSeen += stationsSeen.size();
        }

        void setCostUpperBound(const int c) {
            upperBoundCost = c;
        }

        const std::vector<int> &getDistancesFromStations() const {
            return distFromStations;
        }

        const int getDistanceFromStation(const int stationId) const {
            assert(stationId >= -1 && stationId < static_cast<int>(distFromStations.size()));
            if (stationId == INVALID_ID)
                return INFTY;
            return distFromStations[stationId];
        }

        // trip time to the station
        const int getCostForStation(const int stationId) const {
            assert(stationId >= -1 && stationId < static_cast<int>(distFromStations.size()));
            if (stationId == INVALID_ID)
                return INFTY;
            const int &dist = distFromStations[stationId];
            return CostCalculator::CostFunction::calcTripCost(dist);
        }

    private:
        void init() {
            numEntriesVisited = 0;
            stationsSeen.clear();
            std::fill(distFromStations.begin(), distFromStations.end(), INFTY);
        }

        bool exceedsGlobalBestCost(const int &dist) const {
            const auto tripCost = CostCalculator::CostFunction::calcTripCost(dist);
            return tripCost > upperBoundCost;
        }

        typename CHEnvT::template UpwardSearch<ScanSourceBucket, StopSearch, BasicLabelSet<0, ParentInfo::NO_PARENT_INFO>> reverseUpwardSearch;

        const InputGraphT &inputGraph;
        const CH &ch;

        int upperBoundCost;

        const StationBucketContainer &stationBucketContainer;

        std::vector<int> distFromStations;

        int64_t numEntriesVisited = 0;
        LightweightSubset stationsSeen;
    };
}
