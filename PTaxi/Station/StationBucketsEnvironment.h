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
#include <KARRI/Tools/BinaryIO.h>
#include <KARRI/Algorithms/Buckets/BucketEntry.h>
#include <KARRI/Algorithms/CH/CH.h>

#include "StationBucketContainer.h"

namespace parrot {

    template<typename InputGraphT, typename CHEnvT, bool isPsg>
    class StationBucketsEnvironment {

        public:
        static constexpr bool SORTED = true;
        
        struct CompareEntries {
            bool operator()(const BucketEntry &e1, const BucketEntry &e2) const {
                return e1.distToTarget < e2.distToTarget;
            }
        };
        
        // .targetId is station ID, .distToTarget is distance from vertex to station
        using BucketContainer = StationBucketContainer<BucketEntry, CompareEntries>;
        using BucketPosition = ValueBlockPosition;

    private:

        struct StopWhenDistanceExceeded {
            explicit StopWhenDistanceExceeded(const int &maxDist) : maxDist(maxDist) {}

            template<typename DistLabelT, typename DistLabelContT>
            bool operator()(const int, DistLabelT &distToV, const DistLabelContT &) {
                return distToV[0] > maxDist;
            }

        private:
            const int &maxDist;
        };

        struct GenerateTargetEntry {
            explicit GenerateTargetEntry(BucketContainer &targetBucketContainer, int &curStationId, int &verticesVisited)
                    : targetBucketContainer(targetBucketContainer), curStationId(curStationId), verticesVisited(verticesVisited) {}

            template<typename DistLabelT, typename DistLabelContT>
            bool operator()(const int v, DistLabelT &distToV, const DistLabelContT &) {
                auto entry = BucketEntry(curStationId, distToV[0]);
                targetBucketContainer.insert(v, entry);
                ++verticesVisited;
                return false;
            }

            BucketContainer &targetBucketContainer;
            int &curStationId;
            int &verticesVisited;
        };

        struct GenerateSourceEntry {
            explicit GenerateSourceEntry(BucketContainer &sourceBucketContainer, int &curStationId, int &verticesVisited)
                    : sourceBucketContainer(sourceBucketContainer), curStationId(curStationId), verticesVisited(verticesVisited) {}

            template<typename DistLabelT, typename DistLabelContT>
            bool operator()(const int v, DistLabelT &distToV, const DistLabelContT &) {
                auto entry = BucketEntry(curStationId, distToV[0]);
                sourceBucketContainer.insert(v, entry);
                ++verticesVisited;
                return false;
            }

            BucketContainer &sourceBucketContainer;
            int &curStationId;
            int &verticesVisited;
        };
        
    public:

        StationBucketsEnvironment(const InputGraphT &inputGraph, const CHEnvT &chEnv)
                : inputGraph(inputGraph),
                  ch(chEnv.getCH()),
                  searchGraph(ch.downwardGraph()),
                  sourceBucketContainer(searchGraph.numVertices()),
                  targetBucketContainer(searchGraph.numVertices()),
                  sourceEntryGenSearch(
                          chEnv.getForwardSearch(GenerateSourceEntry(sourceBucketContainer, stationId, verticesVisitedInSearch),
                                                 StopWhenDistanceExceeded(INFTY))),
                  targetEntryGenSearch(
                          chEnv.getReverseSearch(GenerateTargetEntry(targetBucketContainer, stationId, verticesVisitedInSearch),
                                                 StopWhenDistanceExceeded(INFTY)))
                {}


        const BucketContainer &getSourceBuckets() const {
            return sourceBucketContainer;
        }

        const BucketContainer &getTargetBuckets() const {
            return targetBucketContainer;
        }

        void generateBucketEntries(const Station &station) {
            verticesVisitedInSearch = 0;
            stationId = station.stationId;
            const auto edgeId = isPsg ? station.psgEdgeId : station.vehEdgeId;
            const auto stationHead = inputGraph.edgeHead(edgeId);
            const auto stationTail = inputGraph.edgeTail(edgeId);
            const auto stationEdge = inputGraph.travelTime(edgeId);

            sourceEntryGenSearch.run(ch.rank(stationHead));
            targetEntryGenSearch.runWithOffset(ch.rank(stationTail), stationEdge);
        }

        // Reads the bucket container from a binary file. 
        void readBucketsFrom(std::ifstream &in) {
            sourceBucketContainer.readFrom(in);
            targetBucketContainer.readFrom(in);
        }

        // Writes the bucket container to a binary file
        void writeBucketsTo(std::ofstream &out) const {
            sourceBucketContainer.writeTo(out);
            targetBucketContainer.writeTo(out);
        }

    private:
        using GenerateSourceEntriesSearch = typename CHEnvT::template UpwardSearch<GenerateSourceEntry, StopWhenDistanceExceeded>;
        using GenerateTargetEntriesSearch = typename CHEnvT::template UpwardSearch<GenerateTargetEntry, StopWhenDistanceExceeded>;

        const InputGraphT &inputGraph;
        const CH &ch;
        const CH::SearchGraph &searchGraph;

        BucketContainer sourceBucketContainer;
        BucketContainer targetBucketContainer;

        int stationId;
        int verticesVisitedInSearch;

        GenerateSourceEntriesSearch sourceEntryGenSearch;
        GenerateTargetEntriesSearch targetEntryGenSearch;

    };
}