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

namespace karri {

    template<typename InputGraphT, typename CHEnvT>
    class StationBucketsEnvironment {

        
        public:
        static constexpr bool SORTED = false;
        
        // .targetId is vehicle ID, .distToTarget is distance from last stop to vertex
        using BucketContainer = StationBucketContainer<BucketEntry>;
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

        struct GenerateEntry {
            explicit GenerateEntry(BucketContainer &bucketContainer, int &stationId, int &verticesVisited)
                    : bucketContainer(bucketContainer), curStation(stationId), verticesVisited(verticesVisited) {}

            template<typename DistLabelT, typename DistLabelContT>
            bool operator()(const int v, DistLabelT &distToV, const DistLabelContT &) {
                auto entry = BucketEntry(curStation, distToV[0]);
                bucketContainer.insert(v, entry);
                ++verticesVisited;
                return false;
            }

            BucketContainer &bucketContainer;
            int &curStation;
            int &verticesVisited;
        };
        
    public:

        StationBucketsEnvironment(const InputGraphT &inputGraph, const CHEnvT &chEnv)
                : inputGraph(inputGraph),
                  ch(chEnv.getCH()),
                  searchGraph(ch.downwardGraph()),
                  bucketContainer(searchGraph.numVertices()),
                  entryGenSearch(
                          chEnv.getReverseSearch(GenerateEntry(bucketContainer, stationId, verticesVisitedInSearch),
                                                 StopWhenDistanceExceeded(maxDetourUntilEndOfServiceTime)))
                {}


        const BucketContainer &getBuckets() const {
            return bucketContainer;
        }

        void generateBucketEntries(int &stationVertex) {
            
            verticesVisitedInSearch = 0;
            entryGenSearch.run(ch.rank(stationVertex));
        }

        // Reads the bucket container from a binary file. 
        void readFrom(std::ifstream &inPositions, std::ifstream &inEntries) {
            std::vector<BucketPosition> bucketPositions;
            std::vector<BucketEntry> entries;
            bio::read(inPositions, bucketPositions);
            bio::read(inEntries, entries);

            bucketContainer = BucketContainer(bucketPositions, entries);
        }

        // Writes the bucket container to a binary file
        void writeTo(std::ofstream &outPositions, std::ofstream &outEntries) const {
            bio::write(outPositions, bucketContainer.getBucketPositions());
            bio::write(outEntries, bucketContainer.getEntries());
        }

    private:
        // no DownwardSearch in CH
        using GenerateEntriesSearch = typename CHEnvT::template UpwardSearch<GenerateEntry, StopWhenDistanceExceeded>;


        const InputGraphT &inputGraph;
        const CH &ch;
        const CH::SearchGraph &searchGraph;

        BucketContainer bucketContainer;

        int stationId;
        int maxDetourUntilEndOfServiceTime;
        int entriesVisitedInSearch;
        int verticesVisitedInSearch;

        GenerateEntriesSearch entryGenSearch;

    };
}