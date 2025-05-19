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

#include <cassert>
#include <vector>
#include "KARRI/Tools/Constants.h"
#include "KARRI/DataStructures/Containers/TimestampedVector.h"

namespace karri {

// Data structure for dynamically tracking distances from pickups -> stations.
// Allocates entries for distances to a station s from all pickups when one relevant distance to s is found
// for the first time.
    template<typename LabelSetT>
    class TentativeStationDistances {

        static constexpr int K = LabelSetT::K;
        using DistanceLabel = typename LabelSetT::DistanceLabel;
        using LabelMask = typename LabelSetT::LabelMask;

    public:

        TentativeStationDistances(const int numberOfStations)
                : startIdxForStation(numberOfStations, INVALID_INDEX),
                  distances() {}

        void init(const int &numBatches) {
            curNumBatches = numBatches;
            startIdxForStation.clear();
            distances.clear();
        }

        void setCurBatchIdx(const int &batchIdx) {
            curBatchIdx = batchIdx;
        }

        int getDistance(const int &stationId, const int &pdLocId) {
            assert(stationId < startIdxForStation.size());
            const int startIdx = startIdxForStation[stationId];
            if (startIdx == INVALID_INDEX)
                return INFTY;

            const int batchIdx = pdLocId / K;
            return distances[startIdx + batchIdx][pdLocId % K];
        }

        DistanceLabel getDistancesForCurBatch(const int &stationId) {
            assert(stationId < startIdxForStation.size());
            const int startIdx = startIdxForStation[stationId];
            if (startIdx == INVALID_INDEX)
                return DistanceLabel(INFTY);
            return distances[startIdx + curBatchIdx];
        }

        void
        setDistancesForCurBatchIf(const int &stationId, const DistanceLabel &distanceBatch,
                                  const LabelMask &batchInsertMask) {
            if (!anySet(batchInsertMask))
                return;

            if (startIdxForStation[stationId] == INVALID_INDEX) {
                startIdxForStation[stationId] = distances.size();
                distances.insert(distances.end(), curNumBatches, DistanceLabel(INFTY));
            }

            distances[startIdxForStation[stationId] + curBatchIdx].setIf(distanceBatch, batchInsertMask);
        }


    private:

        int curNumBatches;
        TimestampedVector<int> startIdxForStation;
        std::vector<DistanceLabel> distances; // curNumBatches DistanceLabels per station

        int curBatchIdx;

    };
}