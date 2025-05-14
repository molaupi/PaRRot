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


// Data structure for tracking distances from origin to stations.
    template<typename LabelSetT>
    class TentativeStationDistances {

        static constexpr int K = LabelSetT::K;
        using DistanceLabel = typename LabelSetT::DistanceLabel;
        using LabelMask = typename LabelSetT::LabelMask;

    public:

        TentativeStationDistances(const int numberOfStations)
                : distances(numberOfStations, DistanceLabel(INFTY)) {}

        void init(const int &numBatches) {
            curNumBatches = numBatches;
            distances.clear();
        }

        void setCurBatchIdx(const int &batchIdx) {
            curBatchIdx = batchIdx;
        }

        int getDistance(const int &stationId, const int &pdLocId) {
            return distances[stationId][0];
        }

        DistanceLabel getDistancesForCurBatch(const int &stationId) {
            return distances[stationId];
        }

        void
        setDistancesForCurBatchIf(const int &stationId, const DistanceLabel &distanceBatch,
                                  const LabelMask &batchInsertMask) {
            if (!anySet(batchInsertMask))
                return;

            distances[stationId].setIf(distanceBatch, batchInsertMask);
        }


    private:

        int curNumBatches;
        std::vector<DistanceLabel> distances; // curNumBatches DistanceLabels per vehicle

        int curBatchIdx;

    };
}