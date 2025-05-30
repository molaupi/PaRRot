/// ******************************************************************************
/// MIT License
///
/// Copyright (c) 2020 Valentin Buchhold
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

#include <algorithm>
#include <cassert>
#include <vector>

#include <KARRI/DataStructures/Utilities/DynamicRagged2DArrays.h>

template<typename BucketEntryT>
class StationBucketContainer {
public:

    using Bucket = ConstantVectorRange<BucketEntryT>;
    using BucketPosition = ValueBlockPosition;

    // Constructs a container that can maintain buckets for the specified number of vertices.
    explicit StationBucketContainer(const int numVertices) : bucketPositions(numVertices) {
        assert(numVertices >= 0);
    }

    // TODO: Change to R Value Reference to avoid copying
    StationBucketContainer(std::vector<BucketPosition> bucketPositions,
                           std::vector<BucketEntryT> entries)
            : bucketPositions(bucketPositions), entries(entries) {
        assert(!bucketPositions.empty());
        assert(!entries.empty());
    }
    
    // Returns the bucket of the specified vertex.
    Bucket getBucketOf(const int vertex) const {
        assert(vertex >= 0);
        assert(vertex < bucketPositions.size());
        const auto &pos = bucketPositions[vertex];
        return Bucket(entries.begin() + pos.start, entries.begin() + pos.end);
    }

    Bucket getUnsortedBucketOf(const int vertex) const {
        return getBucketOf(vertex);
    }

    // Inserts the given entry into the bucket of the specified vertex.
    bool insert(const int vertex, const BucketEntryT &entry) {
        insertion(vertex, entry, bucketPositions, entries);
        return true;
    }

    void clearBucket(const int vertex) {
        assert(vertex >= 0);
        assert(vertex < bucketPositions.size());
        auto &bucketPos = bucketPositions[vertex];
        std::fill(entries.begin() + bucketPos.start, entries.begin() + bucketPos.end, BucketEntryT());
        bucketPos.end = bucketPos.start;
    }

    // Removes all entries from all buckets.
    void clear() {
        for (auto &bucketPos: bucketPositions)
            bucketPos.end = bucketPos.start;
        std::fill(entries.begin(), entries.end(), BucketEntryT());
    }

    bool allEmpty() const {
        const bool posEmpty = std::all_of(bucketPositions.begin(), bucketPositions.end(),
                                          [](const BucketPosition &bucketPos) {
                                              return bucketPos.end == bucketPos.start;
                                          });
        if (!posEmpty) return false;

        const auto hole = BucketEntryT();
        return std::all_of(entries.begin(), entries.end(), [hole](const BucketEntryT &entry) { return entry == hole; });
    }

    // Reads the bucket container from a binary file. 
    void readFrom(std::ifstream &in) {
        bio::read(in, bucketPositions);
        bio::read(in, entries);
    }

    // Writes the bucket container to a binary file
    void writeTo(std::ofstream &out) const {
        bio::write(out, bucketPositions);
        bio::write(out, entries);
    }

private:

    std::vector<BucketPosition> bucketPositions;
    std::vector<BucketEntryT> entries;
};
