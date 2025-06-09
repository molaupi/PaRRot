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

template<typename BucketEntryT, typename ComparatorT>
class StationBucketContainer {
public:

    using Bucket = ConstantVectorRange<BucketEntryT>;
    using BucketPosition = ValueBlockPosition;

    // Constructs a container that can maintain buckets for the specified number of vertices.
    explicit StationBucketContainer(const int numVertices) 
        : bucketPositions(numVertices), comparator() {
            assert(numVertices >= 0);
        }

    StationBucketContainer(std::vector<BucketPosition>&& bucketPositions,
                           std::vector<BucketEntryT>&& entries)
            : bucketPositions(std::move(bucketPositions)), entries(std::move(entries)) {
        assert(!this->bucketPositions.empty());
        assert(!this->entries.empty());
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
    bool insert(const int v, const BucketEntryT &entry) {
        const auto &pos = bucketPositions[v];
        const auto col = searchForInsertionIdx(entry, pos.start, comparator) - pos.start;
        stableInsertion(v, col, entry, bucketPositions, entries);
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

    // Returns index between start and end (inclusive) where entry should be inserted to preserve order according to
    // comp. start, end, and return value are indices in entries vector.
    template<typename CompT>
    int searchForInsertionIdx(const BucketEntryT &entry, const int start, const int end, CompT &comp) {

        // Check if idle bucket is currently empty or new entry needs to become first element:
        if (start == end || comp(entry, entries[start]))
            return start;

        // Check if new entry needs to become last element:
        if (!comp(entry, entries[end - 1]))
            return end;

        // Binary search with invariant: !idleComp(entry, entries[l]) && idleComp(entry, entries[r])
        int l = start;
        int r = end - 1;
        while (l < r - 1) {
            assert(!comp(entry, entries[l]) && comp(entry, entries[r]));
            int m = (l + r) / 2;
            if (comp(entry, entries[m])) {
                r = m;
            } else {
                l = m;
            }
        }

        return r;
    }

    // Returns whether e1 and e2 are equivalent wrt comp.
    template<typename CompT>
    bool equiv(const BucketEntryT &e1, const BucketEntryT &e2, CompT &comp) const {
        return !comp(e1, e2) && !comp(e2, e1);
    }


    ComparatorT comparator;
    std::vector<BucketPosition> bucketPositions;
    std::vector<BucketEntryT> entries;
};
