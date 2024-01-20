#include <algorithm>
#include <iomanip>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "../../Algorithms/CH/CH.h"
#include "../../Algorithms/CH/Query/BucketQuery.h"
#include "../../DataStructures/Graph/Graph.h"
#include "../../DataStructures/RAPTOR/Data.h"

namespace RIDERAPTOR {

struct DistanceMatrix {
    DistanceMatrix() { }

    DistanceMatrix(const int numberOfStops)
        : matrix(numberOfStops * numberOfStops)
        , numberOfStops(numberOfStops)
    {
    }

    DistanceMatrix(const std::string& fileName) { readFrom(fileName); }

private:
    std::vector<int> matrix;
    int numberOfStops;

public:
    inline int setDistance(const int from, const int to, const int value)
    {
        return matrix[getIndex(from, to)] = value;
    }

    inline int getDistance(const int from, const int to) const noexcept
    {
        return matrix[getIndex(from, to)];
    }

    void readFrom(const std::string& fileName,
        const std::string& separator = ".")
    {
        IO::deserialize(fileName + separator + "matrix", matrix);
        IO::deserialize(fileName + separator + "numberOfStops", numberOfStops);
    }

    void writeTo(const std::string& fileName,
        const std::string& separator = ".")
    {
        IO::serialize(fileName + separator + "matrix", matrix);
        IO::serialize(fileName + separator + "numberOfStops", numberOfStops);
    }

private:
    int getIndex(const int from, const int to) const noexcept
    {
        return from * numberOfStops + to;
    }
};

inline void fillDistanceMatrix(DistanceMatrix& matrix, RAPTOR::Data& raptorData,
    CH::CH& networkCH)
{
    using CHQuery = CH::Query<CHGraph, true, false, true>;
    CHQuery query(networkCH, FORWARD, raptorData.numberOfStops());

    for (const auto fromStop : raptorData.stops()) {
        if (fromStop.value() % 500 == 0) {
            std::cout << "Stop " << fromStop << " done." << std::endl;
        }

        query.run<FORWARD, BACKWARD>(Vertex(fromStop));
        for (const auto toStop : raptorData.stops()) {
            matrix.setDistance(fromStop, toStop,
                query.getForwardDistance(Vertex(toStop)));
        }
    }
}

inline void fillDistanceMatrixUsingBCH(DistanceMatrix& matrix,
    RAPTOR::Data& raptorData,
    CH::CH& networkCH)
{
    using BucketCH = CH::BucketQuery<CHGraph, true, false>;
    BucketCH bucketCH(networkCH, FORWARD, raptorData.numberOfStops());

    for (const auto fromStop : raptorData.stops()) {
        if (fromStop.value() % 500 == 0) {
            std::cout << "Stop " << fromStop << " done." << std::endl;
        }

        bucketCH.run<FORWARD, BACKWARD, false>(Vertex(fromStop));
        for (const auto toStop : raptorData.stops()) {
            matrix.setDistance(fromStop, toStop,
                bucketCH.getForwardDistance(Vertex(toStop)));
        }
    }
}
} // namespace RIDERAPTOR