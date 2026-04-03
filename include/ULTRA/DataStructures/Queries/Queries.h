#pragma once

#include <random>
#include <vector>

#include "../../Helpers/Types.h"
#include "../../../Common/Constants.h"

struct VertexQuery {
    VertexQuery(const Vertex source = noVertex, const Vertex target = noVertex, const int departureTime = never) :
        source(source),
        target(target),
        departureTime(departureTime) {
    }

    inline friend std::ostream& operator<<(std::ostream& out, const VertexQuery& query) noexcept {
        return out << query.source << " -> " << query.target << " @ " << query.departureTime << std::endl;
    }

    Vertex source;
    Vertex target;
    int departureTime;
};

struct EdgeQuery {
    EdgeQuery(const int originVehEdge = INVALID_ID, const int originPsgEdge = INVALID_ID,
              const int destinationVehEdge = INVALID_ID, const int destinationPsgEdge = INVALID_ID,
              const int departureTime = never) :
        originVehEdge(originVehEdge),
        originPsgEdge(originPsgEdge),
        destinationVehEdge(destinationVehEdge),
        destinationPsgEdge(destinationPsgEdge),
        departureTime(departureTime) {
    }

    inline friend std::ostream& operator<<(std::ostream& out, const EdgeQuery& query) noexcept {
        return out << "veh:" << query.originVehEdge << " (psg:" << query.originPsgEdge << ") -> "
                   << "veh:" << query.destinationVehEdge << " (psg:" << query.destinationPsgEdge << ") @ "
                   << query.departureTime << std::endl;
    }

    int originVehEdge;
    int originPsgEdge;
    int destinationVehEdge;
    int destinationPsgEdge;
    int departureTime;
};

inline std::vector<VertexQuery> generateRandomVertexQueries(const size_t numVertices, const size_t numQueries, const int startTime = 0, const int endTime = 24 * 60 * 60) noexcept {
    std::mt19937 randomGenerator(42);
    std::uniform_int_distribution<> vertexDistribution(0, numVertices - 1);
    std::uniform_int_distribution<> timeDistribution(startTime, endTime - 1);
    std::vector<VertexQuery> queries;
    for (size_t i = 0; i < numQueries; i++) {
        queries.emplace_back(Vertex(vertexDistribution(randomGenerator)), Vertex(vertexDistribution(randomGenerator)), timeDistribution(randomGenerator));
    }
    return queries;
}

struct OneToAllQuery {
    OneToAllQuery(const Vertex source = noVertex, const int departureTime = never) :
        source(source),
        departureTime(departureTime) {
    }

    Vertex source;
    int departureTime;
};

inline std::vector<OneToAllQuery> generateRandomOneToAllQueries(const size_t numVertices, const size_t numQueries, const int startTime = 0, const int endTime = 24 * 60 * 60) noexcept {
    std::mt19937 randomGenerator(42);
    std::uniform_int_distribution<> vertexDistribution(0, numVertices - 1);
    std::uniform_int_distribution<> timeDistribution(startTime, endTime - 1);
    std::vector<OneToAllQuery> queries;
    for (size_t i = 0; i < numQueries; i++) {
        queries.emplace_back(Vertex(vertexDistribution(randomGenerator)), timeDistribution(randomGenerator));
    }
    return queries;
}

struct StopQuery {
    StopQuery(const StopId source = noStop, const StopId target = noStop, const int departureTime = never) :
        source(source),
        target(target),
        departureTime(departureTime) {
    }

    inline friend std::ostream& operator<<(std::ostream& out, const StopQuery& query) noexcept {
        return out << query.source << " -> " << query.target << " @ " << query.departureTime << std::endl;
    }

    StopId source;
    StopId target;
    int departureTime;
};

inline std::vector<StopQuery> generateRandomStopQueries(const size_t numStops, const size_t numQueries, const int startTime = 0, const int endTime = 24 * 60 * 60) noexcept {
    std::mt19937 randomGenerator(42);
    std::uniform_int_distribution<> stopDistribution(0, numStops - 1);
    std::uniform_int_distribution<> timeDistribution(startTime, endTime - 1);
    std::vector<StopQuery> queries;
    for (size_t i = 0; i < numQueries; i++) {
        queries.emplace_back(StopId(stopDistribution(randomGenerator)), StopId(stopDistribution(randomGenerator)), timeDistribution(randomGenerator));
    }
    return queries;
}
