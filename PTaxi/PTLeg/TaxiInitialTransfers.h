#pragma once

#include <vector>
#include <iostream>
#include <algorithm>

#include <KARRI/Tools/Constants.h>
#include <ULTRA/DataStructures/Container/IndexedSet.h>
#include <ULTRA/DataStructures/Graph/Graph.h>
#include <ULTRA/Helpers/Types.h>
#include <ULTRA/Helpers/Meta.h>

#include <KARRI/Algorithms/CH/CH.h>
#include <KARRI/Algorithms/Buckets/BucketEntry.h>
#include <KARRI/DataStructures/Labels/BasicLabelSet.h>
#include "../Station/StationBucketsEnvironment.h"

namespace RAPTOR {

/**
 * TaxiInitialTransfers is a class that substitutes the ULTRA initialTransfers 
 * (e.g., BucketCHInitialTransfers) in the TaxiULTRARAPTOR algorithm.
 * 
 * This class uses KaRRi CH queries with pre-built pedestrian station buckets
 * to compute walking distances:
 * - origin to all stations (forward direction)
 * - all stations to destination (backward direction)  
 * - direct origin to destination
 * 
 */
template<typename InputGraphT, typename CHEnvT, typename StationBucketsEnvT>
class TaxiInitialTransfers {
public:
    using LabelSetT = BasicLabelSet<0, ParentInfo::FULL_PARENT_INFO>;
    using Graph = TransferGraph;
    using Type = TaxiInitialTransfers<InputGraphT, CHEnvT, StationBucketsEnvT>;
    
    static constexpr int K = LabelSetT::K;
    using DistanceLabel = typename LabelSetT::DistanceLabel;
    using LabelMask = typename LabelSetT::LabelMask;

private:
    // For origin -> station distances
    struct ScanSourceBuckets {
        explicit ScanSourceBuckets(TaxiInitialTransfers& search) : search(search) {}

        template<typename DistLabelT, typename DistLabelContainerT>
        bool operator()(const int v, DistLabelT& distFromV, const DistLabelContainerT&) {
            for (const auto& entry : search.sourceBucketContainer.getBucketOf(v)) {
                const int stationId = entry.targetId;
                const int distViaV = distFromV[0] + entry.distToTarget;
                if (distViaV < search.targetDistance && distViaV < search.distance[BACKWARD][stationId]) {
                    if (search.distance[BACKWARD][stationId] == INFTY) {
                        search.reachedPOIs[BACKWARD].emplace_back(Vertex(stationId));
                    }
                    search.distance[BACKWARD][stationId] = distViaV;
                }
            }
            return false; // Never prune
        }

        TaxiInitialTransfers& search;
    };

    // For station -> destination distances
    struct ScanTargetBuckets {
        explicit ScanTargetBuckets(TaxiInitialTransfers& search) : search(search) {}

        template<typename DistLabelT, typename DistLabelContainerT>
        bool operator()(const int v, DistLabelT& distToV, const DistLabelContainerT&) {
            for (const auto& entry : search.targetBucketContainer.getBucketOf(v)) {
                const int stationId = entry.targetId;
                const int distViaV = distToV[0] + entry.distToTarget;
                if (distViaV < search.targetDistance && distViaV < search.distance[FORWARD][stationId]) {
                    if (search.distance[FORWARD][stationId] == INFTY) {
                        search.reachedPOIs[FORWARD].emplace_back(Vertex(stationId));
                    }
                    search.distance[FORWARD][stationId] = distViaV;
                }
            }
            return false; // Never prune
        }

        TaxiInitialTransfers& search;
    };


public:
    /**
     * Constructor that initializes the BCH query infrastructure.
     * @param inputGraph The pedestrian input graph
     * @param chEnv The CH environment for the pedestrian graph
     * @param stationBucketsEnv The pre-built pedestrian station buckets
     * @param numStations The number of PT stations (= number of stops)
     */
    TaxiInitialTransfers(const InputGraphT& inputGraph, const CHEnvT& chEnv,
                         const StationBucketsEnvT& stationBucketsEnv, const int numStations)
        : inputGraph(inputGraph)
        , ch(chEnv.getCH())
        , chQuery(chEnv.template getFullCHQuery<LabelSetT>())
        , forwardSearch(chEnv.template getForwardSearch<ScanTargetBuckets, dij::NoCriterion, LabelSetT>(
              ScanTargetBuckets(*this)))
        , backwardSearch(chEnv.template getReverseSearch<ScanSourceBuckets, dij::NoCriterion, LabelSetT>(
              ScanSourceBuckets(*this)))
        , sourceBucketContainer(stationBucketsEnv.getSourceBuckets())
        , targetBucketContainer(stationBucketsEnv.getTargetBuckets())
        , distance{std::vector<int>(numStations, INFTY), std::vector<int>(numStations, INFTY)}
        , originEdge(INVALID_ID)
        , destEdge(INVALID_ID)
        , numStations(numStations)
        , reachedPOIs{std::vector<Vertex>(), std::vector<Vertex>()}
        , targetDistance(INFTY) {
    }

    /**
     * Run the initial transfer queries for the given origin and destination edges.
     * This computes:
     * 1. Direct origin -> destination distance
     * 2. Origin -> all stations distances (forward BCH)
     * 3. All stations -> destination distances (backward BCH)
     * 
     * @param origin The origin edge ID in the pedestrian graph
     * @param destination The destination edge ID in the pedestrian graph
     * @param targetPruningFactor Unused, for interface compatibility
     */
    template<bool TARGET_PRUNING = true>
    inline void run(const int origin, const int destination, const double /* targetPruningFactor */ = 1) noexcept {
        if (originEdge == origin && destEdge == destination) return;

        originEdge = origin;
        destEdge = destination;

        clear();

        // Get ranks and offsets for origin and destination
        const int originHeadRank = ch.rank(inputGraph.edgeHead(origin));
        const int destTailRank = ch.rank(inputGraph.edgeTail(destination));
        const int destOffset = inputGraph.travelTime(destination);

        // 1. Compute direct origin -> destination distance
        chQuery.run(originHeadRank, destTailRank);
        targetDistance = chQuery.getDistance() + destOffset;

        // 2. Forward BCH: origin -> all stations
        // Run upward search from origin head, scanning source buckets
        forwardSearch.run(originHeadRank);

        // 3. Backward BCH: all stations -> destination
        // Run upward search from destination tail with offset, scanning target buckets
        backwardSearch.runWithOffset(destTailRank, destOffset);
    }

    /**
     * Clear all stored distances and POIs.
     */
    inline void clear() noexcept {
        clear<FORWARD>();
        clear<BACKWARD>();
        targetDistance = INFTY;
    }

    // ==================== Standard Interface Methods ====================
    // These methods provide the interface required by TaxiULTRARAPTOR

    /**
     * Get the direct distance from origin to destination.
     */
    inline int getDistance(const Vertex = noVertex) const noexcept {
        return targetDistance;
    }

    /**
     * Get the array of forward distances (origin -> stations).
     */
    inline const std::vector<int>& getForwardDistance() const noexcept {
        return distance[FORWARD];
    }

    /**
     * Get the distance from origin to a specific station.
     */
    inline int getForwardDistance(const Vertex vertex) const noexcept {
        return distance[FORWARD][vertex];
    }

    /**
     * Get the array of backward distances (stations -> destination).
     */
    inline const std::vector<int>& getBackwardDistance() const noexcept {
        return distance[BACKWARD];
    }

    /**
     * Get the distance from a specific station to destination.
     */
    inline int getBackwardDistance(const Vertex vertex) const noexcept {
        return distance[BACKWARD][vertex];
    }

    /**
     * Get the list of stations reachable from origin.
     */
    inline const std::vector<Vertex>& getForwardPOIs() const noexcept {
        return reachedPOIs[FORWARD];
    }

    /**
     * Get the list of stations that can reach destination.
     */
    inline const std::vector<Vertex>& getBackwardPOIs() const noexcept {
        return reachedPOIs[BACKWARD];
    }

    /**
     * Get the origin edge ID.
     */
    inline int getOriginEdge() const noexcept {
        return originEdge;
    }

    /**
     * Get the destination edge ID.
     */
    inline int getDestEdge() const noexcept {
        return destEdge;
    }

private:
    template<int DIRECTION>
    inline void clear() noexcept {
        for (const Vertex vertex : reachedPOIs[DIRECTION]) {
            distance[DIRECTION][vertex] = INFTY;
        }
        reachedPOIs[DIRECTION].clear();
    }

    const InputGraphT& inputGraph;
    const CH& ch;
    
    typename CHEnvT::template FullCHQuery<LabelSetT> chQuery;
    typename CHEnvT::template UpwardSearch<ScanTargetBuckets, dij::NoCriterion, LabelSetT> forwardSearch;
    typename CHEnvT::template UpwardSearch<ScanSourceBuckets, dij::NoCriterion, LabelSetT> backwardSearch;
    
    const typename StationBucketsEnvT::BucketContainer& sourceBucketContainer;
    const typename StationBucketsEnvT::BucketContainer& targetBucketContainer;

    std::vector<int> distance[2];
    int originEdge;
    int destEdge;
    int numStations;
    std::vector<Vertex> reachedPOIs[2];
    int targetDistance;
};

} // namespace RAPTOR
