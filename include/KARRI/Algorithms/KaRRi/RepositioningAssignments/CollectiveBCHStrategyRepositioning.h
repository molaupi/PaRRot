/// *******************************************************************************
/// Individual BCH Strategy for Repositioning Assignments (batched implementation)
/// *******************************************************************************
#pragma once

#include "../../../Tools/Timer.h"
#include "../RequestState/RequestState.h"
#include "../BaseObjects/Assignment.h"
#include "../BaseObjects/PDLocs.h"
#include "../RouteState.h"
#include "../CostCalculator.h"
#include "../LastStopSearches/RepositioningBucketsEnvironment.h"
#include "../PbnsAssignments/VehicleLocator.h"
#include "../../CH/CH.h"
#include "../../Dijkstra/Dijkstra.h"
#include "../../../DataStructures/Containers/LightweightSubset.h"
#include "../../../DataStructures/Labels/BasicLabelSet.h"
#include "../PDDistanceQueries/PDDistances.h"
#include "../Stats/PerformanceStats.h"
#include "../BaseObjects/InternalTaxiResult.h"
#include <array>

namespace karri {
    template<typename InputGraphT, typename CHEnvT, typename RepositioningBucketsEnvT, typename VehicleLocatorT,
        typename CurVehLocToPickupSearchesT>
    class CollectiveBCHStrategyRepositioning {
    public:
        CollectiveBCHStrategyRepositioning(const InputGraphT &inputGraph,
                                           const Fleet &fleet,
                                           const CHEnvT &chEnv,
                                           const CostCalculator &calculator,
                                           const RepositioningBucketsEnvT &repositionBucketsEnv,
                                           const RouteState &routeState,
                                           VehicleLocatorT &vehicleLocator,
                                           CurVehLocToPickupSearchesT &curVehLocToPickupSearches)
            : inputGraph(inputGraph), fleet(fleet), chEnv(chEnv), ch(chEnv.getCH()),
              calculator(calculator), repositionBuckets(repositionBucketsEnv.getBuckets()), routeState(routeState),
              curVehLocToPickupSearches(curVehLocToPickupSearches),
              vehicleLocator(vehicleLocator), candidateSubset(static_cast<int>(fleet.size())),
              curReqState(nullptr), curResult(nullptr),
              reverseSearch(
                  chEnv.template getReverseSearch<ScanBucket, dij::NoCriterion>(
                      ScanBucket(*this), dij::NoCriterion())),
              oneToAnyQuery(chEnv.template getFullCHQuery<>()) {
        }

        void init() {
            // no-op
        }

        void tryRepositioningAssignments(const RequestState &requestState, const PDDistances &pdDistances,
                                         const PDLocs &pdLocs, InternalTaxiResult &result,
                                         stats::RepositioningAssignmentsPerformanceStats &stats) {
            curReqState = &requestState;
            curResult = &result;

            KaRRiTimer timer;
            // Use lightweight subset for deduplicated candidates
            candidateSubset.clear();

            // Run collective BCH search to find candidate vehicles for repositioning assignments
            const int numPickups = static_cast<int>(pdLocs.pickups.size());
            int pickupIdx = 0;
            numEntriesScanned = 0;
            int64_t numVerticesSettled = 0;
            int64_t numEdgesRelaxed = 0;

            std::vector<int> pickupRanks(numPickups);
            std::vector<int> pickupOffsets(numPickups);
            for (int pId = 0; pId < numPickups; ++pId) {
                const auto &p = pdLocs.pickups[pId];
                pickupRanks[pId] = ch.rank(inputGraph.edgeTail(p.loc));
                pickupOffsets[pId] = inputGraph.travelTime(p.loc);
            }
            minPdDistance = pdDistances.getMinDirectDistance();
            reverseSearch.runWithMultipleRoots(pickupRanks, pickupOffsets);
            numVerticesSettled += reverseSearch.getNumVerticesSettled();
            numEdgesRelaxed += reverseSearch.getNumEdgeRelaxations();

            const auto searchTime = timer.elapsed<std::chrono::nanoseconds>();
            stats.searchTime += searchTime;
            stats.numEntriesScanned += numEntriesScanned;
            stats.numVerticesOrLabelsSettled += numVerticesSettled;
            stats.numEdgeRelaxationsInSearchGraph += numEdgesRelaxed;
            timer.restart();

            // For each candidate vehicle, compute exact location and exact CH distances to pickups, then try assignments
            int64_t numAssignmentsTried = 0;
            computeExactAndTryAssignments(requestState, pdDistances, pdLocs, numAssignmentsTried);

            const auto tryAssignmentsTime = timer.elapsed<std::chrono::nanoseconds>();
            stats.numCandidateVehicles += candidateSubset.size();
            stats.numAssignmentsTried += numAssignmentsTried;
            stats.tryAssignmentsTime += tryAssignmentsTime;

            curReqState = nullptr;
            curResult = nullptr;
        }

    private:
        // Scan functor used by reverse CH search. Reads per-search context from parent fields current*.
        struct ScanBucket {
            explicit ScanBucket(CollectiveBCHStrategyRepositioning &parent) : parent(parent) {
            }

            template<typename DistLabelT, typename DistLabelContT>
            bool operator()(const int v, DistLabelT &distFromV, const DistLabelContT &) {
                // distFromV[i] is the distance from pickup i head to v in the CH downward graph
                for (const auto &entry: parent.repositionBuckets.getBucketOf(v)) {
                    ++parent.numEntriesScanned;
                    const int vehId = entry.targetId;
                    const int lbDistToPickup = entry.distToTarget + distFromV[0];
                    // Compute lower-bound cost
                    const int lowerBoundCost = parent.calculator.calcCostLowerBoundForRepositioningAssignment(
                        lbDistToPickup, parent.minPdDistance, *parent.curReqState);
                    if (lowerBoundCost >= parent.curResult->getBestCost()) {
                        // worse than best known cost -> due to sortedness, all further entries will be worse too
                        break;
                    }
                    // Add vehicle to candidate subset
                    parent.candidateSubset.insert(vehId);
                }
                return false;
            }

            CollectiveBCHStrategyRepositioning &parent;
        };

        void computeExactAndTryAssignments(const RequestState &requestState, const PDDistances &pdDistances,
                                           const PDLocs &pdLocs,
                                           int64_t &numAssignmentsTried) {
            Assignment asgn;
            asgn.pickupStopIdx = 0;
            asgn.dropoffStopIdx = 0;

            std::vector<int> pickupRanks(pdLocs.numPickups());
            std::vector<int> pickupOffsets(pdLocs.numPickups());
            for (int pId = 0; pId < pdLocs.numPickups(); ++pId) {
                const auto &p = pdLocs.pickups[pId];
                pickupRanks[pId] = ch.rank(inputGraph.edgeTail(p.loc));
                pickupOffsets[pId] = inputGraph.travelTime(p.loc);
            }

            for (const auto vehId: candidateSubset) {
                const auto &veh = fleet[vehId];
                asgn.vehicle = &veh;

                // Compute current vehicle location
                int64_t dummyStat;
                const auto vehLocation = vehicleLocator.getCurrentLocation(vehId, requestState.now(), dummyStat);

                // Compute lower bound cost for this vehicle using minimum distance from vehicle location to any pickup
                // obtained with one-to-any CH query
                const auto vehSrcRank = ch.rank(inputGraph.edgeHead(vehLocation.location));
                oneToAnyQuery.runOneToAny(vehSrcRank, pickupRanks, pickupOffsets);
                const int minDistToPickup = oneToAnyQuery.getDistance();
                const int minCost = calculator.calcCostLowerBoundForRepositioningAssignment(
                    minDistToPickup, pdDistances.getMinDirectDistance(), requestState);
                if (minCost > curResult->getBestCost())
                    continue;

                // If necessary, compute exact distances from current vehicle location to all pickups
                for (const auto &p: pdLocs.pickups) {
                    curVehLocToPickupSearches.addPickupForProcessing(p.id);
                }
                curVehLocToPickupSearches.computeDistances(vehId, vehLocation.location, pdLocs, dummyStat, dummyStat);

                // Try all assignments for this vehicle and all pickup-dropoff pairs
                for (const auto &p: pdLocs.pickups) {
                    asgn.pickup = p;
                    KASSERT(curVehLocToPickupSearches.knowsDistance(veh.vehicleId, p.id));
                    asgn.distToPickup = curVehLocToPickupSearches.getDistance(veh.vehicleId, p.id);

                    const int minCostForP = calculator.calcCostLowerBoundForRepositioningAssignment(
                        asgn.distToPickup, pdDistances.getMinDirectDistanceForPickup(p.id), requestState);
                    if (minCostForP > curResult->getBestCost())
                        continue;

                    for (const auto &d: pdLocs.dropoffs) {
                        asgn.dropoff = d;
                        asgn.distToDropoff = pdDistances.getDirectDistance(asgn.pickup.id, asgn.dropoff.id);
                        const int cost = calculator.calc(asgn, *curReqState);
                        curResult->tryAssignmentWithKnownCost(asgn, cost);
                        ++numAssignmentsTried;
                    }
                }
            }
        }

        const InputGraphT &inputGraph;
        const Fleet &fleet;
        const CHEnvT &chEnv;
        const CH &ch;
        const CostCalculator &calculator;
        const typename RepositioningBucketsEnvT::BucketContainer &repositionBuckets;
        const RouteState &routeState;
        CurVehLocToPickupSearchesT &curVehLocToPickupSearches;

        VehicleLocatorT &vehicleLocator;
        LightweightSubset candidateSubset;

        // Transient per-request context, set at the start of tryRepositioningAssignments().
        const RequestState *curReqState;
        InternalTaxiResult *curResult;

        // Current per-search context accessed by ScanBucket: K pickups in a batch
        int minPdDistance;
        int64_t numEntriesScanned;

        using ReverseSearchType = typename CHEnvT::template UpwardSearch<ScanBucket, dij::NoCriterion>;
        ReverseSearchType reverseSearch;

        using OneToAnyQuery = typename CHEnvT::template FullCHQuery<>;
        OneToAnyQuery oneToAnyQuery;
    };
}
