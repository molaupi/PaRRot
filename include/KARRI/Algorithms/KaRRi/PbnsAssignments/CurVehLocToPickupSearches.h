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

#include "../../../Tools/Timer.h"
#include "../../../DataStructures/Labels/SimdLabelSet.h"
#include "../../../DataStructures/Containers/TimestampedVector.h"
#include "../../CH/CH.h"
#include "../RouteState.h"

namespace karri {


    template<typename InputGraphT, typename CHEnvT, typename LabelSetT>
    class CurVehLocToPickupSearches {


        static constexpr int K = LabelSetT::K;
        using DistanceLabel = typename LabelSetT::DistanceLabel;

        static constexpr int unknownDist = INFTY + 1;


        struct StopWhenMaxDistExceeded {

            explicit StopWhenMaxDistExceeded(const int &maxDist) : maxDist(maxDist) {}

            template<typename DistLabelT, typename DistLabelContainerT>
            bool operator()(const int, DistLabelT &distToV, const DistLabelContainerT & /*distLabels*/) {
                return !anySet(~(maxDist < distToV));
            }


        private:
            const int &maxDist;
        };

        struct WriteVehicleDistLabel {
            explicit WriteVehicleDistLabel(CurVehLocToPickupSearches &searches) : searches(searches) {}

            template<typename DistLabelT, typename DistLabelContT>
            bool operator()(const int v, const DistLabelT &distToV, const DistLabelContT &) {
                searches.distFromCurVehLocation[v] = distToV[0];
                return false;
            }

        private:

            CurVehLocToPickupSearches &searches;
        };

        struct ScanLabelAndUpdateDistances {
            explicit ScanLabelAndUpdateDistances(CurVehLocToPickupSearches &searches) : searches(
                    searches) {}

            template<typename DistLabelT, typename DistLabelContT>
            bool operator()(const int v, const DistLabelT &distToV, const DistLabelContT &) {
                const auto &distFromVehLocToV = searches.distFromCurVehLocation[v];
                if (distFromVehLocToV >= INFTY)
                    return false;

                DistanceLabel dists = distToV + DistanceLabel(distFromVehLocToV);
                dists.setIf(DistanceLabel(INFTY), ~(distToV < INFTY));
                searches.tentativeDistances.min(dists);
                searches.maxTentativeDist = std::min(searches.maxTentativeDist,
                                                     searches.tentativeDistances.horizontalMax());
                return false;
            }

        private:

            CurVehLocToPickupSearches &searches;
        };

        using WriteVehLabelsSearch = typename CHEnvT::template UpwardSearch<WriteVehicleDistLabel, StopWhenMaxDistExceeded>;
        using FindDistancesSearch = typename CHEnvT::template UpwardSearch<ScanLabelAndUpdateDistances, StopWhenMaxDistExceeded, LabelSetT>;


    public:

        CurVehLocToPickupSearches(const InputGraphT &graph,
                                  const CHEnvT &chEnv,
                                  const RouteState &routeState,
                                  const int fleetSize)
                : inputGraph(graph),
                  ch(chEnv.getCH()),
                  routeState(routeState),
                  fleetSize(fleetSize),
                  distances(),
                  prevNumPickups(0),
                  vehiclesWithDistances(fleetSize),
                  writeVehLabelsSearch(
                          chEnv.template getForwardSearch<WriteVehicleDistLabel, StopWhenMaxDistExceeded>(
                                  WriteVehicleDistLabel(*this), StopWhenMaxDistExceeded(curLeeway))),
                  findDistancesSearch(
                          chEnv.template getReverseSearch<ScanLabelAndUpdateDistances, StopWhenMaxDistExceeded, LabelSetT>(
                                  ScanLabelAndUpdateDistances(*this),
                                  StopWhenMaxDistExceeded(maxTentativeDist))),
                  maxTentativeDist(INFTY),
                  curPickupIds(),
                  currentTime(-1),
                  waitingQueue(),
                  distFromCurVehLocation(chEnv.getCH().upwardGraph().numVertices(), INFTY) {}

        void initialize(const int now, const PDLocs& pdLocs) {
            currentTime = now;

            clearDistances();
            waitingQueue.clear();

            curNumPickups = pdLocs.numPickups();
            const int numDistances = curNumPickups * fleetSize;
            if (numDistances > distances.size()) {
                const int diff = numDistances - distances.size();
                distances.insert(distances.end(), diff, unknownDist);
            }
        }

        bool knowsDistance(const int vehId, const unsigned int pickupId) const {
            assert(vehId >= 0 && vehId < fleetSize);
            assert(pickupId < curNumPickups);
            const int idx = vehId * curNumPickups + pickupId;
            return distances[idx] != unknownDist;
        }

        int getDistance(const int vehId, const unsigned int pickupId) const {
            assert(vehId >= 0 && vehId < fleetSize);
            assert(pickupId < curNumPickups);
            const int idx = vehId * curNumPickups + pickupId;
            return distances[idx];
        }

        // Register pickups for which we want to know the distance from the current location of a vehicle to this pickup.
        // All pickups registered until the next call to computeExactDistancesVia() will be processed with the same vehicle.
        void addPickupForProcessing(const int pickupId) {
            assert(pickupId >= 0);
            assert(pickupId < curNumPickups);
            waitingQueue.push_back(pickupId);
        }

        // Computes the exact distances from the previous stop via the current location of a vehicle to all pickups added
        // using addPickupForProcessing() (since the last call to this function or initialize()). Skips pickups for
        // which the distance via the given vehicle is already known.
        void computeDistances(const int vehId, const int curVehLocation, const PDLocs& pdLocs,
            int64_t &vehLocToPickupTimeStat,
            int64_t &vehLocToPickupNumSearchesStat) {

            curLeeway = routeState.leewayOfLegStartingAt(routeState.stopIdsFor(vehId)[0]);
            if (waitingQueue.empty()) return;

            vehiclesWithDistances.insert(vehId);

            int numChSearchesRun = 0;
            KaRRiTimer timer;
            std::array<int, K> targets;
            std::array<int, K> targetOffsets;

            unsigned int i = 0;
            bool builtLabelsForVeh = false;

            for (auto it = waitingQueue.begin(); it != waitingQueue.end();) {
                const auto pickupId = *it;

                if (!knowsDistance(vehId, pickupId)) {
                    const auto pickupLocation = pdLocs.pickups[pickupId].loc;
                    if (curVehLocation == pickupLocation) {
                        const int idx = vehId * curNumPickups + pickupId;
                        distances[idx] = 0;
                    } else {
                        targets[i] = ch.rank(inputGraph.edgeTail(pickupLocation));
                        targetOffsets[i] = inputGraph.travelTime(pickupLocation);
                        curPickupIds[i] = pickupId;
                        ++i;
                    }
                }

                ++it;
                if (i == K || (it == waitingQueue.end() && i > 0)) {
                    // If there were any pairs left but fewer than K, fill the sources and targets with duplicates of the first pair
                    int endOfBatch = i;
                    for (; i < K; ++i) {
                        targets[i] = targets[0];
                        targetOffsets[i] = targetOffsets[0];
                        curPickupIds[i] = curPickupIds[0];
                    }

                    maxTentativeDist = curLeeway;
                    tentativeDistances = DistanceLabel(INFTY);
                    // Build distance labels for the vehicle
                    if (!builtLabelsForVeh) {
                        distFromCurVehLocation.clear();
                        const auto source = ch.rank(inputGraph.edgeHead(curVehLocation));
                        writeVehLabelsSearch.run(source);
                        builtLabelsForVeh = true;
                    }

                    // Run search from pickups against the vehicle distance labels
                    findDistancesSearch.runWithOffset(targets, targetOffsets);
                    ++numChSearchesRun;

                    // Set found distances.
                    for (int j = 0; j < endOfBatch; ++j) {
                        const int idx = vehId * curNumPickups + curPickupIds[j];
                        distances[idx] = tentativeDistances[j];
                    }

                    i = 0;
                }
            }

            waitingQueue.clear();
            prevNumPickups = curNumPickups;

            vehLocToPickupTimeStat += timer.elapsed<std::chrono::nanoseconds>();
            vehLocToPickupNumSearchesStat += numChSearchesRun;
        }

    private:

        void clearDistances() {

            // Clear the distances for every vehicle for which we computed distances:
            for (const auto &vehId: vehiclesWithDistances) {
                const int start = vehId * prevNumPickups;
                const int end = start + prevNumPickups;
                std::fill(distances.begin() + start, distances.begin() + end, unknownDist);
            }
            assert(std::all_of(distances.begin(), distances.end(), [&](const auto &d) { return d == unknownDist; }));
            vehiclesWithDistances.clear();
        }

        const InputGraphT &inputGraph;
        const CH &ch;
        const RouteState &routeState;
        const int fleetSize;


        std::vector<int> distances;
        int prevNumPickups;
        LightweightSubset vehiclesWithDistances;

        WriteVehLabelsSearch writeVehLabelsSearch;
        FindDistancesSearch findDistancesSearch;
        DistanceLabel tentativeDistances;
        int maxTentativeDist;
        std::array<unsigned int, K> curPickupIds;
        int curLeeway;

        int currentTime;
        int curNumPickups;

        // Entry in waiting queue is pickupId + dist from previous stop of vehicle to pickup.
        std::vector<unsigned int> waitingQueue;

        TimestampedVector<int> distFromCurVehLocation;
    };

}