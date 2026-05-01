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

#include <KARRI/Tools/Timer.h>
#include <KARRI/Algorithms/KaRRi/RequestState/RequestState.h>
#include <KARRI/Algorithms/KaRRi/LastStopSearches/LastStopBCHQuery.h>
#include <KARRI/Algorithms/KaRRi/CostCalculator.h>
#include <KARRI/DataStructures/Containers/FastResetFlagArray.h>
#include <sys/stat.h>

#include "StationsAtLocations.h"

namespace parrot {
    using namespace karri;

    template<typename InputGraphT,
        typename CHEnvT,
        typename CurVehLocToPickupSearchesT,
        typename StationBucketsEnvT,
        typename LabelSet>
    class DALSToStations {
    private:
        static constexpr int K = LabelSet::K;
        using LabelMask = typename LabelSet::LabelMask;
        using DistanceLabel = typename LabelSet::DistanceLabel;

        struct ScanBucket {
        public:
            explicit ScanBucket(DALSToStations &search) : search(search) {
            }

            template<typename DistLabelT, typename DistLabelContainerT>
            bool operator()(const int v, DistLabelT &distToV, const DistLabelContainerT & /*distLabels*/) {
                int numEntriesScannedHere = 0;

                if constexpr (!StationBucketsEnvT::SORTED) {
                    auto bucket = search.bucketContainer.getBucketOf(v);
                    for (const auto &entry: bucket) {
                        ++numEntriesScannedHere;

                        const int &stationId = entry.targetId;

                        const DistanceLabel distViaV = distToV + DistanceLabel(entry.distToTarget);
                        tryUpdatingDistance(stationId, distViaV);
                    }
                } else {
                    auto bucket = search.bucketContainer.getBucketOf(v);

                    for (const auto &entry: bucket) {
                        ++numEntriesScannedHere;

                        const int &stationId = entry.targetId;
                        const DistanceLabel distViaV = distToV + DistanceLabel(entry.distToTarget);
                        const auto atLeastAsGoodAsCurBest = ~search.
                                canPrune(distViaV, search.curMinCostToLastStop);

                        if (!anySet(atLeastAsGoodAsCurBest))
                            break;

                        tryUpdatingDistance(stationId, distViaV, atLeastAsGoodAsCurBest);
                    }
                }

                search.numEntriesVisited += numEntriesScannedHere;
                ++search.numVerticesSettled;

                return false;
            }

        private:
            void tryUpdatingDistance(const int stationId, const DistanceLabel &distToStation,
                                     LabelMask needToUpdate) {
                KASSERT(allSet((distToStation < INFTY) | ~needToUpdate));
                // Update tentative distances to v for any searches where distViaV admits a possible better assignment
                // than the current best and where distViaV is at least as good as the current tentative distance.
                auto &cur = search.currentLastStopDistances[stationId];
                needToUpdate &= distToStation < cur;
                if (!anySet(needToUpdate))
                    return;

                // Mark station as having valid distance if seen for the first time
                if (allSet(cur == INFTY))
                    search.stationsWithValidDistances.push_back(stationId);

                cur.setIf(distToStation, needToUpdate);
            }

            DALSToStations &search;
        };


        struct StopStationBCH {
            explicit StopStationBCH(const DALSToStations &search) : search(search) {
            }

            template<typename DistLabelT, typename DistLabelContainerT>
            bool operator()(const int, DistLabelT &distToV, const DistLabelContainerT & /*distLabels*/) const {
                return allSet(search.canPrune(distToV, search.curMinCostToLastStop));
            }

        private:
            const DALSToStations &search;
        };

        friend ScanBucket;
        friend StopStationBCH;

    public:
        DALSToStations(const InputGraphT &inputGraph,
                       const Fleet &fleet,
                       const CHEnvT &chEnv,
                       CurVehLocToPickupSearchesT &curVehLocToPickupSearchesT,
                       const RouteState &routeState,
                       const StationBucketsEnvT &stationBucketsEnv,
                       const PTStations &ptStations,
                       const StationsAtLocations &stationsAtLocations)
            : inputGraph(inputGraph),
              ch(chEnv.getCH()),
              fleet(fleet),
              calculator(routeState),
              upwardSearch(chEnv.template getForwardSearch<ScanBucket, StopStationBCH, LabelSet>(
                  ScanBucket(*this), StopStationBCH(*this))),
              curVehLocToPickupSearches(curVehLocToPickupSearchesT),
              routeState(routeState),
              checkPBNSForVehicle(fleet.size()),
              idxOfVehicleInRelevant(fleet.size(), INVALID_INDEX),
              bucketContainer(stationBucketsEnv.getTargetBuckets()),
              ptStations(ptStations),
              stationsAtLocations(stationsAtLocations),
              stationsWithValidDistances(),
              currentLastStopDistances(ptStations.size(), DistanceLabel(INFTY)),
              untilLastStopUpperBounds() {
            stationsWithValidDistances.reserve(ptStations.size());
            untilLastStopUpperBounds.reserve(ptStations.size());
        }

        void tryDropoffAfterLastStop(const RequestState &requestState, const PDLocs &pdLocs,
                                     const RelevantPDLocs &relevantOrdinaryPickups,
                                     const RelevantPDLocs &relevantPickupsBeforeNextStop,
                                     stats::DalsAssignmentsPerformanceStats &stats,
                                     FirstTaxiLegResult &firstTaxiLegResult) {
            curReqState = &requestState;
            curRelOrdinaryPickups = &relevantOrdinaryPickups;
            curRelPickupsBns = &relevantPickupsBeforeNextStop;

            initLastStopSearches(requestState);

            const int64_t pbnsTimeBefore = curVehLocToPickupSearches.getTotalLocatingVehiclesTimeForRequest() +
                                           curVehLocToPickupSearches.getTotalVehicleToPickupSearchTimeForRequest();

            initRelevantVehicles(relevantOrdinaryPickups, relevantPickupsBeforeNextStop, requestState, pdLocs, stats);

            totalNumberOfCandidateDropoffs = 0;
            for (unsigned int i = 0; i < relevantVehicleIdsForRequest.size(); i += K) {
                KaRRiTimer timer;
                for (const auto &s: stationsWithValidDistances) {
                    currentLastStopDistances[s] = INFTY;
                }
                stationsWithValidDistances.clear();
                untilLastStopUpperBounds.clear();

                curMinCostToLastStop = DistanceLabel(INFTY);
                for (int j = 0; j < K; ++j) {
                    if (i + j >= relevantVehicleIdsForRequest.size())
                        break;
                    curMinCostToLastStop[j] = relevantVehiclesMinCostToLastStop[i];
                    KASSERT(curMinCostToLastStop[j] >= 0 && curMinCostToLastStop[j] < INFTY);
                }
                stats.searchTime += timer.elapsed<std::chrono::nanoseconds>();

                initializeDistancesForStationsAtLastStops(i, stats);
                runSearchesForVehicleBatch(i, stats);
                totalNumberOfCandidateDropoffs += stationsWithValidDistances.size() * K;
                enumerateAssignmentsForVehicleBatch(i, relevantOrdinaryPickups, relevantPickupsBeforeNextStop,
                                                    requestState, pdLocs, stats, firstTaxiLegResult);
            }

            // Time spent to locate vehicles and compute distances from current vehicle locations to pickups is counted
            // into PBNS time so subtract it here.
            const int64_t pbnsTime = curVehLocToPickupSearches.getTotalLocatingVehiclesTimeForRequest() +
                                     curVehLocToPickupSearches.getTotalVehicleToPickupSearchTimeForRequest() -
                                     pbnsTimeBefore;

            stats.tryAssignmentsTime -= pbnsTime;

            stats.numCandidateDropoffsAcrossAllVehicles += totalNumberOfCandidateDropoffs;
            stats.numEdgeRelaxationsInSearchGraph += totalNumEdgeRelaxations;
            stats.numVerticesOrLabelsSettled += totalNumVerticesSettled;
            stats.numEntriesOrLastStopsScanned += totalNumEntriesScanned;
            stats.numCandidateVehicles += relevantVehicleIdsForRequest.size();
        }

        void setExternalCostUpperBound(const int bestCost) {
            upperBoundCost = bestCost;
        }

    private:
        LabelMask canPrune(const DistanceLabel &distancesToDropoffs, const DistanceLabel &minCostToLastStop) const {
            using F = CostCalculator::CostFunction;

            DistanceLabel costLowerBound = minCostToLastStop +
                                           F::calcKVehicleCosts(distancesToDropoffs) +
                                           F::calcKTripCosts(distancesToDropoffs);
            return distancesToDropoffs >= INFTY | costLowerBound > upperBoundCost;
        }


        void initLastStopSearches(const RequestState &requestState) {
            totalNumEdgeRelaxations = 0;
            totalNumVerticesSettled = 0;
            totalNumEntriesScanned = 0;

            curReqState = &requestState;
        }

        void initSingleRelevantVehicle(const int vehId, const RelevantPDLocs &relPdLocs,
                                       const RequestState &rs, const PDLocs &pdLocs) {
            using namespace time_utils;
            using F = CostCalculator::CostFunction;

            // Find minimum residual detour at end of route for any relevant pickups and set min trip time to last stop accordingly
            const auto numStops = routeState.numStopsOf(vehId);
            const auto schedArrTimeAtLastStop = routeState.schedArrTimesFor(vehId)[numStops - 1];
            int bestCostToLastStop = INFTY;
            for (const auto &e: relPdLocs.relevantSpotsFor(vehId)) {
                const auto &p = pdLocs.pickups[e.pdId];
                const auto depTime = getActualDepTimeAtPickup(vehId, e.stopIndex, e.distToPDLoc, p, rs, routeState);
                const auto initialPickupDetour = calcInitialPickupDetour(
                    vehId, e.stopIndex, numStops - 1, depTime, e.distFromPDLocToNextStop, rs, routeState);
                const auto residualDetourAtEnd = calcResidualPickupDetour(
                    vehId, e.stopIndex, numStops - 1, initialPickupDetour, routeState);
                const auto minTripTimeToLastStop =
                        schedArrTimeAtLastStop + residualDetourAtEnd - rs.earliestDeparture();

                const auto costLowerBound = F::calcVehicleCost(residualDetourAtEnd) +
                                            F::calcTripCost(minTripTimeToLastStop) +
                                            F::calcChangeInTripCostsOfExistingPassengers(
                                                calcAddedTripTimeInInterval(
                                                    vehId, e.stopIndex, numStops - 1, initialPickupDetour, routeState));
                bestCostToLastStop = std::min(bestCostToLastStop, costLowerBound);
            }

            if (idxOfVehicleInRelevant[vehId] != INVALID_INDEX) {
                // If this vehicle is already in the relevant set, possibly update its best cost.
                auto &best = relevantVehiclesMinCostToLastStop[idxOfVehicleInRelevant[vehId]];
                KASSERT(best <= upperBoundCost);
                best = std::min(best, bestCostToLastStop);
            } else {
                // If this vehicle is not in the relevant set, add it if its best cost to last stop is within the upper bound.
                if (bestCostToLastStop <= upperBoundCost) {
                    idxOfVehicleInRelevant[vehId] = relevantVehicleIdsForRequest.size();
                    relevantVehicleIdsForRequest.push_back(vehId);
                    relevantVehiclesMinCostToLastStop.push_back(bestCostToLastStop);
                }
            }
        }

        void initRelevantVehicles(const RelevantPDLocs &relevantOrdinaryPickups,
                                  const RelevantPDLocs &relevantPickupsBeforeNextStop,
                                  const RequestState &requestState, const PDLocs &pdLocs,
                                  stats::DalsAssignmentsPerformanceStats &stats) {
            KaRRiTimer timer;
            for (const auto &vehId: relevantVehicleIdsForRequest)
                idxOfVehicleInRelevant[vehId] = INVALID_INDEX;
            relevantVehicleIdsForRequest.clear();
            relevantVehiclesMinCostToLastStop.clear();

            for (const auto &vehId: relevantOrdinaryPickups.getVehiclesWithRelevantPDLocs()) {
                initSingleRelevantVehicle(vehId, relevantOrdinaryPickups, requestState, pdLocs);
            }
            for (const auto &vehId: relevantPickupsBeforeNextStop.getVehiclesWithRelevantPDLocs()) {
                initSingleRelevantVehicle(vehId, relevantPickupsBeforeNextStop, requestState, pdLocs);
            }
            stats.initializationTime += timer.elapsed<std::chrono::nanoseconds>();
        }

        void initializeDistancesForStationsAtLastStops(const int indexOfFirstVeh,
                                                       stats::DalsAssignmentsPerformanceStats &stats) {
            KaRRiTimer timer;

            for (int i = 0; i < K; ++i) {
                const auto &veh = indexOfFirstVeh + i < relevantVehicleIdsForRequest.size()
                                      ? fleet[relevantVehicleIdsForRequest[indexOfFirstVeh + i]]
                                      : fleet[relevantVehicleIdsForRequest[indexOfFirstVeh]];
                const int lastStopIndex = routeState.numStopsOf(veh.vehicleId) - 1;
                const int lastStopLocation = routeState.stopLocationsFor(veh.vehicleId)[lastStopIndex];

                DistanceLabel zeroAtI = INFTY;
                zeroAtI[i] = 0;

                for (const int stationId: stationsAtLocations.getIdsOfStationsAtVehEdge(lastStopLocation)) {
                    const auto &station = ptStations[stationId];

                    // Mark station as having valid distance if seen for the first time
                    if (allSet(currentLastStopDistances[stationId] == INFTY))
                        stationsWithValidDistances.push_back(stationId);

                    currentLastStopDistances[stationId].min(zeroAtI);
                }
            }

            const int64_t searchTime = timer.elapsed<std::chrono::nanoseconds>();
            stats.searchTime += searchTime;
        }

        void runSearchesForVehicleBatch(const int indexOfFirstVeh, stats::DalsAssignmentsPerformanceStats &stats) {
            KaRRiTimer timer;

            KASSERT(indexOfFirstVeh % K == 0 && indexOfFirstVeh < relevantVehicleIdsForRequest.size());

            std::array<int, K> lastStopHeads;
            for (int i = 0; i < K; ++i) {
                const auto &veh =
                        indexOfFirstVeh + i < relevantVehicleIdsForRequest.size()
                            ? fleet[relevantVehicleIdsForRequest[indexOfFirstVeh + i]]
                            : fleet[relevantVehicleIdsForRequest[indexOfFirstVeh]];
                const int lastStopIndex = routeState.numStopsOf(veh.vehicleId) - 1;
                const int lastStopLocation = routeState.stopLocationsFor(veh.vehicleId)[lastStopIndex];
                lastStopHeads[i] = inputGraph.edgeHead(lastStopLocation);
            }

            run(lastStopHeads);
            const int64_t searchTime = timer.elapsed<std::chrono::nanoseconds>();

            stats.searchTime += searchTime;
            totalNumEdgeRelaxations += getNumEdgeRelaxations();
            totalNumVerticesSettled += getNumVerticesSettled();
            totalNumEntriesScanned += getNumEntriesScanned();
        }

        void run(const std::array<int, K> &sources,
                 const std::array<int, K> offsets = {}) {
            numVerticesSettled = 0;
            numEntriesVisited = 0;
            std::array<int, K> sources_ranks = {};
            std::transform(sources.begin(), sources.end(), sources_ranks.begin(),
                           [&](const int v) { return ch.rank(v); });
            upwardSearch.runWithOffset(sources_ranks, offsets);
        }

        int getNumEdgeRelaxations() const {
            return upwardSearch.getNumEdgeRelaxations();
        }

        int getNumVerticesSettled() const {
            return numVerticesSettled;
        }

        int getNumEntriesScanned() const {
            return numEntriesVisited;
        }

        void enumerateAssignmentsForVehicleBatch(const int indexOfFirstVeh,
                                                 const RelevantPDLocs &relevantOrdinaryPickups,
                                                 const RelevantPDLocs &relevantPickupsBeforeNextStop,
                                                 const RequestState &requestState,
                                                 const PDLocs &pdLocs, stats::DalsAssignmentsPerformanceStats &stats,
                                                 FirstTaxiLegResult &firstTaxiLegResult) {
            int numAssignmentsTried = 0;
            KaRRiTimer timer;

            // Assume that a station S which has been seen by the search has a known worst cost of an access RP trip of
            // c (i.e. the worst cost of any known pareto-optimal label). If no results are known for S, c = INFTY.
            // Also, let d = dist(last stop of route, S).
            // We compute an upper bound on the cost of the pickup part of the DALS assignment based on c:
            // If a DALS insertion is only relevant if its cost is <= c, then the pickup detour and rider trip to the
            // last stop of the route must have a cost of at most c - F::calcVehicleCosts(d) - F::calcTripCost(d)
            // for any relevant DALS assignment.
            for (const int stationId: stationsWithValidDistances) {
                // Find upper bound of known Pareto-optimal access RP trips to station
                int upperBoundCostWithoutTripToStation = -1;
                int upperBoundArrTimeAtStation = -1;
                for (const auto &knownResult: firstTaxiLegResult.getResultsForStation(stationId)) {
                    if (knownResult.costWithoutTrip > upperBoundCostWithoutTripToStation)
                        upperBoundCostWithoutTripToStation = knownResult.costWithoutTrip;
                    if (knownResult.arrivalTime > upperBoundArrTimeAtStation)
                        upperBoundArrTimeAtStation = knownResult.arrivalTime;
                }
                // If no access RP trips to this station are known, its upper bound is INFTY
                if (upperBoundCostWithoutTripToStation == -1) {
                    untilLastStopUpperBounds.emplace_back(INFTY, INFTY);
                    continue;
                }

                const int minDistToStation = currentLastStopDistances[stationId].horizontalMin();
                const int costNoTripAfterLastStop = CostCalculator::CostFunction::calcVehicleCost(minDistToStation);
                untilLastStopUpperBounds.emplace_back(
                    upperBoundCostWithoutTripToStation - costNoTripAfterLastStop,
                    upperBoundArrTimeAtStation - minDistToStation
                );
            }

            enumerateAssignmentsWithOrdinaryPickupForVehicleBatch(indexOfFirstVeh, numAssignmentsTried,
                                                                  relevantOrdinaryPickups, requestState, pdLocs,
                                                                  firstTaxiLegResult);
            enumerateAssignmentsWithPBNSForVehicleBatch(indexOfFirstVeh, numAssignmentsTried,
                                                        relevantPickupsBeforeNextStop,
                                                        requestState, pdLocs, firstTaxiLegResult);

            const int64_t tryAssignmentsTime = timer.elapsed<std::chrono::nanoseconds>();
            stats.tryAssignmentsTime += tryAssignmentsTime;
            stats.numAssignmentsTried += numAssignmentsTried;
        }

        // Enumerate assignments where pickup is after next stop (ordinary pickup):
        void enumerateAssignmentsWithOrdinaryPickupForVehicleBatch(const int indexOfFirstVeh,
                                                                   int &numAssignmentsTried,
                                                                   const RelevantPDLocs &relevantOrdinaryPickups,
                                                                   const RequestState &requestState,
                                                                   const PDLocs &pdLocs,
                                                                   FirstTaxiLegResult &firstTaxiLegResult) {
            const int reqTime = requestState.earliestDeparture();
            Assignment asgn;

            checkPBNSForVehicle.reset();
            for (int i = 0; i < K; ++i) {
                const auto &veh = indexOfFirstVeh + i < relevantVehicleIdsForRequest.size()
                                      ? fleet[relevantVehicleIdsForRequest[indexOfFirstVeh + i]]
                                      : fleet[relevantVehicleIdsForRequest[indexOfFirstVeh]];
                const int vehId = veh.vehicleId;

                if (!relevantOrdinaryPickups.hasRelevantSpotsFor(vehId)) {
                    // vehicle may still have relevant assignment with pickup before next stop
                    checkPBNSForVehicle.set(vehId);
                    continue;
                }

                const auto &numStops = routeState.numStopsOf(vehId);
                const auto &occupancies = routeState.occupanciesFor(vehId);
                const auto lastStopLoc = routeState.stopLocationsFor(vehId)[numStops - 1];
                const auto relevantPickupsInRevOrder = relevantOrdinaryPickups.relevantSpotsForInReverseOrder(vehId);
                asgn.vehicle = &fleet[vehId];
                asgn.dropoffStopIdx = numStops - 1;

                KASSERT(asgn.distToDropoff >= 0 && asgn.distToDropoff < INFTY);
                // int curPickupIndex = numStops - 1;
                auto pickupIt = relevantPickupsInRevOrder.begin();
                for (; pickupIt < relevantPickupsInRevOrder.end(); ++pickupIt) {
                    const auto &entry = *pickupIt;

                    // If capacity is broken in leg of pickup, no earlier pickups are possible either, break
                    if (occupancies[entry.stopIndex] + requestState.originalRequest.numRiders > asgn.vehicle->capacity)
                        break;

                    asgn.pickup = pdLocs.pickups[entry.pdId];
                    asgn.pickupStopIdx = entry.stopIndex;
                    asgn.distToPickup = entry.distToPDLoc;
                    asgn.distFromPickup = entry.distFromPDLocToNextStop;

                    using namespace time_utils;
                    const int depTimeAtPickup = getActualDepTimeAtPickup(
                        vehId, asgn.pickupStopIdx, asgn.distToPickup, asgn.pickup, requestState, routeState);
                    const int initialPickupDetour = calcInitialPickupDetour(
                        vehId, asgn.pickupStopIdx, numStops - 1, depTimeAtPickup, asgn.distFromPickup, requestState,
                        routeState);
                    const auto residualDetourAtLastStop = calcResidualPickupDetour(
                        veh.vehicleId, asgn.pickupStopIdx, numStops - 1, initialPickupDetour, routeState);
                    const int costNoTripUntilDepAtLastStop = calculator.calcCostWithoutTripUntilLastStopForDALS(
                        veh, asgn.pickup, asgn.pickupStopIdx, depTimeAtPickup, initialPickupDetour,
                        residualDetourAtLastStop,
                        requestState);
                    const int arrTimeAtLastStop =
                            routeState.schedArrTimesFor(vehId)[numStops - 1] + residualDetourAtLastStop;
                    const int depTimeAtLastStop =
                            routeState.schedDepTimesFor(vehId)[numStops - 1] + residualDetourAtLastStop;

                    for (int idxInValid = 0; idxInValid < stationsWithValidDistances.size(); ++idxInValid) {
                        if (untilLastStopUpperBounds[idxInValid].dominates(
                            costNoTripUntilDepAtLastStop, arrTimeAtLastStop))
                            continue;

                        const int stationId = stationsWithValidDistances[idxInValid];
                        const auto &station = ptStations[stationId];
                        KASSERT(station.stationId == stationId);
                        asgn.dropoff = {
                            station.stationId, // PDLoc ID
                            station.vehEdgeId, // Location in road network
                            station.psgEdgeId, // Location in passenger road network
                            0, // Walking time from this dropoff to destination
                            0, // Vehicle driving time from this dropoff to the destination
                            0, // Vehicle driving time from destination to this dropoff,
                            true
                        };

                        asgn.distToDropoff = currentLastStopDistances[station.stationId][i];
                        KASSERT(asgn.distToDropoff >= 0 && asgn.distToDropoff < INFTY);

                        if (asgn.pickup.loc == asgn.dropoff.loc)
                            continue;

                        const bool stationAtExistingStop = asgn.dropoff.loc == lastStopLoc;

                        // Check hard constraints
                        if (isAnyHardConstraintViolated(asgn, requestState, initialPickupDetour, 0,
                                                        residualDetourAtLastStop, stationAtExistingStop,
                                                        routeState))
                            continue;

                        using F = CostCalculator::CostFunction;
                        const int arrivalTime =
                                (stationAtExistingStop ? arrTimeAtLastStop : depTimeAtLastStop) + asgn.distToDropoff;
                        KASSERT(arrivalTime == calcArrivalTime(asgn, requestState, routeState));
                        static const int &stopTime = InputConfig::getInstance().stopTime;
                        const int vehDist = asgn.distToDropoff + (stationAtExistingStop ? 0 : stopTime);
                        const int cost = costNoTripUntilDepAtLastStop +
                                         F::calcVehicleCost(vehDist) +
                                         F::calcTripCost(arrivalTime - reqTime);
                        KASSERT(cost == calculator.calc(asgn, requestState),
                                "RequestState=" << requestState << ", Assignment=" << asgn << ", Route=" << routeState.
                                printRouteOf(vehId));
                        firstTaxiLegResult.tryAssignmentForStation(
                            station.stationId, asgn, cost, arrivalTime, InsertionType::DALS);
                        ++numAssignmentsTried;
                    }
                }

                if (pickupIt == relevantPickupsInRevOrder.end()) {
                    // If the reverse scan of the vehicle route did not break early at a later stop, then we also
                    // need to consider the pickup before next stop case.
                    checkPBNSForVehicle.set(vehId);
                }
            }
        }

        // Enumerate assignments where the pickup is before the next stop (PBNS + DALS):
        void enumerateAssignmentsWithPBNSForVehicleBatch(const int indexOfFirstVeh,
                                                         int &numAssignmentsTried,
                                                         const RelevantPDLocs &relevantPickupsBeforeNextStop,
                                                         const RequestState &requestState,
                                                         const PDLocs &pdLocs,
                                                         FirstTaxiLegResult &firstTaxiLegResult) {
            const int reqTime = requestState.earliestDeparture();
            Assignment asgn;
            asgn.pickupStopIdx = 0;

            for (int i = 0; i < K; ++i) {
                const auto &veh = indexOfFirstVeh + i < relevantVehicleIdsForRequest.size()
                                      ? fleet[relevantVehicleIdsForRequest[indexOfFirstVeh + i]]
                                      : fleet[relevantVehicleIdsForRequest[indexOfFirstVeh]];
                const int vehId = veh.vehicleId;

                if (!relevantPickupsBeforeNextStop.hasRelevantSpotsFor(vehId))
                    continue;

                if (!checkPBNSForVehicle.isSet(vehId))
                    continue;

                if (routeState.numStopsOf(vehId) == 0 ||
                    routeState.occupanciesFor(vehId)[0] + requestState.originalRequest.numRiders > fleet[vehId].
                    capacity)
                    continue;

                pbnsContinuations.clear();

                const auto numStops = routeState.numStopsOf(vehId);
                const int lastStopLoc = routeState.stopLocationsFor(vehId)[numStops - 1];
                asgn.vehicle = &fleet[vehId];
                asgn.dropoffStopIdx = numStops - 1;


                for (auto &entry: relevantPickupsBeforeNextStop.relevantSpotsFor(vehId)) {
                    asgn.pickup = pdLocs.pickups[entry.pdId];
                    asgn.distFromPickup = entry.distFromPDLocToNextStop;

                    using namespace time_utils;
                    const int depTimeAtPickup = getActualDepTimeAtPickup(
                        vehId, 0, entry.distToPDLoc, asgn.pickup, requestState, routeState);
                    const int initialPickupDetour = calcInitialPickupDetour(
                        vehId, 0, numStops - 1, depTimeAtPickup, asgn.distFromPickup, requestState, routeState);
                    const auto residualDetourAtLastStop = calcResidualPickupDetour(
                        veh.vehicleId, 0, numStops - 1, initialPickupDetour, routeState);
                    const int lowerBoundCostNoTripUntilDepAtLastStop = calculator.
                            calcCostWithoutTripUntilLastStopForDALS(
                                veh, asgn.pickup, 0, depTimeAtPickup, initialPickupDetour,
                                residualDetourAtLastStop, requestState);
                    const int lowerBoundArrTimeAtLastStop =
                            routeState.schedArrTimesFor(vehId)[numStops - 1] + residualDetourAtLastStop;
                    const int lowerBoundDepTimeAtLastStop =
                            routeState.schedDepTimesFor(vehId)[numStops - 1] + residualDetourAtLastStop;

                    for (int idxInValid = 0; idxInValid < stationsWithValidDistances.size(); ++idxInValid) {
                        if (untilLastStopUpperBounds[idxInValid].dominates(
                            lowerBoundCostNoTripUntilDepAtLastStop, lowerBoundArrTimeAtLastStop))
                            continue;

                        const auto &station = ptStations[stationsWithValidDistances[idxInValid]];
                        KASSERT(station.stationId == stationsWithValidDistances[idxInValid]);
                        asgn.dropoff = {
                            station.stationId, // PDLoc ID
                            station.vehEdgeId, // Location in road network
                            station.psgEdgeId, // Location in passenger road network
                            0, // Walking time from this dropoff to destination
                            0, // Vehicle driving time from this dropoff to the destination
                            0, // Vehicle driving time from destination to this dropoff
                            true
                        };

                        if (asgn.pickup.loc == asgn.dropoff.loc)
                            continue;

                        asgn.distToDropoff = currentLastStopDistances[station.stationId][i];
                        if (asgn.distToDropoff >= INFTY)
                            continue;

                        const bool stationAtExistingStop = asgn.dropoff.loc == lastStopLoc;

                        if (curVehLocToPickupSearches.knowsDistance(vehId, asgn.pickup.id)) {
                            asgn.distToPickup = curVehLocToPickupSearches.getDistance(vehId, asgn.pickup.id);
                            // requestState.tryAssignmentWithKnownCost(asgn, calculator.calc(asgn, requestState));
                            firstTaxiLegResult.tryAssignmentForStation(
                                station.stationId, asgn, calculator.calc(asgn, requestState),
                                time_utils::calcArrivalTime(asgn, requestState, routeState), InsertionType::DALS_PBNS);
                            ++numAssignmentsTried;
                        } else {
                            asgn.distToPickup = entry.distToPDLoc;
                            using F = CostCalculator::CostFunction;
                            const auto lowerBoundArrTime = (stationAtExistingStop
                                                                ? lowerBoundArrTimeAtLastStop
                                                                : lowerBoundDepTimeAtLastStop) + asgn.distToDropoff;;
                            KASSERT(lowerBoundArrTime == calcArrivalTime(asgn, requestState, routeState));
                            static const int &stopTime = InputConfig::getInstance().stopTime;
                            const int vehDist = asgn.distToDropoff + (stationAtExistingStop ? 0 : stopTime);
                            const auto lowerBoundCost = lowerBoundCostNoTripUntilDepAtLastStop +
                                                        F::calcVehicleCost(
                                                            asgn.distToDropoff + InputConfig::getInstance().stopTime) +
                                                        F::calcTripCost(lowerBoundArrTime - reqTime);
                            if (lowerBoundCost < upperBoundCost) {
                                // In this case, we need the exact distance to the pickup via the current location of the
                                // vehicle. We postpone computation of that distance to be able to bundle it with the
                                // computation of distances to other pickups via the vehicle location. Then all remaining
                                // assignments with this pickup can be tried with the exact distance later.
                                curVehLocToPickupSearches.addPickupForProcessing(asgn.pickup.id, asgn.distToPickup);
                                pbnsContinuations.push_back({asgn.pickup.id, asgn.distFromPickup, idxInValid});
                                break;
                            }
                        }
                    }
                }

                // Continue with assignments for pickups where exact distance via vehicle location is needed
                curVehLocToPickupSearches.computeExactDistancesVia(fleet[vehId], pdLocs);
                for (const auto &continuation: pbnsContinuations) {
                    KASSERT(continuation.pickupID >= 0 && continuation.pickupID < pdLocs.numPickups());
                    KASSERT(
                        continuation.fromIdxInValidStations >= 0 && continuation.fromIdxInValidStations <
                        stationsWithValidDistances.size());
                    asgn.pickup = pdLocs.pickups[continuation.pickupID];

                    asgn.distToPickup = curVehLocToPickupSearches.getDistance(vehId, continuation.pickupID);
                    asgn.distFromPickup = continuation.distFromPickup;
                    if (asgn.distToPickup >= INFTY)
                        continue;

                    using namespace time_utils;
                    const int depTimeAtPickup = getActualDepTimeAtPickup(
                        vehId, 0, asgn.distToPickup, asgn.pickup, requestState, routeState);
                    const int initialPickupDetour = calcInitialPickupDetour(
                        vehId, 0, numStops - 1, depTimeAtPickup, asgn.distFromPickup, requestState, routeState);
                    const auto residualDetourAtLastStop = calcResidualPickupDetour(
                        veh.vehicleId, 0, numStops - 1, initialPickupDetour, routeState);
                    const int costNoTripUntilDepAtLastStop = calculator.calcCostWithoutTripUntilLastStopForDALS(
                        veh, asgn.pickup, 0, depTimeAtPickup, initialPickupDetour, residualDetourAtLastStop,
                        requestState);
                    const int arrTimeAtLastStop =
                            routeState.schedArrTimesFor(vehId)[numStops - 1] + residualDetourAtLastStop;
                    const int depTimeAtLastStop =
                            routeState.schedDepTimesFor(vehId)[numStops - 1] + residualDetourAtLastStop;

                    for (int idxInValid = continuation.fromIdxInValidStations;
                         idxInValid < stationsWithValidDistances.size(); ++idxInValid) {
                        if (untilLastStopUpperBounds[idxInValid].dominates(
                            costNoTripUntilDepAtLastStop, arrTimeAtLastStop))
                            continue;

                        const auto &station = ptStations[stationsWithValidDistances[idxInValid]];

                        asgn.dropoff = {
                            station.stationId, // PDLoc ID
                            station.vehEdgeId, // Location in road network
                            station.psgEdgeId, // Location in passenger road network
                            0, // Walking time from this dropoff to destination
                            0, // Vehicle driving time from this dropoff to the destination
                            0, // Vehicle driving time from destination to this dropoff
                            true
                        };

                        if (asgn.pickup.loc == asgn.dropoff.loc)
                            continue;

                        asgn.distToDropoff = currentLastStopDistances[station.stationId][i];
                        if (asgn.distToDropoff >= INFTY)
                            continue;

                        ++numAssignmentsTried;
                        asgn.dropoffStopIdx = numStops - 1;

                        const bool stationAtExistingStop = asgn.dropoff.loc == lastStopLoc;

                        // Check hard constraints
                        if (isAnyHardConstraintViolated(asgn, requestState, initialPickupDetour, 0,
                                                        residualDetourAtLastStop, stationAtExistingStop,
                                                        routeState))
                            continue;

                        // requestState.tryAssignmentWithKnownCost(asgn, calculator.calc(asgn, requestState));
                        using F = CostCalculator::CostFunction;
                        const int arrivalTime =
                                (stationAtExistingStop ? arrTimeAtLastStop : depTimeAtLastStop) + asgn.distToDropoff;
                        KASSERT(arrivalTime == calcArrivalTime(asgn, requestState, routeState));
                        static const int &stopTime = InputConfig::getInstance().stopTime;
                        const int vehDist = asgn.distToDropoff + (stationAtExistingStop ? 0 : stopTime);
                        const int cost = costNoTripUntilDepAtLastStop +
                                         F::calcVehicleCost(vehDist) +
                                         F::calcTripCost(arrivalTime - reqTime);
                        KASSERT(cost == calculator.calc(asgn, requestState),
                                "RequestState=" << requestState << ", Assignment=" << asgn << ", Route=" << routeState.
                                printRouteOf(vehId));
                        firstTaxiLegResult.tryAssignmentForStation(
                            station.stationId, asgn, cost, arrivalTime, InsertionType::DALS_PBNS);
                    }
                }
            }
        }


        typename CHEnvT::template UpwardSearch<ScanBucket, StopStationBCH, LabelSet> upwardSearch;
        const typename StationBucketsEnvT::BucketContainer &bucketContainer;

        const InputGraphT &inputGraph;
        const CH &ch;
        const Fleet &fleet;
        CostCalculator calculator;
        CurVehLocToPickupSearchesT &curVehLocToPickupSearches;
        const RouteState &routeState;

        const PTStations &ptStations;
        const StationsAtLocations &stationsAtLocations;

        // Flag per vehicle that tells us if we still have to consider a pickup before the next stop of the vehicle.
        FastResetFlagArray<> checkPBNSForVehicle;

        // Records for postponing and bundling computations of distances from current locations of vehicles to
        // pickups needed in the PBNS+DALS case.
        struct PickupBeforeNextStopContinuation {
            int pickupID;
            int distFromPickup;
            int fromIdxInValidStations;
        };

        std::vector<PickupBeforeNextStopContinuation> pbnsContinuations;

        int upperBoundCost;

        std::vector<int> idxOfVehicleInRelevant;
        // Maps every vehicle to its index in relevantVehicleIdsForRequest and in relevantVehiclesMinTripTimesToLastStop
        std::vector<int> relevantVehicleIdsForRequest;
        // std::vector<int> relevantVehiclesMinTripTimesToLastStop;
        // std::vector<int> relevantVehiclesMinResidualPickupDetours;
        std::vector<int> relevantVehiclesMinCostToLastStop;

        std::vector<int> stationsWithValidDistances;
        std::vector<DistanceLabel> currentLastStopDistances;

        // For the station at stationsWithValidDistances[i], untilLastStopUpperBounds[i] contains an upper bound on the
        // cost and departure time until the last stop to allow for a better assignment than the best known to this station.
        struct UpperBoundLabel {
            int cost = INFTY;
            int departureTime = INFTY;

            bool dominates(int c, int d) const {
                if (cost > c || departureTime > d)
                    return false;
                return cost < c || departureTime < d;
            }
        };

        std::vector<UpperBoundLabel> untilLastStopUpperBounds;

        // Pointers to request state and relevant PD locs so Dijkstra search callback has access to them
        RequestState const *curReqState;
        RelevantPDLocs const *curRelOrdinaryPickups;
        RelevantPDLocs const *curRelPickupsBns;

        // // Minimum trip time for any DALS insertion on current vehicles needed to get to last stop.
        // // Can be used for cost lower bounds when pruning BCH query.
        // DistanceLabel curMinTripTimesToLastStop;
        //
        // // Minimum residual detour at last stop for any DALS insertion on current vehicles.
        // // Can be used for cost lower bounds when pruning BCH query.
        // DistanceLabel curMinResidualPickupDetours;

        // Minimum cost for part of route until last stop for any DALS insertion on current vehicles.
        // Can be used for cost lower bounds when pruning BCH query.
        DistanceLabel curMinCostToLastStop;

        int numVerticesSettled;
        int numEntriesVisited;

        int totalNumEdgeRelaxations;
        int totalNumVerticesSettled;
        int totalNumEntriesScanned;

        int totalNumberOfCandidateDropoffs;
    };
}
