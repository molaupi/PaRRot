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

namespace karri {

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

            explicit ScanBucket(DALSToStations &search) : search(search){}

            template<typename DistLabelT, typename DistLabelContainerT>
            bool operator()(const int v, DistLabelT &distToV, const DistLabelContainerT & /*distLabels*/) {
                // Check if we can prune at this vertex based only on the distance to v
                if (allSet(search.canPrune(distToV)))
                    return true;
            
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
                            const auto atLeastAsGoodAsCurBest = ~search.canPrune(distViaV);

                            if (!anySet(atLeastAsGoodAsCurBest))
                                break;

                            tryUpdatingDistance(stationId, distViaV);
                        }
                }
                
                search.numEntriesVisited += numEntriesScannedHere;
                ++search.numVerticesSettled;

                return false;
            }

        private:

            void tryUpdatingDistance(const int stationId, const DistanceLabel& distFromPDLoc) {
                // Update tentative distances to v for any searches where distViaV admits a possible better assignment
                // than the current best and where distViaV is at least as good as the current tentative distance.
                LabelMask curMask = ~(search.currentLastStopDistances[stationId] < distFromPDLoc);
                curMask &= distFromPDLoc < INFTY;
                if (!anySet(curMask))
                    return;
                    
                search.currentLastStopDistances[stationId].setIf(distFromPDLoc, curMask);
            }

            DALSToStations &search;
        };


        struct StopStationBCH {
            explicit StopStationBCH(const DALSToStations &search) : search(search) {}

            template<typename DistLabelT, typename DistLabelContainerT>
            bool operator()(const int, DistLabelT &distToV, const DistLabelContainerT & /*distLabels*/) const {
                if constexpr (!StationBucketsEnvT::SORTED) {
                    return false;
                }
                return allSet(search.canPrune(distToV));
            }

        private:
            const DALSToStations &search;

        };


    public:

        DALSToStations(const InputGraphT &inputGraph,
                              const Fleet &fleet,
                              const CHEnvT &chEnv,
                              CurVehLocToPickupSearchesT &curVehLocToPickupSearchesT,
                              const RouteState &routeState,
                              const StationBucketsEnvT &stationBucketsEnv,
                              PTStations &stations)
                : inputGraph(inputGraph),
                  ch(chEnv.getCH()),
                  fleet(fleet),
                  calculator(routeState),
                  upwardSearch(chEnv.template getForwardSearch<ScanBucket, StopStationBCH, LabelSet>(
                        ScanBucket(*this), StopStationBCH(*this))),
                  curVehLocToPickupSearches(curVehLocToPickupSearchesT),
                  routeState(routeState),
                  checkPBNSForVehicle(fleet.size()),
                  bucketContainer(stationBucketsEnv.getBuckets()),
                  stations(stations),
                  currentLastStopDistances(stations.size(), DistanceLabel(INFTY)) {}

        void tryDropoffAfterLastStop(const RelevantPDLocs &relevantOrdinaryPickups,
                                     const RelevantPDLocs &relevantPickupsBeforeNextStop,
                                     RequestState& requestState,
                                     const PDLocs& pdLocs, stats::DalsAssignmentsPerformanceStats& stats,
                                     FirstTaxiLegResult &firstTaxiLegResult) {
            curReqState = &requestState;
            curRelOrdinaryPickups = &relevantOrdinaryPickups;
            curRelPickupsBns = &relevantPickupsBeforeNextStop;

            initLastStopSearches(requestState, firstTaxiLegResult);

            const int64_t pbnsTimeBefore = curVehLocToPickupSearches.getTotalLocatingVehiclesTimeForRequest() +
                                           curVehLocToPickupSearches.getTotalVehicleToPickupSearchTimeForRequest();

            for (unsigned int i = 0; i < fleet.size(); i += K) {
                runSearchesForVehicleBatch(i, stats);
                enumerateAssignmentsForVehicleBatch(i, relevantOrdinaryPickups, relevantPickupsBeforeNextStop, requestState, pdLocs, stats, firstTaxiLegResult);
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
            stats.numCandidateVehicles += curRelOrdinaryPickups->getVehiclesWithRelevantPDLocs().size() +
                                         curRelPickupsBns->getVehiclesWithRelevantPDLocs().size();
        }

        // Sets a known upper bound on the cost of a PALS insertion.
        void setExternalCostUpperBound(const int c) {
            externalUpperBoundCost = c;
        }

        LabelMask canPrune(const DistanceLabel &distancesToDropoffs) const {
            if (upperBoundCost >= INFTY) {
                // If current best is INFTY, only indices i with distancesToPickups[i] >= INFTY or
                // minDirectDistances[i] >= INFTY are worse than the current best.
                return ~(distancesToDropoffs < INFTY);
            }

            const DistanceLabel costLowerBound = calculator.template calcKVehicleIndependentCostLowerBoundsForDALSWithKnownMinDistToDropoff<LabelSet>(
                    0, distancesToDropoffs, 0, *curReqState);

            return upperBoundCost < costLowerBound;
        }

    private:

        void initLastStopSearches(const RequestState& requestState, FirstTaxiLegResult &firstTaxiLegResult) {
            totalNumberOfCandidateDropoffs = 0;
            totalNumEdgeRelaxations = 0;
            totalNumVerticesSettled = 0;
            totalNumEntriesScanned = 0;
            
            curReqState = &requestState;
            upperBoundCost = std::min(firstTaxiLegResult.getWorstCostForAllStations(), externalUpperBoundCost);
            externalUpperBoundCost = INFTY;
            std::fill(currentLastStopDistances.begin(), currentLastStopDistances.end(), DistanceLabel(INFTY));
        }

        void runSearchesForVehicleBatch(const int firstVehId, stats::DalsAssignmentsPerformanceStats& stats) {
            KaRRiTimer timer;

            assert(firstVehId % K == 0 && firstVehId < fleet.size());

            std::array<int, K> lastStopTails;
            std::array<int, K> travelTimes;
            for (int i = 0; i < K; ++i) {
                const auto &veh =
                        firstVehId + i < fleet.size() ? fleet[firstVehId + i]
                                                                      : fleet[firstVehId];
                const int lastStopIndex = routeState.numStopsOf(veh.vehicleId) - 1;
                const int lastStopLocation = routeState.stopLocationsFor(veh.vehicleId)[lastStopIndex];
                lastStopTails[i] = inputGraph.edgeTail(lastStopLocation);
                travelTimes[i] = inputGraph.travelTime(lastStopLocation);
            }
            
            run(lastStopTails, travelTimes);
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

        void enumerateAssignmentsForVehicleBatch(const int firstVehId,
                                                 const RelevantPDLocs &relevantOrdinaryPickups,
                                                 const RelevantPDLocs &relevantPickupsBeforeNextStop,
                                                 RequestState& requestState,
                                                 const PDLocs& pdLocs, stats::DalsAssignmentsPerformanceStats& stats,
                                                 FirstTaxiLegResult &firstTaxiLegResult) {

            int numAssignmentsTried = 0;
            KaRRiTimer timer;

            enumerateAssignmentsWithOrdinaryPickupForVehicleBatch(firstVehId, numAssignmentsTried, relevantOrdinaryPickups, requestState, pdLocs, firstTaxiLegResult);
            enumerateAssignmentsWithPBNSForVehicleBatch(firstVehId, numAssignmentsTried, relevantPickupsBeforeNextStop, requestState, pdLocs, firstTaxiLegResult);

            const int64_t tryAssignmentsTime = timer.elapsed<std::chrono::nanoseconds>();
            stats.tryAssignmentsTime += tryAssignmentsTime;
            stats.numAssignmentsTried += numAssignmentsTried;            
        }
        
        // Enumerate assignments where pickup is after next stop (ordinary pickup):
        void enumerateAssignmentsWithOrdinaryPickupForVehicleBatch(const int firstVehId,
                                                                   int &numAssignmentsTried,
                                                                   const RelevantPDLocs &relevantOrdinaryPickups,
                                                                   RequestState& requestState,
                                                                   const PDLocs& pdLocs,
                                                                   FirstTaxiLegResult &firstTaxiLegResult) {
            
            Assignment asgn;

            checkPBNSForVehicle.reset();
            for (int i = 0; i < K; ++i) {
                const auto &veh =
                        firstVehId + i < fleet.size() ? fleet[firstVehId + i]
                                                                      : fleet[firstVehId];
                const int vehId = veh.vehicleId;

                if (!relevantOrdinaryPickups.hasRelevantSpotsFor(vehId)) {
                    // vehicle may still have relevant assignment with pickup before next stop
                    checkPBNSForVehicle.set(vehId);
                    continue;
                }

                const auto &numStops = routeState.numStopsOf(vehId);
                const auto &occupancies = routeState.occupanciesFor(vehId);
                const auto relevantPickupsInRevOrder = relevantOrdinaryPickups.relevantSpotsForInReverseOrder(vehId);
                asgn.vehicle = &fleet[vehId];
                asgn.dropoffStopIdx = numStops - 1;

                for (const auto &station: stations) {
                    asgn.dropoff = {
                        station.stationId, // PDLoc ID
                        station.vehEdgeId, // Location in road network
                        station.psgEdgeId, // Location in passenger road network
                        0, // Walking time from this dropoff to destination
                        0, // Vehicle driving time from this dropoff to the destination
                        0 // Vehicle driving time from destination to this dropoff
                    };

                    asgn.distToDropoff = currentLastStopDistances[station.stationId][i];
                    if (asgn.distToDropoff >= INFTY)
                        continue; // no need to check pickup before next stop

                    assert(asgn.distToDropoff >= 0 && asgn.distToDropoff < INFTY);
                    int curPickupIndex = numStops - 1;
                    auto pickupIt = relevantPickupsInRevOrder.begin();
                    for (; pickupIt < relevantPickupsInRevOrder.end(); ++pickupIt) {
                        const auto &entry = *pickupIt;

                        if (entry.stopIndex < curPickupIndex) {
                            // New smaller pickup index reached: Check if seating capacity and cost lower bound admit
                            // any valid assignments at this or earlier indices.
                            if (occupancies[entry.stopIndex] + requestState.originalRequest.numRiders > asgn.vehicle->capacity)
                                break;

                            assert(entry.stopIndex < numStops - 1);
                            const auto minTripTimeToLastStop = routeState.schedDepTimesFor(vehId)[numStops - 1] -
                                                               routeState.schedArrTimesFor(vehId)[entry.stopIndex + 1];

                            const auto minCostFromHere = calculator.calcVehicleIndependentCostLowerBoundForDALSWithKnownMinDistToDropoff(
                                    asgn.dropoff.walkingDist, asgn.distToDropoff, minTripTimeToLastStop, requestState);
                            if (minCostFromHere > upperBoundCost)
                                break;

                            curPickupIndex = entry.stopIndex;
                        }

                        asgn.pickup = pdLocs.pickups[entry.pdId];
                        if (asgn.pickup.loc == asgn.dropoff.loc)
                            continue;
                        ++numAssignmentsTried;
                        asgn.pickupStopIdx = entry.stopIndex;
                        asgn.distToPickup = entry.distToPDLoc;
                        asgn.distFromPickup = entry.distFromPDLocToNextStop;
                        // requestState.tryAssignmentWithKnownCost(asgn, calculator.calc(asgn, requestState));
                        firstTaxiLegResult.tryAssignmentWithKnownCostForStation(station.stationId, asgn, calculator.calc(asgn, requestState));
                    }
                    
                    if (pickupIt == relevantPickupsInRevOrder.end()) {
                        // If the reverse scan of the vehicle route did not break early at a later stop, then we also
                        // need to consider the pickup before next stop case.
                        checkPBNSForVehicle.set(vehId);
                    } else {
                        ++totalNumberOfCandidateDropoffs;
                    }
                }
            }
        }

        // Enumerate assignments where the pickup is before the next stop (PBNS + DALS):
        void enumerateAssignmentsWithPBNSForVehicleBatch(const int firstVehId,
                                                         int &numAssignmentsTried,
                                                         const RelevantPDLocs &relevantPickupsBeforeNextStop,
                                                         RequestState& requestState,
                                                         const PDLocs& pdLocs,
                                                         FirstTaxiLegResult &firstTaxiLegResult) {
            
            Assignment asgn;
            asgn.pickupStopIdx = 0;

            for (int i = 0; i < K; ++i) {
                const auto &veh =
                        firstVehId + i < fleet.size() ? fleet[firstVehId + i]
                                                                      : fleet[firstVehId];
                const int vehId = veh.vehicleId;

                if (!relevantPickupsBeforeNextStop.hasRelevantSpotsFor(vehId))
                    continue;

                if (!checkPBNSForVehicle.isSet(vehId))
                    continue;

                if (routeState.numStopsOf(vehId) == 0 ||
                    routeState.occupanciesFor(vehId)[0] + requestState.originalRequest.numRiders > fleet[vehId].capacity)
                    continue;

                pbnsContinuations.clear();

                const auto numStops = routeState.numStopsOf(vehId);
                asgn.vehicle = &fleet[vehId];
                asgn.dropoffStopIdx = numStops - 1;


                for (auto &entry: relevantPickupsBeforeNextStop.relevantSpotsFor(vehId)) {
                    asgn.pickup = pdLocs.pickups[entry.pdId];
                    asgn.distFromPickup = entry.distFromPDLocToNextStop;
                    for (const auto &station: stations) {
                        asgn.dropoff = {
                            station.stationId, // PDLoc ID
                            station.vehEdgeId, // Location in road network
                            station.psgEdgeId, // Location in passenger road network
                            0, // Walking time from this dropoff to destination
                            0, // Vehicle driving time from this dropoff to the destination
                            0 // Vehicle driving time from destination to this dropoff
                        };

                        if (asgn.pickup.loc == asgn.dropoff.loc)
                            continue;

                        asgn.distToDropoff = currentLastStopDistances[station.stationId][i];
                        if (asgn.distToDropoff >= INFTY)
                            continue;
                        ++totalNumberOfCandidateDropoffs;

                        if (curVehLocToPickupSearches.knowsDistance(vehId, asgn.pickup.id)) {
                            asgn.distToPickup = curVehLocToPickupSearches.getDistance(vehId, asgn.pickup.id);
                            // requestState.tryAssignmentWithKnownCost(asgn, calculator.calc(asgn, requestState));
                            firstTaxiLegResult.tryAssignmentWithKnownCostForStation(station.stationId, asgn, calculator.calc(asgn, requestState));
                            ++numAssignmentsTried;
                        } else {
                            asgn.distToPickup = entry.distToPDLoc;
                            const auto lowerBoundCost = calculator.calc(asgn, requestState);
                            if (lowerBoundCost < upperBoundCost ||
                                (lowerBoundCost == firstTaxiLegResult.getWorstCostForAllStations() &&
                                 breakCostTie(asgn, firstTaxiLegResult.getWorstAssignmentForAllStations()))) {
                                // In this case, we need the exact distance to the pickup via the current location of the
                                // vehicle. We postpone computation of that distance to be able to bundle it with the
                                // computation of distances to other pickups via the vehicle location. Then all remaining
                                // assignments with this pickup can be tried with the exact distance later.
                                curVehLocToPickupSearches.addPickupForProcessing(asgn.pickup.id, asgn.distToPickup);
                                pbnsContinuations.push_back({asgn.pickup.id, asgn.distFromPickup, asgn.dropoff.id});
                                break;
                            }
                        }
                    }
                }

                // Continue with assignments for pickups where exact distance via vehicle location is needed
                curVehLocToPickupSearches.computeExactDistancesVia(fleet[vehId], pdLocs);
                for (const auto &continuation: pbnsContinuations) {
                    assert(continuation.pickupID >= 0 && continuation.pickupID < pdLocs.numPickups());
                    assert(continuation.fromStationID >= 0 && continuation.fromStationID < stations.size());
                    asgn.pickup = pdLocs.pickups[continuation.pickupID];

                    asgn.distToPickup = curVehLocToPickupSearches.getDistance(vehId,
                                                                              continuation.pickupID);
                    if (asgn.distToPickup >= INFTY)
                        continue;

                    asgn.distFromPickup = continuation.distFromPickup;
                    for (int stationId = continuation.fromStationID;
                         stationId < stations.size(); ++stationId) {
                        const auto &station = stations[stationId];
                        asgn.dropoff = {
                            stationId, // PDLoc ID
                            station.vehEdgeId, // Location in road network
                            station.psgEdgeId, // Location in passenger road network
                            0, // Walking time from this dropoff to destination
                            0, // Vehicle driving time from this dropoff to the destination
                            0 // Vehicle driving time from destination to this dropoff
                        };

                        if (asgn.pickup.loc == asgn.dropoff.loc)
                            continue;

                        asgn.distToDropoff = currentLastStopDistances[stationId][i];
                        if (asgn.distToDropoff >= INFTY)
                            continue;

                        ++numAssignmentsTried;
                        asgn.dropoffStopIdx = numStops - 1;
                        // requestState.tryAssignmentWithKnownCost(asgn, calculator.calc(asgn, requestState));
                        firstTaxiLegResult.tryAssignmentWithKnownCostForStation(stationId, asgn, calculator.calc(asgn, requestState));
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

        // Stations
        PTStations &stations;

        // Flag per vehicle that tells us if we still have to consider a pickup before the next stop of the vehicle.
        FastResetFlagArray<> checkPBNSForVehicle;

        // Records for postponing and bundling computations of distances from current locations of vehicles to
        // pickups needed in the PBNS+DALS case.
        struct PickupBeforeNextStopContinuation {
            int pickupID;
            int distFromPickup;
            int fromStationID;
        };
        std::vector<PickupBeforeNextStopContinuation> pbnsContinuations;

        int upperBoundCost;
        int externalUpperBoundCost; // External upper bound on the cost of a PALS insertion

        std::vector<DistanceLabel> currentLastStopDistances;
        
        // Pointers to request state and relevant PD locs so Dijkstra search callback has access to them
        RequestState const *curReqState;
        RelevantPDLocs const *curRelOrdinaryPickups;
        RelevantPDLocs const *curRelPickupsBns;

        int numVerticesSettled;
        int numEntriesVisited;

        int totalNumEdgeRelaxations;
        int totalNumVerticesSettled;
        int totalNumEntriesScanned;

        int totalNumberOfCandidateDropoffs;

    };
    
}