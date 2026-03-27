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

#include "RouteState.h"
#include "LastStopSearches/OnlyLastStopsAtVerticesBucketSubstitute.h"
#include "PathTracker.h"

namespace karri {
    // Updates the system state consisting of the route state (schedules of vehicles and additional information about
    // stops) as well as the bucket state (precomputed information for fast shortest path queries to vehicle stops).
    template<typename InputGraphT,
        typename EllipticBucketsEnvT,
        typename LastStopBucketsEnvT,
        typename StationsInEllipseT,
        typename CurVehLocsT,
        typename PathTrackerT,
        typename LoggerT = NullLogger>
    class SystemStateUpdater {
    public:
        SystemStateUpdater(const InputGraphT &inputGraph,
                           const CurVehLocsT &curVehLocs,
                           PathTrackerT &pathTracker,
                           RouteState &routeState, EllipticBucketsEnvT &ellipticBucketsEnv,
                           LastStopBucketsEnvT &lastStopBucketsEnv,
                           StationsInEllipseT &stationsInEllipse)
            : inputGraph(inputGraph),
              curVehLocs(curVehLocs),
              pathTracker(pathTracker),
              routeState(routeState),
              ellipticBucketsEnv(ellipticBucketsEnv),
              lastStopBucketsEnv(lastStopBucketsEnv),
              stationsInEllipse(stationsInEllipse),
              bestAssignmentsLogger(LogManager<LoggerT>::getLogger("bestassignments.csv",
                                                                   "request_id, "
                                                                   "request_time, "
                                                                   "direct_od_dist, "
                                                                   "vehicle_id, "
                                                                   "pickup_insertion_point, "
                                                                   "dropoff_insertion_point, "
                                                                   "dist_to_pickup, "
                                                                   "dist_from_pickup, "
                                                                   "dist_to_dropoff, "
                                                                   "dist_from_dropoff, "
                                                                   "pickup_id, "
                                                                   "pickup_walking_dist, "
                                                                   "dropoff_id, "
                                                                   "dropoff_walking_dist, "
                                                                   "num_stops, "
                                                                   "veh_dep_time_at_stop_before_pickup, "
                                                                   "veh_dep_time_at_stop_before_dropoff, "
                                                                   "not_using_vehicle, "
                                                                   "cost\n")),
              // tripTypeLogger(LogManager<LoggerT>::getLogger("triptypes.csv",
              //                                               "request_id,"
              //                                               "is_taxi_only,"
              //                                               "is_pt_only,"
              //                                               "is_walking_only,"
              //                                               "is_combined,"
              //                                               "has_first_taxi_leg,"
              //                                               "has_pt_leg,"
              //                                               "has_second_taxi_leg\n")),
              roadCatLogger(LogManager<LoggerT>::getLogger(karri::stats::OsmRoadCategoryStats::LOGGER_NAME,
                                                           "type," +
                                                           karri::stats::OsmRoadCategoryStats::getLoggerCols())) {
        }


        void insertBestAssignment(const RequestState &requestState, const TaxiResult &result,
                                  stats::UpdatePerformanceStats &stats,
                                  const int externalMaxArrTimeAtDropoff = INFTY) {
            KaRRiTimer timer;

            const auto &asgn = result.getBestAssignment();
            chosenPDLocsRoadCatStats.incCountForCat(inputGraph.osmRoadCategory(asgn.pickup.loc));
            chosenPDLocsRoadCatStats.incCountForCat(inputGraph.osmRoadCategory(asgn.dropoff.loc));
            assert(asgn.vehicle != nullptr);

            const auto vehId = asgn.vehicle->vehicleId;
            const auto numStopsBefore = routeState.numStopsOf(vehId);
            const auto depTimeAtLastStopBefore = routeState.schedDepTimesFor(vehId)[numStopsBefore - 1];

            timer.restart();
            auto [pickupIndex, dropoffIndex] = routeState.insert(asgn, requestState, externalMaxArrTimeAtDropoff);
            const auto routeUpdateTime = timer.elapsed<std::chrono::nanoseconds>();
            stats.updateRoutesTime += routeUpdateTime;

            updateBucketState(asgn, pickupIndex, dropoffIndex, depTimeAtLastStopBefore, requestState.now(),
                              stats);

            // If the vehicle has to be rerouted at its current location for a PBNS assignment, we introduce an
            // intermediate stop at its current location representing the rerouting.
            if (asgn.pickupStopIdx == 0 && numStopsBefore > 1 &&
                routeState.schedDepTimesFor(vehId)[0] < requestState.now()) {
                createIntermediateStopAtCurrentLocationForReroute(*asgn.vehicle, requestState.now(), stats);
                ++pickupIndex;
                ++dropoffIndex;
            }

            int pickupStopId = routeState.stopIdsFor(vehId)[pickupIndex];
            int dropoffStopId = routeState.stopIdsFor(vehId)[dropoffIndex];

            assert(pickupStopId >= 0 && dropoffStopId >= 0);

            // Register the inserted pickup and dropoff with the path data
            pathTracker.registerPdEventsForBestAssignment(requestState, pickupStopId, dropoffStopId);
        }

        void notifyStopStarted(const Vehicle &veh) {
            // Update buckets and route state
            ellipticBucketsEnv.deleteSourceBucketEntries(veh, 0);
            ellipticBucketsEnv.deleteTargetBucketEntries(veh, 1);
            // remove the stations from the ellipse that are no longer relevant
            stationsInEllipse.removeStationsForStop(0, veh.vehicleId);
            routeState.removeStartOfCurrentLeg(veh.vehicleId);
        }


        void notifyStopCompleted(const Vehicle &veh) {
            pathTracker.logCompletedStop(veh);

            // If vehicle has become idle, update last stop bucket entries
            if (routeState.numStopsOf(veh.vehicleId) == 1) {
                int64_t updateTimePlaceholder = 0;
                lastStopBucketsEnv.updateBucketEntries(veh, 0, updateTimePlaceholder);
            }
        }

        void notifyVehicleReachedEndOfServiceTime(const Vehicle &veh) {
            const auto vehId = veh.vehicleId;
            assert(routeState.numStopsOf(vehId) == 1);

            stats::UpdatePerformanceStats finalRemovalStatsPlaceholder;
            lastStopBucketsEnv.removeIdleBucketEntries(veh, 0, finalRemovalStatsPlaceholder);

            routeState.removeStartOfCurrentLeg(vehId);
        }


        void writeBestAssignmentToLogger(const RequestState &requestState, const TaxiResult &result) {
            bestAssignmentsLogger
                    << requestState.originalRequest.requestId << ", "
                    << requestState.originalRequest.requestTime << ", "
                    << requestState.originalReqDirectDist << ", ";

            if (result.getBestCost() == INFTY) {
                bestAssignmentsLogger << "-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,inf\n";
                return;
            }

            const auto &bestAsgn = result.getBestAssignment();

            const auto &vehId = bestAsgn.vehicle->vehicleId;
            const auto &numStops = routeState.numStopsOf(vehId);
            using time_utils::getVehDepTimeAtStopForRequest;
            const auto &vehDepTimeBeforePickup = getVehDepTimeAtStopForRequest(vehId, bestAsgn.pickupStopIdx,
                                                                               requestState.now(), routeState);
            const auto &vehDepTimeBeforeDropoff = getVehDepTimeAtStopForRequest(vehId, bestAsgn.dropoffStopIdx,
                requestState.now(), routeState);
            bestAssignmentsLogger
                    << vehId << ", "
                    << bestAsgn.pickupStopIdx << ", "
                    << bestAsgn.dropoffStopIdx << ", "
                    << bestAsgn.distToPickup << ", "
                    << bestAsgn.distFromPickup << ", "
                    << bestAsgn.distToDropoff << ", "
                    << bestAsgn.distFromDropoff << ", "
                    << bestAsgn.pickup.id << ", "
                    << bestAsgn.pickup.walkingDist << ", "
                    << bestAsgn.dropoff.id << ", "
                    << bestAsgn.dropoff.walkingDist << ", "
                    << numStops << ", "
                    << vehDepTimeBeforePickup << ", "
                    << vehDepTimeBeforeDropoff << ", "
                    << "false, "
                    << result.getBestCost() << "\n";
        }

        void writeReceiveRequestLogs(const int requestId, const stats::RequestReceiveStats &stats) const {
            logPerformance(requestId, "", stats);
            writeTaxiPerformanceLogs(requestId, "taxi_only.", stats.taxiOnlyStats);
            writeTaxiPerformanceLogs(requestId, "first_leg.", stats.taxiFirstLegStats);
            logPerformance(requestId, "pt_only.", stats.ptOnlyStats);
            logPerformance(requestId, "pt_with_taxi.", stats.ptWithTaxiStats);
        }

        // template<typename PtAndTaxiTripFinderResponse>
        // void writeTripTypeLogs(const int requestId, const PtAndTaxiTripFinderResponse &response) {
        //     const bool isTaxiOnly = response.isValidTaxiOnlyTrip();
        //     const bool isPtOnly = response.isValidPTOnlyTrip() && !response.getPTLeg().isJourneyWalking();
        //     const bool isWalkingOnly = response.isValidPTOnlyTrip() && response.getPTLeg().isJourneyWalking();
        //     const bool isCombined = !isTaxiOnly && !isPtOnly && !isWalkingOnly;
        //     tripTypeLogger << requestId << ", "
        //             << isTaxiOnly << ", "
        //             << isPtOnly << ", "
        //             << isWalkingOnly << ", "
        //             << isCombined << ", "
        //             << response.hasValidFirstTaxiLeg() << ", "
        //             << response.hasValidPTLeg() << ", "
        //             << (response.hasValidPTLeg() && response.getPTLeg().isFinalTransferByTaxi()) << "\n";
        // }

        void writeSecondTaxiLegLogs(const int requestId, const stats::SecondTaxiLegStats &stats) const {
            writeTaxiPerformanceLogs(requestId, "second_leg.", stats.taxiSecondLegStats);
            logPerformance(requestId, "second_leg.", stats.updateStats);
        }

        void writeRoadCatLogs() {
            roadCatLogger << "all_pd_locs, " << allPDLocsRoadCatStats.getLoggerRow() << "\n";
            roadCatLogger << "chosen_pd_locs, " << chosenPDLocsRoadCatStats.getLoggerRow() << "\n";
            allPDLocsRoadCatStats.reset();
            chosenPDLocsRoadCatStats.reset();
        }

    private:
        void writeTaxiPerformanceLogs(const int requestId, const std::string &name_prefix,
                                      const stats::TaxiPerformanceStats &stats) const {
            logPerformance(requestId, name_prefix,
                           stats,
                           stats.initializationStats,
                           stats.ellipticBchStats,
                           stats.pdDistancesStats,
                           stats.ordAssignmentsStats,
                           stats.pbnsAssignmentsStats,
                           stats.palsAssignmentsStats,
                           stats.dalsAssignmentsStats,
                           stats.stationBchStats);
        }

#define GET_RAW_TYPE_OF(x) std::remove_reference_t<decltype(x)>

        template<typename... PerfStatssT>
        void logPerformance(const int requestId, const std::string &name_prefix, PerfStatssT... perfStatss) const {
            RUN_FORALL(
                LogManager<LoggerT>::getLogger(name_prefix + std::string(GET_RAW_TYPE_OF(perfStatss)::LOGGER_NAME),
                    "request_id," + std::string(GET_RAW_TYPE_OF(perfStatss)::LOGGER_COLS)) << requestId << "," <<
                perfStatss.getLoggerRow() << "\n"
            );
        }


        // If vehicle is rerouted from its current position to a newly inserted stop (PBNS assignment), create new
        // intermediate stop at the vehicle's current position to maintain the invariant of the schedule for the
        // first stop, i.e. dist(s[i], s[i+1]) = schedArrTime(s[i+1]) - schedDepTime(s[i]).
        // Intermediate stop gets an arrival time equal to the request time so the stop is reached immediately,
        // making it the new stop 0. Thus, we do not need to compute target bucket entries for the stop.
        void createIntermediateStopAtCurrentLocationForReroute(const Vehicle &veh, const int now,
                                                               karri::stats::UpdatePerformanceStats &stats) {
            assert(curVehLocs.knowsCurrentLocationOf(veh.vehicleId));
            auto loc = curVehLocs.getCurrentLocationOf(veh.vehicleId);
            LIGHT_KASSERT(loc.depTimeAtHead >= now);
            routeState.createIntermediateStopForReroute(veh.vehicleId, loc.location, now, loc.depTimeAtHead);
            ellipticBucketsEnv.generateSourceBucketEntries(veh, 1, stats);
        }


        // Updates the bucket state (elliptic buckets, last stop buckets, lastStopsAtVertices structure) given an
        // assignment that has already been inserted into routeState as well as the stop index of the pickup and
        // dropoff after the insertion.
        void updateBucketState(const Assignment &asgn,
                               const int pickupIndex, const int dropoffIndex,
                               const int depTimeAtLastStopBefore,
                               const int now,
                               karri::stats::UpdatePerformanceStats &stats) {
            generateBucketStateForNewStops(asgn, pickupIndex, dropoffIndex, depTimeAtLastStopBefore, now, stats);

            // If we use buckets sorted by remaining leeway, we have to update the leeway of all
            // entries for stops of this vehicle.
            if constexpr (EllipticBucketsEnvT::SORTED_BY_REM_LEEWAY) {
                ellipticBucketsEnv.updateLeewayInSourceBucketsForAllStopsOf(*asgn.vehicle, stats);
                ellipticBucketsEnv.updateLeewayInTargetBucketsForAllStopsOf(*asgn.vehicle, stats);
            }

            // If last stop does not change but departure time at last stop does change, update last stop bucket entries
            // accordingly.
            const int vehId = asgn.vehicle->vehicleId;
            const auto numStopsAfter = routeState.numStopsOf(vehId);
            const bool pickupAtExistingStop = pickupIndex == asgn.pickupStopIdx;
            const bool dropoffAtExistingStop = dropoffIndex == asgn.dropoffStopIdx + !pickupAtExistingStop;
            const auto depTimeAtLastStopAfter = routeState.schedDepTimesFor(vehId)[numStopsAfter - 1];
            const bool depTimeAtLastChanged = depTimeAtLastStopAfter != depTimeAtLastStopBefore;

            if ((dropoffAtExistingStop || dropoffIndex < numStopsAfter - 1) && depTimeAtLastChanged) {
                lastStopBucketsEnv.updateBucketEntries(*asgn.vehicle, numStopsAfter - 1,
                                                       stats.lastStopBucketsUpdateEntriesTime);
            }
        }

        void generateBucketStateForNewStops(const Assignment &asgn, const int pickupIndex, const int dropoffIndex,
                                            const int depTimeAtLastStopBefore,
                                            const int now,
                                            karri::stats::UpdatePerformanceStats &stats) {
            const auto vehId = asgn.vehicle->vehicleId;
            const auto &numStops = routeState.numStopsOf(vehId);
            const bool pickupAtExistingStop = pickupIndex == asgn.pickupStopIdx;
            const bool dropoffAtExistingStop = dropoffIndex == asgn.dropoffStopIdx + !pickupAtExistingStop;

            if (!pickupAtExistingStop) {
                ellipticBucketsEnv.generateTargetBucketEntries(*asgn.vehicle, pickupIndex, stats);
                ellipticBucketsEnv.generateSourceBucketEntries(*asgn.vehicle, pickupIndex, stats);
                // calculate the relevant stations for this new pickup stop
                stationsInEllipse.recomputeStationsInEllipseForStop(pickupIndex - 1, vehId);
                stationsInEllipse.computeNewStationsInEllipsesForStop(pickupIndex, vehId);
            }

            // If no new stop was inserted for the pickup, we do not need to generate any new entries for it.
            if (dropoffAtExistingStop)
                return;

            ellipticBucketsEnv.generateTargetBucketEntries(*asgn.vehicle, dropoffIndex, stats);

            // If dropoff is not the new last stop, we generate elliptic source buckets for it.
            if (dropoffIndex < numStops - 1) {
                ellipticBucketsEnv.generateSourceBucketEntries(*asgn.vehicle, dropoffIndex, stats);
                // calculate the relevant stations for this new dropoff stop
                stationsInEllipse.recomputeStationsInEllipseForStop(dropoffIndex - 1, vehId);
                stationsInEllipse.computeNewStationsInEllipsesForStop(dropoffIndex, vehId);
                return;
            }

            // If dropoff is the new last stop, the former last stop becomes a regular stop:
            // Generate elliptic source bucket entries for former last stop
            const auto pickupAtEnd = pickupIndex + 1 == dropoffIndex && pickupIndex > asgn.pickupStopIdx;
            const int formerLastStopIdx = dropoffIndex - pickupAtEnd - 1;
            ellipticBucketsEnv.generateSourceBucketEntries(*asgn.vehicle, formerLastStopIdx, stats);
            stationsInEllipse.computeNewStationsInEllipsesForStop(formerLastStopIdx, vehId);

            // Remove last stop bucket entries for former last stop and generate them for dropoff
            if (formerLastStopIdx == 0 && depTimeAtLastStopBefore < now) {
                lastStopBucketsEnv.removeIdleBucketEntries(*asgn.vehicle, formerLastStopIdx, stats);
            } else {
                lastStopBucketsEnv.removeNonIdleBucketEntries(*asgn.vehicle, formerLastStopIdx, stats);
            }
            lastStopBucketsEnv.generateNonIdleBucketEntries(*asgn.vehicle, stats);
        }

        const InputGraphT &inputGraph;
        const CurVehLocsT &curVehLocs;
        PathTrackerT &pathTracker;

        // Route state
        RouteState &routeState;

        // Bucket state
        EllipticBucketsEnvT &ellipticBucketsEnv;
        LastStopBucketsEnvT &lastStopBucketsEnv;

        // Stations in Ellipse
        StationsInEllipseT &stationsInEllipse;

        // Performance Loggers
        LoggerT &bestAssignmentsLogger;
        // LoggerT &tripTypeLogger;
        LoggerT &roadCatLogger;

        // Road Category Stats
        stats::OsmRoadCategoryStats allPDLocsRoadCatStats;
        stats::OsmRoadCategoryStats chosenPDLocsRoadCatStats;
    };
}
