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

#include "../../DataStructures/Queues/AddressableKHeap.h"
#include "../../Tools/CommandLine/ProgressBar.h"
#include "../../Tools/Workarounds.h"
#include "../../Tools/Logging/LogManager.h"
#include "../../Tools/Timer.h"
#include "BaseObjects/Request.h"
#include "BaseObjects/Vehicle.h"

namespace karri {
    template<typename PTAndTaxiTripFinderT,
        typename SystemStateUpdaterT,
        typename ScheduledStopsT>
    class EventSimulation {
        enum VehicleState {
            OUT_OF_SERVICE,
            IDLING,
            DRIVING,
            STOPPING
        };

        enum RiderState {
            NOT_RECEIVED,
            WAITING_FOR_PICKUP,
            IN_TAXI_VEHICLE,
            NON_TAXI_TO_DESTINATION,
            FINISHED
        };


        enum VehicleEventType {
            VEHICLE_NO_EVENT,
            STARTUP,
            ARRIVE_AT_STOP,
            DEPART_FROM_STOP,
            SHUTDOWN
        };

        enum RiderEventType {
            RIDER_NO_EVENT,
            RECEIVE,
            SCHEDULE_2ND_TAXI_LEG,
            ARRIVE_AT_DESTINATION // Any non-taxi arrival
        };

        // Stores information about assignment and departure time of a request needed for logging on arrival of the
        // request.
        struct RequestData {
            int depTime;
            int walkingTimeToPickup;
            int walkingTimeFromDropoff = 0;
            int taxiLegCost = 0;
            int ptLegCost = 0;
            int secondTaxiLegApproxCost = 0;

            // related to combined trip
            bool isCombinedTrip = false;
            bool hasFirstTaxiLeg = false;
            bool hasSecondTaxiLeg = false;
            bool completedFirstTaxiLeg = false;
            int arrivalTimeAtEndOfPtJourney = 0; // arr time at last station if 2nd taxi leg or at destination if not
            int secondTaxiLegDepTime = 0;
            int secondTaxiLegCost = 0;
            int walkingTimeToSecondTaxiLegPickup = 0;
        };

        // Prints representation of request data to output stream for debugging purposes
        inline friend std::ostream& operator<<(std::ostream& os, const RequestData& data) {
            os << "depTime: " << data.depTime
               << ", walkingTimeToPickup: " << data.walkingTimeToPickup
               << ", walkingTimeFromDropoff: " << data.walkingTimeFromDropoff
               << ", taxiLegCost: " << data.taxiLegCost
               << ", ptLegCost: " << data.ptLegCost
               << ", secondTaxiLegApproxCost: " << data.secondTaxiLegApproxCost
               << ", isCombinedTrip: " << data.isCombinedTrip
               << ", hasFirstTaxiLeg: " << data.hasFirstTaxiLeg
               << ", hasSecondTaxiLeg: " << data.hasSecondTaxiLeg
               << ", completedFirstTaxiLeg: " << data.completedFirstTaxiLeg
               << ", arrivalTimeAtEndOfPtJourney: " << data.arrivalTimeAtEndOfPtJourney
               << ", secondTaxiLegDepTime: " << data.secondTaxiLegDepTime
               << ", secondTaxiLegCost: " << data.secondTaxiLegCost
               << ", walkingTimeToSecondTaxiLegPickup: " << data.walkingTimeToSecondTaxiLegPickup;
            return os;
        }

        const int TRIGGER_TAXI_TIME = 9000;

    public:
        EventSimulation(
            const Fleet &fleet, const std::vector<Request> &requests,
            PTAndTaxiTripFinderT &ptAndTaxiTripFinder,
            SystemStateUpdaterT &systemStateUpdater,
            const ScheduledStopsT &scheduledStops,
            PTStations &stations,
            const bool verbose = false)
            : fleet(fleet),
              requests(requests),
              ptAndTaxiTripFinder(ptAndTaxiTripFinder),
              systemStateUpdater(systemStateUpdater),
              scheduledStops(scheduledStops),
              stations(stations),
              vehicleEvents(fleet.size()),
              requestEvents(requests.size()),
              nextVehicleEvents(fleet.size(), VEHICLE_NO_EVENT),
              nextRiderEvents(requests.size(), RIDER_NO_EVENT),
              vehicleState(fleet.size(), OUT_OF_SERVICE),
              riderState(requests.size(), NOT_RECEIVED),
              requestData(requests.size(), RequestData()),
              ptStationsForSecondTaxiLeg(requests.size(), INVALID_ID),
              eventSimulationStatsLogger(LogManager<std::ofstream>::getLogger("eventsimulationstats.csv",
                                                                              "occurrence_time,"
                                                                              "type,"
                                                                              "running_time\n")),
              assignmentQualityStats(LogManager<std::ofstream>::getLogger("assignmentquality.csv",
                                                                          "request_id,"
                                                                          "arr_time,"
                                                                          "wait_time,"
                                                                          "ride_time,"
                                                                          "trip_time,"
                                                                          "walk_to_pickup_time,"
                                                                          "walk_to_dropoff_time,"
                                                                          "cost\n")),
              legStatsLogger(LogManager<std::ofstream>::getLogger("legstats.csv",
                                                                  "vehicle_id,"
                                                                  "stop_time,"
                                                                  "dep_time,"
                                                                  "arr_time,"
                                                                  "drive_time,"
                                                                  "occupancy\n")),
              progressBar(requests.size(), verbose) {
            progressBar.setDotOutputInterval(1);
            progressBar.setPercentageOutputInterval(5);
            for (const auto &veh: fleet) {
                nextVehicleEvents[veh.vehicleId] = STARTUP;
                vehicleEvents.insert(veh.vehicleId, veh.startOfServiceTime);
            }
            for (const auto &req: requests) {
                nextRiderEvents[req.requestId] = RECEIVE;
                requestEvents.insert(req.requestId, req.requestTime);
            }
        }

        void run() {
            while (!(vehicleEvents.empty() && requestEvents.empty())) {
                // Pop next event from either queue. Request event has precedence if at the same time as vehicle event.
                int id, occTime;

                if (requestEvents.empty()) {
                    vehicleEvents.min(id, occTime);
                    handleVehicleEvent(id, occTime);
                    continue;
                }

                if (vehicleEvents.empty()) {
                    requestEvents.min(id, occTime);
                    handleRequestEvent(id, occTime);
                    continue;
                }

                if (vehicleEvents.minKey() < requestEvents.minKey()) {
                    vehicleEvents.min(id, occTime);
                    handleVehicleEvent(id, occTime);
                    continue;
                }

                requestEvents.min(id, occTime);
                handleRequestEvent(id, occTime);
            }
        }

    private:
        void handleVehicleEvent(const int vehId, const int occTime) {
            switch (nextVehicleEvents[vehId]) {
                case STARTUP:
                    handleVehicleStartup(vehId, occTime);
                    break;
                case SHUTDOWN:
                    handleVehicleShutdown(vehId, occTime);
                    break;
                case ARRIVE_AT_STOP:
                    handleVehicleArrivalAtStop(vehId, occTime);
                    break;
                case DEPART_FROM_STOP:
                    handleVehicleDepartureFromStop(vehId, occTime);
                    break;
                case VEHICLE_NO_EVENT:
                    KASSERT(false);
                    break;
                default:
                    break;
            }
        }

        void handleRequestEvent(const int reqId, const int occTime) {
            switch (nextRiderEvents[reqId]) {
                case RECEIVE:
                    handleRequestReceipt(reqId, occTime);
                    break;
                case SCHEDULE_2ND_TAXI_LEG:
                    handleSecondTaxiLeg(reqId, occTime);
                    break;
                case ARRIVE_AT_DESTINATION:
                    handleRiderArrivalAtDest(reqId, occTime);
                    break;
                case FINISHED:
                    KASSERT(false);
                    break;
                case RIDER_NO_EVENT:
                    KASSERT(false);
                    break;
                default:
                    break;
            }
        }

        void handleVehicleStartup(const int vehId, const int occTime) {
            KASSERT(vehicleState[vehId] == OUT_OF_SERVICE);
            KASSERT(fleet[vehId].startOfServiceTime == occTime);
            unused(occTime);
            KaRRiTimer timer;

            // Vehicle may have already been assigned stops. In this case it will start driving right away:
            if (scheduledStops.hasNextScheduledStop(vehId)) {
                vehicleState[vehId] = DRIVING;
                nextVehicleEvents[vehId] = ARRIVE_AT_STOP;
                vehicleEvents.increaseKey(vehId, scheduledStops.getNextScheduledStop(vehId).arrTime);
            } else {
                vehicleState[vehId] = IDLING;
                nextVehicleEvents[vehId] = SHUTDOWN;
                vehicleEvents.increaseKey(vehId, fleet[vehId].endOfServiceTime);
            }

            const auto time = timer.elapsed<std::chrono::nanoseconds>();
            eventSimulationStatsLogger << occTime << ",VehicleStartup," << time << '\n';
        }

        void handleVehicleShutdown(const int vehId, const int occTime) {
            KASSERT(vehicleState[vehId] == IDLING);
            KASSERT(fleet[vehId].endOfServiceTime == occTime);
            unused(occTime);
            KASSERT(!scheduledStops.hasNextScheduledStop(vehId));
            KaRRiTimer timer;

            vehicleState[vehId] = OUT_OF_SERVICE;
            nextVehicleEvents[vehId] = VEHICLE_NO_EVENT;

            int id, key;
            vehicleEvents.deleteMin(id, key);
            KASSERT(id == vehId && key == occTime);
            systemStateUpdater.notifyVehicleReachedEndOfServiceTime(fleet[vehId]);

            const auto time = timer.elapsed<std::chrono::nanoseconds>();
            eventSimulationStatsLogger << occTime << ",VehicleShutdown," << time << '\n';
        }

        void handleVehicleArrivalAtStop(const int vehId, const int occTime) {
            KASSERT(vehicleState[vehId] == DRIVING);
            KASSERT(scheduledStops.getNextScheduledStop(vehId).arrTime == occTime);
            KaRRiTimer timer;

            const auto prevStop = scheduledStops.getCurrentOrPrevScheduledStop(vehId);
            legStatsLogger << vehId << ','
                    << prevStop.depTime - prevStop.arrTime << ','
                    << prevStop.depTime << ','
                    << occTime << ','
                    << occTime - prevStop.depTime << ','
                    << prevStop.occupancyInFollowingLeg << '\n';

            const auto reachedStop = scheduledStops.getNextScheduledStop(vehId);

            // Handle dropoffs at reached stop: Insert walking arrival event at the time when passenger will arrive at
            // destination. Thus, all requests are logged in the order of the arrival at their destination.
            for (const auto &reqId: reachedStop.requestsDroppedOffHere) {
                KASSERT(riderState[reqId] == IN_TAXI_VEHICLE, "request " << reqId << " data: " << requestData[reqId]);
                auto &reqData = requestData[reqId];

                const bool lastTaxiLegFinished =
                        !reqData.isCombinedTrip || // finished first leg in taxi only trip
                        !reqData.hasSecondTaxiLeg || // finished first leg in combined trip without second taxi leg
                        reqData.completedFirstTaxiLeg; // finished second leg of combined trip

                if (lastTaxiLegFinished) {
                    // No more taxis used, so arrival time at destination is fixed.
                    // Insert rider event for arrival at destination.
                    riderState[reqId] = NON_TAXI_TO_DESTINATION;
                    int arrTimeAtDest;
                    if (ptStationsForSecondTaxiLeg[reqId] == INVALID_ID) {
                        // If finished first leg in combined trip without second taxi leg, arrival time at destination is
                        // arrival time with PT journey
                        arrTimeAtDest = reqData.arrivalTimeAtEndOfPtJourney;
                    } else {
                        // If finished a taxi leg with no more PT, walk to destination.
                        arrTimeAtDest = occTime + reqData.walkingTimeFromDropoff;
                    }
                    nextRiderEvents[reqId] = ARRIVE_AT_DESTINATION;
                    requestEvents.insert(reqId, arrTimeAtDest);
                } else {
                    KASSERT(reqData.hasSecondTaxiLeg);
                    // If finished first leg of a trip with more than one leg, wait for pickup by next vehicle.
                    riderState[reqId] = WAITING_FOR_PICKUP;
                    reqData.completedFirstTaxiLeg = true;
                }
            }


            // Next event for this vehicle is the departure at this stop:
            vehicleState[vehId] = STOPPING;
            nextVehicleEvents[vehId] = DEPART_FROM_STOP;
            vehicleEvents.increaseKey(vehId, reachedStop.depTime);
            systemStateUpdater.notifyStopStarted(fleet[vehId]);

            const auto time = timer.elapsed<std::chrono::nanoseconds>();
            eventSimulationStatsLogger << occTime << ",VehicleArrival," << time << '\n';
        }

        void handleVehicleDepartureFromStop(const int vehId, const int occTime) {
            KASSERT(vehicleState[vehId] == STOPPING);
            KASSERT(scheduledStops.getCurrentOrPrevScheduledStop(vehId).depTime == occTime);
            KaRRiTimer timer;

            if (!scheduledStops.hasNextScheduledStop(vehId)) {
                vehicleState[vehId] = IDLING;
                nextVehicleEvents[vehId] = SHUTDOWN;
                vehicleEvents.increaseKey(vehId, fleet[vehId].endOfServiceTime);
            } else {
                // Remember departure time for all requests picked up at this stop:
                const auto curStop = scheduledStops.getCurrentOrPrevScheduledStop(vehId);
                for (const auto &reqId: curStop.requestsPickedUpHere) {
                    KASSERT(riderState[reqId] == WAITING_FOR_PICKUP, "request " << reqId << ", occTime " << occTime << ", data: " << requestData[reqId]);

                    riderState[reqId] = IN_TAXI_VEHICLE;
                    if (requestData[reqId].completedFirstTaxiLeg) {
                        requestData[reqId].secondTaxiLegDepTime = occTime;
                    } else {
                        requestData[reqId].depTime = occTime;
                    }
                }
                vehicleState[vehId] = DRIVING;
                nextVehicleEvents[vehId] = ARRIVE_AT_STOP;
                vehicleEvents.increaseKey(vehId, scheduledStops.getNextScheduledStop(vehId).arrTime);
            }

            systemStateUpdater.notifyStopCompleted(fleet[vehId]);

            const auto time = timer.elapsed<std::chrono::nanoseconds>();
            eventSimulationStatsLogger << occTime << ",VehicleDeparture," << time << '\n';
        }

        void handleRequestReceipt(const int reqId, const int occTime) {
            ++progressBar;
            KASSERT(riderState[reqId] == NOT_RECEIVED);
            KASSERT(requests[reqId].requestTime == occTime);
            KaRRiTimer timer;

            const auto &request = requests[reqId];
            // Find best assignment -> PTAndTaxiTripFinder
            karri::stats::RequestReceiveStats stats;
            auto ptAndTaxiTripFinderResponse = ptAndTaxiTripFinder.findBestAssignment(request, stats);
            auto firstTaxiLeg = ptAndTaxiTripFinderResponse.getFirstTaxiLeg();
            auto ptLeg = ptAndTaxiTripFinderResponse.getPTLeg();

            // Calculated costs
            requestData[reqId].taxiLegCost = ptAndTaxiTripFinderResponse.firstTaxiLegCost;
            requestData[reqId].ptLegCost = ptAndTaxiTripFinderResponse.ptLegCost;
            requestData[reqId].secondTaxiLegApproxCost = ptAndTaxiTripFinderResponse.secondTaxiLegApproxCost;

            int id, key;
            requestEvents.deleteMin(id, key);
            KASSERT(id == reqId && key == occTime);
            nextRiderEvents[reqId] = RIDER_NO_EVENT;

            if (ptAndTaxiTripFinderResponse.isValidTaxiOnlyTrip()) {
                systemStateUpdater.writeBestAssignmentToLogger(firstTaxiLeg);
                riderState[reqId] = WAITING_FOR_PICKUP;
                applyAssignment(firstTaxiLeg, reqId, occTime, stats.updateStats);
            } else if (ptAndTaxiTripFinderResponse.isValidPTOnlyTrip()) {
                applyPtOnlyJourney(ptLeg, reqId, occTime);
            } else {
                riderState[reqId] = WAITING_FOR_PICKUP;
                applyCombinedTrip(firstTaxiLeg, ptLeg, reqId, occTime,
                                  ptAndTaxiTripFinderResponse.hasValidFirstTaxiLeg(),
                                  stats.updateStats);
            }

            systemStateUpdater.writeReceiveRequestLogs(reqId, stats);

            const auto time = timer.elapsed<std::chrono::nanoseconds>();
            eventSimulationStatsLogger << occTime << ",RequestReceipt," << time << '\n';
        }

        void applyAssignment(RequestState &asgnFinderResponse, const int reqId, const int,
                            karri::stats::UpdatePerformanceStats &updateStats,
                             const bool isSecondTaxiLeg = false) {
            KASSERT(!asgnFinderResponse.isNotUsingVehicleBest());

            const auto &bestAsgn = asgnFinderResponse.getBestAssignment();
            // || !bestAsgn.pickup || !bestAsgn.dropoff
            if (!bestAsgn.vehicle) {
                riderState[reqId] = FINISHED;
                return;
            }

            if (isSecondTaxiLeg) {
                requestData[reqId].walkingTimeToSecondTaxiLegPickup = bestAsgn.pickup.walkingDist;
                requestData[reqId].walkingTimeFromDropoff += bestAsgn.dropoff.walkingDist;
                requestData[reqId].secondTaxiLegCost = asgnFinderResponse.getBestCost();
            } else {
                requestData[reqId].walkingTimeToPickup = bestAsgn.pickup.walkingDist;
                requestData[reqId].walkingTimeFromDropoff = bestAsgn.dropoff.walkingDist;
            }

            systemStateUpdater.insertBestAssignment(asgnFinderResponse, updateStats);

            const auto vehId = bestAsgn.vehicle->vehicleId;
            switch (vehicleState[vehId]) {
                case STOPPING:
                    // Update event time to departure time at current stop since it may have changed
                    vehicleEvents.updateKey(vehId, scheduledStops.getCurrentOrPrevScheduledStop(vehId).depTime);
                    break;
                case IDLING:
                    vehicleState[vehId] = DRIVING;
                    nextVehicleEvents[vehId] = ARRIVE_AT_STOP;
                    [[fallthrough]];
                case DRIVING:
                    // Update event time to arrival time at next stop since it may have changed (also for case of idling).
                    vehicleEvents.updateKey(vehId, scheduledStops.getNextScheduledStop(vehId).arrTime);
                    [[fallthrough]];
                default:
                    break;
            }
        }

        template<typename JourneyResponseT>
        void applyPtOnlyJourney(JourneyResponseT &ptResponse, const int reqId, const int) {
            auto &reqData = requestData[reqId];

            const bool isWalkingOnly = ptResponse.isJourneyWalking();
            riderState[reqId] = NON_TAXI_TO_DESTINATION;
            nextRiderEvents[reqId] = ARRIVE_AT_DESTINATION;
            requestEvents.insert(reqId, ptResponse.getArrivalTime());
            reqData.depTime = isWalkingOnly
                                  ? ptResponse.getDepartureTime()
                                  : ptResponse.getDepartureTimeAtFirstStation();
            reqData.walkingTimeToPickup = isWalkingOnly ? 0 : ptResponse.getWalkingTimeToFirstStation();
            reqData.walkingTimeFromDropoff = ptResponse.getWalkingTimeFromLastStation();
            reqData.arrivalTimeAtEndOfPtJourney = ptResponse.getArrivalTime();
        }

        template<typename AssignmentFinderResponseT, typename JourneyResponseT>
        void applyCombinedTrip(AssignmentFinderResponseT &asgnFinderResponse, JourneyResponseT &ptResponse,
                               const int reqId, const int occTime, bool validFirstTaxiLeg,
                               karri::stats::UpdatePerformanceStats &updateStats) {
            KASSERT(ptResponse.getDepartureTimeAtFirstStation() < ptResponse.getArrivalTimeAtLastStation(), "Journey: " << ptResponse.getBestJourney());
            auto &reqData = requestData[reqId];
            reqData.isCombinedTrip = true;
            reqData.arrivalTimeAtEndOfPtJourney = ptResponse.getArrivalTimeAtLastStation();

            // Apply first taxi leg assignment
            if (validFirstTaxiLeg) {
                asgnFinderResponse.setMaxArrTimeAtDropoffStation(ptResponse.getDepartureTimeAtFirstStation());
                applyAssignment(asgnFinderResponse, reqId, occTime, updateStats);
                reqData.hasFirstTaxiLeg = true;
            } else {
                // without first taxi leg
                KASSERT(!ptResponse.isJourneyWalking());
                reqData.depTime = ptResponse.getDepartureTimeAtFirstStation();
                reqData.walkingTimeToPickup = ptResponse.getWalkingTimeToFirstStation();
                reqData.walkingTimeFromDropoff = ptResponse.getWalkingTimeFromLastStation();
            }

            if (ptResponse.isFinalTransferByTaxi()) {
                reqData.hasSecondTaxiLeg = true;
                ptStationsForSecondTaxiLeg[reqId] = ptResponse.getLastStation();

                // Insert request event for second taxi leg 15 minutes before arrival
                const int schedule2ndLegTime = std::max(occTime, ptResponse.getArrivalTimeAtLastStation() - TRIGGER_TAXI_TIME);
                nextRiderEvents[reqId] = SCHEDULE_2ND_TAXI_LEG;
                requestEvents.insert(reqId, schedule2ndLegTime);
            } else {
                const auto arrivalTimeAtDestination = ptResponse.getArrivalTime();
                requestData[reqId].arrivalTimeAtEndOfPtJourney = arrivalTimeAtDestination;
            }
        }

        void handleSecondTaxiLeg(const int reqId, const int occTime) {
            KASSERT(riderState[reqId] == WAITING_FOR_PICKUP || riderState[reqId] == IN_TAXI_VEHICLE);
            KASSERT(ptStationsForSecondTaxiLeg[reqId] != INVALID_ID);

            int id, key;
            requestEvents.deleteMin(id, key);
            KASSERT(id == reqId && key == occTime);
            nextRiderEvents[reqId] = RIDER_NO_EVENT;

            const auto &request = requests[reqId];
            const auto stationEdgeId = stations[ptStationsForSecondTaxiLeg[reqId]].vehEdgeId;
            const Request &newReq = {
                reqId,
                stationEdgeId,
                request.destination,
                requestData[reqId].arrivalTimeAtEndOfPtJourney,
                request.numRiders
            };

            karri::stats::SecondTaxiLegStats secondTaxiLegStats;
            auto asgnFinderResponse = ptAndTaxiTripFinder.findBestSecondTaxiLeg(newReq, occTime, secondTaxiLegStats.taxiSecondLegStats);

            // Before the second taxi leg
            const auto &reqData = requestData[reqId];
            const auto waitTime = reqData.depTime - requests[reqId].requestTime;
            asgnFinderResponse.setCurrentWaitTime(waitTime);

            applyAssignment(asgnFinderResponse, reqId, occTime, secondTaxiLegStats.updateStats, true);

            systemStateUpdater.writeSecondTaxiLegLogs(reqId, secondTaxiLegStats);
        }

        void handleRiderArrivalAtDest(const int reqId, const int occTime) {
            KASSERT(riderState[reqId] == NON_TAXI_TO_DESTINATION);
            KaRRiTimer timer;

            const auto &reqData = requestData[reqId];
            riderState[reqId] = FINISHED;
            int id, key;
            requestEvents.deleteMin(id, key);
            KASSERT(id == reqId && key == occTime);
            nextRiderEvents[reqId] = RIDER_NO_EVENT;

            const auto firstTaxiLegWaitTime = reqData.depTime - requests[reqId].requestTime;
            const auto secondTaxiLegWaitTime = reqData.hasSecondTaxiLeg
                                                   ? reqData.secondTaxiLegDepTime - reqData.arrivalTimeAtEndOfPtJourney
                                                   : 0;
            const auto waitTime = firstTaxiLegWaitTime + secondTaxiLegWaitTime;
            const auto arrTime = occTime;
            const auto rideTime = occTime - reqData.walkingTimeFromDropoff - secondTaxiLegWaitTime - reqData.
                                  walkingTimeToSecondTaxiLegPickup - reqData.depTime;
            const auto tripTime = arrTime - requests[reqId].requestTime;
            const auto asgnCost = reqData.hasSecondTaxiLeg
                                      ? reqData.taxiLegCost + reqData.ptLegCost + reqData.secondTaxiLegCost
                                      : reqData.taxiLegCost + reqData.ptLegCost;

            assignmentQualityStats << reqId << ','
                    << arrTime << ','
                    << waitTime << ','
                    << rideTime << ','
                    << tripTime << ','
                    << reqData.walkingTimeToPickup << ','
                    << reqData.walkingTimeFromDropoff << ','
                    << asgnCost << '\n';


            const auto time = timer.elapsed<std::chrono::nanoseconds>();
            eventSimulationStatsLogger << occTime << ",RequestWalkingArrival," << time << '\n';
        }


        const Fleet &fleet;
        const std::vector<Request> &requests;
        PTAndTaxiTripFinderT &ptAndTaxiTripFinder;
        SystemStateUpdaterT &systemStateUpdater;
        const ScheduledStopsT &scheduledStops;
        PTStations &stations;

        AddressableQuadHeap vehicleEvents;
        AddressableQuadHeap requestEvents;

        std::vector<VehicleEventType> nextVehicleEvents;
        std::vector<RiderEventType> nextRiderEvents;
        std::vector<VehicleState> vehicleState;
        std::vector<RiderState> riderState;

        std::vector<RequestData> requestData;

        std::vector<int> ptStationsForSecondTaxiLeg;

        std::ofstream &eventSimulationStatsLogger;
        std::ofstream &assignmentQualityStats;
        std::ofstream &legStatsLogger;
        KaRRiProgressBar progressBar;
    };
}
