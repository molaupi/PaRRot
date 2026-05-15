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

#include "../../../../PTaxi/RiderModeChoice/TransportMode.h"
#include "../../DataStructures/Queues/AddressableKHeap.h"
#include "../../Tools/CommandLine/ProgressBar.h"
#include "../../Tools/Workarounds.h"
#include "../../Tools/Logging/LogManager.h"
#include "../../Tools/Timer.h"
#include "BaseObjects/Request.h"
#include "BaseObjects/Vehicle.h"
#include "Stats/PerformanceStats.h"

namespace karri {
    template<typename RequestStateInitializerT,
        typename KaRRiPreparatorT,
        typename WalkTripFinderT,
        typename CarTripFinderT,
        typename TaxiTripFinderT,
        typename PTTripFinderT,
        typename PTAndTaxiTripFinderT,
        typename ModeChoiceT,
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
            int directOdDist = 0;

            int taxiLegCost = 0;
            int ptLegCost = 0;
            int secondTaxiLegApproxCost = 0; // expected cost based on heuristic
            parrot::mode_choice::TransportMode mode = parrot::mode_choice::TransportMode::None;

            // related to combined trip
            bool hasFirstTaxiLeg = false;
            bool hasSecondTaxiLeg = false;
            bool completedFirstTaxiLeg = false;
            int secondTaxiLegCost = 0; // actual observed cost

            // First taxi leg
            int firstTaxiLegPickupWalkTime = 0;
            int firstTaxiLegDepAtPickup = 0;
            int firstTaxiLegArrAtDropoff = 0;
            int firstTaxiLegDropoffWalkTime = 0;
            int firstTaxiLegVehicleId = INVALID_ID;

            // PT leg
            int ptLegDepTime = 0;
            int ptLegArrTime = 0;
            int ptLegWalkTime = 0;
            int ptLegRideTime = 0;

            // Second taxi leg
            int secondTaxiLegPickupWalkTime = 0;
            int secondTaxiLegDepAtPickup = 0;
            int secondTaxiLegArrAtDropoff = 0;
            int secondTaxiLegDropoffWalkTime = 0;
            int secondTaxiLegVehicleId = INVALID_ID;
        };

        // Prints representation of request data to output stream for debugging purposes
        inline friend std::ostream &operator<<(std::ostream &os, const RequestData &data) {
            os << "directOdDist: " << data.directOdDist
                    << ", taxiLegCost: " << data.taxiLegCost
                    << ", ptLegCost: " << data.ptLegCost
                    << ", secondTaxiLegApproxCost: " << data.secondTaxiLegApproxCost
                    << ", mode: " << static_cast<int>(data.mode)
                    << ", hasFirstTaxiLeg: " << data.hasFirstTaxiLeg
                    << ", hasSecondTaxiLeg: " << data.hasSecondTaxiLeg
                    << ", completedFirstTaxiLeg: " << data.completedFirstTaxiLeg
                    << ", secondTaxiLegCost: " << data.secondTaxiLegCost
                    << ", firstTaxiLegPickupWalkTime: " << data.firstTaxiLegPickupWalkTime
                    << ", firstTaxiLegDepAtPickup: " << data.firstTaxiLegDepAtPickup
                    << ", firstTaxiLegArrAtDropoff: " << data.firstTaxiLegArrAtDropoff
                    << ", firstTaxiLegDropoffWalkTime: " << data.firstTaxiLegDropoffWalkTime
                    << ", firstTaxiLegVehicleId: " << data.firstTaxiLegVehicleId
                    << ", ptLegDepTime: " << data.ptLegDepTime
                    << ", ptLegArrTime: " << data.ptLegArrTime
                    << ", ptLegWalkTime: " << data.ptLegWalkTime
                    << ", ptLegRideTime: " << data.ptLegRideTime
                    << ", secondTaxiLegPickupWalkTime: " << data.secondTaxiLegPickupWalkTime
                    << ", secondTaxiLegDepAtPickup: " << data.secondTaxiLegDepAtPickup
                    << ", secondTaxiLegArrAtDropoff: " << data.secondTaxiLegArrAtDropoff
                    << ", secondTaxiLegDropoffWalkTime: " << data.secondTaxiLegDropoffWalkTime
                    << ", secondTaxiLegVehicleId: " << data.secondTaxiLegVehicleId;
            return os;
        }

        const int TRIGGER_TAXI_TIME = 9000;

    public:
        EventSimulation(
            const Fleet &fleet, const std::vector<Request> &requests,
            RequestStateInitializerT &requestStateInitializer,
            KaRRiPreparatorT &preparator,
            WalkTripFinderT &walkTripFinder,
            CarTripFinderT &carTripFinder,
            TaxiTripFinderT &taxiTripFinder,
            PTTripFinderT &ptTripFinder,
            PTAndTaxiTripFinderT &ptAndTaxiTripFinder,
            ModeChoiceT &modeChoice,
            SystemStateUpdaterT &systemStateUpdater,
            const ScheduledStopsT &scheduledStops,
            const parrot::PTStations &stations,
            const bool verbose = false)
            : fleet(fleet),
              requests(requests),
              rsInitializer(requestStateInitializer),
              karriPrep(preparator),
              walkTripFinder(walkTripFinder),
              carTripFinder(carTripFinder),
              taxiTripFinder(taxiTripFinder),
              ptTripFinder(ptTripFinder),
              ptAndTaxiTripFinder(ptAndTaxiTripFinder),
              modeChoice(modeChoice),
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
                                                                          "direct_od_dist,"
                                                                          "arr_time,"
                                                                          "trip_time,"
                                                                          "wait_time,"
                                                                          "taxi_ride_time,"
                                                                          "pt_ride_time,"
                                                                          "walk_time,"
                                                                          "cost_1st_taxi_leg,"
                                                                          "cost_pt_leg,"
                                                                          "cost_2nd_taxi_leg,"
                                                                          "cost\n")),
              tripStatsLogger(LogManager<std::ofstream>::getLogger("tripstats.csv",
                                                                   "request_id,"
                                                                   "firstTaxiLegPickupWalkTime,"
                                                                   "firstTaxiLegDepAtPickup,"
                                                                   "firstTaxiLegArrAtDropoff,"
                                                                   "firstTaxiLegDropoffWalkTime,"
                                                                   "firstTaxiLegVehicleId,"
                                                                   "ptLegDepTime,"
                                                                   "ptLegArrTime,"
                                                                   "ptLegWalkTime,"
                                                                   "ptLegRideTime,"
                                                                   "secondTaxiLegPickupWalkTime,"
                                                                   "secondTaxiLegDepAtPickup,"
                                                                   "secondTaxiLegArrAtDropoff,"
                                                                   "secondTaxiLegDropoffWalkTime,"
                                                                   "secondTaxiLegVehicleId\n"
              )),
              legStatsLogger(LogManager<std::ofstream>::getLogger("legstats.csv",
                                                                  "vehicle_id,"
                                                                  "from_edge,"
                                                                  "to_edge,"
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
            const auto reachedStop = scheduledStops.getNextScheduledStop(vehId);
            legStatsLogger << vehId << ',' << prevStop.stopLocation << ',' << reachedStop.stopLocation << ','
                    << prevStop.depTime - prevStop.arrTime << ','
                    << prevStop.depTime << ','
                    << reachedStop.arrTime << ','
                    << reachedStop.arrTime - prevStop.depTime << ','
                    << prevStop.occupancyInFollowingLeg << '\n';

            // Handle dropoffs at reached stop: Insert walking arrival event at the time when passenger will arrive at
            // destination. Thus, all requests are logged in the order of the arrival at their destination.
            for (const auto &reqId: reachedStop.requestsDroppedOffHere) {
                KASSERT(riderState[reqId] == IN_TAXI_VEHICLE, "request " << reqId << " data: " << requestData[reqId]);
                auto &reqData = requestData[reqId];

                if (reqData.mode == parrot::mode_choice::TransportMode::Taxi) {
                    // If the request is on a taxi-only trip, the rider now walks from here to the destination.
                    riderState[reqId] = NON_TAXI_TO_DESTINATION;
                    nextRiderEvents[reqId] = ARRIVE_AT_DESTINATION;
                    reqData.firstTaxiLegArrAtDropoff = occTime;
                    requestEvents.insert(reqId, occTime + reqData.firstTaxiLegDropoffWalkTime);
                    continue;
                }

                KASSERT(reqData.mode == parrot::mode_choice::TransportMode::TaxiAndPT);

                if (!reqData.hasSecondTaxiLeg) {
                    // If the request is on a combined trip without an egress RP trip, they now take the PT leg until the destination.
                    riderState[reqId] = NON_TAXI_TO_DESTINATION;
                    nextRiderEvents[reqId] = ARRIVE_AT_DESTINATION;
                    reqData.firstTaxiLegArrAtDropoff = occTime;
                    requestEvents.insert(reqId, reqData.ptLegArrTime);
                    continue;
                }

                if (reqData.completedFirstTaxiLeg || !reqData.hasFirstTaxiLeg) {
                    // If the request is on a combined trip with an egress RP trip and has previously completed the access RP trip,
                    // they are now done with the second taxi leg, and walk to the destination.
                    riderState[reqId] = NON_TAXI_TO_DESTINATION;
                    nextRiderEvents[reqId] = ARRIVE_AT_DESTINATION;
                    reqData.secondTaxiLegArrAtDropoff = occTime;
                    requestEvents.insert(reqId, occTime + reqData.secondTaxiLegDropoffWalkTime);
                    continue;
                }

                // If the rider has finished the access RP trip in a combined journey with an egress RP trip,
                // they now wait for pickup by the egress vehicle. (In reality, they start the PT leg but there are no
                // more rider events until the next pickup.)
                riderState[reqId] = WAITING_FOR_PICKUP;
                reqData.firstTaxiLegArrAtDropoff = occTime;
                reqData.completedFirstTaxiLeg = true;
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
                    KASSERT(riderState[reqId] == WAITING_FOR_PICKUP,
                            "request " << reqId << ", occTime " << occTime << ", data: " << requestData[reqId]);

                    riderState[reqId] = IN_TAXI_VEHICLE;
                    auto &reqData = requestData[reqId];
                    if (reqData.mode == parrot::mode_choice::TransportMode::TaxiAndPT && (
                            reqData.completedFirstTaxiLeg || !reqData.hasFirstTaxiLeg)) {
                        reqData.secondTaxiLegDepAtPickup = occTime;
                    } else {
                        reqData.firstTaxiLegDepAtPickup = occTime;
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

            auto requestState = rsInitializer.initializeRequestState(request, request.requestTime,
                                                                     stats.taxiPrepStats.initializationStats);
            const auto baseInfo = karriPrep.prepareBaseInfo(requestState, stats.taxiPrepStats);
            const auto walkOnlyResult = walkTripFinder.findWalkingTrip(requestState, stats.walkOnlyStats);
            const auto carOnlyResult = carTripFinder.findCarTrip(requestState, stats.carOnlyStats);
            const auto taxiOnlyResult = taxiTripFinder.findBestAssignment(requestState, baseInfo, stats.taxiOnlyStats);
            systemStateUpdater.writeBestAssignmentToLogger(requestState, taxiOnlyResult);
            const auto ptOnlyResult = ptTripFinder.findBestJourney(requestState, stats.ptOnlyStats);

            // const int ptOnlyCostBound = ptOnlyResult.getCost();
            const int ptOnlyCostBound = ptTripFinder.getBestCostOfAnyJourneyFromLastRun(request.requestTime);
            const auto ptAndTaxiResult = ptAndTaxiTripFinder.findBestAssignment(
                requestState, baseInfo, taxiOnlyResult.getBestCost(), ptOnlyCostBound, ptTripFinder,
                stats.taxiAndPtPerformanceStats);


            int id, key;
            requestEvents.deleteMin(id, key);
            KASSERT(id == reqId && key == occTime);
            nextRiderEvents[reqId] = RIDER_NO_EVENT;

            const auto mode = modeChoice.chooseMode(requestState, walkOnlyResult, carOnlyResult, taxiOnlyResult,
                                                    ptOnlyResult,
                                                    ptAndTaxiResult);
            auto &reqData = requestData[reqId];
            reqData.directOdDist = requestState.originalReqDirectDist;
            reqData.mode = mode;

            using parrot::mode_choice::TransportMode;
            if (mode == TransportMode::Ped || mode == TransportMode::Car) {
                const int arrTime = mode == TransportMode::Ped
                                        ? request.requestTime + walkOnlyResult.walkingDist
                                        : request.requestTime + requestState.originalReqDirectDist;
                processChoiceOtherMode(reqId, occTime, arrTime);
            } else if (mode == TransportMode::PublicTransport) {
                reqData.ptLegCost = ptOnlyResult.getCost();
                reqData.ptLegDepTime = request.requestTime;
                reqData.ptLegArrTime = ptOnlyResult.getArrivalTime();
                reqData.ptLegRideTime = ptOnlyResult.getTotalInVehicleTime();
                reqData.ptLegWalkTime = ptOnlyResult.getTotalTransferTime();
                processChoiceOtherMode(reqId, occTime, ptOnlyResult.getArrivalTime());
            } else if (mode == TransportMode::Taxi) {
                riderState[reqId] = WAITING_FOR_PICKUP;
                const auto &asgn = taxiOnlyResult.getBestAssignment();
                reqData.taxiLegCost = taxiOnlyResult.getBestCost();
                reqData.firstTaxiLegPickupWalkTime = asgn.pickup.walkingDist;
                reqData.firstTaxiLegDropoffWalkTime = asgn.dropoff.walkingDist;
                reqData.firstTaxiLegVehicleId = asgn.vehicle ? asgn.vehicle->vehicleId : INVALID_ID;
                applyAssignment(requestState, asgn, reqId, stats.updateStats);
            } else if (mode == TransportMode::TaxiAndPT) {
                riderState[reqId] = WAITING_FOR_PICKUP;
                // Calculated costs
                reqData.taxiLegCost = ptAndTaxiResult.getFirstTaxiLegCost();
                reqData.ptLegCost = ptAndTaxiResult.getPTLegCost();
                reqData.secondTaxiLegApproxCost = ptAndTaxiResult.getSecondTaxiLegCost();

                if (ptAndTaxiResult.isInitialTransferByTaxi()) {
                    const auto &firstTaxiLeg = ptAndTaxiResult.getFirstTaxiLeg();
                    const auto &firstTaxiLegAsgn = firstTaxiLeg.getBestAssignment();
                    reqData.firstTaxiLegPickupWalkTime = firstTaxiLegAsgn.pickup.walkingDist;
                    reqData.firstTaxiLegDropoffWalkTime = firstTaxiLegAsgn.dropoff.walkingDist;
                    reqData.firstTaxiLegVehicleId = firstTaxiLegAsgn.vehicle
                                                        ? firstTaxiLegAsgn.vehicle->vehicleId
                                                        : INVALID_ID;
                    reqData.hasFirstTaxiLeg = true;
                }

                KASSERT(!ptAndTaxiResult.getPTLeg().journey.empty());
                reqData.ptLegCost = ptAndTaxiResult.getPTLeg().getCost();
                reqData.ptLegDepTime = ptAndTaxiResult.getPTLeg().getDepartureTime();
                reqData.ptLegArrTime = ptAndTaxiResult.getPTLeg().getArrivalTime();
                reqData.ptLegRideTime = ptAndTaxiResult.getPTLeg().getTotalInVehicleTime();
                reqData.ptLegWalkTime = ptAndTaxiResult.getPTLeg().getTotalTransferTime();

                reqData.hasSecondTaxiLeg = ptAndTaxiResult.isFinalTransferByTaxi();

                applyCombinedTrip(requestState, ptAndTaxiResult, reqId, occTime, stats.updateStats);
            } else if (mode == TransportMode::None) {
                processNoMode(reqId);
            } else {
                KASSERT(false);
            }

            // systemStateUpdater.writeTripTypeLogs(reqId, ptAndTaxiTripFinderResponse);
            systemStateUpdater.writeReceiveRequestLogs(reqId, stats);

            const auto time = timer.elapsed<std::chrono::nanoseconds>();
            eventSimulationStatsLogger << occTime << ",RequestReceipt," << time << '\n';
        }

        void applyAssignment(const RequestState &requestState, const Assignment &asgn, const int reqId,
                             karri::stats::UpdatePerformanceStats &updateStats,
                             const int externalMaxArrTimeAtDropoff = INFTY) {
            // || !bestAsgn.pickup || !bestAsgn.dropoff
            if (!asgn.vehicle) {
                riderState[reqId] = FINISHED;
                return;
            }

            systemStateUpdater.insertBestAssignment(requestState, asgn, updateStats, externalMaxArrTimeAtDropoff);

            const auto vehId = asgn.vehicle->vehicleId;
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

        void processChoiceOtherMode(const int reqId, const int occTime, const int arrivalTime) {
            KASSERT(!requestEvents.contains(reqId));

            // Assign rider to take other mode to their destination and insert event for their arrival.
            riderState[reqId] = NON_TAXI_TO_DESTINATION;
            nextRiderEvents[reqId] = ARRIVE_AT_DESTINATION;
            requestEvents.insert(reqId, arrivalTime);
        }

        void processNoMode(const int reqId) {
            KASSERT(!requestEvents.contains(reqId));
            riderState[reqId] = FINISHED;
            nextRiderEvents[reqId] = RIDER_NO_EVENT;
            assignmentQualityStats << reqId << ','
                    << -1 << ','
                    << -1 << ','
                    << -1 << ','
                    << -1 << ','
                    << -1 << ','
                    << -1 << ','
                    << -1 << ','
                    << -1 << ','
                    << -1 << ','
                    << -1 << ','
                    << -1 << '\n';
            tripStatsLogger << reqId << ','
                    << -1 << ','
                    << -1 << ','
                    << -1 << ','
                    << -1 << ','
                    << -1 << ','
                    << -1 << ','
                    << -1 << ','
                    << -1 << ','
                    << -1 << ','
                    << -1 << ','
                    << -1 << ','
                    << -1 << ','
                    << -1 << ','
                    << -1 << '\n';
        }

        void applyCombinedTrip(const RequestState &requestState,
                               const parrot::ApproximateCombinedTripResult &combinedResult,
                               const int reqId, const int occTime,
                               karri::stats::UpdatePerformanceStats &updateStats) {
            const auto &ptLeg = combinedResult.getPTLeg();
            const auto &firstTaxiLeg = combinedResult.getFirstTaxiLeg();
            KASSERT(ptLeg.getDepartureTimeAtFirstStation() < ptLeg.getArrivalTimeAtLastStation(),
                    "Journey: " << ptLeg.getBestJourney());

            // Apply first taxi leg assignment
            if (combinedResult.isInitialTransferByTaxi()) {
                KASSERT(firstTaxiLeg.isValid());
                KASSERT(ptLeg.journey.front().usesRoute);
                applyAssignment(requestState, firstTaxiLeg.getBestAssignment(), reqId, updateStats,
                                ptLeg.getDepartureTimeAtFirstStation());
            }

            if (combinedResult.isFinalTransferByTaxi()) {
                ptStationsForSecondTaxiLeg[reqId] = ptLeg.getLastStation();

                // Insert request event for second taxi leg 15 minutes before arrival
                const int schedule2ndLegTime = std::max(
                    occTime, ptLeg.getArrivalTimeAtLastStation() - TRIGGER_TAXI_TIME);
                nextRiderEvents[reqId] = SCHEDULE_2ND_TAXI_LEG;
                requestEvents.insert(reqId, schedule2ndLegTime);
            }
        }

        void handleSecondTaxiLeg(const int reqId, const int occTime) {
            KASSERT(riderState[reqId] == WAITING_FOR_PICKUP || riderState[reqId] == IN_TAXI_VEHICLE);
            KASSERT(ptStationsForSecondTaxiLeg[reqId] != INVALID_ID);

            int id, key;
            requestEvents.deleteMin(id, key);
            KASSERT(id == reqId && key == occTime);
            nextRiderEvents[reqId] = RIDER_NO_EVENT;

            const auto &originalRequest = requests[reqId];
            const auto stationEdgeId = stations[ptStationsForSecondTaxiLeg[reqId]].vehEdgeId;
            const Request &newReq = {
                reqId,
                stationEdgeId,
                originalRequest.destination,
                requestData[reqId].ptLegArrTime,
                originalRequest.numRiders
            };

            karri::stats::SecondTaxiLegStats secondTaxiLegStats;
            auto requestState = rsInitializer.initializeRequestState(newReq, occTime,
                                                                     secondTaxiLegStats.taxiPrepStats.
                                                                     initializationStats);
            auto baseInfo = karriPrep.prepareBaseInfo(requestState, secondTaxiLegStats.taxiPrepStats);
            for (auto &p: baseInfo.pdLocs.pickups)
                p.isStation = true;
            const auto secondLegResult = taxiTripFinder.findBestAssignment(
                requestState, baseInfo, secondTaxiLegStats.taxiSecondLegStats);
            // auto asgnFinderResponse = ptAndTaxiTripFinder.findBestSecondTaxiLeg(
            //     newReq, occTime, secondTaxiLegStats.taxiSecondLegStats);

            // todo: potentially allow mode choice here again, just exclude car

            // Before the second taxi leg
            // const auto &reqData = requestData[reqId];
            // const auto waitTime = reqData.depTime - requests[reqId].requestTime;
            // requestState.setCurrentWaitTime(waitTime);


            auto &reqData = requestData[reqId];
            const auto &asgn = secondLegResult.getBestAssignment();
            reqData.secondTaxiLegCost = secondLegResult.getBestCost();
            reqData.secondTaxiLegPickupWalkTime = asgn.pickup.walkingDist;
            reqData.secondTaxiLegDropoffWalkTime = asgn.dropoff.walkingDist;
            reqData.secondTaxiLegVehicleId = asgn.vehicle ? asgn.vehicle->vehicleId : INVALID_ID;

            applyAssignment(requestState, asgn, reqId, secondTaxiLegStats.updateStats);

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


            int arrTime = 0, tripTime = 0, waitTime = 0, taxiRideTime = 0, ptRideTime = 0, walkTime = 0,
                    asgnCostFirstTaxiLeg = 0, asgnCostPtLeg = 0, asgnCostSecondTaxiLeg = 0, asgnCostTotal = 0;
            if (reqData.mode == parrot::mode_choice::TransportMode::Ped || reqData.mode ==
                parrot::mode_choice::TransportMode::Car) {
                arrTime = occTime;
                tripTime = arrTime - requests[reqId].requestTime;
                walkTime = reqData.mode == parrot::mode_choice::TransportMode::Ped ? tripTime : 0;
                asgnCostTotal = INFTY;
            } else if (reqData.mode == parrot::mode_choice::TransportMode::PublicTransport) {
                KASSERT(occTime == reqData.ptLegArrTime && requests[reqId].requestTime == reqData.ptLegDepTime);
                arrTime = occTime;
                tripTime = arrTime - requests[reqId].requestTime;
                waitTime = tripTime - reqData.ptLegRideTime - reqData.ptLegWalkTime;
                ptRideTime = reqData.ptLegRideTime;
                walkTime = reqData.ptLegWalkTime;
                asgnCostPtLeg = reqData.ptLegCost;
                asgnCostTotal = reqData.ptLegCost;
            } else if (reqData.mode == parrot::mode_choice::TransportMode::Taxi) {
                arrTime = occTime;
                tripTime = arrTime - requests[reqId].requestTime;
                waitTime = reqData.firstTaxiLegDepAtPickup - requests[reqId].requestTime - reqData.
                           firstTaxiLegPickupWalkTime;
                taxiRideTime = reqData.firstTaxiLegArrAtDropoff - reqData.firstTaxiLegDepAtPickup;
                walkTime = reqData.firstTaxiLegPickupWalkTime + reqData.firstTaxiLegDropoffWalkTime;
                asgnCostFirstTaxiLeg = reqData.taxiLegCost;
                asgnCostTotal = reqData.taxiLegCost;
            } else if (reqData.mode == parrot::mode_choice::TransportMode::TaxiAndPT) {
                arrTime = occTime;
                tripTime = arrTime - requests[reqId].requestTime;
                if (reqData.hasFirstTaxiLeg) {
                    KASSERT(reqData.firstTaxiLegDepAtPickup < reqData.firstTaxiLegArrAtDropoff,
                            "Request ID = " << reqId << ", Request data = " << reqData);
                    waitTime += reqData.firstTaxiLegDepAtPickup - requests[reqId].requestTime - reqData.
                            firstTaxiLegPickupWalkTime;
                    waitTime += reqData.ptLegDepTime - (
                        reqData.firstTaxiLegArrAtDropoff + reqData.firstTaxiLegDropoffWalkTime);
                    walkTime += reqData.firstTaxiLegPickupWalkTime + reqData.firstTaxiLegDropoffWalkTime;
                    taxiRideTime += reqData.firstTaxiLegArrAtDropoff - reqData.firstTaxiLegDepAtPickup;
                }
                waitTime += reqData.ptLegArrTime - reqData.ptLegDepTime - reqData.ptLegRideTime - reqData.ptLegWalkTime;
                walkTime += reqData.ptLegWalkTime;
                ptRideTime = reqData.ptLegRideTime;
                if (reqData.hasSecondTaxiLeg) {
                    KASSERT(reqData.secondTaxiLegDepAtPickup >= reqData.ptLegArrTime,
                            "Request ID = " << reqId << ", Request data = " << reqData);
                    KASSERT(reqData.secondTaxiLegDepAtPickup < reqData.secondTaxiLegArrAtDropoff,
                            "Request ID = " << reqId << ", Request data = " << reqData);
                    waitTime += reqData.secondTaxiLegDepAtPickup - reqData.ptLegArrTime - reqData.
                            secondTaxiLegPickupWalkTime;
                    walkTime += reqData.secondTaxiLegPickupWalkTime + reqData.secondTaxiLegDropoffWalkTime;
                    taxiRideTime += reqData.secondTaxiLegArrAtDropoff - reqData.secondTaxiLegDepAtPickup;
                }
                asgnCostFirstTaxiLeg = reqData.taxiLegCost;
                asgnCostPtLeg = reqData.ptLegCost;
                asgnCostSecondTaxiLeg = reqData.secondTaxiLegCost;
                asgnCostTotal = reqData.taxiLegCost + reqData.ptLegCost + reqData.secondTaxiLegCost;
            }
            KASSERT(
                reqData.mode == parrot::mode_choice::TransportMode::Car || tripTime == waitTime + taxiRideTime +
                ptRideTime + walkTime,
                "Request ID = " << reqId << ", Request data = " << reqData << ", arrTime = " << arrTime <<
                ", tripTime = " << tripTime << ", waitTime = " << waitTime << ", taxiRideTime = " << taxiRideTime <<
                ", ptRideTime = " << ptRideTime << ", walkTime = " << walkTime);

            assignmentQualityStats << reqId << ','
                    << reqData.directOdDist << ','
                    << arrTime << ','
                    << tripTime << ','
                    << waitTime << ','
                    << taxiRideTime << ','
                    << ptRideTime << ','
                    << walkTime << ','
                    << asgnCostFirstTaxiLeg << ','
                    << asgnCostPtLeg << ','
                    << asgnCostSecondTaxiLeg << ','
                    << asgnCostTotal << '\n';
            tripStatsLogger << reqId << ','
                    << reqData.firstTaxiLegPickupWalkTime << ','
                    << reqData.firstTaxiLegDepAtPickup << ','
                    << reqData.firstTaxiLegArrAtDropoff << ','
                    << reqData.firstTaxiLegDropoffWalkTime << ','
                    << reqData.firstTaxiLegVehicleId << ','
                    << reqData.ptLegDepTime << ','
                    << reqData.ptLegArrTime << ','
                    << reqData.ptLegWalkTime << ','
                    << reqData.ptLegRideTime << ','
                    << reqData.secondTaxiLegPickupWalkTime << ','
                    << reqData.secondTaxiLegDepAtPickup << ','
                    << reqData.secondTaxiLegArrAtDropoff << ','
                    << reqData.secondTaxiLegDropoffWalkTime << ','
                    << reqData.secondTaxiLegVehicleId << '\n';


            const auto time = timer.elapsed<std::chrono::nanoseconds>();
            eventSimulationStatsLogger << occTime << ",RequestWalkingArrival," << time << '\n';
        }


        const Fleet &fleet;
        const std::vector<Request> &requests;
        RequestStateInitializerT &rsInitializer;
        KaRRiPreparatorT &karriPrep;
        WalkTripFinderT &walkTripFinder;
        CarTripFinderT &carTripFinder;
        TaxiTripFinderT &taxiTripFinder;
        PTTripFinderT &ptTripFinder;
        PTAndTaxiTripFinderT &ptAndTaxiTripFinder;
        ModeChoiceT &modeChoice;
        SystemStateUpdaterT &systemStateUpdater;
        const ScheduledStopsT &scheduledStops;
        const parrot::PTStations &stations;

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
        std::ofstream &tripStatsLogger;
        std::ofstream &legStatsLogger;
        KaRRiProgressBar progressBar;
    };
}
