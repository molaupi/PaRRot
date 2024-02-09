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
#include "../../Tools/Logging/LogManager.h"
#include "../../Tools/Timer.h"
#include "../../Tools/Workarounds.h"
#include "BaseObjects/Request.h"
#include "BaseObjects/Vehicle.h"

namespace karri {

template <typename AssignmentFinderT,
    typename SystemStateUpdaterT,
    typename ScheduledStopsT>
class SimulateOnlyRequests {

    enum VehicleState {
        OUT_OF_SERVICE,
        IDLING,
        DRIVING,
        STOPPING
    };

    enum RequestState {
        NOT_RECEIVED,
        ASSIGNED_TO_VEH,
        WALKING_TO_DEST,
        FINISHED
    };

    // Stores information about assignment and departure time of a request needed for logging on arrival of the
    // request.
    struct RequestData {
        int depTime;
        int walkingTimeToPickup;
        int walkingTimeFromDropoff;
        int assignmentCost;
    };

public:
    SimulateOnlyRequests(
        const Fleet& fleet, const std::vector<Request>& requests, const int stopTime,
        AssignmentFinderT& assignmentFinder, SystemStateUpdaterT& systemStateUpdater,
        const ScheduledStopsT& scheduledStops,
        const bool verbose = false)
        : fleet(fleet)
        , requests(requests)
        , stopTime(stopTime)
        , assignmentFinder(assignmentFinder)
        , systemStateUpdater(systemStateUpdater)
        , scheduledStops(scheduledStops)
        , requestEvents(requests.size())
        , vehicleState(fleet.size(), OUT_OF_SERVICE)
        , requestState(requests.size(), NOT_RECEIVED)
        , requestData(requests.size(), RequestData())
        , eventSimulationStatsLogger(LogManager<std::ofstream>::getLogger("eventsimulationstats.csv",
              "occurrence_time,"
              "type,"
              "running_time\n"))
        , assignmentQualityStats(LogManager<std::ofstream>::getLogger("assignmentquality.csv",
              "request_id,"
              "arr_time,"
              "wait_time,"
              "ride_time,"
              "trip_time,"
              "walk_to_pickup_time,"
              "walk_to_dropoff_time,"
              "cost\n"))
        , legStatsLogger(LogManager<std::ofstream>::getLogger("legstats.csv",
              "vehicle_id,"
              "stop_time,"
              "dep_time,"
              "arr_time,"
              "drive_time,"
              "occupancy\n"))
        , progressBar(requests.size(), verbose)
    {
        for (const auto& req : requests)
            requestEvents.insert(req.requestId, req.requestTime);
    }

    void run()
    {
        while (!requestEvents.empty()) {
            // Pop next event from either queue. Request event has precedence if at the same time as vehicle event.
            int id, occTime;

            requestEvents.min(id, occTime);
            // only receipt, no drop off or anything
            handleRequestReceipt(id, occTime);
        }
    }

    void handleRequestReceipt(const int reqId, const int occTime)
    {
        ++progressBar;
        assert(requestState[reqId] == NOT_RECEIVED);
        assert(requests[reqId].requestTime == occTime);
        Timer timer;

        const auto& request = requests[reqId];
        const auto& asgnFinderResponse = assignmentFinder.findBestAssignment(request);
        systemStateUpdater.writeBestAssignmentToLogger();

        applyAssignment(asgnFinderResponse, reqId, occTime);

        const auto time = timer.elapsed<std::chrono::nanoseconds>();
        eventSimulationStatsLogger << occTime << ",RequestReceipt," << time << '\n';
    }

    template <typename AssignmentFinderResponseT>
    void applyAssignment(const AssignmentFinderResponseT& asgnFinderResponse, const int reqId, const int occTime)
    {
        if (asgnFinderResponse.isNotUsingVehicleBest()) {
            requestState[reqId] = WALKING_TO_DEST;
            requestData[reqId].assignmentCost = asgnFinderResponse.getBestCost();
            requestData[reqId].depTime = occTime;
            requestData[reqId].walkingTimeToPickup = 0;
            requestData[reqId].walkingTimeFromDropoff = asgnFinderResponse.getNotUsingVehicleDist();
            requestEvents.increaseKey(reqId, occTime + asgnFinderResponse.getNotUsingVehicleDist());
            systemStateUpdater.writePerformanceLogs();
            return;
        }

        int id, key;
        requestEvents.deleteMin(id, key); // event for walking arrival at dest inserted at dropoff
        assert(id == reqId && key == occTime);

        const auto& bestAsgn = asgnFinderResponse.getBestAssignment();
        if (!bestAsgn.vehicle || !bestAsgn.pickup || !bestAsgn.dropoff) {
            requestState[reqId] = FINISHED;
            systemStateUpdater.writePerformanceLogs();
            return;
        }

        requestState[reqId] = ASSIGNED_TO_VEH;
        requestData[reqId].walkingTimeToPickup = bestAsgn.pickup->walkingDist;
        requestData[reqId].walkingTimeFromDropoff = bestAsgn.dropoff->walkingDist;
        requestData[reqId].assignmentCost = asgnFinderResponse.getBestCost();

        int pickupStopId, dropoffStopId;
        systemStateUpdater.insertBestAssignment(pickupStopId, dropoffStopId);
        systemStateUpdater.writePerformanceLogs();
        assert(pickupStopId >= 0 && dropoffStopId >= 0);
        // ...
    }

    const Fleet& fleet;
    const std::vector<Request>& requests;
    const int stopTime;
    AssignmentFinderT& assignmentFinder;
    SystemStateUpdaterT& systemStateUpdater;
    const ScheduledStopsT& scheduledStops;

    AddressableQuadHeap requestEvents;

    std::vector<VehicleState> vehicleState;
    std::vector<RequestState> requestState;

    std::vector<RequestData> requestData;

    std::ofstream& eventSimulationStatsLogger;
    std::ofstream& assignmentQualityStats;
    std::ofstream& legStatsLogger;
    ProgressBar progressBar;
};
}
