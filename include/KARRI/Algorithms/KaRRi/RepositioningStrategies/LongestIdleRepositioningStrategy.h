/// ******************************************************************************
/// MIT License
///
/// Copyright (c) 2026 Moritz Laupichler <moritz.laupichler@kit.edu>
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

#include <random>
#include <vector>
#include "../BaseObjects/Request.h"
#include "../RouteState.h"
#include "../../../Tools/Constants.h"
#include "../../../Tools/Workarounds.h"
#include "../../../DataStructures/Queues/AddressableFIFOQueue.h"


namespace karri::RepositioningStrategies {

    // Strategy that always repositions the idle vehicle that has been idle the longest (ties broken
    // arbitrarily). Vehicles becoming idle/non-idle are tracked via notifyBecameIdle()/notifyBecameNonIdle()
    // in an addressable FIFO queue: a vehicle is appended to the back of the queue when it becomes idle, and
    // removed from wherever it is in the queue when it becomes non-idle. pickRepositioningVehicleAndTarget()
    // pops the vehicle at the front of the queue, i.e. the one that has been idle the longest.
    // Repositioning targets are chosen exactly as in RandomRepositioningStrategy: from previously seen request
    // origins, weighted by frequency.
    class LongestIdleRepositioningStrategy {

    public:
        static constexpr bool USE_DETERMINISTIC = true;

        explicit LongestIdleRepositioningStrategy(const Fleet &fleet)
            : seenOriginLocations(),
              gen(USE_DETERMINISTIC ? 0 : std::random_device{}()),
              idleQueue(static_cast<int>(fleet.size())) {}

        // Notify the strategy about an incoming request.
        // Tracks the origin location for future repositioning target selection.
        void notifyRequestIncoming(const Request &request) {
            seenOriginLocations.push_back(request.origin);
        }

        // Notify the strategy that a vehicle has become idle. Appends it to the back of the idle queue.
        void notifyBecameIdle(const int vehId) {
            idleQueue.push(vehId);
        }

        // Notify the strategy that a vehicle is no longer idle. Removes it from the idle queue, wherever it
        // currently is.
        void notifyBecameNonIdle(const int vehId) {
            idleQueue.remove(vehId);
        }

        // Pick the idle vehicle that has been idle the longest and a repositioning target location.
        // Returns a pair of (vehicle ID, target location).
        // Returns (INVALID_ID, INVALID_EDGE) if no valid choice can be made.
        std::pair<int, int> pickRepositioningVehicleAndTarget(const RouteState &routeState) {
            unused(routeState);
            if (idleQueue.empty() || seenOriginLocations.empty()) {
                return {INVALID_ID, INVALID_EDGE};
            }

            // Vehicle that has been idle the longest is at the front of the queue.
            // Since vehicle becomes non-idle, next call to notifyBecameNonIdle will also try to remove this
            // vehicle from the queue, but that's okay.
            const int vehId = idleQueue.popFront();

            // Choose a repositioning target from previously seen origins
            std::uniform_int_distribution<size_t> targetLocDis(0, seenOriginLocations.size() - 1);
            const int target = seenOriginLocations[targetLocDis(gen)];

            return {vehId, target};
        }

    private:
        // Track seen origin locations of requests for target selection
        std::vector<int> seenOriginLocations;

        // Random number generator
        std::mt19937 gen;

        // Idle vehicles in order of becoming idle, i.e. the vehicle at the front has been idle the longest.
        AddressableFIFOQueue idleQueue;
    };

}
