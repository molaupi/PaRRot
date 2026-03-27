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


#include "PDLocsFinder.h"
#include "../BaseObjects/Request.h"
#include "../BaseObjects/PDLocs.h"
#include "../Stats/PerformanceStats.h"
#include "../Stats/OsmRoadCategoryStats.h"
#include "../BaseObjects/Assignment.h"
#include "../InputConfig.h"
#include "../CostCalculator.h"
#include "../../../Tools/Simd/AlignedVector.h"
#include "../../../DataStructures/Containers/Subset.h"
#include "KARRI/Tools/Logging/LogManager.h"

namespace karri {

// Holds information relating to a specific request.
    struct RequestState {

        RequestState()
                : originalRequest(),
                  originalReqDirectDist(-1)
        // , currentWaitTime(0)
        {}


        // Information about current request itself
        Request originalRequest;
        int originalReqDirectDist;
        int requestIssueTime;

        // Shorthand for requestIssueTime
        int now() const {
            return requestIssueTime;
        }

        void setRequestIssueTime(const int issueTime) {
            requestIssueTime = issueTime;
        }

        // requestTime
        int earliestDeparture() const {
            return originalRequest.requestTime;
        }

        int getOriginalReqMaxTripTime() const {
            assert(originalReqDirectDist >= 0);
            return static_cast<int>(InputConfig::getInstance().alpha * static_cast<double>(originalReqDirectDist)) + InputConfig::getInstance().beta;
        }

        int getPassengerArrAtPickup(const PDLoc& pickup) const {
            return originalRequest.requestTime + pickup.walkingDist;
        }

        int getMaxArrTimeAtDropoff(const PDLoc& dropoff) const {
            return originalRequest.requestTime + getOriginalReqMaxTripTime() - dropoff.walkingDist;
        }


        int getMaxDepTimeAtPickup() const {
            // return originalRequest.requestTime + InputConfig::getInstance().maxWaitTime - currentWaitTime;
            return originalRequest.requestTime + InputConfig::getInstance().maxWaitTime ;
        }

        // void setCurrentWaitTime(const int waitTime) {
        //     currentWaitTime = waitTime;
        // }


        int getArrivalTime(const Assignment &asgn, const RouteState &routeState) const {
            using namespace time_utils;
            const int actualDepTimeAtPickup = getActualDepTimeAtPickup(asgn, *this, routeState);
            const int initialPickupDetour = calcInitialPickupDetour(asgn, actualDepTimeAtPickup, *this, routeState);
            const bool dropoffAtExistingStop = isDropoffAtExistingStop(asgn, routeState);
            return getArrTimeAtDropoff(actualDepTimeAtPickup, asgn, initialPickupDetour, dropoffAtExistingStop, routeState);
        }

        void reset() {
            originalRequest = {};
            originalReqDirectDist = INFTY;
            // currentWaitTime = 0;
        }

    // private:

        // int currentWaitTime;

    };
}