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

// Holds information relating to a specific request like its pickups and dropoffs and the best known assignment.
    struct RequestState {

        RequestState()
                : originalRequest(),
                  originalReqDirectDist(-1),
                  minDirectPDDist(-1) {}


        ~RequestState() {
            auto &roadCatLogger = LogManager<std::ofstream>::getLogger(karri::stats::OsmRoadCategoryStats::LOGGER_NAME,
                                                                       "type," +
                                                                       karri::stats::OsmRoadCategoryStats::getLoggerCols());
            roadCatLogger << "all_pd_locs, " << allPDLocsRoadCatStats.getLoggerRow() << "\n";
            roadCatLogger << "chosen_pd_locs, " << chosenPDLocsRoadCatStats.getLoggerRow() << "\n";
        }


        // Information about current request itself
        Request originalRequest;
        int originalReqDirectDist;
        int minDirectPDDist;

        // Shorthand for requestTime
        int now() const {
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
            return originalRequest.requestTime + InputConfig::getInstance().maxWaitTime;
        }

        // Information about best known assignment for current request

        const Assignment &getBestAssignment() const {
            return bestAssignment;
        }

        const int &getBestCost() const {
            return bestCost;
        }

        bool isNotUsingVehicleBest() const {
            return notUsingVehicleIsBest;
        }

        const int &getNotUsingVehicleDist() const {
            return notUsingVehicleDist;
        }

        bool tryAssignmentWithKnownCost(const Assignment &asgn, const int cost) {
            if (cost < INFTY && (cost < bestCost || (cost == bestCost &&
                                    breakCostTie(asgn, bestAssignment)))) {

                bestAssignment = asgn;
                bestCost = cost;
                notUsingVehicleIsBest = false;
                notUsingVehicleDist = INFTY;
                return true;
            }
            return false;
        }

        void tryNotUsingVehicleAssignment(const int notUsingVehDist, const int travelTimeOfDestEdge) {
            const int cost = CostCalculator::calcCostForNotUsingVehicle(notUsingVehDist, travelTimeOfDestEdge, *this);
            if (cost < bestCost) {
                bestAssignment = Assignment();
                bestCost = cost;
                notUsingVehicleIsBest = true;
                notUsingVehicleDist = notUsingVehDist;
            }
        }

        stats::DispatchingPerformanceStats &stats() {
            return perfStats;
        }

        const stats::DispatchingPerformanceStats &stats() const {
            return perfStats;
        }

        stats::OsmRoadCategoryStats &allPDLocsRoadCategoryStats() {
            return allPDLocsRoadCatStats;
        }

        stats::OsmRoadCategoryStats &chosenPDLocsRoadCategoryStats() {
            return chosenPDLocsRoadCatStats;
        }

        void reset() {
            perfStats.clear();

            originalRequest = {};
            originalReqDirectDist = INFTY;
            minDirectPDDist = INFTY;

            bestAssignment = Assignment();
            bestCost = INFTY;
            notUsingVehicleIsBest = false;
            notUsingVehicleDist = INFTY;
        }

    private:

        stats::DispatchingPerformanceStats perfStats;
        stats::OsmRoadCategoryStats allPDLocsRoadCatStats;
        stats::OsmRoadCategoryStats chosenPDLocsRoadCatStats;

        // Information about best known assignment for current request
        Assignment bestAssignment;
        int bestCost;
        bool notUsingVehicleIsBest;
        int notUsingVehicleDist;
    };
}