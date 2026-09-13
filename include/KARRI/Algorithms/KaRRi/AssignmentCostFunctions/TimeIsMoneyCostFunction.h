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

#include <cassert>
#include <cmath>
#include <cstdint>

namespace karri {


    template<double PASSENGER_COST_SCALE = 1.0, double WALKING_COST_SCALE = 1.0, double VEHICLE_COST_SCALE = 1.0,
             double TRANSFER_COST_SCALE = 0.0, int TRANSFER_INCONVENIENCE_WEIGHT = 10>
    struct TimeIsMoneyCostFunction {

        static constexpr double PSG_WEIGHT = PASSENGER_COST_SCALE;
        static constexpr double WALK_WEIGHT = WALKING_COST_SCALE;
        static constexpr double VEH_WEIGHT = VEHICLE_COST_SCALE;

    private:

        // Since the weights are now real-valued parameters of a mathematical expression instead of
        // integers, a weight times an integer quantity is in general not an integer anymore. Rounds
        // to the nearest integer, breaking ties away from zero (matching std::lround), instead of
        // relying on the truncation of the implicit double-to-int conversion on return.
        static inline int roundToInt(const double x) {
            return static_cast<int>(std::lround(x));
        }

        // Returns the smallest non-negative integer x with x * scale >= value. Used to invert cost
        // terms of the form x * scale for pruning purposes. Since scale and value are now
        // floating-point, a plain std::ceil(value / scale) can be off by one due to floating-point
        // rounding; the result is therefore verified and, if necessary, corrected using exact
        // integer arithmetic so it is never an over-estimate (which would make pruning unsound).
        static inline int ceilDiv(const double value, const double scale) {
            assert(scale > 0);
            auto x = static_cast<int64_t>(std::ceil(value / scale));
            while (x > 0 && static_cast<double>(x - 1) * scale >= value)
                --x;
            while (static_cast<double>(x) * scale < value)
                ++x;
            return static_cast<int>(x);
        }

    public:

        template<typename RequestContext>
        static inline int calcUpperBoundTripCostDifference(const int tripTimeDifference, const RequestContext &) {
            return roundToInt(PASSENGER_COST_SCALE * tripTimeDifference);
        }

        template<typename DistanceLabel, typename RequestContext>
        static inline DistanceLabel
        calcKUpperBoundTripCostDifferences(const DistanceLabel &tripTimeDifference, const RequestContext &) {
            auto diff = tripTimeDifference;
            diff.multiplyWithScalar(PASSENGER_COST_SCALE);
            return diff;
        }

        static inline int calcUpperBoundTripViolationCostDifference(const int tripTimeDifference) {
            assert(tripTimeDifference >= 0);
            return roundToInt(PASSENGER_COST_SCALE * tripTimeDifference);
        }

        template<typename RequestContext>
        static inline int calcLowerBoundTripCostDifference(const int tripTimeDifference, const RequestContext &) {
            return roundToInt(PASSENGER_COST_SCALE * tripTimeDifference);
        }

        template<typename DistanceLabel, typename RequestContext>
        static inline DistanceLabel
        calcKLowerBoundTripCostDifferences(const DistanceLabel &tripTimeDifference, const RequestContext &) {
            auto diff = tripTimeDifference;
            diff.multiplyWithScalar(PASSENGER_COST_SCALE);
            return diff;
        }

        static inline int calcTripCost(const int tripTime) {
            const auto regularCost = roundToInt(PASSENGER_COST_SCALE * tripTime);
            return regularCost;
        }

        template<typename DistanceLabel>
        static inline DistanceLabel calcKTripCosts(const DistanceLabel &tripTime) {

            DistanceLabel regularCost = tripTime;
            regularCost.multiplyWithScalar(PASSENGER_COST_SCALE);
            return regularCost;
        }

        // Overload for call sites that pass a request context (unused, since trip cost here does not depend on it).
        template<typename DistanceLabel, typename RequestContext>
        static inline DistanceLabel calcKTripCosts(const DistanceLabel &tripTime, const RequestContext &) {
            return calcKTripCosts(tripTime);
        }

        static inline int calcWalkingCost(const int walkingDist, const int) {
            // Time is money => walking time is part of passengers trip time so do not count it again
            return roundToInt(WALKING_COST_SCALE * walkingDist);
        }

        static inline int calcWalkingCost(const int walkingDist) {
            // Time is money => walking time is part of passengers trip time so do not count it again
            return roundToInt(WALKING_COST_SCALE * walkingDist);
        }

        template<typename DistanceLabel>
        static inline DistanceLabel calcKWalkingCosts(const DistanceLabel &walkingDist, const int) {
            // Time is money => walking time is part of passengers trip time so do not count it again
            auto cost = walkingDist;
            cost.multiplyWithScalar(WALKING_COST_SCALE);
            return cost;
        }

        template<typename DistanceLabel>
        static inline DistanceLabel calcKWalkingCosts(const DistanceLabel &walkingDist) {
            // Time is money => walking time is part of passengers trip time so do not count it again
            auto cost = walkingDist;
            cost.multiplyWithScalar(WALKING_COST_SCALE);
            return cost;
        }

        template<typename RequestContext>
        static inline int calcWaitViolationCost(const int, const RequestContext &) {
            // No wait time soft constraint considered
            return 0;
            // return WAIT_TIME_VIOLATION_WEIGHT * std::max(actualDepTimeAtPickup - context.getMaxDepTimeAtPickup(), 0);
        }

        template<typename DistanceLabel, typename RequestContext>
        static inline DistanceLabel calcKWaitViolationCosts(const DistanceLabel &,
                                                            const RequestContext &) {
            // No wait time soft constraint considered
            return 0;
            // DistanceLabel violationCost = actualDepTimeAtPickup - DistanceLabel(context.getMaxDepTimeAtPickup());
            // violationCost.max(0);
            // violationCost.multiplyWithScalar(WAIT_TIME_VIOLATION_WEIGHT);
            // return violationCost;
        }

        static inline int calcUpperBoundWaitViolationCostDifference(const int diffInTimeTillDepAtPickup) {
            assert(diffInTimeTillDepAtPickup >= 0);
            return 0;
        }

        static inline int calcChangeInTripCostsOfExistingPassengers(const int addedTripTimeForExistingPassengers) {
            return roundToInt(PASSENGER_COST_SCALE * addedTripTimeForExistingPassengers);
        }

        static inline int calcUpperBoundVehicleCostDifference(const int detourDiff) {
            return calcVehicleCost(detourDiff);
        }

        static inline int calcLowerBoundVehicleCostDifference(const int detourDiff) {
            return calcVehicleCost(detourDiff);
        }

        static inline int calcVehicleCost(const int residualDetourAtEnd) {
            return roundToInt(VEHICLE_COST_SCALE * residualDetourAtEnd);
        }

        // waiting time / trip time
        static inline int calcTransferCost(const int totalTransferTime) {
            return roundToInt(TRANSFER_COST_SCALE * totalTransferTime);
        }

        static inline int calcTransferPenalty(const int numberOfTransfers) {
            return TRANSFER_INCONVENIENCE_WEIGHT * numberOfTransfers;
        }

        template<typename DistanceLabel>
        static inline DistanceLabel calcKVehicleCosts(const DistanceLabel &totalDetour) {
            auto cost = totalDetour;
            cost.multiplyWithScalar(VEHICLE_COST_SCALE);
            return cost;
        }

        // Returns the smallest distance from a pickup or to a dropoff (distance that is part of the detour)
        // s.t. the vehicle cost alone leads to a greater cost than the one given. Uses the maximum length of
        // any route leg to get a global lower bound on the detour.
        static inline int
        calcMinDistFromOrToPDLocSuchThatVehCostReachesMinCost(const int cost, const int maxLegLength) {
            if constexpr (VEHICLE_COST_SCALE == 0.0)
                return INFTY;
            else
                return ceilDiv(cost, VEHICLE_COST_SCALE) + maxLegLength;
        }

        // Returns the smallest distance from a pickup or to a dropoff (distance that is part of the detour and the trip
        // time) s.t. the vehicle cost and trip cost lead to a greater cost than the one given. Uses the maximum length of
        // any route leg to get a global lower bound on the detour.
        static inline int
        calcMinDistFromOrToPDLocSuchThatVehAndTripCostsReachMinCost(const int cost, const int maxLegLength) {
            const double c = cost + VEHICLE_COST_SCALE * maxLegLength;
            const double d = VEHICLE_COST_SCALE + PASSENGER_COST_SCALE;
            assert(d != 0);
            return ceilDiv(c, d);
        }

    };
}
