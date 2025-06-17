/// ******************************************************************************
/// MIT License
///
/// Copyright (c) 2020 Valentin Buchhold
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

#include <cstdint>
#include <limits>

// This class represents an entry of a relevant station for dropoff in a stop pair. 
// It stores the distance from the previous stop of a vehicle to the station
// and the distance from the station to the next stop of the vehicle.
struct StationEntry {
  StationEntry() noexcept = default;

  StationEntry(const int stationId, 
               const int distFromStopToStation, 
               const int distFromStationToStop) noexcept
      : targetId(stationId), 
        distFromStopToStation(distFromStopToStation), 
        distFromStationToStop(distFromStationToStop) {}

  constexpr bool operator==(const StationEntry& rhs) const noexcept {
    return targetId == rhs.targetId;
  }

  int32_t targetId = std::numeric_limits<int32_t>::max(); // station ID
  int32_t distFromStopToStation;
  int32_t distFromStationToStop;
};
