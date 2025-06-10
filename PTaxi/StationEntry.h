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

// This class represents an entry in the bucket of a station s. 
// It stores the distance from the previous stop of a vehicle to the station
// and the distance from the station to the next stop of the vehicle.
struct StationEntry {
  StationEntry() noexcept = default;

  StationEntry(const int stationId, const int vehId, const int stopIndex, 
               const int distFromStopToStation, const int distFromStationToStop) noexcept
      : stationId(stationId), 
        vehId(vehId), 
        stopIndex(stopIndex), 
        distFromStopToStation(distFromStopToStation), 
        distFromStationToStop(distFromStationToStop) {}

  constexpr bool operator==(const StationEntry& rhs) const noexcept {
    return stationId == rhs.stationId && 
           vehId == rhs.vehId && 
           stopIndex == rhs.stopIndex;
  }

  int32_t stationId = std::numeric_limits<int32_t>::max();
  int32_t vehId = std::numeric_limits<int32_t>::max();
  int32_t stopIndex = std::numeric_limits<int32_t>::max();
  int32_t distFromStopToStation;
  int32_t distFromStationToStop;
};
