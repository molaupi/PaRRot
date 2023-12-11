#pragma once

#include "../../../DataStructures/RideRAPTOR/Entities/InsertionInfo.h"
#include "Request.h"

namespace karri {

struct RideRAPTORRequest : public Request {
  int32_t vehicleId;
  int32_t directDistance;
  RIDERAPTOR::StopInsertionInfo pickupInfo;
  RIDERAPTOR::StopInsertionInfo dropoffInfo;
};

}  // namespace karri