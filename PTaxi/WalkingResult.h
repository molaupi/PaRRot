#pragma once

#include "Common/Constants.h"

namespace parrot {
    struct WalkingResult {
        int walkingDist = INFTY;
        int cost = INFTY;

        bool isValid() const {
            return walkingDist != INFTY;
        }
    };
}