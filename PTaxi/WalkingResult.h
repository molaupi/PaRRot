#pragma once

struct WalkingResult {
    int walkingDist = INFTY;
    int cost = INFTY;

    bool isValid() const {
        return walkingDist != INFTY;
    }
};