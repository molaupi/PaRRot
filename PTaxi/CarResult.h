#pragma once

namespace parrot {
    struct CarResult {
        int carDist = INFTY;

        bool isValid() const {
            return carDist != INFTY;
        }
    };
}
