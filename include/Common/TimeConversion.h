#pragma once

namespace parrot {
    static constexpr int karriToULTRATime(int timeInOneTenthSeconds) {
        return timeInOneTenthSeconds / 10 + (timeInOneTenthSeconds % 10 != 0);
    }

    static constexpr int ultraToKarriTime(int timeInSeconds) {
        return timeInSeconds * 10;
    }
}