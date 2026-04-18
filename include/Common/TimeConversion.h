#pragma once

namespace parrot {
    static constexpr int karriToULTRATime(int timeInOneTenthSeconds) {
        return timeInOneTenthSeconds / 10;
    }

    static constexpr int ultraToKarriTime(int timeInSeconds) {
        return timeInSeconds * 10;
    }
}