#pragma once

#include "KaRRiBaseInfo.h"
#include "ApproximateCombinedTripResult.h"

namespace parrot {
    using namespace karri;

    // No-op implementation of PTAndTaxiTripFinder that always returns an invalid result.
    // Used to evaluate algorithm without combined journeys.
    class NoOpPTAndTaxiTripFinder {
    public:
        NoOpPTAndTaxiTripFinder() = default;

        ApproximateCombinedTripResult findBestAssignment(const RequestState &,
                                                         const KaRRiBaseInfo &,
                                                         const int,
                                                         const int,
                                                         const auto &,
                                                         stats::TaxiAndPtPerformanceStats &) {
            // no op
            return ApproximateCombinedTripResult();
        }
    };
}
