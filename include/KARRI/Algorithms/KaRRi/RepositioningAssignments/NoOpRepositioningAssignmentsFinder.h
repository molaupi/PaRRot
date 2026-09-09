/// *******************************************************************************
/// Minimal RepositioningAssignmentsFinder facade
/// *******************************************************************************
#pragma once

#include "../RequestState/RequestState.h"
#include "../PDDistanceQueries/PDDistances.h"
#include "../Stats/PerformanceStats.h"
#include "../BaseObjects/InternalTaxiResult.h"

namespace karri {

    class NoOpRepositioningAssignmentsFinder {
    public:
        NoOpRepositioningAssignmentsFinder() = default;

        void findAssignments(const RequestState &, const PDDistances &, const PDLocs &, InternalTaxiResult &, stats::RepositioningAssignmentsPerformanceStats &) {}

        void init() {}
    };

}
