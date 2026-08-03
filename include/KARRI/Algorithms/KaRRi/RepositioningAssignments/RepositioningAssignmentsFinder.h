/// *******************************************************************************
/// Minimal RepositioningAssignmentsFinder facade
/// *******************************************************************************
#pragma once

#include "../RequestState/RequestState.h"
#include "../PDDistanceQueries/PDDistances.h"
#include "../Stats/PerformanceStats.h"
#include <TaxiResult.h>

namespace karri {

    template<typename StrategyT>
    class RepositioningAssignmentsFinder {
    public:
        explicit RepositioningAssignmentsFinder(StrategyT &strategy) : strategy(strategy) {}

        void findAssignments(const RequestState &requestState, const PDDistances &pdDistances, const PDLocs &pdLocs,
                             TaxiResult &result, stats::RepositioningAssignmentsPerformanceStats &stats) {
            strategy.tryRepositioningAssignments(requestState, pdDistances, pdLocs, result, stats);
        }

        void init() {
            strategy.init();
        }

    private:
        StrategyT &strategy;
    };

}
