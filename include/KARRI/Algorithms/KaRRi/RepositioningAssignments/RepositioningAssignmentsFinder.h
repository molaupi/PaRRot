/// *******************************************************************************
/// Minimal RepositioningAssignmentsFinder facade
/// *******************************************************************************
#pragma once

#include "../RequestState/RequestState.h"
#include "../PDDistanceQueries/PDDistances.h"
#include "../Stats/PerformanceStats.h"
#include "../BaseObjects/InternalTaxiResult.h"

namespace karri {

    template<typename StrategyT>
    class RepositioningAssignmentsFinder {
    public:
        explicit RepositioningAssignmentsFinder(StrategyT &strategy) : strategy(strategy) {}

        void findAssignments(const RequestState &requestState, const PDDistances &pdDistances, const PDLocs &pdLocs,
                             InternalTaxiResult &result, stats::RepositioningAssignmentsPerformanceStats &stats) {
            strategy.tryRepositioningAssignments(requestState, pdDistances, pdLocs, result, stats);
        }

        void init() {
            strategy.init();
        }

    private:
        StrategyT &strategy;
    };

}
