/// *******************************************************************************
/// MIT License
///
/// Copyright (c) 2026 Copied/Adapted
///
/// *******************************************************************************

#pragma once

#include <type_traits>
#include "../../Buckets/SortedBucketContainer.h"
#include "../../Buckets/BucketEntry.h"
#include "../../CH/CH.h"
#include "../BaseObjects/Vehicle.h"
#include "../../../Tools/Timer.h"
#include "../../../DataStructures/Containers/Subset.h"
#include "../Stats/PerformanceStats.h"

namespace karri {

    // No-op facility that satisfies interface for generating and removing buckets in SystemStateUpdater
    // (though the methods should never be called if repositioning is fully disabled).
    class NoOpRepositioningBucketsEnvironment {

    public:

        NoOpRepositioningBucketsEnvironment() = default;

        void generateRepositioningBucketEntries(const Vehicle &, const std::vector<int> &) {}

        void removeRepositioningBucketEntries(const Vehicle &, const std::vector<int> &) {}

    };

} // namespace karri
