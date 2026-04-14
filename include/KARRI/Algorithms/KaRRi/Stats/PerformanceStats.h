/// ******************************************************************************
/// MIT License
///
/// Copyright (c) 2023 Moritz Laupichler <moritz.laupichler@kit.edu>
///
/// Permission is hereby granted, free of charge, to any person obtaining a copy
/// of this software and associated documentation files (the "Software"), to deal
/// in the Software without restriction, including without limitation the rights
/// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
/// copies of the Software, and to permit persons to whom the Software is
/// furnished to do so, subject to the following conditions:
///
/// The above copyright notice and this permission notice shall be included in all
/// copies or substantial portions of the Software.
///
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
/// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
/// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
/// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
/// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
/// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
/// SOFTWARE.
/// ******************************************************************************


#pragma once

#include <cstdint>

namespace karri::stats {
    struct InitializationPerformanceStats {
        int64_t findPDLocsInRadiusTime = 0;
        int64_t findVehicleToPdLocsDistancesTime = 0;
        int64_t notUsingVehicleTime = 0;
        int64_t computeODDistanceTime = 0;

        int64_t getTotalTime() const {
            return findPDLocsInRadiusTime + findVehicleToPdLocsDistancesTime + notUsingVehicleTime +
                   computeODDistanceTime;
        }

        void clear() {
            findPDLocsInRadiusTime = 0;
            findVehicleToPdLocsDistancesTime = 0;
            notUsingVehicleTime = 0;
            computeODDistanceTime = 0;
        }

        static constexpr auto LOGGER_NAME = "perf_initreq.csv";
        static constexpr auto LOGGER_COLS =
                "find_pd_locs_in_radius_time,"
                "find_vehicle_to_pd_locs_distances_time,"
                "not_using_veh_time,"
                "compute_od_distance_time,"
                "total_time\n";


        std::string getLoggerRow() const {
            std::stringstream ss;
            ss << findPDLocsInRadiusTime << ","
                    << findVehicleToPdLocsDistancesTime << ","
                    << notUsingVehicleTime << ","
                    << computeODDistanceTime << ","
                    << getTotalTime();
            return ss.str();
        }
    };


    struct EllipticBCHPerformanceStats {
        int64_t initializationTime = 0;
        int64_t pickupTime = 0;
        int64_t dropoffTime = 0;
        int64_t pickupNumEdgeRelaxations = 0;
        int64_t pickupNumVerticesSettled = 0;
        int64_t pickupNumEntriesScanned = 0;
        int64_t dropoffNumEdgeRelaxations = 0;
        int64_t dropoffNumVerticesSettled = 0;
        int64_t dropoffNumEntriesScanned = 0;

        int64_t getTotalTime() const {
            return initializationTime + pickupTime + dropoffTime;
        }

        void clear() {
            initializationTime = 0;
            pickupTime = 0;
            dropoffTime = 0;
            pickupNumEdgeRelaxations = 0;
            pickupNumVerticesSettled = 0;
            pickupNumEntriesScanned = 0;
            dropoffNumEdgeRelaxations = 0;
            dropoffNumVerticesSettled = 0;
            dropoffNumEntriesScanned = 0;
        }

        static constexpr auto LOGGER_NAME = "perf_ellipticbch.csv";
        static constexpr auto LOGGER_COLS =
                "initialization_time,"
                "pickup_time,"
                "dropoff_time,"
                "pickup_num_edge_relaxations,"
                "pickup_num_vertices_settled,"
                "pickup_num_entries_scanned,"
                "dropoff_num_edge_relaxations,"
                "dropoff_num_vertices_settled,"
                "dropoff_num_entries_scanned,"
                "total_time\n";


        std::string getLoggerRow() const {
            std::stringstream ss;
            ss << initializationTime << ","
                    << pickupTime << ","
                    << dropoffTime << ","
                    << pickupNumEdgeRelaxations << ","
                    << pickupNumVerticesSettled << ","
                    << pickupNumEntriesScanned << ","
                    << dropoffNumEdgeRelaxations << ","
                    << dropoffNumVerticesSettled << ","
                    << dropoffNumEntriesScanned << ","
                    << getTotalTime();
            return ss.str();
        }
    };

    struct PDDistancesPerformanceStats {
        int64_t initializationTime = 0;
        int64_t dropoffBucketEntryGenTime = 0;
        int64_t pickupBchSearchTime = 0;

        int64_t getTotalTime() const {
            return initializationTime + dropoffBucketEntryGenTime + pickupBchSearchTime;
        }

        void clear() {
            initializationTime = 0;
            dropoffBucketEntryGenTime = 0;
            pickupBchSearchTime = 0;
        }

        static constexpr auto LOGGER_NAME = "perf_pddistances.csv";
        static constexpr auto LOGGER_COLS =
                "initialization_time,"
                "dropoff_bucket_entry_gen_time,"
                "pickup_bch_search_time,"
                "total_time\n";


        std::string getLoggerRow() const {
            std::stringstream ss;
            ss << initializationTime << ","
                    << dropoffBucketEntryGenTime << ","
                    << pickupBchSearchTime << ","
                    << getTotalTime();
            return ss.str();
        }
    };

    struct FilterRelevantPdLocsPerformanceStats {
        int64_t numRelevantStopsForPickups = 0;
        int64_t numRelevantStopsForDropoffs = 0;
        int64_t filterRelevantPDLocsTime = 0;

        int64_t getTotalTime() const {
            return filterRelevantPDLocsTime;
        }

        void clear() {
            numRelevantStopsForPickups = 0;
            numRelevantStopsForDropoffs = 0;
            filterRelevantPDLocsTime = 0;
        }

        static constexpr auto LOGGER_NAME = "perf_filterrelevantpdlocs.csv";
        static constexpr auto LOGGER_COLS =
                "num_relevant_stops_for_pickups,"
                "num_relevant_stops_for_dropoffs,"
                "filter_relevant_pd_locs_time,"
                "total_time\n";

        std::string getLoggerRow() const {
            std::stringstream ss;
            ss << numRelevantStopsForPickups << ","
                    << numRelevantStopsForDropoffs << ","
                    << filterRelevantPDLocsTime << ","
                    << getTotalTime();
            return ss.str();
        }
    };

    struct OrdAssignmentsPerformanceStats {
        int64_t initializationTime = 0;

        int64_t numCandidateVehicles = 0;
        int64_t numNonPairedAssignmentsTried = 0;
        int64_t numPairedAssignmentsTried = 0;
        int64_t tryNonPairedAssignmentsTime = 0;
        int64_t tryPairedAssignmentsTime = 0;

        int64_t getTotalTime() const {
            return initializationTime + tryNonPairedAssignmentsTime +
                   tryPairedAssignmentsTime;
        }

        void clear() {
            initializationTime = 0;

            numCandidateVehicles = 0;
            numNonPairedAssignmentsTried = 0;
            numPairedAssignmentsTried = 0;
            tryNonPairedAssignmentsTime = 0;
            tryPairedAssignmentsTime = 0;
        }

        static constexpr auto LOGGER_NAME = "perf_ord.csv";
        static constexpr auto LOGGER_COLS =
                "initialization_time,"
                "num_candidate_vehicles,"
                "num_non_paired_assignments_tried,"
                "num_paired_assignments_tried,"
                "try_non_paired_assignments_time,"
                "try_paired_assignments_time,"
                "total_time\n";


        std::string getLoggerRow() const {
            std::stringstream ss;
            ss << initializationTime << ","
                    << numCandidateVehicles << ","
                    << numNonPairedAssignmentsTried << ","
                    << numPairedAssignmentsTried << ","
                    << tryNonPairedAssignmentsTime << ","
                    << tryPairedAssignmentsTime << ","
                    << getTotalTime();
            return ss.str();
        }
    };

    struct PbnsAssignmentsPerformanceStats {
        int64_t initializationTime = 0;

        int64_t locatingVehiclesTime = 0;
        int64_t numCHSearches = 0;
        int64_t directCHSearchTime = 0;

        int64_t numCandidateVehicles = 0;
        int64_t numAssignmentsTried = 0;
        int64_t tryAssignmentsTime = 0;

        int64_t getTotalTime() const {
            return initializationTime + tryAssignmentsTime + locatingVehiclesTime;
        }

        void clear() {
            initializationTime = 0;

            locatingVehiclesTime = 0;
            numCHSearches = 0;
            directCHSearchTime = 0;

            numCandidateVehicles = 0;
            numAssignmentsTried = 0;
            tryAssignmentsTime = 0;
        }

        static constexpr auto LOGGER_NAME = "perf_pbns.csv";
        static constexpr auto LOGGER_COLS =
                "initialization_time,"
                "locating_vehicles_time,"
                "num_ch_searches,"
                "direct_ch_search_time,"
                "num_candidate_vehicles,"
                "num_assignments_tried,"
                "try_assignments_time,"
                "total_time\n";


        std::string getLoggerRow() const {
            std::stringstream ss;
            ss << initializationTime << ","
                    << locatingVehiclesTime << ","
                    << numCHSearches << ","
                    << directCHSearchTime << ","
                    << numCandidateVehicles << ","
                    << numAssignmentsTried << ","
                    << tryAssignmentsTime << ","
                    << getTotalTime();
            return ss.str();
        }
    };

    struct PalsAssignmentsPerformanceStats {
        int64_t initializationTime = 0;

        int64_t numEdgeRelaxationsInSearchGraph = 0;
        int64_t numVerticesOrLabelsSettled = 0;
        int64_t numEntriesOrLastStopsScanned = 0;
        int64_t searchTime = 0;

        int64_t numCandidateVehicles = 0;
        int64_t numAssignmentsTried = 0;
        int64_t tryAssignmentsTime = 0;

        // Stats about pickup coinciding with last stop (same independent of PALS strategy):
        int64_t pickupAtLastStop_numCandidateVehicles = 0;
        int64_t pickupAtLastStop_numAssignmentsTried = 0;
        int64_t pickupAtLastStop_tryAssignmentsTime = 0;

        // Stats only relevant for collective PALS strategy:
        int64_t collective_pickupVehDistQueryTime = 0;
        int64_t collective_numPromisingDropoffs = 0;
        int64_t collective_numInitialLabelsGenerated = 0;
        int64_t collective_numInitialLabelsNotPruned = 0;
        int64_t collective_initializationTime = 0;
        int64_t collective_numDominationRelationTests = 0;
        bool collective_usedFallback = false;

        int64_t getTotalTime() const {
            return initializationTime + searchTime + tryAssignmentsTime + pickupAtLastStop_tryAssignmentsTime;
        }

        void clear() {
            initializationTime = 0;
            numEdgeRelaxationsInSearchGraph = 0;
            numVerticesOrLabelsSettled = 0;
            numEntriesOrLastStopsScanned = 0;
            searchTime = 0;
            numCandidateVehicles = 0;
            numAssignmentsTried = 0;
            tryAssignmentsTime = 0;
            pickupAtLastStop_numCandidateVehicles = 0;
            pickupAtLastStop_numAssignmentsTried = 0;
            pickupAtLastStop_tryAssignmentsTime = 0;
            collective_pickupVehDistQueryTime = 0;
            collective_numPromisingDropoffs = 0;
            collective_numInitialLabelsGenerated = 0;
            collective_numInitialLabelsNotPruned = 0;
            collective_initializationTime = 0;
            collective_numDominationRelationTests = 0;
            collective_usedFallback = false;
        }

        static constexpr auto LOGGER_NAME = "perf_pals.csv";
        static constexpr auto LOGGER_COLS =
                "initialization_time,"
                "num_edge_relaxations,"
                "num_vertices_or_labels_settled,"
                "num_entries_or_last_stops_scanned,"
                "search_time,"
                "num_candidate_vehicles,"
                "num_assignments_tried,"
                "try_assignments_time,"
                "pickup_at_last_stop.num_candidate_vehicles,"
                "pickup_at_last_stop.num_assignments_tried,"
                "pickup_at_last_stop.try_assignments_time,"
                "collective.pickup_veh_dist_query_time,"
                "collective.num_promising_dropoffs,"
                "collective.num_initial_labels_generated,"
                "collective.num_initial_labels_not_pruned,"
                "collective.initialization_time,"
                "collective.num_domination_relation_tests,"
                "collective.used_fallback,"
                "total_time\n";


        std::string getLoggerRow() const {
            std::stringstream ss;
            ss << initializationTime << ","
                    << numEdgeRelaxationsInSearchGraph << ","
                    << numVerticesOrLabelsSettled << ","
                    << numEntriesOrLastStopsScanned << ","
                    << searchTime << ","
                    << numCandidateVehicles << ","
                    << numAssignmentsTried << ","
                    << tryAssignmentsTime << ","
                    << pickupAtLastStop_numCandidateVehicles << ","
                    << pickupAtLastStop_numAssignmentsTried << ","
                    << pickupAtLastStop_tryAssignmentsTime << ","
                    << collective_pickupVehDistQueryTime << ","
                    << collective_numPromisingDropoffs << ","
                    << collective_numInitialLabelsGenerated << ","
                    << collective_numInitialLabelsNotPruned << ","
                    << collective_initializationTime << ","
                    << collective_numDominationRelationTests << ","
                    << collective_usedFallback << ","
                    << getTotalTime();
            return ss.str();
        }
    };

    struct DalsAssignmentsPerformanceStats {
        int64_t initializationTime = 0;

        int64_t numEdgeRelaxationsInSearchGraph = 0;
        int64_t numVerticesOrLabelsSettled = 0;
        int64_t numEntriesOrLastStopsScanned = 0;
        int64_t searchTime = 0;

        int64_t numCandidateVehicles = 0;
        int64_t numCandidateDropoffsAcrossAllVehicles = 0;
        int64_t numAssignmentsTried = 0;
        int64_t tryAssignmentsTime = 0;

        // Stats only relevant for collective DALS strategy:
        int64_t collective_numDominationRelationTests = 0;
        bool collective_ranClosestDropoffSearch = false;
        int64_t collective_numDirectCHSearches = 0;
        int64_t collective_initializationTime = 0;

        int64_t getTotalTime() const {
            return initializationTime + searchTime + tryAssignmentsTime;
        }

        void clear() {
            initializationTime = 0;
            numEdgeRelaxationsInSearchGraph = 0;
            numVerticesOrLabelsSettled = 0;
            numEntriesOrLastStopsScanned = 0;
            searchTime = 0;
            numCandidateVehicles = 0;
            numCandidateDropoffsAcrossAllVehicles = 0;
            numAssignmentsTried = 0;
            tryAssignmentsTime = 0;
            collective_numDominationRelationTests = 0;
            collective_ranClosestDropoffSearch = false;
            collective_numDirectCHSearches = 0;
            collective_initializationTime = 0;
        }


        static constexpr auto LOGGER_NAME = "perf_dals.csv";
        static constexpr auto LOGGER_COLS =
                "initialization_time,"
                "num_edge_relaxations,"
                "num_vertices_or_labels_settled,"
                "num_entries_or_last_stops_scanned,"
                "search_time,"
                "num_candidate_vehicles,"
                "num_candidate_dropoffs,"
                "num_assignments_tried,"
                "try_assignments_time,"
                "collective.num_domination_relation_tests,"
                "collective.ran_closest_dropoff_search,"
                "collective.num_direct_ch_searches,"
                "collective.initialization_time,"
                "total_time\n";


        std::string getLoggerRow() const {
            std::stringstream ss;
            ss << initializationTime << ","
                    << numEdgeRelaxationsInSearchGraph << ","
                    << numVerticesOrLabelsSettled << ","
                    << numEntriesOrLastStopsScanned << ","
                    << searchTime << ","
                    << numCandidateVehicles << ","
                    << numCandidateDropoffsAcrossAllVehicles << ","
                    << numAssignmentsTried << ","
                    << tryAssignmentsTime << ","
                    << collective_numDominationRelationTests << ","
                    << collective_ranClosestDropoffSearch << ","
                    << collective_numDirectCHSearches << ","
                    << collective_initializationTime << ","
                    << getTotalTime();
            return ss.str();
        }
    };


    struct StationBchPerformanceStats {
        int64_t pickupInitializationTime = 0;
        int64_t pickupBchSearchTime = 0;
        int64_t pickupNumEdgeRelaxations = 0;
        int64_t pickupNumVerticesSettled = 0;
        int64_t pickupNumEntriesScanned = 0;
        int64_t pickupNumStationsSeen = 0;

        int64_t destInitializationTime = 0;
        int64_t destBchSearchTime = 0;
        int64_t destNumEdgeRelaxations = 0;
        int64_t destNumVerticesSettled = 0;
        int64_t destNumEntriesScanned = 0;
        int64_t destNumStationsSeen = 0;


        int64_t getTotalTime() const {
            return pickupInitializationTime + pickupBchSearchTime + destInitializationTime + destBchSearchTime;
        }

        void clear() {
            pickupInitializationTime = 0;
            pickupBchSearchTime = 0;
            pickupNumEdgeRelaxations = 0;
            pickupNumVerticesSettled = 0;
            pickupNumEntriesScanned = 0;
            pickupNumStationsSeen = 0;
            destInitializationTime = 0;
            destBchSearchTime = 0;
            destNumEdgeRelaxations = 0;
            destNumVerticesSettled = 0;
            destNumEntriesScanned = 0;
            destNumStationsSeen = 0;
        }

        static constexpr auto LOGGER_NAME = "perf_stationbch.csv";
        static constexpr auto LOGGER_COLS =
                "pickup_initialization_time,"
                "pickup_bch_search_time,"
                "pickup_num_edge_relaxations,"
                "pickup_num_vertices_settled,"
                "pickup_num_entries_scanned,"
                "pickup_num_stations_seen,"
                "dest_initialization_time,"
                "dest_bch_search_time,"
                "dest_num_edge_relaxations,"
                "dest_num_vertices_settled,"
                "dest_num_entries_scanned,"
                "dest_num_stations_seen,"
                "total_time\n";


        std::string getLoggerRow() const {
            std::stringstream ss;
            ss << pickupInitializationTime << ","
                    << pickupBchSearchTime << ","
                    << pickupNumEdgeRelaxations << ","
                    << pickupNumVerticesSettled << ","
                    << pickupNumEntriesScanned << ","
                    << pickupNumStationsSeen << ","
                    << destInitializationTime << ","
                    << destBchSearchTime << ","
                    << destNumEdgeRelaxations << ","
                    << destNumVerticesSettled << ","
                    << destNumEntriesScanned << ","
                    << destNumStationsSeen << ","
                    << getTotalTime();
            return ss.str();
        }
    };

    struct TaxiPrepStats {
        int32_t numPickups = 0;
        int32_t numDropoffs = 0;
        InitializationPerformanceStats initializationStats{};
        EllipticBCHPerformanceStats ellipticBchStats{};
        PDDistancesPerformanceStats pdDistancesStats{};
        FilterRelevantPdLocsPerformanceStats filterOrdinaryPdLocsStats{};
        FilterRelevantPdLocsPerformanceStats filterBnsPdLocsStats{};

        int64_t getTotalTime() const {
            return initializationStats.getTotalTime() +
                   ellipticBchStats.getTotalTime() +
                   pdDistancesStats.getTotalTime() +
                   filterOrdinaryPdLocsStats.getTotalTime() +
                   filterBnsPdLocsStats.getTotalTime();
        }

        void clear() {
            numPickups = 0;
            numDropoffs = 0;
            initializationStats.clear();
            ellipticBchStats.clear();
            pdDistancesStats.clear();
            filterOrdinaryPdLocsStats.clear();
            filterBnsPdLocsStats.clear();
        }

        static constexpr auto LOGGER_NAME = "perf_taxi_prep.csv";
        static constexpr auto LOGGER_COLS =
                "num_pickups,"
                "num_dropoffs,"
                "initialization_time,"
                "elliptic_bch_time,"
                "pd_distances_time,"
                "filter_ordinary_pd_locs_time,"
                "filter_bns_pd_locs_time,"
                "total_time\n";

        std::string getLoggerRow() const {
            std::stringstream ss;
            ss << numPickups << ","
                    << numDropoffs << ","
                    << initializationStats.getTotalTime() << ","
                    << ellipticBchStats.getTotalTime() << ","
                    << pdDistancesStats.getTotalTime() << ","
                    << filterOrdinaryPdLocsStats.getTotalTime() << ","
                    << filterBnsPdLocsStats.getTotalTime() << ","
                    << getTotalTime();
            return ss.str();
        }
    };

    struct UpdatePerformanceStats {
        int64_t elliptic_generate_numVerticesInSearchSpace = 0;
        int64_t elliptic_generate_numEntriesInserted = 0;
        int64_t elliptic_generate_time = 0;

        int64_t elliptic_update_numVerticesVisited = 0;
        int64_t elliptic_update_numEntriesScanned = 0;
        int64_t elliptic_update_time = 0;

        int64_t elliptic_delete_numVerticesVisited = 0;
        int64_t elliptic_delete_numEntriesScanned = 0;
        int64_t elliptic_delete_time = 0;

        int64_t stationsInEllipse_generate_time = 0;
        int64_t stationsInEllipse_update_time = 0;
        int64_t stationsInEllipse_remove_time = 0;

        int64_t lastStopBucketsGenerateEntriesTime = 0;
        int64_t lastStopBucketsUpdateEntriesTime = 0;
        int64_t lastStopBucketsDeleteEntriesTime = 0;

        int64_t lastStopsAtVerticesUpdateTime = 0;

        int64_t updateRoutesTime = 0;

        int64_t getTotalTime() const {
            return elliptic_generate_time + elliptic_update_time +  elliptic_delete_time +
                stationsInEllipse_generate_time + stationsInEllipse_update_time + stationsInEllipse_remove_time +
                 lastStopBucketsGenerateEntriesTime +
                   lastStopBucketsDeleteEntriesTime + lastStopsAtVerticesUpdateTime + updateRoutesTime;
        }

        void clear() {
            elliptic_generate_numVerticesInSearchSpace = 0;
            elliptic_generate_numEntriesInserted = 0;
            elliptic_generate_time = 0;
            elliptic_update_numVerticesVisited = 0;
            elliptic_update_numEntriesScanned = 0;
            elliptic_update_time = 0;
            elliptic_delete_numVerticesVisited = 0;
            elliptic_delete_numEntriesScanned = 0;
            elliptic_delete_time = 0;
            stationsInEllipse_generate_time = 0;
            stationsInEllipse_update_time = 0;
            stationsInEllipse_remove_time = 0;
            lastStopBucketsGenerateEntriesTime = 0;
            lastStopBucketsDeleteEntriesTime = 0;
            lastStopsAtVerticesUpdateTime = 0;
            updateRoutesTime = 0;
        }

        static constexpr auto LOGGER_NAME = "perf_update.csv";
        static constexpr auto LOGGER_COLS =
                "elliptic.generate.numVerticesInSearchSpace,"
                "elliptic.generate.numEntriesInserted,"
                "elliptic.generate.time,"
                "elliptic.update.numVerticesVisited,"
                "elliptic.update.numEntriesScanned,"
                "elliptic.update.time,"
                "elliptic.delete.numVerticesVisited,"
                "elliptic.delete.numEntriesScanned,"
                "elliptic.delete.time,"
        "stations_in_ellipse_generate_time,"
        "stations_in_ellipse_update_time,"
        "stations_in_ellipse_remove_time,"
                "last_stop_buckets_generate_entries_time,"
                "last_stop_buckets_delete_entries_time,"
                "last_stop_at_vertices_update_time,"
                "update_routes_time,"
                "total_time\n";


        std::string getLoggerRow() const {
            std::stringstream ss;
            ss << elliptic_generate_numVerticesInSearchSpace << ","
                    << elliptic_generate_numEntriesInserted << ","
                    << elliptic_generate_time << ","
                    << elliptic_update_numVerticesVisited << ","
                    << elliptic_update_numEntriesScanned << ","
                    << elliptic_update_time << ","
                    << elliptic_delete_numVerticesVisited << ","
                    << elliptic_delete_numEntriesScanned << ","
                    << elliptic_delete_time << ","
                    << stationsInEllipse_generate_time << ","
                    << stationsInEllipse_update_time << ","
                    << stationsInEllipse_remove_time << ","
                    << lastStopBucketsGenerateEntriesTime << ","
                    << lastStopBucketsDeleteEntriesTime << ","
                    << lastStopsAtVerticesUpdateTime << ","
                    << updateRoutesTime << ","
                    << getTotalTime();
            return ss.str();
        }
    };

    struct WalkPerformanceStats {
        int64_t time = 0;

        int64_t getTotalTime() const {
            return time;
        }

        void clear() {
            time = 0;
        }

        static constexpr auto LOGGER_NAME = "perf_walk.csv";
        static constexpr auto LOGGER_COLS =
                "time,"
                "total_time\n";

        std::string getLoggerRow() const {
            std::stringstream ss;
            ss << time << ","
                    << getTotalTime();
            return ss.str();
        }
    };

    // Dummy car performance stats
    struct CarPerformanceStats {
        int64_t time = 0;

        int64_t getTotalTime() const {
            return time;
        }

        void clear() {
            time = 0;
        }

        static constexpr auto LOGGER_NAME = "perf_car.csv";
        static constexpr auto LOGGER_COLS =
                "time,"
                "total_time\n";

        std::string getLoggerRow() const {
            std::stringstream ss;
            ss << time << ","
                    << getTotalTime();
            return ss.str();
        }
    };

    struct TaxiPerformanceStats {
        OrdAssignmentsPerformanceStats ordAssignmentsStats{};
        PbnsAssignmentsPerformanceStats pbnsAssignmentsStats{};
        PalsAssignmentsPerformanceStats palsAssignmentsStats{};
        DalsAssignmentsPerformanceStats dalsAssignmentsStats{};

        int64_t getTotalTime() const {
            return ordAssignmentsStats.getTotalTime() +
                   pbnsAssignmentsStats.getTotalTime() +
                   palsAssignmentsStats.getTotalTime() +
                   dalsAssignmentsStats.getTotalTime();
        }

        void clear() {
            ordAssignmentsStats.clear();
            pbnsAssignmentsStats.clear();
            palsAssignmentsStats.clear();
            dalsAssignmentsStats.clear();
        }

        static constexpr auto LOGGER_NAME = "perf_overall.csv";
        static constexpr auto LOGGER_COLS =
                "ord_assignments_time,"
                "pbns_assignments_time,"
                "pals_assignments_time,"
                "dals_assignments_time,"
                "total_time\n";


        std::string getLoggerRow() const {
            std::stringstream ss;
            ss << ordAssignmentsStats.getTotalTime() << ","
                    << pbnsAssignmentsStats.getTotalTime() << ","
                    << palsAssignmentsStats.getTotalTime() << ","
                    << dalsAssignmentsStats.getTotalTime() << ","
                    << getTotalTime();
            return ss.str();
        }
    };

    struct PtPerformanceStats {
        int64_t roundInitializationTime = 0;
        int64_t collectRoutesTime = 0;
        int64_t scanRoutesTime = 0;
        int64_t relaxInitialTransfersTime = 0;
        int64_t relaxIntermediateTransfersTime = 0;

        int64_t numRounds = 0;
        int64_t numRoutesScanned = 0;
        int64_t numRouteSegmentsScanned = 0;
        int64_t numTransferEdgesRelaxed = 0;
        int64_t numStopsImprovedByTrip = 0;
        int64_t numStopsImprovedByTransfer = 0;

        int64_t getTotalTime() const {
            return roundInitializationTime + collectRoutesTime + scanRoutesTime + relaxInitialTransfersTime +
                   relaxIntermediateTransfersTime;
        }

        void clear() {
            roundInitializationTime = 0;
            collectRoutesTime = 0;
            scanRoutesTime = 0;
            relaxInitialTransfersTime = 0;
            relaxIntermediateTransfersTime = 0;

            numRounds = 0;
            numRoutesScanned = 0;
            numRouteSegmentsScanned = 0;
            numTransferEdgesRelaxed = 0;
            numStopsImprovedByTrip = 0;
            numStopsImprovedByTransfer = 0;
        }

        static constexpr auto LOGGER_NAME = "perf_pt.csv";
        static constexpr auto LOGGER_COLS =
                "round_initialization_time,"
                "collect_routes_time,"
                "scan_routes_time,"
                "relax_initial_transfers_time,"
                "relax_intermediate_transfers_time,"
                "num_rounds,"
                "num_routes_scanned,"
                "num_route_segments_scanned,"
                "num_transfer_edges_relaxed,"
                "num_stops_improved_by_trip,"
                "num_stops_improved_by_transfer,"
                "total_time\n";

        std::string getLoggerRow() const {
            std::stringstream ss;
            ss << roundInitializationTime << ","
                    << collectRoutesTime << ","
                    << scanRoutesTime << ","
                    << relaxInitialTransfersTime << ","
                    << relaxIntermediateTransfersTime << ","
                    << numRounds << ","
                    << numRoutesScanned << ","
                    << numRouteSegmentsScanned << ","
                    << numTransferEdgesRelaxed << ","
                    << numStopsImprovedByTrip << ","
                    << numStopsImprovedByTransfer << ","
                    << getTotalTime();
            return ss.str();
        }
    };

    struct TaxiAndPtPerformanceStats {
        StationBchPerformanceStats stationBchStats{};
        TaxiPerformanceStats taxiFirstLegStats{};
        PtPerformanceStats ptWithTaxiStats{};

        int64_t getTotalTime() const {
            return stationBchStats.getTotalTime() +
                   taxiFirstLegStats.getTotalTime() +
                   ptWithTaxiStats.getTotalTime();
        }

        void clear() {
            stationBchStats.clear();
            taxiFirstLegStats.clear();
            ptWithTaxiStats.clear();
        }

        static constexpr auto LOGGER_NAME = "perf_taxi_and_pt.csv";
        static constexpr auto LOGGER_COLS =
                "station_bch_time,"
                "taxi_first_leg_time,"
                "pt_with_taxi_time,"
                "total_time\n";

        std::string getLoggerRow() const {
            std::stringstream ss;
            ss << stationBchStats.getTotalTime() << ","
                    << taxiFirstLegStats.getTotalTime() << ","
                    << ptWithTaxiStats.getTotalTime() << ","
                    << getTotalTime();
            return ss.str();
        }
    };

    struct RequestReceiveStats {
        WalkPerformanceStats walkOnlyStats{};
        CarPerformanceStats carOnlyStats{};
        TaxiPrepStats taxiPrepStats{};
        TaxiPerformanceStats taxiOnlyStats{};
        PtPerformanceStats ptOnlyStats{};
        TaxiAndPtPerformanceStats taxiAndPtPerformanceStats{};
        UpdatePerformanceStats updateStats{};


        int64_t getTotalTime() const {
            return walkOnlyStats.getTotalTime() +
                   carOnlyStats.getTotalTime() +
                   taxiPrepStats.getTotalTime() +
                   taxiOnlyStats.getTotalTime() +
                   ptOnlyStats.getTotalTime() +
                   taxiAndPtPerformanceStats.getTotalTime() +
                   updateStats.getTotalTime();
        }

        void clear() {
            walkOnlyStats.clear();
            carOnlyStats.clear();
            taxiPrepStats.clear();
            taxiOnlyStats.clear();
            ptOnlyStats.clear();
            taxiAndPtPerformanceStats.clear();
            updateStats.clear();
        }

        static constexpr auto LOGGER_NAME = "perf_request_receive.csv";
        static constexpr auto LOGGER_COLS =
                "walk_only_time,"
                "car_only_time,"
                "taxi_prep_time,"
                "taxi_only_time,"
                "pt_only_time,"
                "taxi_and_pt_time,"
                "update_time,"
                "total_time\n";

        std::string getLoggerRow() const {
            std::stringstream ss;
            ss << walkOnlyStats.getTotalTime() << ","
                    << carOnlyStats.getTotalTime() << ","
                    << taxiPrepStats.getTotalTime() << ","
                    << taxiOnlyStats.getTotalTime() << ","
                    << ptOnlyStats.getTotalTime() << ","
                    << taxiAndPtPerformanceStats.getTotalTime() << ","
                    << updateStats.getTotalTime() << ","
                    << getTotalTime();
            return ss.str();
        }
    };

    struct SecondTaxiLegStats {
        TaxiPrepStats taxiPrepStats{};
        TaxiPerformanceStats taxiSecondLegStats{};
        UpdatePerformanceStats updateStats{};

        int64_t getTotalTime() const {
            return taxiPrepStats.getTotalTime() + taxiSecondLegStats.getTotalTime() +
                   updateStats.getTotalTime();
        }

        void clear() {
            taxiPrepStats.clear();
            taxiSecondLegStats.clear();
            updateStats.clear();
        }

        static constexpr auto LOGGER_NAME = "perf_second_taxi_leg.csv";
        static constexpr auto LOGGER_COLS =
                "taxi_prep_time,"
                "taxi_second_leg_time,"
                "update_time,"
                "total_time\n";

        std::string getLoggerRow() const {
            std::stringstream ss;
            ss << taxiPrepStats.getTotalTime() << ","
                    << taxiSecondLegStats.getTotalTime() << ","
                    << updateStats.getTotalTime() << ","
                    << getTotalTime();
            return ss.str();
        }
    };
}
