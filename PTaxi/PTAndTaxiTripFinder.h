#pragma once

#include "Station/Station.h"
#include "Stats/IntermediateResultStats.h"
#include <KARRI/Algorithms/CH/CH.h>
#include <KARRI/Algorithms/KaRRi/RequestState/RelevantPDLocs.h>

#include "KaRRiBaseInfo.h"
#include "ApproximateCombinedTripResult.h"
#include "BestPTJourneyChoice.h"
#include "FirstTaxiLeg/StationsAtLocations.h"

namespace karri {
    // Core of the PTaxi algorithm: Given a ride request r, this facility finds the optimal assignment of r to the route
    // of a vehicle and a pickup and dropoff location, according to the current state of all vehicle routes.
    template<
        typename VehicleInputGraphT,
        typename VehCHEnvT,
        typename StationBucketsEnvT,
        typename StationDistanceFinderT,
        typename PALSToStationsT,
        typename StationsInEllipseT,
        typename OrdinaryToStationsT,
        typename DALSToStationsT,
        typename PBNSToStationsT,
        typename PTQueryT,
        typename PTAlgorithmWithTaxiT,
        typename TaxiLegApproximationT
    >
    class PTAndTaxiTripFinder {
    public:
        PTAndTaxiTripFinder(
            const VehicleInputGraphT &vehInputGraph,
            const VehCHEnvT &vehChEnv,
            const Fleet &fleet,
            RouteState &routeState,
            const PTStations &stations,
            const std::vector<PTQueryT> &queries,
            StationBucketsEnvT &stationBucketsEnv,
            StationsAtLocations &stationsAtLocations,
            PALSToStationsT &palsToStations,
            StationsInEllipseT &stationsInEllipse,
            DALSToStationsT &dalsToStations,
            PBNSToStationsT &pbnsToStations,
            PTAlgorithmWithTaxiT &ptAlgorithmWithTaxi)
            : vehInputGraph(vehInputGraph),
              vehCh(vehChEnv.getCH()),
              routeState(routeState),
              fleet(fleet),
              stations(stations),
              queries(queries),
              stationBucketsEnv(stationBucketsEnv),
              stationDistanceFinder(vehInputGraph, vehChEnv, routeState, stationBucketsEnv, stations, stationsAtLocations),
              palsToStations(palsToStations),
              stationsInEllipse(stationsInEllipse),
              ordinaryToStations(fleet, routeState),
              dalsToStations(dalsToStations),
              pbnsToStations(pbnsToStations),
              ptAlgorithmWithTaxi(ptAlgorithmWithTaxi),
              taxiLegApproximation(vehInputGraph, vehChEnv, stationBucketsEnv, stations.size()),
              intermediateLogger(LogManager<std::ofstream>::getLogger(stats::IntermediateResultStats::LOGGER_NAME,
                                                                      "request_id,"
                                                                      "request_time," +
                                                                      std::string(
                                                                          stats::IntermediateResultStats::LOGGER_COLS))),
              firstTaxiLegLogger(LogManager<std::ofstream>::getLogger(stats::FirstTaxiLegResultStats::LOGGER_NAME,
                                                                      "request_id," +
                                                                      std::string(
                                                                          stats::FirstTaxiLegResultStats::LOGGER_COLS))),
              ptLogger(LogManager<std::ofstream>::getLogger(stats::PTResultStats::LOGGER_NAME,
                                                            "request_id," +
                                                            std::string(stats::PTResultStats::LOGGER_COLS))) {
        }

        ApproximateCombinedTripResult findBestAssignment(const RequestState &requestState,
                                                         const KaRRiBaseInfo &baseInfo,
                                                         const int taxiOnlyCost,
                                                         stats::TaxiAndPtPerformanceStats &stats) {
            const Request &req = requestState.originalRequest;
            const auto query = queries[req.requestId];
            const int originPsgEdge = query.originPsgEdge;
            const int originVehEdge = query.originVehEdge;
            const int destPsgEdge = query.destinationPsgEdge;
            const int destVehEdge = query.destinationVehEdge;

            const FirstTaxiLegResult &firstTaxiLeg = runFirstTaxiSharingLeg(
                requestState, baseInfo.pdLocs, baseInfo.relOrdinaryPickups, baseInfo.relPickupsBeforeNextStop,
                taxiOnlyCost, stats.stationBchStats, stats.taxiFirstLegStats);

            const int maxTripTime = requestState.getOriginalReqMaxTripTime();
            taxiLegApproximation.findDistancesFromStationsToDest(req.destination, maxTripTime,
                                                                 stats.stationBchStats);
            const auto &distFromStations = taxiLegApproximation.getDistancesFromStations();

            ptAlgorithmWithTaxi.runWithTaxi(originPsgEdge, originVehEdge, destPsgEdge, destVehEdge,
                                            query.departureTime, firstTaxiLeg, distFromStations, stats.ptWithTaxiStats);
            auto ptLegParetoFront = ptAlgorithmWithTaxi.getJourneys();

            auto journey = chooseBestJourney(ptLegParetoFront, maxTripTime);
            const bool firstLegByTaxi = !journey.empty() && journey.front().usesTaxi;
            const bool lastLegByTaxi = !journey.empty() && journey.back().usesTaxi;
            if (lastLegByTaxi) {
                // remove taxi leg in end
                journey.pop_back();
            }
            if (firstLegByTaxi) {
                // remove taxi leg in beginning
                journey.erase(journey.begin());
            }
            PTResult ptLegResponse(journey, maxTripTime);

            // size_t ptLegCount = ptLegResponse.getBestJourney().size();

            // first taxi leg + PT journey + 2nd taxi leg approximation
            ApproximateCombinedTripResult intermediateResult(req.requestTime,
                                                             maxTripTime,
                                                             firstLegByTaxi,
                                                             lastLegByTaxi,
                                                             firstTaxiLeg,
                                                             ptLegResponse,
                                                             taxiLegApproximation.getCostForStation(
                                                                 ptLegResponse.getLastStation()),
                                                             taxiLegApproximation.getDistanceFromStation(
                                                                 ptLegResponse.getLastStation())
            );


            constexpr const char *InsertionTypes[] = {"PALS", "DALS", "DALS_PBNS", "ORDINARY", "PBNS", "UNDEFINED"};
            // LOGS: Cost of taxi, PT, combined; arrivalTimes

            intermediateLogger << req.requestId << ", "
                    << req.requestTime << ", "
                    << intermediateResult.getBestCost() << ", "
                    << intermediateResult.getFirstTaxiLegCost() << ", "
                    << intermediateResult.getPTLegCost() << ", "
                    << intermediateResult.getSecondTaxiLegCost() << ", "
                    << InsertionTypes[intermediateResult.getFirstTaxiLegInsertionType()] << ", "
                    << intermediateResult.getArrivalTime() << "\n";

            const auto firstTaxiLegResults = firstTaxiLeg.getValidResults();
            const size_t validResultsCount = firstTaxiLegResults.size();

            int sumCost = 0, sumArrivalTime = 0;
            int insertionTypeCounts[] = {0, 0, 0, 0, 0, 0}; // PALS, DALS, DALS_PBNS, ORDINARY, PBNS, UNDEFINED

            for (const auto &result: firstTaxiLegResults) {
                sumCost += result.bestCost;
                sumArrivalTime += result.arrivalTime;
                ++insertionTypeCounts[result.insertionType];
            }

            const double averageCost = firstTaxiLegResults.empty()
                                           ? 0.0
                                           : static_cast<double>(sumCost) / static_cast<double>(validResultsCount);
            const double averageArrivalTime =
                    firstTaxiLegResults.empty()
                        ? 0.0
                        : static_cast<double>(sumArrivalTime) / static_cast<double>(validResultsCount);

            firstTaxiLegLogger << req.requestId << ", "
                    << averageCost << ", "
                    << averageArrivalTime << ", "
                    << insertionTypeCounts[0] << ", "
                    << insertionTypeCounts[1] << ", "
                    << insertionTypeCounts[2] << ", "
                    << insertionTypeCounts[3] << ", "
                    << insertionTypeCounts[4] << ", "
                    << insertionTypeCounts[5] << ", "
                    << validResultsCount << "\n";

            return intermediateResult;
        }

    private:
        FirstTaxiLegResult runFirstTaxiSharingLeg(const RequestState &rs, const PDLocs &pdLocs,
                                                  const RelevantPDLocs &relOrdinaryPickups,
                                                  const RelevantPDLocs &relPickupsBeforeNextStop,
                                                  const int taxiOnlyCost,
                                                  stats::StationBchPerformanceStats &stationBchStats,
                                                  stats::TaxiPerformanceStats &stats) {
            FirstTaxiLegResult firstTaxiLegResult(routeState, rs, stations.size(), taxiOnlyCost);

            runStationBCH(rs, pdLocs, taxiOnlyCost, stationBchStats);
            runPALS(rs, pdLocs, taxiOnlyCost, stats.palsAssignmentsStats, firstTaxiLegResult);
            runOrdinary(rs, pdLocs, relOrdinaryPickups, stats.ordAssignmentsStats, firstTaxiLegResult);
            runDALS(rs, pdLocs, relOrdinaryPickups, relPickupsBeforeNextStop, taxiOnlyCost,
                    stats.dalsAssignmentsStats, firstTaxiLegResult);
            runPBNS(rs, pdLocs, relPickupsBeforeNextStop, taxiOnlyCost, stats.pbnsAssignmentsStats, firstTaxiLegResult);

            // -> assignment with best cost for each PT station
            return firstTaxiLegResult;
        }

        void runStationBCH(const RequestState &rs, const PDLocs &pdLocs, const int taxiOnlyCost,
                           stats::StationBchPerformanceStats &stats) {
            // Run BCH queries from origin to all stations reachable pickups from origin from KaRRi
            stationDistanceFinder.setExternalCostUpperBound(taxiOnlyCost);
            stationDistanceFinder.run(rs, pdLocs, stats);
        }

        void runPALS(const RequestState &rs, const PDLocs &pdLocs,
                     const int taxiOnlyCost,
                     stats::PalsAssignmentsPerformanceStats &stats,
                     FirstTaxiLegResult &firstTaxiLegResult) {
            // last stop -> pickups
            // PALS Individual BCH
            palsToStations.setExternalCostUpperBound(taxiOnlyCost, firstTaxiLegResult.getWorstCostForAllStations());
            palsToStations.tryPickupAfterLastStop(rs, pdLocs, stationDistanceFinder.getDistancesToStations(),
                                                  stationDistanceFinder.getStationsSeen(), stations, stats, firstTaxiLegResult);
        }

        void runOrdinary(const RequestState &rs, const PDLocs &pdLocs,
                         const RelevantPDLocs &relOrdinaryPickpus, stats::OrdAssignmentsPerformanceStats &stats,
                         FirstTaxiLegResult &firstTaxiLegResult) {
            ordinaryToStations.enumerateAssignments(rs, pdLocs, relOrdinaryPickpus, stations, stationsInEllipse,
                                                    stationDistanceFinder.getDistancesToStations(), stats, firstTaxiLegResult);
        }

        void runDALS(const RequestState &rs, const PDLocs &pdLocs,
                     const RelevantPDLocs &relOrdinaryPickups, const RelevantPDLocs &relPickupsBeforeNextStop,
                     const int taxiOnlyCost,
                     stats::DalsAssignmentsPerformanceStats &stats,
                     FirstTaxiLegResult &firstTaxiLegResult) {
            dalsToStations.setExternalCostUpperBound(taxiOnlyCost, firstTaxiLegResult.getWorstCostForAllStations());
            dalsToStations.tryDropoffAfterLastStop(rs, pdLocs, relOrdinaryPickups, relPickupsBeforeNextStop, stats,
                                                   firstTaxiLegResult);
        }

        void runPBNS(const RequestState &rs, const PDLocs &pdLocs,
                     const RelevantPDLocs &relPickupsBns,
                     const int taxiOnlyCost,
                     stats::PbnsAssignmentsPerformanceStats &stats,
                     FirstTaxiLegResult &firstTaxiLegResult) {
            pbnsToStations.setExternalCostUpperBound(taxiOnlyCost, firstTaxiLegResult.getWorstCostForAllStations());
            pbnsToStations.findAssignments(rs, pdLocs, relPickupsBns, stations, stationsInEllipse,
                                           stationDistanceFinder.getDistancesToStations(), stats, firstTaxiLegResult);
        }

        const VehicleInputGraphT &vehInputGraph;
        const CH &vehCh;

        const Fleet &fleet;
        RouteState &routeState;

        const PTStations &stations;
        StationBucketsEnvT &stationBucketsEnv;
        StationDistanceFinderT stationDistanceFinder;
        PALSToStationsT &palsToStations;

        StationsInEllipseT &stationsInEllipse;
        OrdinaryToStationsT ordinaryToStations;

        DALSToStationsT &dalsToStations;
        PBNSToStationsT &pbnsToStations;

        const std::vector<PTQueryT> &queries;
        PTAlgorithmWithTaxiT &ptAlgorithmWithTaxi;

        TaxiLegApproximationT taxiLegApproximation;

        std::ofstream &intermediateLogger;
        std::ofstream &firstTaxiLegLogger;
        std::ofstream &ptLogger;
    };
}
