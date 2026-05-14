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


#include <cassert>
#include <kassert/kassert.hpp>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <csv.h>
#include <functional>

#include <Common/Constants.h>

#include "../PTaxi/PTAndTaxiTripFinder.h"
#include "../PTaxi/FirstTaxiLeg/StationDistanceFinder.h"
#include "../PTaxi/FirstTaxiLeg/PALSToStations.h"
#include "../PTaxi/FirstTaxiLeg/DALSToStations.h"
#include "../PTaxi/FirstTaxiLeg/OrdinaryToStations.h"
#include "../PTaxi/FirstTaxiLeg/PBNSToStations.h"
#include "../PTaxi/Station/StationBucketsEnvironment.h"
#include "../PTaxi/Station/StationsInEllipse.h"
#include "../PTaxi/SecondTaxiLeg/HeuristicEgressTripFinder.h"
#include "../PTaxi/PTLeg/ParrotCombinedULTRAMcRAPTOR.h"
#include "../PTaxi/PTLeg/ParrotPTOnlyULTRAMcRAPTOR.h"
#include "../PTaxi/PTLeg/ParrotInitialTransfers.h"

#include <ULTRA/DataStructures/Queries/Queries.h>

#include <KARRI/Algorithms/CH/CH.h>
#include <KARRI/Tools/custom_assertion_levels.h>
#include <KARRI/Tools/CommandLine/CommandLineParser.h>
#include <KARRI/Tools/Logging/LogManager.h>
#include <KARRI/DataStructures/Graph/Graph.h>
#include <KARRI/DataStructures/Graph/Attributes/LatLngAttribute.h>
#include <KARRI/DataStructures/Graph/Attributes/EdgeIdAttribute.h>
#include <KARRI/DataStructures/Graph/Attributes/OsmNodeIdAttribute.h>
#include <KARRI/DataStructures/Graph/Attributes/FreeFlowSpeedAttribute.h>
#include <KARRI/DataStructures/Graph/Attributes/EdgeTailAttribute.h>
#include <KARRI/DataStructures/Graph/Attributes/TravelTimeAttribute.h>
#include <KARRI/DataStructures/Graph/Attributes/PsgEdgeToCarEdgeAttribute.h>
#include <KARRI/DataStructures/Graph/Attributes/CarEdgeToPsgEdgeAttribute.h>
#include <KARRI/Algorithms/KaRRi/InputConfig.h>
#include <KARRI/Algorithms/KaRRi/BaseObjects/Vehicle.h>
#include <KARRI/Algorithms/KaRRi/BaseObjects/Request.h>
#include <KARRI/Algorithms/KaRRi/PbnsAssignments/VehicleLocator.h>
#include <KARRI/Algorithms/KaRRi/EllipticBCH/FeasibleEllipticDistances.h>
#include <KARRI/Algorithms/KaRRi/EllipticBCH/EllipticBucketsEnvironment.h>
#include <KARRI/Algorithms/KaRRi/EllipticBCH/EllipticBCHSearches.h>
#include <KARRI/Algorithms/KaRRi/EllipticBCH/PDLocsAtExistingStopsFinder.h>
#include <KARRI/Algorithms/KaRRi/CostCalculator.h>
#include <KARRI/Algorithms/KaRRi/RequestState/RequestState.h>
#include <KARRI/Algorithms/KaRRi/RequestState/RelevantPDLocsFilter.h>
#include <KARRI/Algorithms/KaRRi/OrdinaryAssignments/OrdinaryAssignmentsFinder.h>
#include <KARRI/Algorithms/KaRRi/PbnsAssignments/PBNSAssignmentsFinder.h>
#include <KARRI/Algorithms/KaRRi/PalsAssignments/PALSAssignmentsFinder.h>
#include <KARRI/Algorithms/KaRRi/DalsAssignments/DALSAssignmentsFinder.h>
#include <KARRI/Algorithms/KaRRi/LastStopSearches/SortedLastStopBucketsEnvironment.h>
#include <KARRI/Algorithms/KaRRi/LastStopSearches/UnsortedLastStopBucketsEnvironment.h>
#include <KARRI/Algorithms/KaRRi/RequestState/VehicleToPDLocQuery.h>
#include <KARRI/Algorithms/KaRRi/RequestState/RequestStateInitializer.h>
#include <KARRI/Algorithms/KaRRi/SystemStateUpdater.h>
#include <KARRI/Algorithms/KaRRi/EventSimulation.h>
#include <KARRI/Algorithms/KaRRi/RequestState/PDLocsFinder.h>

#include "../PTaxi/KaRRiBaseInfoPreparator.h"
#include "../PTaxi/TaxiTripFinder.h"
#include "../PTaxi/PTJourneyFinder.h"
#include "../PTaxi/WalkingTripFinder.h"
#include <../PTaxi/RiderModeChoice/ModeChoice.h>
#include <../PTaxi/RiderModeChoice/UtilityLogitCriterion.h>

#include "NoOpPTAndTaxiTripFinder.h"
#include "../PTaxi/CarTripFinder.h"
#include "PTLeg/ParrotPTOnlyULTRARAPTOR.h"
#include "Station/NoOpStationsInEllipse.h"
#include "ULTRA/Algorithms/RAPTOR/Profiler.h"

#ifdef KARRI_USE_CCHS
#include <KARRI/Algorithms/KaRRi/CCHEnvironment.h>
#else

#include <KARRI/Algorithms/KaRRi/CHEnvironment.h>

#endif

#if KARRI_PD_STRATEGY == KARRI_BCH_PD_STRAT

#include <KARRI/Algorithms/KaRRi/PDDistanceQueries/BCHStrategy.h>

#else // KARRI_PD_STRATEGY == KARRI_CH_PD_STRAT
#include <KARRI/Algorithms/KaRRi/PDDistanceQueries/CHStrategy.h>
#endif


#if KARRI_PALS_STRATEGY == KARRI_COL

#include <KARRI/Algorithms/KaRRi/PalsAssignments/CollectiveBCHStrategy.h>

#elif KARRI_PALS_STRATEGY == KARRI_IND

#include <KARRI/Algorithms/KaRRi/PalsAssignments/IndividualBCHStrategy.h>

#else // KARRI_PALS_STRATEGY == KARRI_DIJ

#include <KARRI/Algorithms/KaRRi/PalsAssignments/DijkstraStrategy.h>

#endif

#if KARRI_DALS_STRATEGY == KARRI_COL

#include <KARRI/Algorithms/KaRRi/DalsAssignments/CollectiveBCHStrategy.h>

#elif KARRI_DALS_STRATEGY == KARRI_IND

#include <KARRI/Algorithms/KaRRi/DalsAssignments/IndividualBCHStrategy.h>

#else // KARRI_DALS_STRATEGY == KARRI_DIJ

#include <KARRI/Algorithms/KaRRi/DalsAssignments/DijkstraStrategy.h>

#endif


inline void printUsage() {
    std::cout <<
            "Usage: karri -veh-g <vehicle network> -psg-g <passenger network> -r <requests> -v <vehicles> -o <file>\n"
            "Runs Karlsruhe Rapid Ridesharing (KaRRi) simulation with given vehicle and passenger road networks,\n"
            "requests, and vehicles. Writes output files to specified base path."
            "  -veh-g <file>            vehicle road network in binary format.\n"
            "  -psg-g <file>            passenger road (and path) network in binary format.\n"
            "  -r <file>                requests in CSV format.\n"
            "  -v <file>                vehicles in CSV format.\n"
            "  -s <sec>                 stop time (in s). (dflt: 60s)\n"
            "  -mc-taxi-w <sec>         if the wait time for a taxi pickup exceeds this value, the trip is not considered for mode choice (in s). (dflt: no bound)\n"
            "  -w <sec>                   maximum added wait time after accepting ride (in s). (dflt: 600s)\n"
            "  -a <factor>               model parameter alpha' for max trip time after being assigned = pa * asgn trip time + pb (dflt: 1.4)\n"
            "  -b <seconds>              model parameter beta' for max trip time after being assigned = pa * asgn trip time + pb (dflt: 1200)\n"
            "  -p-radius <sec>          walking radius (in s) for pickup locations around origin. (dflt: 300s)\n"
            "  -d-radius <sec>          walking radius (in s) for dropoff locations around destination. (dflt: 300s)\n"
            "  -max-num-p <int>         max number of pickup locations to consider, sampled from all in radius. Set to 0 for no limit (dflt).\n"
            "  -max-num-d <int>         max number of dropoff locations to consider, sampled from all in radius. Set to 0 for no limit (dflt).\n"
            "  -cost-tolerance <factor> factor that decides cost upper bound for combined journeys relative to cost of best RP-only and PT-only journeys (dflt: 1.0)\n"
            "  -egr-cost-m <factor>     model parameter for slope of linear approximation of taxi egress cost relative to direct distance (dflt: 1.0); prefix 'neg' for negative\n"
            "  -egr-cost-b <sec>        model parameter for intercept of linear approximation of taxi egress cost (dflt: 0s); prefix 'neg' for negative\n"
            "  -egr-tt-m <factor>       model parameter for slope of linear approximation of taxi egress travel time relative to direct distance (dflt: 1.0); prefix 'neg' for negative\n"
            "  -egr-tt-b <sec>          model parameter for intercept of linear approximation of taxi egress travel time (dflt: 0s); prefix 'neg' for negative\n"
            "  -veh-h <file>            contraction hierarchy for the vehicle network in binary format.\n"
            "  -psg-h <file>            contraction hierarchy for the passenger network in binary format.\n"
            "  -veh-d <file>            separator decomposition for the vehicle network in binary format (needed for CCHs).\n"
            "  -psg-d <file>            separator decomposition for the passenger network in binary format (needed for CCHs).\n"
            "  -csv-in-LOUD-format      if set, assumes that input files are in the format used by LOUD.\n"
            "  -o <file>                generate output files at name <file> (specify name without file suffix).\n"
            "  -raptor-data <file>      file with the precomputed RAPTOR data.\n"
            "  -station-mapping <file>  file which maps the station to edge-ids in the given passenger road graph.\n"
            "  -station-buckets <file>  precomputed station buckets (vehicle) for use in KaRRi in binary format.\n"
            "  -psg-station-buckets <file>  precomputed station buckets (pedestrian) for walking transfers in binary format.\n"
            "  -help                    show usage help text.\n";
}

int readSignedIntParam(const std::string &s) {
    if (startsWith(s, "neg")) {
        return -std::stoi(s.substr(3));
    }
    return std::stoi(s);
}

double readSignedDoubleParam(const std::string &s) {
    if (startsWith(s, "neg")) {
        return -std::stod(s.substr(3));
    }
    return std::stod(s);
}

int main(int argc, char *argv[]) {
    using namespace karri;
    try {
        CommandLineParser clp(argc, argv);
        if (clp.isSet("help")) {
            printUsage();
            return EXIT_SUCCESS;
        }


        // Parse the command-line options.
        InputConfig &inputConfig = InputConfig::getInstance();
        inputConfig.stopTime = clp.getValue<int>("s", 60) * 10;
        if (clp.isSet("mc-taxi-w")) {
            inputConfig.modeChoiceMaxTaxiWaitTime = clp.getValue<int>("mc-taxi-w") * 10;
        } else {
            inputConfig.modeChoiceMaxTaxiWaitTime = INFTY;
        }
        inputConfig.hardConstraintMaxAddedWaitTime = clp.getValue<int>("w", 600) * 10;
        inputConfig.hardConstraintAlpha = clp.getValue<double>("a", 1.4);
        inputConfig.hardConstraintBeta = clp.getValue<int>("b", 600) * 10;
        inputConfig.parrotCostTolerance = clp.getValue<double>("cost-tolerance", 1.0);
        inputConfig.parrotEgressCostHeuristicSlope = readSignedDoubleParam(clp.getValue<std::string>("egr-cost-m", "1.0"));
        inputConfig.parrotEgressCostHeuristicIntercept = readSignedIntParam(clp.getValue<std::string>("egr-cost-b", "0")) * 10;
        inputConfig.parrotEgressTravelTimeHeuristicSlope = readSignedDoubleParam(clp.getValue<std::string>("egr-tt-m", "1.0"));
        inputConfig.parrotEgressTravelTimeHeuristicIntercept = readSignedIntParam(clp.getValue<std::string>("egr-tt-b", "0")) * 10;
        inputConfig.pickupRadius = clp.getValue<int>("p-radius", 300) * 10;
        inputConfig.dropoffRadius = clp.getValue<int>("d-radius", 300) * 10;
        inputConfig.maxNumPickups = clp.getValue<int>("max-num-p", INFTY);
        inputConfig.maxNumDropoffs = clp.getValue<int>("max-num-d", INFTY);
        if (inputConfig.maxNumPickups == 0) inputConfig.maxNumPickups = INFTY;
        if (inputConfig.maxNumDropoffs == 0) inputConfig.maxNumDropoffs = INFTY;
        const auto vehicleNetworkFileName = clp.getValue<std::string>("veh-g");
        const auto passengerNetworkFileName = clp.getValue<std::string>("psg-g");
        const auto vehicleFileName = clp.getValue<std::string>("v");
        const auto requestFileName = clp.getValue<std::string>("r");
        const auto vehHierarchyFileName = clp.getValue<std::string>("veh-h");
        const auto psgHierarchyFileName = clp.getValue<std::string>("psg-h");
        const auto vehSepDecompFileName = clp.getValue<std::string>("veh-d");
        const auto psgSepDecompFileName = clp.getValue<std::string>("psg-d");
        const bool csvFilesInLoudFormat = clp.isSet("csv-in-LOUD-format");
        // new
        const auto raptorFileName = clp.getValue<std::string>("raptor-data");
        const auto stationMappingFileName = clp.getValue<std::string>("station-mapping");
        auto stationBucketsFilename = clp.getValue<std::string>("station-buckets");
        if (!endsWith(stationBucketsFilename, ".bucket.bin")) stationBucketsFilename += ".bucket.bin";
        auto psgStationBucketsFilename = clp.getValue<std::string>("psg-station-buckets");
        if (!endsWith(psgStationBucketsFilename, ".bucket.bin")) psgStationBucketsFilename += ".bucket.bin";

        auto outputFileName = clp.getValue<std::string>("o");
        if (endsWith(outputFileName, ".csv"))
            outputFileName = outputFileName.substr(0, outputFileName.size() - 4);


        LogManager<std::ofstream>::setBaseFileName(outputFileName + ".");

        // Read the vehicle network from file.
        std::cout << "Reading vehicle network from file... " << std::flush;
        using VehicleVertexAttributes = VertexAttrs<LatLngAttribute, OsmNodeIdAttribute>;
        using VehicleEdgeAttributes = EdgeAttrs<
            EdgeIdAttribute, EdgeTailAttribute, FreeFlowSpeedAttribute, TravelTimeAttribute, CarEdgeToPsgEdgeAttribute,
            OsmRoadCategoryAttribute>;
        using VehicleInputGraph = KaRRiStaticGraph<VehicleVertexAttributes, VehicleEdgeAttributes>;
        std::ifstream vehicleNetworkFile(vehicleNetworkFileName, std::ios::binary);
        if (!vehicleNetworkFile.good())
            throw std::invalid_argument("file not found -- '" + vehicleNetworkFileName + "'");
        VehicleInputGraph vehicleInputGraph(vehicleNetworkFile);
        vehicleNetworkFile.close();
        std::vector<int32_t> vehGraphOrigIdToSeqId;
        if (vehicleInputGraph.numEdges() > 0 && vehicleInputGraph.edgeId(0) == INVALID_ID) {
            vehGraphOrigIdToSeqId.assign(vehicleInputGraph.numEdges(), INVALID_ID);
            std::iota(vehGraphOrigIdToSeqId.begin(), vehGraphOrigIdToSeqId.end(), 0);
            FORALL_VALID_EDGES(vehicleInputGraph, v, e) {
                assert(vehicleInputGraph.edgeId(e) == INVALID_ID);
                vehicleInputGraph.edgeTail(e) = v;
                vehicleInputGraph.edgeId(e) = e;
            }
        } else {
            FORALL_VALID_EDGES(vehicleInputGraph, v, e) {
                assert(vehicleInputGraph.edgeId(e) != INVALID_ID);
                if (vehicleInputGraph.edgeId(e) >= vehGraphOrigIdToSeqId.size()) {
                    const auto numElementsToBeInserted =
                            vehicleInputGraph.edgeId(e) + 1 - vehGraphOrigIdToSeqId.size();
                    vehGraphOrigIdToSeqId.insert(vehGraphOrigIdToSeqId.end(), numElementsToBeInserted, INVALID_ID);
                }
                assert(vehGraphOrigIdToSeqId[vehicleInputGraph.edgeId(e)] == INVALID_ID);
                vehGraphOrigIdToSeqId[vehicleInputGraph.edgeId(e)] = e;
                vehicleInputGraph.edgeTail(e) = v;
                vehicleInputGraph.edgeId(e) = e;
            }
        }
        std::cout << "done.\n";

        // Read the passenger network from file.
        std::cout << "Reading passenger network from file... " << std::flush;
        using PsgVertexAttributes = VertexAttrs<LatLngAttribute, OsmNodeIdAttribute>;
        using PsgEdgeAttributes = EdgeAttrs<EdgeIdAttribute, EdgeTailAttribute, PsgEdgeToCarEdgeAttribute,
            TravelTimeAttribute>;
        using PsgInputGraph = KaRRiStaticGraph<PsgVertexAttributes, PsgEdgeAttributes>;
        std::ifstream psgNetworkFile(passengerNetworkFileName, std::ios::binary);
        if (!psgNetworkFile.good())
            throw std::invalid_argument("file not found -- '" + passengerNetworkFileName + "'");
        PsgInputGraph psgInputGraph(psgNetworkFile);
        psgNetworkFile.close();
        assert(psgInputGraph.numEdges() > 0 && psgInputGraph.edgeId(0) == INVALID_ID);
        int numEdgesWithMappingToCar = 0;
        FORALL_VALID_EDGES(psgInputGraph, v, e) {
            assert(psgInputGraph.edgeId(e) == INVALID_ID);
            psgInputGraph.edgeTail(e) = v;
            psgInputGraph.edgeId(e) = e;

            const int eInVehGraph = psgInputGraph.toCarEdge(e);
            if (eInVehGraph != PsgEdgeToCarEdgeAttribute::defaultValue()) {
                ++numEdgesWithMappingToCar;
                assert(eInVehGraph < vehGraphOrigIdToSeqId.size());
                psgInputGraph.toCarEdge(e) = vehGraphOrigIdToSeqId[eInVehGraph];
                assert(psgInputGraph.toCarEdge(e) < vehicleInputGraph.numEdges());
                vehicleInputGraph.toPsgEdge(psgInputGraph.toCarEdge(e)) = e;

                assert(psgInputGraph.latLng(psgInputGraph.edgeHead(e)).latitude() ==
                    vehicleInputGraph.latLng(vehicleInputGraph.edgeHead(psgInputGraph.toCarEdge(e))).latitude());
                assert(psgInputGraph.latLng(psgInputGraph.edgeHead(e)).longitude() == vehicleInputGraph.latLng(
                    vehicleInputGraph.edgeHead(psgInputGraph.toCarEdge(e))).longitude());
            }
        }
        unused(numEdgesWithMappingToCar);
        assert(numEdgesWithMappingToCar > 0);
        std::cout << "done.\n";


        // Read the vehicle data from file.
        std::cout << "Reading vehicle data from file... " << std::flush;
        Fleet fleet;
        int location, capacity, startOfServiceTime, endOfServiceTime;
        io::CSVReader<4, io::trim_chars<' '> > vehiclesFileReader(vehicleFileName);

        if (csvFilesInLoudFormat) {
            vehiclesFileReader.read_header(io::ignore_no_column, "initial_location", "seating_capacity",
                                           "start_service_time", "end_service_time");
        } else {
            vehiclesFileReader.read_header(io::ignore_no_column,
                                           "initial_location", "start_of_service_time",
                                           "end_of_service_time", "capacity");
        }

        int maxCapacity = 0;
        while ((csvFilesInLoudFormat &&
                vehiclesFileReader.read_row(location, capacity, startOfServiceTime, endOfServiceTime)) ||
               (!csvFilesInLoudFormat &&
                vehiclesFileReader.read_row(location, startOfServiceTime, endOfServiceTime, capacity))) {
            if (location < 0 || location >= vehGraphOrigIdToSeqId.size() ||
                vehGraphOrigIdToSeqId[location] == INVALID_ID)
                throw std::invalid_argument("invalid location -- '" + std::to_string(location) + "'");
            if (endOfServiceTime <= startOfServiceTime)
                throw std::invalid_argument("start of service time needs to be before end of service time");
            const int vehicleId = static_cast<int>(fleet.size());
            fleet.push_back({
                vehicleId, vehGraphOrigIdToSeqId[location], startOfServiceTime * 10,
                endOfServiceTime * 10, capacity
            });
            maxCapacity = std::max(maxCapacity, capacity);
        }
        std::cout << "done.\n";

        // Read the request data from file.
        std::cout << "Reading request data from file... " << std::flush;
        std::vector<Request> requests;
        std::vector<EdgeQuery> queries;
        int origin, destination, requestTime, numRiders;
        double allowPrivateCarProbability;
        io::CSVReader<5, io::trim_chars<' '> > reqFileReader(requestFileName);

        if (csvFilesInLoudFormat) {
            reqFileReader.read_header(io::ignore_extra_column | io::ignore_missing_column, "pickup_spot",
                                      "dropoff_spot", "min_dep_time",
                                      "num_riders", "allow_private_car_probability");
        } else {
            reqFileReader.read_header(io::ignore_extra_column | io::ignore_missing_column, "origin", "destination",
                                      "req_time",
                                      "num_riders", "allow_private_car_probability");
        }


        numRiders = 1; // If number of riders was not specified, assume one rider
        allowPrivateCarProbability = 1.0; // If not specified, assume that all riders can use private car
        while (reqFileReader.read_row(origin, destination, requestTime, numRiders, allowPrivateCarProbability)) {
            if (origin < 0 || origin >= vehGraphOrigIdToSeqId.size() || vehGraphOrigIdToSeqId[origin] == INVALID_ID)
                throw std::invalid_argument("invalid location -- '" + std::to_string(origin) + "'");
            if (destination < 0 || destination >= vehGraphOrigIdToSeqId.size() ||
                vehGraphOrigIdToSeqId[destination] == INVALID_ID)
                throw std::invalid_argument("invalid location -- '" + std::to_string(destination) + "'");
            if (numRiders > maxCapacity)
                throw std::invalid_argument(
                    "number of riders '" + std::to_string(numRiders) + "' is larger than max vehicle capacity (" +
                    std::to_string(maxCapacity) + ")");
            const auto originSeqId = vehGraphOrigIdToSeqId[origin];
            assert(vehicleInputGraph.toPsgEdge(originSeqId) != CarEdgeToPsgEdgeAttribute::defaultValue());
            const auto destSeqId = vehGraphOrigIdToSeqId[destination];
            assert(vehicleInputGraph.toPsgEdge(destSeqId) != CarEdgeToPsgEdgeAttribute::defaultValue());

            const int requestId = static_cast<int>(requests.size());
            requests.push_back({
                requestId, originSeqId, destSeqId, requestTime * 10, numRiders, allowPrivateCarProbability
            });
            // Reset defaults in case next request does not specify all values
            numRiders = 1;
            allowPrivateCarProbability = 1.0;

            // Create EdgeQuery with both vehicle and passenger edge IDs
            const int originPsgEdge = vehicleInputGraph.toPsgEdge(originSeqId);
            const int destPsgEdge = vehicleInputGraph.toPsgEdge(destSeqId);
            queries.push_back(EdgeQuery(originSeqId, originPsgEdge, destSeqId, destPsgEdge, requestTime));
        }
        std::cout << "done.\n";

#ifdef KARRI_USE_CCHS

        // Prepare vehicle CH environment
        using VehCHEnv = CCHEnvironment<VehicleInputGraph, TravelTimeAttribute>;
        std::unique_ptr<VehCHEnv> vehChEnv;
        if (vehSepDecompFileName.empty()) {
            std::cout << "Building Separator Decomposition and CCH... " << std::flush;
            vehChEnv = std::make_unique<VehCHEnv>(vehicleInputGraph);
            std::cout << "done.\n";
        } else {
            // Read the separator decomposition from file, construct and customize CCH.
            std::cout << "Reading Seperator Decomposition from file and building CCH... " << std::flush;
            std::ifstream vehSepDecompFile(vehSepDecompFileName, std::ios::binary);
            if (!vehSepDecompFile.good())
                throw std::invalid_argument("file not found -- '" + vehSepDecompFileName + "'");
            SeparatorDecomposition vehSepDecomp;
            vehSepDecomp.readFrom(vehSepDecompFile);
            vehSepDecompFile.close();
            std::cout << "done.\n";
            vehChEnv = std::make_unique<VehCHEnv>(vehicleInputGraph, vehSepDecomp);
        }

        // Prepare passenger CH environment
        using PsgCHEnv = CCHEnvironment<PsgInputGraph, TravelTimeAttribute>;
        std::unique_ptr<PsgCHEnv> psgChEnv;
        if (psgSepDecompFileName.empty()) {
            std::cout << "Building Separator Decomposition and CCH... " << std::flush;
            psgChEnv = std::make_unique<PsgCHEnv>(psgInputGraph);
            std::cout << "done.\n";
        } else {
            // Read the separator decomposition from file, construct and customize CCH.
            std::cout << "Reading Seperator Decomposition from file and building CCH... " << std::flush;
            std::ifstream psgSepDecompFile(psgSepDecompFileName, std::ios::binary);
            if (!psgSepDecompFile.good())
                throw std::invalid_argument("file not found -- '" + psgSepDecompFileName + "'");
            SeparatorDecomposition psgSepDecomp;
            psgSepDecomp.readFrom(psgSepDecompFile);
            psgSepDecompFile.close();
            std::cout << "done.\n";
            psgChEnv = std::make_unique<PsgCHEnv>(psgInputGraph, psgSepDecomp);
        }

#else
        // Prepare vehicle CH environment
        using VehCHEnv = CHEnvironment<VehicleInputGraph, TravelTimeAttribute>;
        std::unique_ptr<VehCHEnv> vehChEnv;
        if (vehHierarchyFileName.empty()) {
            std::cout << "Building CH... " << std::flush;
            vehChEnv = std::make_unique<VehCHEnv>(vehicleInputGraph);
            std::cout << "done.\n";
        } else {
            // Read the CH from file.
            std::cout << "Reading CH from file... " << std::flush;
            std::ifstream vehHierarchyFile(vehHierarchyFileName, std::ios::binary);
            if (!vehHierarchyFile.good())
                throw std::invalid_argument("file not found -- '" + vehHierarchyFileName + "'");
            CH vehCh(vehHierarchyFile);
            vehHierarchyFile.close();
            std::cout << "done.\n";
            vehChEnv = std::make_unique<VehCHEnv>(std::move(vehCh));
        }

        // Prepare passenger CH environment
        using PsgCHEnv = CHEnvironment<PsgInputGraph, TravelTimeAttribute>;
        std::unique_ptr<PsgCHEnv> psgChEnv;
        if (psgHierarchyFileName.empty()) {
            std::cout << "Building passenger CH... " << std::flush;
            psgChEnv = std::make_unique<PsgCHEnv>(psgInputGraph);
            std::cout << "done.\n";
        } else {
            // Read the passenger CH from file.
            std::cout << "Reading passenger CH from file... " << std::flush;
            std::ifstream psgHierarchyFile(psgHierarchyFileName, std::ios::binary);
            if (!psgHierarchyFile.good())
                throw std::invalid_argument("file not found -- '" + psgHierarchyFileName + "'");
            CH psgCh(psgHierarchyFile);
            psgHierarchyFile.close();
            std::cout << "done.\n";
            psgChEnv = std::make_unique<PsgCHEnv>(std::move(psgCh));
        }
#endif

        // Create Route State for empty routes.
        RouteState routeState(fleet);

        // Set up the distance checker callback to verify shortest path distances.
        // This captures the vehicle graph and CH environment for use in checkDirectDistance().
        routeState.setDistanceChecker([&vehicleInputGraph, &vehChEnv](int curStop, int nextStop) {
            if (curStop == nextStop)
                return 0;
            const auto &ch = vehChEnv->getCH();
            auto chQuery = vehChEnv->template getFullCHQuery<>();
            chQuery.run(ch.rank(vehicleInputGraph.edgeHead(curStop)), ch.rank(vehicleInputGraph.edgeTail(nextStop)));
            return chQuery.getDistance() + vehicleInputGraph.travelTime(nextStop);
        });

        using VehicleLocatorImpl = VehicleLocator<VehicleInputGraph, VehCHEnv>;
        VehicleLocatorImpl locator(vehicleInputGraph, *vehChEnv, routeState);

        // Construct Elliptic BCH bucket environment:
        static constexpr bool ELLIPTIC_SORTED_BUCKETS = KARRI_ELLIPTIC_BCH_SORTED_BUCKETS;
        using EllipticBucketsEnv = EllipticBucketsEnvironment<VehicleInputGraph, VehCHEnv, ELLIPTIC_SORTED_BUCKETS>;
        EllipticBucketsEnv ellipticBucketsEnv(vehicleInputGraph, *vehChEnv, routeState);

        // If we use any BCH queries in the PALS or DALS strategies, we construct the according bucket data structure.
        // Otherwise, we use a last stop buckets substitute that only stores which vehicles' last stops are at a vertex.
#if KARRI_PALS_STRATEGY == KARRI_COL || KARRI_PALS_STRATEGY == KARRI_IND || \
KARRI_DALS_STRATEGY == KARRI_COL || KARRI_DALS_STRATEGY == KARRI_IND

        static constexpr bool LAST_STOP_SORTED_BUCKETS = KARRI_LAST_STOP_BCH_SORTED_BUCKETS;
        using LastStopBucketsEnv = std::conditional_t<LAST_STOP_SORTED_BUCKETS,
            SortedLastStopBucketsEnvironment<VehicleInputGraph, VehCHEnv>,
            UnsortedLastStopBucketsEnvironment<VehicleInputGraph, VehCHEnv>
        >;
        LastStopBucketsEnv lastStopBucketsEnv(vehicleInputGraph, *vehChEnv, routeState);

#else
        using LastStopBucketsEnv = OnlyLastStopsAtVerticesBucketSubstitute<VehicleInputGraph>;
        LastStopBucketsEnv lastStopBucketsEnv(vehicleInputGraph, routeState, fleet.size());
#endif
        // Last stop bucket environment (or substitute) also serves as a source of information on the last stops at vertices.
        using LastStopAtVerticesInfo = LastStopBucketsEnv;

        using EllipticBCHLabelSet = std::conditional_t<KARRI_ELLIPTIC_BCH_USE_SIMD,
            SimdLabelSet<KARRI_ELLIPTIC_BCH_LOG_K, ParentInfo::NO_PARENT_INFO>,
            BasicLabelSet<KARRI_ELLIPTIC_BCH_LOG_K, ParentInfo::NO_PARENT_INFO> >;
        using FeasibleEllipticDistancesImpl = FeasibleEllipticDistances<EllipticBCHLabelSet>;

        using PDLocsAtExistingStopsFinderImpl = PDLocsAtExistingStopsFinder<VehicleInputGraph, VehCHEnv, typename
            EllipticBucketsEnv::BucketContainer, LastStopAtVerticesInfo>;
        PDLocsAtExistingStopsFinderImpl pdLocsAtExistingStops(vehicleInputGraph, *vehChEnv,
                                                              ellipticBucketsEnv.getSourceBuckets(), lastStopBucketsEnv,
                                                              routeState);

        using EllipticBCHSearchesImpl = EllipticBCHSearches<VehicleInputGraph, VehCHEnv, CostCalculator::CostFunction,
            EllipticBucketsEnv, LastStopAtVerticesInfo, FeasibleEllipticDistancesImpl, EllipticBCHLabelSet>;
        EllipticBCHSearchesImpl ellipticSearches(vehicleInputGraph, fleet, ellipticBucketsEnv, lastStopBucketsEnv,
                                                 *vehChEnv, routeState);


        // Construct remaining request state
        using RelevantPDLocsFilterImpl = RelevantPDLocsFilter<FeasibleEllipticDistancesImpl, VehicleInputGraph,
            VehCHEnv>;
        RelevantPDLocsFilterImpl relevantPdLocsFilter(fleet, vehicleInputGraph, *vehChEnv, routeState);


        const auto revVehicleGraph = vehicleInputGraph.getReverseGraph();
        const auto revPsgGraph = psgInputGraph.getReverseGraph();


        using VehicleToPDLocQueryImpl = VehicleToPDLocQuery<VehicleInputGraph>;
        VehicleToPDLocQueryImpl vehicleToPdLocQuery(vehicleInputGraph, revVehicleGraph);


        // Construct PD-distance query
        using PDDistancesLabelSet = std::conditional_t<KARRI_PD_DISTANCES_USE_SIMD,
            SimdLabelSet<KARRI_PD_DISTANCES_LOG_K, ParentInfo::NO_PARENT_INFO>,
            BasicLabelSet<KARRI_PD_DISTANCES_LOG_K, ParentInfo::NO_PARENT_INFO> >;

#if KARRI_PD_STRATEGY == KARRI_BCH_PD_STRAT
        using PDDistanceQueryImpl = PDDistanceQueryStrategies::BCHStrategy<VehicleInputGraph, VehCHEnv,
            PDDistancesLabelSet>;
        PDDistanceQueryImpl pdDistanceSearches(vehicleInputGraph, *vehChEnv);

#else // KARRI_PD_STRATEGY == KARRI_CH_PD_STRAT
        using FFPDDistanceQueryImpl = PDDistanceQueryStrategies::CHStrategy<VehicleInputGraph, VehCHEnv,
            PDDistancesLabelSet>;
        FFPDDistanceQueryImpl ffPDDistanceQuery(vehicleInputGraph, *vehChEnv);
#endif

        // Construct ordinary assignments finder:
        using OrdinaryAssignmentsFinderImpl = OrdinaryAssignmentsFinder;
        OrdinaryAssignmentsFinderImpl ordinaryInsertionsFinder(fleet, routeState);

        // Construct PBNS assignments finder:
        using CurVehLocToPickupLabelSet = PDDistancesLabelSet;
        using CurVehLocToPickupSearchesImpl = CurVehLocToPickupSearches<VehicleInputGraph, VehicleLocatorImpl, VehCHEnv,
            CurVehLocToPickupLabelSet>;
        CurVehLocToPickupSearchesImpl curVehLocToPickupSearches(vehicleInputGraph, locator, *vehChEnv, routeState,
                                                                fleet.size());


        using PBNSInsertionsFinderImpl = PBNSAssignmentsFinder<CurVehLocToPickupSearchesImpl>;
        PBNSInsertionsFinderImpl pbnsInsertionsFinder(curVehLocToPickupSearches, fleet, routeState);

        // Construct PALS strategy and assignment finder:
        using PALSLabelSet = std::conditional_t<KARRI_PALS_USE_SIMD,
            SimdLabelSet<KARRI_PALS_LOG_K, ParentInfo::NO_PARENT_INFO>,
            BasicLabelSet<KARRI_PALS_LOG_K, ParentInfo::NO_PARENT_INFO> >;


#if KARRI_PALS_STRATEGY == KARRI_COL
        // Use Collective-BCH PALS Strategy
        using PALSStrategy = PickupAfterLastStopStrategies::CollectiveBCHStrategy<VehicleInputGraph, VehCHEnv,
            LastStopBucketsEnv, VehicleToPDLocQueryImpl, PDDistancesLabelSet, PALSLabelSet>;
        PALSStrategy palsStrategy(vehicleInputGraph, fleet, *vehChEnv,
                                  vehicleToPdLocQuery, lastStopBucketsEnv, routeState);
#elif KARRI_PALS_STRATEGY == KARRI_IND
        // Use Individual-BCH PALS Strategy
        using PALSStrategy = PickupAfterLastStopStrategies::IndividualBCHStrategy<VehicleInputGraph, VehCHEnv,
            LastStopBucketsEnv, PDDistancesImpl, PALSLabelSet>;
        PALSStrategy palsStrategy(vehicleInputGraph, fleet, *vehChEnv, lastStopBucketsEnv,
                                  routeState);
#else // KARRI_PALS_STRATEGY == KARRI_DIJ
        // Use Dijkstra PALS Strategy
        using PALSStrategy = PickupAfterLastStopStrategies::DijkstraStrategy<VehicleInputGraph, LastStopBucketsEnv,
            PDDistancesImpl, PALSLabelSet>;
        PALSStrategy palsStrategy(vehicleInputGraph, revVehicleGraph, fleet, routeState, lastStopBucketsEnv);
#endif

        using PALSInsertionsFinderImpl = PALSAssignmentsFinder<VehicleInputGraph, PALSStrategy, LastStopAtVerticesInfo>;
        PALSInsertionsFinderImpl palsInsertionsFinder(palsStrategy, vehicleInputGraph, fleet, lastStopBucketsEnv,
                                                      routeState);


        // Construct DALS strategy and assignment finder:

        using DALSLabelSet = std::conditional_t<KARRI_DALS_USE_SIMD,
            SimdLabelSet<KARRI_DALS_LOG_K, ParentInfo::NO_PARENT_INFO>,
            BasicLabelSet<KARRI_DALS_LOG_K, ParentInfo::NO_PARENT_INFO> >;
#if KARRI_DALS_STRATEGY == KARRI_COL
        // Use Collective-BCH DALS Strategy
        using DALSStrategy = DropoffAfterLastStopStrategies::CollectiveBCHStrategy<VehicleInputGraph, VehCHEnv,
            LastStopBucketsEnv, CurVehLocToPickupSearchesImpl>;
        DALSStrategy dalsStrategy(vehicleInputGraph, fleet, routeState, *vehChEnv, lastStopBucketsEnv,
                                  curVehLocToPickupSearches);
#elif KARRI_DALS_STRATEGY == KARRI_IND
        // Use Individual-BCH DALS Strategy
        using DALSStrategy = DropoffAfterLastStopStrategies::IndividualBCHStrategy<VehicleInputGraph, VehCHEnv,
            LastStopBucketsEnv, CurVehLocToPickupSearchesImpl, DALSLabelSet>;
        DALSStrategy dalsStrategy(vehicleInputGraph, fleet, *vehChEnv, lastStopBucketsEnv, curVehLocToPickupSearches,
                                  routeState);
#else // KARRI_DALS_STRATEGY == KARRI_DIJ
        // Use Dijkstra DALS Strategy
        using DALSLabelSet = std::conditional_t<KARRI_DALS_USE_SIMD,
            SimdLabelSet<KARRI_DALS_LOG_K, ParentInfo::NO_PARENT_INFO>,
            BasicLabelSet<KARRI_DALS_LOG_K, ParentInfo::NO_PARENT_INFO> >;
        using DALSStrategy = DropoffAfterLastStopStrategies::DijkstraStrategy<VehicleInputGraph, LastStopBucketsEnv,
            CurVehLocToPickupSearchesImpl, DALSLabelSet>;
        DALSStrategy dalsStrategy(vehicleInputGraph, revVehicleGraph, fleet, curVehLocToPickupSearches, routeState,
                                  lastStopBucketsEnv);
#endif

        using DALSInsertionsFinderImpl = DALSAssignmentsFinder<DALSStrategy>;
        DALSInsertionsFinderImpl dalsInsertionsFinder(dalsStrategy);

        using RequestStateInitializerImpl = RequestStateInitializer<VehicleInputGraph, VehCHEnv>;
        RequestStateInitializerImpl requestStateInitializer(vehicleInputGraph, *vehChEnv);

        using PDLocsFinderImpl = PDLocsFinder<VehicleInputGraph, PsgInputGraph, VehicleToPDLocQueryImpl>;
        PDLocsFinderImpl pdLocsFinder(vehicleInputGraph, psgInputGraph, revPsgGraph, vehicleToPdLocQuery);


        // Read the RAPTOR data
        std::cout << "Reading RAPTOR data from file... " << std::flush;
        RAPTOR::Data raptor(raptorFileName);
        raptor.useImplicitDepartureBufferTimes();
        std::cout << "done.\n";

        raptor.printInfo();

        // Read the station mapping file
        std::cout << "Reading station mapping from file... " << std::flush;
        parrot::PTStations stations;
        int edgeId;
        int stationId = 0;
        io::CSVReader<1> stationMappingFileReader(stationMappingFileName);

        stationMappingFileReader.read_header(io::ignore_extra_column, "initial_location");

        while (stationMappingFileReader.read_row(edgeId)) {
            if (edgeId < 0) {
                throw std::invalid_argument("invalid edge id for a station-- '" + std::to_string(edgeId) + "'");
            }

            // edge id in the station mapping file is the edge id in the road network
            int psgEdgeId = vehicleInputGraph.toPsgEdge(edgeId);
            if (psgEdgeId == CarEdgeToPsgEdgeAttribute::defaultValue()) {
                psgEdgeId = INVALID_EDGE;
            }
            stations.push_back({stationId, psgEdgeId, edgeId});
            stationId++;
        }
        std::cout << "done.\n";

        // Pedestrian Station Buckets for walking transfers
        using PsgStationBucketsEnv = parrot::StationBucketsEnvironment<PsgInputGraph, PsgCHEnv, true>;

        std::cout << "Reading pedestrian station buckets from file... " << std::flush;
        std::ifstream inPsg(psgStationBucketsFilename, std::ios::binary);
        if (!inPsg.good())
            throw std::invalid_argument("file not found -- '" + psgStationBucketsFilename + "'");
        PsgStationBucketsEnv psgStationBucketsEnv(psgInputGraph, *psgChEnv);
        psgStationBucketsEnv.readBucketsFrom(inPsg);
        std::cout << "done.\n";

        // Create ParrotInitialTransfers using pedestrian BCH infrastructure
        using ParrotInitialTransfersType = RAPTOR::ParrotInitialTransfers<PsgInputGraph, PsgCHEnv, PsgStationBucketsEnv>
                ;
        ParrotInitialTransfersType parrotInitialTransfers(psgInputGraph, *psgChEnv, psgStationBucketsEnv,
                                                          stations.size());

        // Create PT journey finder for PT-only journey with our custom ParrotInitialTransfers
        using PTAlgorithm = RAPTOR::ParrotPTOnlyULTRARAPTOR<BasicLabelSet<0, ParentInfo::NO_PARENT_INFO>,
            RAPTOR::NoProfiler, false, ParrotInitialTransfersType>;
        // using PTAlgorithm = RAPTOR::ParrotPTOnlyULTRAMcRAPTOR<ParrotInitialTransfersType, RAPTOR::NoProfiler>;
        PTAlgorithm ptAlgorithm(raptor, parrotInitialTransfers, stations, psgInputGraph.numEdges());


#if PARROT_NO_COMBINED
        using PTAndTaxiTripFinderImpl = parrot::NoOpPTAndTaxiTripFinder;
        PTAndTaxiTripFinderImpl ptAndTaxiTripFinder;

        using StationsInEllipseImpl = parrot::NoOpStationsInEllipse;
        StationsInEllipseImpl stationsInEllipse;
#else

        parrot::StationsAtLocations stationsAtLocations(stations, vehicleInputGraph.numEdges());

        // Buckets for PT stations (vehicle graph)
        using StationBucketsEnv = parrot::StationBucketsEnvironment<VehicleInputGraph, VehCHEnv, false>;

        std::cout << "Reading station buckets from file... " << std::flush;
        std::ifstream in(stationBucketsFilename, std::ios::binary);
        if (!in.good())
            throw std::invalid_argument("file not found -- '" + stationBucketsFilename + "'");
        StationBucketsEnv stationBucketsEnv(vehicleInputGraph, *vehChEnv);
        stationBucketsEnv.readBucketsFrom(in);
        std::cout << "done.\n";

        // Create cost-criterion PT router to use for combined journeys
        using PTAlgorithmWithTaxi = RAPTOR::ParrotCombinedULTRAMcRAPTOR<ParrotInitialTransfersType, RAPTOR::NoProfiler>;
        PTAlgorithmWithTaxi ptAlgorithmWithTaxi(raptor, parrotInitialTransfers, stations, psgInputGraph.numEdges());

        using StationBCH = parrot::StationDistanceFinder<VehicleInputGraph, VehCHEnv, StationBucketsEnv, PALSLabelSet>;

        // PALS for stations
        using PALSToStationsImpl = parrot::PALSToStations<VehicleInputGraph, VehCHEnv, LastStopBucketsEnv,
            LastStopAtVerticesInfo, StationBCH::StationDistances, PALSLabelSet>;
        PALSToStationsImpl palsToStations(vehicleInputGraph, fleet, *vehChEnv, lastStopBucketsEnv, lastStopBucketsEnv,
                                          routeState);

        // DALS for stations
        using DALSToStationsLabelSet = BasicLabelSet<0, ParentInfo::NO_PARENT_INFO>;
        using DALSToStationsImpl = parrot::DALSToStations<VehicleInputGraph, VehCHEnv, CurVehLocToPickupSearchesImpl,
            StationBucketsEnv, DALSToStationsLabelSet>;
        DALSToStationsImpl dalsToStations(vehicleInputGraph, fleet, *vehChEnv, curVehLocToPickupSearches, routeState,
                                          stationBucketsEnv, stations, stationsAtLocations);

        using StationsInEllipseImpl = parrot::StationsInEllipse<VehicleInputGraph, VehCHEnv, StationBucketsEnv>;
        StationsInEllipseImpl stationsInEllipse(vehicleInputGraph, *vehChEnv, routeState, stationBucketsEnv, stations,
                                                stationsAtLocations);

        // Ordinary for stations
        using OrdinaryToStationsImpl = parrot::OrdinaryToStations<StationsInEllipseImpl, StationBCH::StationDistances>;

        // PBNS for stations
        using PBNSToStationsImpl = parrot::PBNSToStations<CurVehLocToPickupSearchesImpl, StationsInEllipseImpl,
            StationBCH::StationDistances>;
        PBNSToStationsImpl pbnsToStations(curVehLocToPickupSearches, fleet, routeState);

        using TaxiLegApproximationImpl = parrot::HeuristicEgressTripFinder<VehicleInputGraph, VehCHEnv, StationBucketsEnv>;

        // -> pass ULTRA algorithm instance and stationBucketsEnv, palsToStations to PTAndTaxiTripFinder
        using PTAndTaxiTripFinderImpl = parrot::PTAndTaxiTripFinder<
            VehicleInputGraph,
            VehCHEnv,
            StationBucketsEnv,
            StationBCH,
            PALSToStationsImpl,
            StationsInEllipseImpl,
            OrdinaryToStationsImpl,
            DALSToStationsImpl,
            PBNSToStationsImpl,
            EdgeQuery,
            PTAlgorithmWithTaxi,
            TaxiLegApproximationImpl>;
        PTAndTaxiTripFinderImpl ptAndTaxiTripFinder(vehicleInputGraph, *vehChEnv, fleet, routeState,
                                                    stations, queries, stationBucketsEnv, stationsAtLocations,
                                                    palsToStations, stationsInEllipse, dalsToStations, pbnsToStations,
                                                    ptAlgorithmWithTaxi);
#endif


        using KaRRiBaseInfoPreparatorImpl = KaRRiBaseInfoPreparator<VehicleInputGraph, VehCHEnv,
            FeasibleEllipticDistancesImpl, PDLocsFinderImpl, PDLocsAtExistingStopsFinderImpl, EllipticBCHSearchesImpl,
            PDDistanceQueryImpl, RelevantPDLocsFilterImpl>;
        KaRRiBaseInfoPreparatorImpl kaRRiBaseInfoPreparator(vehicleInputGraph, *vehChEnv, fleet, routeState,
                                                            pdLocsFinder, pdLocsAtExistingStops, ellipticSearches,
                                                            pdDistanceSearches);

        using WalkTripFinderImpl = parrot::WalkingTripFinder<VehicleInputGraph, PsgInputGraph, PsgCHEnv>;
        WalkTripFinderImpl walkTripFinder(vehicleInputGraph, psgInputGraph, *psgChEnv);

        using CarTripFinderImpl = parrot::CarTripFinder;
        CarTripFinderImpl carTripFinder;

        using TaxiTripFinderImpl = TaxiTripFinder<OrdinaryAssignmentsFinderImpl, PBNSInsertionsFinderImpl,
            PALSInsertionsFinderImpl, DALSInsertionsFinderImpl>;
        TaxiTripFinderImpl taxiTripFinder(ordinaryInsertionsFinder, pbnsInsertionsFinder, palsInsertionsFinder,
                                          dalsInsertionsFinder);

        using PTTripFinderImpl = parrot::PTJourneyFinder<EdgeQuery, PTAlgorithm>;
        PTTripFinderImpl ptTripFinder(queries, ptAlgorithm);

#if KARRI_OUTPUT_VEHICLE_PATHS
        using VehPathTracker = PathTracker<VehicleInputGraph, VehCHEnv, std::ofstream>;
        VehPathTracker pathTracker(vehicleInputGraph, *vehChEnv, routeState, fleet);
#else
        using VehPathTracker = NoOpPathTracker;
        VehPathTracker pathTracker;
#endif

        using SystemStateUpdaterImpl = SystemStateUpdater<VehicleInputGraph, EllipticBucketsEnv, LastStopBucketsEnv,
            StationsInEllipseImpl, CurVehLocToPickupSearchesImpl, VehPathTracker, std::ofstream>;
        SystemStateUpdaterImpl
                systemStateUpdater(vehicleInputGraph, curVehLocToPickupSearches,
                                   pathTracker, routeState, ellipticBucketsEnv, lastStopBucketsEnv, stationsInEllipse);


        // Initialize last stop state for initial locations of vehicles
        stats::UpdatePerformanceStats genInitialLastStopBucketsStats;
        for (const auto &veh: fleet) {
            lastStopBucketsEnv.generateIdleBucketEntries(veh, genInitialLastStopBucketsStats);
        }

        using ModeChoiceImpl = parrot::mode_choice::ModeChoice<parrot::mode_choice::UtilityLogitCriterion,
            std::ofstream>;
        ModeChoiceImpl modeChoice(routeState);

        // Run simulation:
        using EventSimulationImpl = EventSimulation<RequestStateInitializerImpl, KaRRiBaseInfoPreparatorImpl,
            WalkTripFinderImpl, CarTripFinderImpl, TaxiTripFinderImpl, PTTripFinderImpl, PTAndTaxiTripFinderImpl,
            ModeChoiceImpl, SystemStateUpdaterImpl, RouteState>;
        EventSimulationImpl eventSimulation(fleet, requests, requestStateInitializer, kaRRiBaseInfoPreparator,
                                            walkTripFinder, carTripFinder, taxiTripFinder, ptTripFinder,
                                            ptAndTaxiTripFinder, modeChoice, systemStateUpdater,
                                            routeState, stations, true);
        eventSimulation.run();
    } catch (std::exception &e) {
        std::cerr << "KaRRi error: " << e.what() << '\n';
        std::cerr << "Try '" << argv[0] << " -help' for more information.\n";
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
