#include <csv.h>

#include <cassert>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <random>

#include "../ULTRA/DataStructures/RAPTOR/Data.h"
#include "../ULTRA/DataStructures/RideRAPTOR/Data.h"

#include "../KARRI/Algorithms/CH/CH.h"
#include "../KARRI/Algorithms/KaRRi/AssignmentFinder.h"
#include "../KARRI/Algorithms/KaRRi/BaseObjects/Request.h"
#include "../KARRI/Algorithms/KaRRi/BaseObjects/Vehicle.h"
#include "../KARRI/Algorithms/KaRRi/CostCalculator.h"
#include "../KARRI/Algorithms/KaRRi/DalsAssignments/DALSAssignmentsFinder.h"
#include "../KARRI/Algorithms/KaRRi/EllipticBCH/EllipticBCHSearches.h"
#include "../KARRI/Algorithms/KaRRi/EllipticBCH/EllipticBucketsEnvironment.h"
#include "../KARRI/Algorithms/KaRRi/EllipticBCH/FeasibleEllipticDistances.h"
#include "../KARRI/Algorithms/KaRRi/EventSimulation.h"
#include "../KARRI/Algorithms/KaRRi/InputConfig.h"
#include "../KARRI/Algorithms/KaRRi/LastStopSearches/SortedLastStopBucketsEnvironment.h"
#include "../KARRI/Algorithms/KaRRi/LastStopSearches/UnsortedLastStopBucketsEnvironment.h"
#include "../KARRI/Algorithms/KaRRi/OrdinaryAssignments/OrdinaryAssignmentsFinder.h"
#include "../KARRI/Algorithms/KaRRi/PDDistanceQueries/PDDistances.h"
#include "../KARRI/Algorithms/KaRRi/PalsAssignments/PALSAssignmentsFinder.h"
#include "../KARRI/Algorithms/KaRRi/PbnsAssignments/PBNSAssignmentsFinder.h"
#include "../KARRI/Algorithms/KaRRi/PbnsAssignments/VehicleLocator.h"
#include "../KARRI/Algorithms/KaRRi/RequestState/RelevantPDLocs.h"
#include "../KARRI/Algorithms/KaRRi/RequestState/RelevantPDLocsFilter.h"
#include "../KARRI/Algorithms/KaRRi/RequestState/RequestState.h"
#include "../KARRI/Algorithms/KaRRi/RequestState/RequestStateInitializer.h"
#include "../KARRI/Algorithms/KaRRi/RequestState/VehicleToPDLocQuery.h"
#include "../KARRI/Algorithms/KaRRi/SystemStateUpdater.h"
#include "../KARRI/DataStructures/Graph/Attributes/CarEdgeToPsgEdgeAttribute.h"
#include "../KARRI/DataStructures/Graph/Attributes/EdgeIdAttribute.h"
#include "../KARRI/DataStructures/Graph/Attributes/EdgeTailAttribute.h"
#include "../KARRI/DataStructures/Graph/Attributes/FreeFlowSpeedAttribute.h"
#include "../KARRI/DataStructures/Graph/Attributes/LatLngAttribute.h"
#include "../KARRI/DataStructures/Graph/Attributes/OsmNodeIdAttribute.h"
#include "../KARRI/DataStructures/Graph/Attributes/PsgEdgeToCarEdgeAttribute.h"
#include "../KARRI/DataStructures/Graph/Attributes/TravelTimeAttribute.h"
#include "../KARRI/DataStructures/Graph/Graph.h"
#include "../KARRI/Tools/CommandLine/CommandLineParser.h"
#include "../KARRI/Tools/Logging/LogManager.h"

#ifdef KARRI_USE_CCHS
#include "../KARRI/Algorithms/KaRRi/CCHEnvironment.h"
#else
#include "../KARRI/Algorithms/KaRRi/CHEnvironment.h"
#endif

#if KARRI_PD_STRATEGY == KARRI_BCH_PD_STRAT

#include "../KARRI/Algorithms/KaRRi/PDDistanceQueries/BCHStrategy.h"

#else // KARRI_PD_STRATEGY == KARRI_CH_PD_STRAT
#include "../KARRI/Algorithms/KaRRi/PDDistanceQueries/CHStrategy.h"
#endif

#if KARRI_PALS_STRATEGY == KARRI_COL

#include "../KARRI/Algorithms/KaRRi/PalsAssignments/CollectiveBCHStrategy.h"

#elif KARRI_PALS_STRATEGY == KARRI_IND

#include "../KARRI/Algorithms/KaRRi/PalsAssignments/IndividualBCHStrategy.h"

#else // KARRI_PALS_STRATEGY == KARRI_DIJ

#include "../KARRI/Algorithms/KaRRi/PalsAssignments/DijkstraStrategy.h"

#endif

#if KARRI_DALS_STRATEGY == KARRI_COL

#include "../KARRI/Algorithms/KaRRi/DalsAssignments/CollectiveBCHStrategy.h"

#elif KARRI_DALS_STRATEGY == KARRI_IND

#include "../KARRI/Algorithms/KaRRi/DalsAssignments/IndividualBCHStrategy.h"

#else // KARRI_DALS_STRATEGY == KARRI_DIJ

#include "../KARRI/Algorithms/KaRRi/DalsAssignments/DijkstraStrategy.h"

#endif

inline void printUsage()
{
    std::cout
        << "Usage: EXECUTABLE -veh-g <vehicle network> -psg-g <passenger network> "
           "-v <vehicles> -o <file>\n"
           "Runs Karlsruhe Rapid Ridesharing (KaRRi) simulation with given "
           "vehicle and passenger road networks,\n"
           "requests, and vehicles. Writes output files to specified base path.\n"
           "  -veh-g <file>             vehicle road network in binary format.\n"
           "  -psg-g <file>             passenger road (and path) network in binary "
           "format.\n"
           "  -r <file>                 requests in CSV format.\n"
           "  -v <file>                 vehicles in CSV format.\n"
           "  -s <sec>                  stop time (in s). (dflt: 60s)\n"
           "  -w <sec>                  maximum wait time (in s). (dflt: 300s)\n"
           "  -a <factor>               model parameter alpha for max trip time = a "
           "* OD-dist + b (dflt: 1.7)\n"
           "  -b <seconds>              model parameter beta for max trip time = a "
           "* OD-dist + b (dflt: 120)\n"
           "  -p-radius <sec>           walking radius (in s) for pickup locations "
           "around origin. (dflt: 300s)\n"
           "  -d-radius <sec>           walking radius (in s) for dropoff locations "
           "around destination. (dflt: 300s)\n"
           "  -max-num-p <int>          max number of pickup locations to consider, "
           "sampled from all in radius. Set to 0 for no limit (dflt).\n"
           "  -max-num-d <int>          max number of dropoff locations to "
           "consider, sampled from all in radius. Set to 0 for no limit (dflt).\n"
           "  -always-veh               if set, the rider is not allowed to walk to "
           "their destination without a vehicle trip.\n"
           "  -veh-h <file>             contraction hierarchy for the vehicle "
           "network in binary format.\n"
           "  -psg-h <file>             contraction hierarchy for the passenger "
           "network in binary format.\n"
           "  -veh-d <file>             separator decomposition for the vehicle "
           "network in binary format (needed for CCHs).\n"
           "  -psg-d <file>             separator decomposition for the passenger "
           "network in binary format (needed for CCHs).\n"
           "  -csv-in-LOUD-format       if set, assumes that input files are in the "
           "format used by LOUD.\n"
           "  -o <file>                 generate output files at name <file> "
           "(specify name without file suffix).\n"
           "  -station-mapping <file>   file which maps the station used in RAPTOR to edges in the given road graph\n"
           "  -raptor-data <file>       file with the precomputed RAPTOR data\n"
           "  -help                     show usage help text.\n";
}

int main(int argc, char* argv[])
{
    try {
        CommandLineParser clp(argc, argv);
        if (clp.isSet("help")) {
            printUsage();
            return EXIT_SUCCESS;
        }

        // Parse the command-line options.
        karri::InputConfig inputConfig {};
        inputConfig.stopTime = clp.getValue<int>("s", 60) * 10;
        inputConfig.maxWaitTime = clp.getValue<int>("w", 300) * 10;
        inputConfig.pickupRadius = clp.getValue<int>("p-radius", inputConfig.maxWaitTime / 10) * 10;
        inputConfig.dropoffRadius = clp.getValue<int>("d-radius", inputConfig.maxWaitTime / 10) * 10;
        inputConfig.maxNumPickups = clp.getValue<int>("max-num-p", INFTYKARRI);
        inputConfig.maxNumDropoffs = clp.getValue<int>("max-num-d", INFTYKARRI);
        inputConfig.alwaysUseVehicle = clp.isSet("always-veh");
        if (inputConfig.maxNumPickups == 0)
            inputConfig.maxNumPickups = INFTYKARRI;
        if (inputConfig.maxNumDropoffs == 0)
            inputConfig.maxNumDropoffs = INFTYKARRI;
        inputConfig.alpha = clp.getValue<double>("a", 1.7);
        inputConfig.beta = clp.getValue<int>("b", 120) * 10;
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
        const auto stationMappingFileName = clp.getValue<std::string>("station-mapping");
        const auto raptorFileName = clp.getValue<std::string>("raptor-data");

        auto outputFileName = clp.getValue<std::string>("o");
        if (endsWith(outputFileName, ".csv"))
            outputFileName = outputFileName.substr(0, outputFileName.size() - 4);

        LogManager<std::ofstream>::setBaseFileName(outputFileName + ".");

        // Read the vehicle network from file.
        std::cout << "Reading vehicle network from file... " << std::flush;
        using VehicleVertexAttributes = karri::VertexAttrs<LatLngAttribute, OsmNodeIdAttribute>;
        using VehicleEdgeAttributes = karri::EdgeAttrs<EdgeIdAttribute, EdgeTailAttribute, FreeFlowSpeedAttribute,
            TravelTimeAttribute, CarEdgeToPsgEdgeAttribute,
            OsmRoadCategoryAttribute>;
        using VehicleInputGraph = karri::StaticGraph<VehicleVertexAttributes, VehicleEdgeAttributes>;
        std::ifstream vehicleNetworkFile(vehicleNetworkFileName, std::ios::binary);
        if (!vehicleNetworkFile.good())
            throw std::invalid_argument("file not found -- '" + vehicleNetworkFileName + "'");
        VehicleInputGraph vehicleInputGraph(vehicleNetworkFile);
        vehicleNetworkFile.close();
        std::vector<int32_t> vehGraphOrigIdToSeqId;
        if (vehicleInputGraph.numEdges() > 0 && vehicleInputGraph.edgeId(0) == INVALID_ID) {
            vehGraphOrigIdToSeqId.assign(vehicleInputGraph.numEdges(), INVALID_ID);
            std::iota(vehGraphOrigIdToSeqId.begin(), vehGraphOrigIdToSeqId.end(), 0);
            FORALL_VALID_EDGES(vehicleInputGraph, v, e)
            {
                assert(vehicleInputGraph.edgeId(e) == INVALID_ID);
                vehicleInputGraph.edgeTail(e) = v;
                vehicleInputGraph.edgeId(e) = e;
            }
        } else {
            FORALL_VALID_EDGES(vehicleInputGraph, v, e)
            {
                assert(vehicleInputGraph.edgeId(e) != INVALID_ID);
                if (vehicleInputGraph.edgeId(e) >= vehGraphOrigIdToSeqId.size()) {
                    const auto numElementsToBeInserted = vehicleInputGraph.edgeId(e) + 1 - vehGraphOrigIdToSeqId.size();
                    vehGraphOrigIdToSeqId.insert(vehGraphOrigIdToSeqId.end(),
                        numElementsToBeInserted, INVALID_ID);
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
        using PsgVertexAttributes = karri::VertexAttrs<LatLngAttribute, OsmNodeIdAttribute>;
        using PsgEdgeAttributes = karri::EdgeAttrs<EdgeIdAttribute, EdgeTailAttribute, PsgEdgeToCarEdgeAttribute,
            TravelTimeAttribute>;
        using PsgInputGraph = karri::StaticGraph<PsgVertexAttributes, PsgEdgeAttributes>;
        std::ifstream psgNetworkFile(passengerNetworkFileName, std::ios::binary);
        if (!psgNetworkFile.good())
            throw std::invalid_argument("file not found -- '" + passengerNetworkFileName + "'");
        PsgInputGraph psgInputGraph(psgNetworkFile);
        psgNetworkFile.close();
        assert(psgInputGraph.numEdges() > 0 && psgInputGraph.edgeId(0) == INVALID_ID);
        int numEdgesWithMappingToCar = 0;
        FORALL_VALID_EDGES(psgInputGraph, v, e)
        {
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

                assert(
                    psgInputGraph.latLng(psgInputGraph.edgeHead(e)).latitude() == vehicleInputGraph.latLng(vehicleInputGraph.edgeHead(psgInputGraph.toCarEdge(e))).latitude());
                assert(
                    psgInputGraph.latLng(psgInputGraph.edgeHead(e)).longitude() == vehicleInputGraph.latLng(vehicleInputGraph.edgeHead(psgInputGraph.toCarEdge(e))).longitude());
            }
        }
        unused(numEdgesWithMappingToCar);
        assert(numEdgesWithMappingToCar > 0);
        std::cout << "done.\n";

        // Read the vehicle data from file.
        std::cout << "Reading vehicle data from file... " << std::flush;
        karri::Fleet fleet;
        int location, capacity, startOfServiceTime, endOfServiceTime;
        io::CSVReader<4, io::trim_chars<' '>> vehiclesFileReader(vehicleFileName);

        if (csvFilesInLoudFormat) {
            vehiclesFileReader.read_header(io::ignore_no_column, "initial_location",
                "seating_capacity", "start_service_time",
                "end_service_time");
        } else {
            vehiclesFileReader.read_header(io::ignore_no_column, "initial_location",
                "start_of_service_time",
                "end_of_service_time", "capacity");
        }

        int maxCapacity = 0;
        while ((csvFilesInLoudFormat && vehiclesFileReader.read_row(location, capacity, startOfServiceTime, endOfServiceTime)) || (!csvFilesInLoudFormat && vehiclesFileReader.read_row(location, startOfServiceTime, endOfServiceTime, capacity))) {
            if (location < 0 || location >= vehGraphOrigIdToSeqId.size() || vehGraphOrigIdToSeqId[location] == INVALID_ID)
                throw std::invalid_argument("invalid location -- '" + std::to_string(location) + "'");
            if (endOfServiceTime <= startOfServiceTime)
                throw std::invalid_argument(
                    "start of service time needs to be before end of service time");
            const int vehicleId = static_cast<int>(fleet.size());
            fleet.push_back({ vehicleId, vehGraphOrigIdToSeqId[location],
                startOfServiceTime * 10, endOfServiceTime * 10,
                capacity });
            maxCapacity = std::max(maxCapacity, capacity);
        }
        std::cout << "done.\n";

        // Create Route State for empty routes.
        karri::RouteState routeState(fleet, inputConfig.stopTime);

        // Read the request data from file.
        std::cout << "Reading request data from file... " << std::flush;
        std::vector<karri::Request> requests;
        int origin, destination, requestTime, numRiders;
        io::CSVReader<4, io::trim_chars<' '>> reqFileReader(requestFileName);

        if (csvFilesInLoudFormat) {
            reqFileReader.read_header(io::ignore_missing_column, "pickup_spot",
                "dropoff_spot", "min_dep_time", "num_riders");
        } else {
            reqFileReader.read_header(io::ignore_missing_column, "origin",
                "destination", "req_time", "num_riders");
        }

        numRiders = -1;
        while (
            reqFileReader.read_row(origin, destination, requestTime, numRiders)) {
            if (origin < 0 || origin >= vehGraphOrigIdToSeqId.size() || vehGraphOrigIdToSeqId[origin] == INVALID_ID)
                throw std::invalid_argument("invalid location -- '" + std::to_string(origin) + "'");
            if (destination < 0 || destination >= vehGraphOrigIdToSeqId.size() || vehGraphOrigIdToSeqId[destination] == INVALID_ID)
                throw std::invalid_argument("invalid location -- '" + std::to_string(destination) + "'");
            if (numRiders > maxCapacity)
                throw std::invalid_argument("number of riders '" + std::to_string(numRiders) + "' is larger than max vehicle capacity (" + std::to_string(maxCapacity) + ")");
            const auto originSeqId = vehGraphOrigIdToSeqId[origin];
            assert(vehicleInputGraph.toPsgEdge(originSeqId) != CarEdgeToPsgEdgeAttribute::defaultValue());
            const auto destSeqId = vehGraphOrigIdToSeqId[destination];
            assert(vehicleInputGraph.toPsgEdge(destSeqId) != CarEdgeToPsgEdgeAttribute::defaultValue());
            const int requestId = static_cast<int>(requests.size());
            if (numRiders == -1) // If number of riders was not specified, assume one rider
                numRiders = 1;
            requests.push_back(
                { requestId, originSeqId, destSeqId, requestTime * 10, numRiders });
            numRiders = -1;
        }
        std::cout << "done.\n";

#ifdef KARRI_USE_CCHS

        // Prepare vehicle CH environment
        using VehCHEnv = karri::CCHEnvironment<VehicleInputGraph, TravelTimeAttribute>;
        std::unique_ptr<VehCHEnv> vehChEnv;
        if (vehSepDecompFileName.empty()) {
            std::cout << "Building Separator Decomposition and CCH... " << std::flush;
            vehChEnv = std::make_unique<VehCHEnv>(vehicleInputGraph);
            std::cout << "done.\n";
        } else {
            // Read the separator decomposition from file, construct and customize
            // CCH.
            std::cout
                << "Reading Seperator Decomposition from file and building CCH... "
                << std::flush;
            std::ifstream vehSepDecompFile(vehSepDecompFileName, std::ios::binary);
            if (!vehSepDecompFile.good())
                throw std::invalid_argument("file not found -- '" + vehSepDecompFileName + "'");
            karri::SeparatorDecomposition vehSepDecomp;
            vehSepDecomp.readFrom(vehSepDecompFile);
            vehSepDecompFile.close();
            std::cout << "done.\n";
            vehChEnv = std::make_unique<VehCHEnv>(vehicleInputGraph, vehSepDecomp);
        }

        // Prepare passenger CH environment
        using PsgCHEnv = karri::CCHEnvironment<PsgInputGraph, TravelTimeAttribute>;
        std::unique_ptr<PsgCHEnv> psgChEnv;
        if (psgSepDecompFileName.empty()) {
            std::cout << "Building Separator Decomposition and CCH... " << std::flush;
            psgChEnv = std::make_unique<PsgCHEnv>(psgInputGraph);
            std::cout << "done.\n";
        } else {
            // Read the separator decomposition from file, construct and customize
            // CCH.
            std::cout
                << "Reading Seperator Decomposition from file and building CCH... "
                << std::flush;
            std::ifstream psgSepDecompFile(psgSepDecompFileName, std::ios::binary);
            if (!psgSepDecompFile.good())
                throw std::invalid_argument("file not found -- '" + psgSepDecompFileName + "'");
            karri::SeparatorDecomposition psgSepDecomp;
            psgSepDecomp.readFrom(psgSepDecompFile);
            psgSepDecompFile.close();
            std::cout << "done.\n";
            psgChEnv = std::make_unique<PsgCHEnv>(psgInputGraph, psgSepDecomp);
        }

#else
        // Prepare vehicle CH environment
        using VehCHEnv = karri::CHEnvironment<VehicleInputGraph, TravelTimeAttribute>;
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
            karri::CH vehCh(vehHierarchyFile);
            vehHierarchyFile.close();
            std::cout << "done.\n";
            vehChEnv = std::make_unique<VehCHEnv>(std::move(vehCh));
        }

        // Prepare passenger CH environment
        using PsgCHEnv = karri::CHEnvironment<PsgInputGraph, TravelTimeAttribute>;
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
            karri::CH psgCh(psgHierarchyFile);
            psgHierarchyFile.close();
            std::cout << "done.\n";
            psgChEnv = std::make_unique<PsgCHEnv>(std::move(psgCh));
        }
#endif

        using VehicleLocatorImpl = karri::VehicleLocator<VehicleInputGraph, VehCHEnv>;
        VehicleLocatorImpl locator(vehicleInputGraph, *vehChEnv, routeState);

        karri::CostCalculator calc(routeState, inputConfig);
        karri::RequestState reqState(calc, inputConfig);

        // Construct Elliptic BCH searches:
        static constexpr bool ELLIPTIC_SORTED_BUCKETS = KARRI_ELLIPTIC_BCH_SORTED_BUCKETS;
        using EllipticBucketsEnv = karri::EllipticBucketsEnvironment<VehicleInputGraph, VehCHEnv,
            ELLIPTIC_SORTED_BUCKETS>;
        EllipticBucketsEnv ellipticBucketsEnv(vehicleInputGraph, *vehChEnv,
            routeState, inputConfig,
            reqState.stats().updateStats);

        using EllipticBCHLabelSet = std::conditional_t<
            KARRI_ELLIPTIC_BCH_USE_SIMD,
            SimdLabelSet<KARRI_ELLIPTIC_BCH_LOG_K, ParentInfo::NO_PARENT_INFO>,
            BasicLabelSet<KARRI_ELLIPTIC_BCH_LOG_K, ParentInfo::NO_PARENT_INFO>>;
        using FeasibleEllipticDistancesImpl = karri::FeasibleEllipticDistances<EllipticBCHLabelSet>;
        FeasibleEllipticDistancesImpl feasibleEllipticPickups(fleet.size(),
            routeState);
        FeasibleEllipticDistancesImpl feasibleEllipticDropoffs(fleet.size(),
            routeState);

        karri::LastStopsAtVertices lastStopsAtVertices(vehicleInputGraph.numVertices(),
            fleet.size());

        using EllipticBCHSearchesImpl = karri::EllipticBCHSearches<VehicleInputGraph, VehCHEnv,
            karri::CostCalculator::CostFunction, EllipticBucketsEnv,
            FeasibleEllipticDistancesImpl, EllipticBCHLabelSet>;
        EllipticBCHSearchesImpl ellipticSearches(
            vehicleInputGraph, fleet, ellipticBucketsEnv, lastStopsAtVertices,
            *vehChEnv, routeState, feasibleEllipticPickups,
            feasibleEllipticDropoffs, reqState);

        // Construct remaining request state
        karri::RelevantPDLocs relOrdinaryPickups(fleet.size());
        karri::RelevantPDLocs relOrdinaryDropoffs(fleet.size());
        karri::RelevantPDLocs relPickupsBeforeNextStop(fleet.size());
        karri::RelevantPDLocs relDropoffsBeforeNextStop(fleet.size());
        using RelevantPDLocsFilterImpl = karri::RelevantPDLocsFilter<FeasibleEllipticDistancesImpl, VehicleInputGraph,
            VehCHEnv>;
        RelevantPDLocsFilterImpl relevantPdLocsFilter(
            fleet, vehicleInputGraph, *vehChEnv, calc, reqState, routeState,
            inputConfig, feasibleEllipticPickups, feasibleEllipticDropoffs,
            relOrdinaryPickups, relOrdinaryDropoffs, relPickupsBeforeNextStop,
            relDropoffsBeforeNextStop);

        const auto revVehicleGraph = vehicleInputGraph.getReverseGraph();
        using VehicleToPDLocQueryImpl = karri::VehicleToPDLocQuery<VehicleInputGraph>;
        VehicleToPDLocQueryImpl vehicleToPdLocQuery(vehicleInputGraph, revVehicleGraph);

        // Construct PD-distance query
        using PDDistancesLabelSet = std::conditional_t<KARRI_PD_DISTANCES_USE_SIMD,
            SimdLabelSet<KARRI_PD_DISTANCES_LOG_K, ParentInfo::NO_PARENT_INFO>,
            BasicLabelSet<KARRI_PD_DISTANCES_LOG_K, ParentInfo::NO_PARENT_INFO>>;
        using PDDistancesImpl = karri::PDDistances<PDDistancesLabelSet>;
        PDDistancesImpl pdDistances(reqState);

#if KARRI_PD_STRATEGY == KARRI_BCH_PD_STRAT
        using PDDistanceQueryImpl = karri::PDDistanceQueryStrategies::BCHStrategy<VehicleInputGraph, VehCHEnv, VehicleToPDLocQueryImpl, PDDistancesLabelSet>;
        PDDistanceQueryImpl pdDistanceQuery(vehicleInputGraph, *vehChEnv, pdDistances, reqState, vehicleToPdLocQuery);

#else // KARRI_PD_STRATEGY == KARRI_CH_PD_STRAT
        using PDDistanceQueryImpl = karri::PDDistanceQueryStrategies::CHStrategy<VehicleInputGraph, VehCHEnv, PDDistancesLabelSet>;
        PDDistanceQueryImpl pdDistanceQuery(vehicleInputGraph, *vehChEnv, pdDistances, reqState);
#endif

        // Construct ordinary assignments finder:
        using OrdinaryAssignmentsFinderImpl = karri::OrdinaryAssignmentsFinder<PDDistancesImpl>;
        OrdinaryAssignmentsFinderImpl ordinaryInsertionsFinder(relOrdinaryPickups, relOrdinaryDropoffs,
            pdDistances, fleet, calc, routeState,
            reqState);

        // Construct PBNS assignments finder:
        using CurVehLocToPickupLabelSet = PDDistancesLabelSet;
        using CurVehLocToPickupSearchesImpl = karri::CurVehLocToPickupSearches<VehicleInputGraph, VehicleLocatorImpl, VehCHEnv, CurVehLocToPickupLabelSet>;
        CurVehLocToPickupSearchesImpl curVehLocToPickupSearches(vehicleInputGraph, locator, *vehChEnv, routeState,
            reqState, fleet.size());

        using PBNSInsertionsFinderImpl = karri::PBNSAssignmentsFinder<PDDistancesImpl, CurVehLocToPickupSearchesImpl>;
        PBNSInsertionsFinderImpl pbnsInsertionsFinder(relPickupsBeforeNextStop, relOrdinaryDropoffs,
            relDropoffsBeforeNextStop, pdDistances, curVehLocToPickupSearches,
            fleet, calc, routeState, reqState);

        // If we use any BCH queries in the PALS or DALS strategies, we construct the according bucket data structure.
        // Otherwise, we use no-op last stop buckets.

#if KARRI_PALS_STRATEGY == KARRI_COL || KARRI_PALS_STRATEGY == KARRI_IND || KARRI_DALS_STRATEGY == KARRI_COL || KARRI_DALS_STRATEGY == KARRI_IND

        static constexpr bool LAST_STOP_SORTED_BUCKETS = KARRI_LAST_STOP_BCH_SORTED_BUCKETS;
        using LastStopBucketsEnv = std::conditional_t<LAST_STOP_SORTED_BUCKETS,
            karri::SortedLastStopBucketsEnvironment<VehicleInputGraph, VehCHEnv>,
            karri::UnsortedLastStopBucketsEnvironment<VehicleInputGraph, VehCHEnv>>;
        LastStopBucketsEnv lastStopBucketsEnv(vehicleInputGraph, *vehChEnv, routeState, reqState.stats().updateStats);

#else
        using LastStopBucketsEnv = karri::NoOpLastStopBucketsEnvironment;
        LastStopBucketsEnv lastStopBucketsEnv;
#endif

        // Construct PALS strategy and assignment finder:
        using PALSLabelSet = std::conditional_t<KARRI_PALS_USE_SIMD,
            SimdLabelSet<KARRI_PALS_LOG_K, ParentInfo::NO_PARENT_INFO>,
            BasicLabelSet<KARRI_PALS_LOG_K, ParentInfo::NO_PARENT_INFO>>;

#if KARRI_PALS_STRATEGY == KARRI_COL
        // Use Collective-BCH PALS Strategy
        using PALSStrategy = karri::PickupAfterLastStopStrategies::CollectiveBCHStrategy<VehicleInputGraph, VehCHEnv, LastStopBucketsEnv, VehicleToPDLocQueryImpl, PDDistancesImpl, PALSLabelSet>;
        PALSStrategy palsStrategy(vehicleInputGraph, fleet, *vehChEnv,
            vehicleToPdLocQuery, lastStopBucketsEnv, pdDistances, calc, routeState, reqState,
            inputConfig);
#elif KARRI_PALS_STRATEGY == KARRI_IND
        // Use Individual-BCH PALS Strategy
        using PALSStrategy = karri::PickupAfterLastStopStrategies::IndividualBCHStrategy<VehicleInputGraph, VehCHEnv, LastStopBucketsEnv, PDDistancesImpl, PALSLabelSet>;
        PALSStrategy palsStrategy(vehicleInputGraph, fleet, *vehChEnv, calc, lastStopBucketsEnv, pdDistances,
            routeState, reqState, reqState.getBestCost(), inputConfig);
#else // KARRI_PALS_STRATEGY == KARRI_DIJ
      // Use Dijkstra PALS Strategy
        using PALSStrategy = karri::PickupAfterLastStopStrategies::DijkstraStrategy<VehicleInputGraph, PDDistancesImpl, PALSLabelSet>;
        PALSStrategy palsStrategy(vehicleInputGraph, revVehicleGraph, fleet, routeState, lastStopsAtVertices, calc, pdDistances, reqState, inputConfig);
#endif

        using PALSInsertionsFinderImpl = karri::PALSAssignmentsFinder<VehicleInputGraph, PDDistancesImpl, PALSStrategy>;
        PALSInsertionsFinderImpl palsInsertionsFinder(palsStrategy, vehicleInputGraph, fleet, calc, lastStopsAtVertices,
            routeState, pdDistances, reqState);

        // Construct DALS strategy and assignment finder:

#if KARRI_DALS_STRATEGY == KARRI_COL
        // Use Collective-BCH DALS Strategy
        using DALSStrategy = karri::DropoffAfterLastStopStrategies::CollectiveBCHStrategy<VehicleInputGraph, VehCHEnv, LastStopBucketsEnv, CurVehLocToPickupSearchesImpl>;
        DALSStrategy dalsStrategy(vehicleInputGraph, fleet, routeState, *vehChEnv, lastStopBucketsEnv, calc,
            curVehLocToPickupSearches, reqState, relOrdinaryPickups, relPickupsBeforeNextStop,
            inputConfig);
#elif KARRI_DALS_STRATEGY == KARRI_IND
        // Use Individual-BCH DALS Strategy
        using DALSLabelSet = std::conditional_t<KARRI_DALS_USE_SIMD,
            karri::SimdLabelSet<KARRI_DALS_LOG_K, ParentInfo::NO_PARENT_INFO>,
            karri::BasicLabelSet<KARRI_DALS_LOG_K, ParentInfo::NO_PARENT_INFO>>;
        using DALSStrategy = karri::DropoffAfterLastStopStrategies::IndividualBCHStrategy<VehicleInputGraph, VehCHEnv, LastStopBucketsEnv, CurVehLocToPickupSearchesImpl, DALSLabelSet>;
        DALSStrategy dalsStrategy(vehicleInputGraph, fleet, *vehChEnv, calc, lastStopBucketsEnv,
            curVehLocToPickupSearches, routeState, reqState, relOrdinaryPickups,
            relPickupsBeforeNextStop);
#else // KARRI_DALS_STRATEGY == KARRI_DIJ
      // Use Dijkstra DALS Strategy
        using DALSLabelSet = std::conditional_t<KARRI_DALS_USE_SIMD,
            karri::SimdLabelSet<KARRI_DALS_LOG_K, ParentInfo::NO_PARENT_INFO>,
            karri::BasicLabelSet<KARRI_DALS_LOG_K, ParentInfo::NO_PARENT_INFO>>;
        using DALSStrategy = karri::DropoffAfterLastStopStrategies::DijkstraStrategy<VehicleInputGraph, CurVehLocToPickupSearchesImpl, DALSLabelSet>;
        DALSStrategy dalsStrategy(vehicleInputGraph, revVehicleGraph, fleet, calc, curVehLocToPickupSearches, routeState, lastStopsAtVertices, reqState, relOrdinaryPickups, relPickupsBeforeNextStop);
#endif

        using DALSInsertionsFinderImpl = karri::DALSAssignmentsFinder<DALSStrategy>;
        DALSInsertionsFinderImpl dalsInsertionsFinder(dalsStrategy);

        using RequestStateInitializerImpl = karri::RequestStateInitializer<VehicleInputGraph, PsgInputGraph, VehCHEnv, PsgCHEnv, VehicleToPDLocQueryImpl>;
        RequestStateInitializerImpl requestStateInitializer(vehicleInputGraph, psgInputGraph, *vehChEnv, *psgChEnv,
            reqState, inputConfig, vehicleToPdLocQuery);

        using InsertionFinderImpl = karri::AssignmentFinder<RequestStateInitializerImpl,
            EllipticBCHSearchesImpl,
            PDDistanceQueryImpl,
            OrdinaryAssignmentsFinderImpl,
            PBNSInsertionsFinderImpl,
            PALSInsertionsFinderImpl,
            DALSInsertionsFinderImpl,
            RelevantPDLocsFilterImpl>;
        InsertionFinderImpl insertionFinder(reqState, requestStateInitializer, ellipticSearches, pdDistanceQuery,
            ordinaryInsertionsFinder, pbnsInsertionsFinder, palsInsertionsFinder,
            dalsInsertionsFinder, relevantPdLocsFilter);

#if KARRI_OUTPUT_VEHICLE_PATHS
        using VehPathTracker = PathTracker<VehicleInputGraph, VehCHEnv, std::ofstream>;
        VehPathTracker pathTracker(vehicleInputGraph, *vehChEnv, reqState, routeState, fleet.size());
#else
        using VehPathTracker = karri::NoOpPathTracker;
        VehPathTracker pathTracker;
#endif

        using SystemStateUpdaterImpl = karri::SystemStateUpdater<VehicleInputGraph, EllipticBucketsEnv, LastStopBucketsEnv, CurVehLocToPickupSearchesImpl, VehPathTracker, std::ofstream>;
        SystemStateUpdaterImpl systemStateUpdater(vehicleInputGraph, reqState, inputConfig, curVehLocToPickupSearches,
            pathTracker, routeState, ellipticBucketsEnv, lastStopBucketsEnv,
            lastStopsAtVertices);

        // Initialize last stop state for initial locations of vehicles
        for (const auto& veh : fleet) {
            const auto head = vehicleInputGraph.edgeHead(veh.initialLocation);
            lastStopsAtVertices.insertLastStopAt(head, veh.vehicleId);
            lastStopBucketsEnv.generateIdleBucketEntries(veh);
        }

        // 1) run some requests to start the vehicles
        int numberOfSimulatedRequests = 100;
        int startTimeOfSimulatedRequest = 0;
        int endTimeOfSimulatedRequest = 24 * 60 * 60; // midnight in secs

        /* std::vector<karri::Request> simulatedRequests; */
        /* simulatedRequests.reserve(numberOfSimulatedRequests); */

        /* // generate random requests */
        /* std::mt19937 randomGenerator(2 + 42); */
        /* std::uniform_int_distribution<> locationDistribution(0, psgInputGraph.numEdges() >> 1); */
        /* std::uniform_int_distribution<> timeDistribution(startTimeOfSimulatedRequest, endTimeOfSimulatedRequest + 1); */

        /* for (int reqId = 0; reqId < numberOfSimulatedRequests; ++reqId) { */
        /*     simulatedRequests.push_back({ */
        /*         reqId, */
        /*         locationDistribution(randomGenerator), */
        /*         locationDistribution(randomGenerator), */
        /*         timeDistribution(randomGenerator), */
        /*         1 // num of passenger */
        /*     }); */
        /* } */
        requests.resize(500);

        // Run simulation:
        using EventSimulationImpl = karri::EventSimulation<InsertionFinderImpl, SystemStateUpdaterImpl, karri::RouteState>;
        EventSimulationImpl eventSimulation(fleet, requests, inputConfig.stopTime,
            insertionFinder, systemStateUpdater,
            routeState, true);

        eventSimulation.run();

        // 2) now the RideRAPTOR stuff
        // Read the station mapping file
        std::cout << "Reading station mapping from file... " << std::flush;
        std::vector<int> edgeIdOfStation;
        int edgeId;
        io::CSVReader<1> stationMappingFileReader(stationMappingFileName);

        stationMappingFileReader.read_header(io::ignore_no_column, "initial_location");

        while (stationMappingFileReader.read_row(edgeId)) {
            if (edgeId < 0)
                throw std::invalid_argument("invalid edge id for a station-- '" + std::to_string(edgeId) + "'");
            edgeIdOfStation.push_back(edgeId);
        }
        std::cout << "done.\n";

        // Read the RAPTOR data
        std::cout << "Reading RAPTOR data from file... " << std::flush;
        RAPTOR::Data raptor(raptorFileName);
        std::cout << "done.\n";

        raptor.printInfo();

        // Build the RideRAPTOR data
        std::cout << "Building RideRAPTOR data... " << std::flush;

        RIDERAPTOR::Data<FeasibleEllipticDistancesImpl, VehicleInputGraph, VehCHEnv, EllipticBCHSearchesImpl> rideRaptor(
            raptor,
            fleet,
            vehicleInputGraph,
            *vehChEnv,
            calc,
            reqState,
            routeState,
            inputConfig,
            feasibleEllipticPickups,
            feasibleEllipticDropoffs,
            relOrdinaryPickups,
            relOrdinaryDropoffs,
            relPickupsBeforeNextStop,
            relDropoffsBeforeNextStop,
            ellipticSearches,
            edgeIdOfStation);

        rideRaptor.buildRideTransferGraph();
        std::cout << "done.\n";

        rideRaptor.rideTransferGraph.printAnalysis();

    } catch (std::exception& e) {
        std::cerr << argv[0] << ": " << e.what() << '\n';
        std::cerr << "Try '" << argv[0] << " -help' for more information.\n";
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
