#include <cassert>
#include <kassert/kassert.hpp>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <csv.h>
#include <functional>

#include <Common/Constants.h>
#include <KARRI/Tools/CommandLine/CommandLineParser.h>
#include <KARRI/Tools/Logging/LogManager.h>
#include <KARRI/DataStructures/Graph/Graph.h>
#include <KARRI/DataStructures/Graph/Attributes/LatLngAttribute.h>
#include <KARRI/DataStructures/Graph/Attributes/EdgeIdAttribute.h>
#include <KARRI/DataStructures/Graph/Attributes/OsmNodeIdAttribute.h>
#include <KARRI/DataStructures/Graph/Attributes/EdgeTailAttribute.h>
#include <KARRI/DataStructures/Graph/Attributes/PsgEdgeToCarEdgeAttribute.h>
#include <KARRI/DataStructures/Graph/Attributes/FreeFlowSpeedAttribute.h>
#include <KARRI/DataStructures/Graph/Attributes/CarEdgeToPsgEdgeAttribute.h>
#include <KARRI/DataStructures/Graph/Attributes/OsmRoadCategoryAttribute.h>


#include <KARRI/Algorithms/KaRRi/CHEnvironment.h>
#include <KARRI/Algorithms/CH/CH.h>

#include <ULTRA/Algorithms/CH/Query/BucketQuery.h>
#include <ULTRA/DataStructures/RAPTOR/Data.h>

#include "../PTaxi/Station.h"
#include "../PTaxi/StationBucketsEnvironment.h"


inline void printUsage() {
    std::cout <<
              "Usage: \n"
              "     BuildStaticBuckets -veh-g <file> -psg-g <file> -veh-h <file> -psg-h <file> -raptor-data <file> -station-mapping <file>\n"
              "     -o-bucket-graph <file> -o-station-buckets <file>\n"
              "Build static buckets for the preprocessing of PTaxi and write them to the specified output files.\n"
              "1. Build the bucket graph for the passenger network for use in ULTRA algorithm.\n"
              "2. Build the buckets for the stations in the vehicle network for use in KaRRi.\n"
              "  -veh-g <file>              vehicle road network in binary format.\n"
              "  -psg-g <file>              passenger road (and path) network in binary format.\n"
              "  -veh-h <file>              contraction hierarchy for the vehicle network in binary format.\n"
              "  -psg-h <file>              contraction hierarchy for the passenger network in binary format.\n"
              "  -raptor-data <file>        file with the precomputed RAPTOR data.\n"
              "  -station-mapping <file>    file which maps the station used in RAPTOR to edges in the given road graph.\n"
              "  -ch <file>                 contraction hierarchy for the transfer graph of ULTRA in binary format.\n"
              "  -o-bucket-graph <file>     bucket graph for ULTRA in <file>.\n"
              "  -o-station-buckets <file>  station buckets for KaRRi in <file>.\n"
              "  -help                      display this help and exit.\n";
}

int main(int argc, char *argv[]) {
    
    try {
        CommandLineParser clp(argc, argv);
        if (clp.isSet("help")) {
            printUsage();
            return EXIT_SUCCESS;
        }


        // Parse the command-line options.
        const auto vehicleNetworkFileName = clp.getValue<std::string>("veh-g");
        const auto passengerNetworkFileName = clp.getValue<std::string>("psg-g");
        const auto vehHierarchyFileName = clp.getValue<std::string>("veh-h");
        const auto psgHierarchyFileName = clp.getValue<std::string>("psg-h");
        const auto raptorFileName = clp.getValue<std::string>("raptor-data");
        const auto stationMappingFileName = clp.getValue<std::string>("station-mapping");
        const auto chFileName = clp.getValue<std::string>("ch");
        const auto bucketGraphOutputFileName = clp.getValue<std::string>("o-bucket-graph");
        auto stationBucketsOutputFilename = clp.getValue<std::string>("o-station-buckets");
        if (!endsWith(stationBucketsOutputFilename, ".bucket.bin")) stationBucketsOutputFilename += ".bucket.bin";

        // Read the vehicle network from file.
        std::cout << "Reading vehicle network from file... " << std::flush;
        using VehicleVertexAttributes = VertexAttrs<LatLngAttribute, OsmNodeIdAttribute>;
        using VehicleEdgeAttributes = EdgeAttrs<
                EdgeIdAttribute, EdgeTailAttribute, FreeFlowSpeedAttribute, TravelTimeAttribute, CarEdgeToPsgEdgeAttribute, OsmRoadCategoryAttribute>;
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
        using PsgEdgeAttributes = EdgeAttrs<EdgeIdAttribute, EdgeTailAttribute, PsgEdgeToCarEdgeAttribute, TravelTimeAttribute>;
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
            CH vehCh(vehHierarchyFile);
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
            CH psgCh(psgHierarchyFile);
            psgHierarchyFile.close();
            std::cout << "done.\n";
            psgChEnv = std::make_unique<PsgCHEnv>(std::move(psgCh));
        }

        // Read the RAPTOR data
        std::cout << "Reading RAPTOR data from file... " << std::flush;
        RAPTOR::Data raptor(raptorFileName);
        raptor.useImplicitDepartureBufferTimes();
        std::cout << "done.\n";

        raptor.printInfo();

        // Read the station mapping file
        std::cout << "Reading station mapping from file... " << std::flush;
        PTStations stations;
        int edgeId;
        int stationId = 0;
        io::CSVReader<1> stationMappingFileReader(stationMappingFileName);

        stationMappingFileReader.read_header(io::ignore_no_column, "initial_location");

        while (stationMappingFileReader.read_row(edgeId)) {
            if (edgeId < 0) {
                throw std::invalid_argument("invalid edge id for a station-- '" + std::to_string(edgeId) + "'");
            }

            // edge id in the station mapping file is the edge id in the road network
            int psgEdgeId = vehicleInputGraph.toPsgEdge(edgeId);
            int psgVertexId = psgInputGraph.edgeHead(psgEdgeId);
            int psgChOrder = psgChEnv->getCH().rank(psgVertexId);
            stations.push_back({stationId, psgEdgeId, psgChOrder, edgeId});
            stationId++;
        }
        std::cout << "done.\n";
        
        std::cout << "Reading ULTRA CH from file... " << std::flush;
        ULTRACH::CH ch(chFileName);
        std::cout << "Building bucket graph... " << std::flush;
        ULTRACH::BucketQuery bucketGraph(ch, FORWARD, raptor.numberOfStops());
        std::cout << "done.\n";

        std::cout << "Writing bucket graph to file... " << std::flush;
        bucketGraph.writeBinary(bucketGraphOutputFileName);
        std::cout << "done.\n";

        // Station Buckets for first taxi leg in KaRRi
        using StationBucketsEnv = karri::StationBucketsEnvironment<VehicleInputGraph, VehCHEnv>;
        StationBucketsEnv stationBucketsEnv(vehicleInputGraph, *vehChEnv);

        std::cout << "Building buckets for stations... " << std::flush;
        for (const auto& station : stations) {
            stationBucketsEnv.generateBucketEntries(station);
        }
        std::cout << "done.\n";

        std::cout << "Writing station buckets to file... " << std::flush;
        std::ofstream out(stationBucketsOutputFilename, std::ios::binary);
        stationBucketsEnv.writeBucketsTo(out);
        std::cout << "done.\n";


    } catch (std::exception &e) {
        std::cerr << "Error: " << e.what() << '\n';
        std::cerr << "Try '" << argv[0] << " -help' for more information.\n";
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}