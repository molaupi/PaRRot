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


#include <KARRI/Algorithms/KaRRi/CHEnvironment.h>
#include <KARRI/Algorithms/CH/CH.h>

#include <ULTRA/Algorithms/CH/Query/BucketQuery.h>
#include <ULTRA/DataStructures/RAPTOR/Data.h>

#include "../PTaxi/StationBucketsEnvironment.h"


inline void printUsage() {
    std::cout <<
              "Usage: \n"
              "     TransformLocations -src-g <file> -tar-g <file> -p <file> -o <file>\n"
              "     TransformLocations -src-g <file> -tar-g <file> -v <file> -o <file>\n"
              "     TransformLocations -tar-g <file> -p <file> -in-repr <edge|vertex> -out-repr <vertex|edge> -o <file>\n"
              "     TransformLocations -tar-g <file> -v <file> -in-repr <edge|vertex> -out-repr <vertex|edge> -o <file>\n"
              "Takes a set of origin-destination pairs or initial vehicle locations and transforms the locations to the "
              "geographically closest locations in a target network. The target network should geographically encompass "
              "all locations.\n"
              "Input locations may be specified as vertices or edges in a source network or coordinates in a GCS (see -in-repr).\n"
              "Output locations may be vertices or edges in the target network (see -out-repr).\n"
              "Vertices can be transformed to incident edges in the same network and vice versa by specifying identical source and\n"
              "target networks (mapping done with vertex/edge IDs).\n"
              "  -tar-g <file>          target graph file in binary format\n"
              "  -d <dist>              maximum geographical distance (in m) for transformation from one vertex to another.\n"
              "                              Set to 0 to allow unlimited radius (dflt).\n"
              "  -psg                   if set, restrict eligible locations in target graph to those accessible by vehicles and passengers.\n"
              "  -p <file>              path to CSV file containing input OD-pairs\n"
              "  -o-col-name <name>     name of origin column in OD-pairs file (dflt: 'origin')\n"
              "  -d-col-name <name>     name of destination column in OD-pairs file (dflt: 'destination')\n"
              "  -v <file>              path to CSV file containing input vehicles\n"
              "  -l-col-name <name>     name of initial location column in vehicles file (dflt: 'initial_location')\n"
              "  -in-repr <repr>        Representation of locations in input. Possible values:\n"
              "                             vertex-id   (ID of vertex in source graph; requires -src-g; default)\n"
              "                             edge-id     (ID of edge in source graph; requires -src-g)\n"
              "                             lat-lng     (Latitude and longitude in format '(lat|lng)')\n"
              "                             epsg-31467  (Easting (=X) and northing (=Y) in EPSG 31467 in format '(X|Y)')\n"
              "  -src-g <file>          source graph file in binary format (required for certain values of -in-repr)\n"
              "  -out-repr <repr>       Representation of locations in output. Possible values:\n"
              "                             vertex-id (dflt)\n"
              "                             edge-id\n"
              "  -a <file>              optional .poly file describing area encompassing all OD-pairs/all vehicle locations.\n"
              "  -o <file>              place transformed OD-pairs in <file>\n"
              "  -help                  display this help and exit\n";
}

int main(int argc, char *argv[]) {
    
    try {
        CommandLineParser clp(argc, argv);
        if (clp.isSet("help")) {
            printUsage();
            return EXIT_SUCCESS;
        }


        // Parse the command-line options.
        const auto passengerNetworkFileName = clp.getValue<std::string>("psg-g");
        const auto psgHierarchyFileName = clp.getValue<std::string>("psg-h");
        const auto raptorFileName = clp.getValue<std::string>("raptor-data");
        const auto stationMappingFileName = clp.getValue<std::string>("station-mapping");
        const auto bucketGraphOutputFileName = clp.getValue<std::string>("o-bucket-graph");
        auto stationBucketsOutputFilename = clp.getValue<std::string>("o-station-buckets");
        const auto stationBucketsPositionsFileName = stationBucketsOutputFilename + ".positions.bucket.bin";
        const auto stationBucketsEntriesFileName = stationBucketsOutputFilename + ".entries.bucket.bin";

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
        std::cout << "done.\n";

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
        std::vector<int> vertexIdOfStation;
        int edgeId;
        io::CSVReader<1> stationMappingFileReader(stationMappingFileName);

        stationMappingFileReader.read_header(io::ignore_no_column, "initial_location");

        while (stationMappingFileReader.read_row(edgeId)) {
            if (edgeId < 0) {
                throw std::invalid_argument("invalid edge id for a station-- '" + std::to_string(edgeId) + "'");
            }

            // edge id in the station mapping file is the edge id in the passenger graph
            int vertexId = psgInputGraph.edgeHead(edgeId);
            vertexIdOfStation.push_back(vertexId);
        }
        std::cout << "done.\n";


        std::cout << "Convert the karri::CH to ULTRA::CH... " << std::flush;
        std::cout << "[Passenger CH]... " << std::flush;
        // convert from KARRI:CH to ULTRA::CH
        auto& karriPsgCH = psgChEnv->getCH();
        auto& karriPsgUpCHGraph = karriPsgCH.upwardGraph();
        auto& karriPsgDownCHGraph = karriPsgCH.downwardGraph();

        assert(karriPsgUpCHGraph.numVertices() == karriPsgDownCHGraph.numVertices());

        CHConstructionGraph upPsgCHGraph;
        CHConstructionGraph downPsgCHGraph;

        upPsgCHGraph.addVertices(karriPsgUpCHGraph.numVertices());
        upPsgCHGraph.reserve(karriPsgUpCHGraph.numVertices(), karriPsgUpCHGraph.numEdges());

        FORALL_VALID_EDGES(karriPsgUpCHGraph, v, e)
        {
            auto edgeHandle = upPsgCHGraph.addEdge(Vertex(v), Vertex(karriPsgUpCHGraph.edgeHead(e)));
            edgeHandle.set(Weight, karriPsgUpCHGraph.traversalCost(e));
            // since UnpackingInfoAttribute is defined a little weird (via the two edges directly, rather than the ViaVertex itself), we need to get the Vertex
            auto& unpackingInfoOfEdge = karriPsgUpCHGraph.unpackingInfo(e);
            if (unpackingInfoOfEdge.second == INVALID_EDGE) {
                // this case, the edge was an input edge => set invalid vertex as viavertex
                edgeHandle.set(ViaVertex, noVertex);
            } else {
                // otherwise take the FromVertex / edgeTail from the first edge
                assert(unpackingInfoOfEdge.first < karriPsgDownCHGraph.numEdges());
                edgeHandle.set(ViaVertex, Vertex(karriPsgDownCHGraph.edgeHead(unpackingInfoOfEdge.first)));
            }
        }

        downPsgCHGraph.addVertices(karriPsgDownCHGraph.numVertices());
        downPsgCHGraph.reserve(karriPsgDownCHGraph.numVertices(), karriPsgDownCHGraph.numEdges());

        FORALL_VALID_EDGES(karriPsgDownCHGraph, v, e)
        {
            auto edgeHandle = downPsgCHGraph.addEdge(Vertex(v), Vertex(karriPsgDownCHGraph.edgeHead(e)));
            edgeHandle.set(Weight, karriPsgDownCHGraph.traversalCost(e));
            // TODO
            edgeHandle.set(ViaVertex, noVertex);
            /* // since UnpackingInfoAttribute is defined a little weird (via the two edges directly, rather than the ViaVertex itself), we need to get the Vertex */
            /* auto& unpackingInfoOfEdge = karriPsgDownCHGraph.unpackingInfo(e); */
            /* if (unpackingInfoOfEdge.second == INVALID_EDGE) { */
            /*     // this case, the edge was an input edge => set invalid vertex as viavertex */
            /*     edgeHandle.set(ViaVertex, noVertex); */
            /* } else { */
            /*     // otherwise take the FromVertex / edgeTail from the first edge */
            /*     assert(unpackingInfoOfEdge.first < karriPsgUpCHGraph.numEdges()); */
            /*     edgeHandle.set(ViaVertex, Vertex(karriPsgUpCHGraph.edgeHead(unpackingInfoOfEdge.first))); */
            /* } */
        }

        ULTRACH::CH psgCh(std::move(upPsgCHGraph), std::move(downPsgCHGraph));
        
        // Reorder the vertices in the passenger CH
        Order order(Construct::Id, psgCh.numVertices());
        std::vector<bool> swapped(psgCh.numVertices(), false);
        for (const StopId stop : raptor.stops()) {
            const auto newStopId = vertexIdOfStation[stop];
            assert(newStopId < order.size());

            if (swapped[stop] || swapped[newStopId]) {
                continue;
            }
            order[stop] = newStopId;
            order[newStopId] = stop;
            swapped[stop] = true;
            swapped[newStopId] = true;
        }
        psgCh.applyVertexOrder(order);

        std::cout << "## Psg FORWARD ##" << std::endl;
        psgCh.getGraph(FORWARD).printAnalysis();
        std::cout << "## Psg BACKWARD ##" << std::endl;
        psgCh.getGraph(BACKWARD).printAnalysis();

        std::cout << "done.\n";
        
        std::cout << "Building bucket graph... " << std::flush;
        ULTRACH::BucketQuery bucketGraph(psgCh, FORWARD, raptor.numberOfStops());
        std::cout << "done.\n";

        std::cout << "Writing bucket graph to file... " << std::flush;
        bucketGraph.writeBinary(bucketGraphOutputFileName);
        std::cout << "done.\n";

        // Station Buckets for first taxi leg in KaRRi
        using StationBucketsEnv = karri::StationBucketsEnvironment<PsgInputGraph, PsgCHEnv>;
        StationBucketsEnv stationBucketsEnv(psgInputGraph, *psgChEnv);

        std::cout << "Building buckets for stations... " << std::flush;
        for (auto& vertexId : vertexIdOfStation) {
            stationBucketsEnv.generateBucketEntries(vertexId);
        }
        std::cout << "done.\n";

        std::cout << "Writing station buckets to file... " << std::flush;
        std::ofstream outPositions(stationBucketsPositionsFileName, std::ios::binary);
        std::ofstream outEntries(stationBucketsEntriesFileName, std::ios::binary);
        stationBucketsEnv.writeTo(outPositions, outEntries);
        std::cout << "done.\n";
        
        std::cout << "Read station buckets from file... " << std::flush;
        std::ifstream inPositions(stationBucketsPositionsFileName, std::ios::binary);
        std::ifstream inEntries(stationBucketsEntriesFileName, std::ios::binary);
        StationBucketsEnv readStationBucketsEnv(psgInputGraph, *psgChEnv);
        readStationBucketsEnv.readFrom(inPositions, inEntries);

        std::cout << "done.\n";


    } catch (std::exception &e) {
        std::cerr << "Error: " << e.what() << '\n';
        std::cerr << "Try '" << argv[0] << " -help' for more information.\n";
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}