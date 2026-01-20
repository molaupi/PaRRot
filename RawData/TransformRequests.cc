/// ******************************************************************************
/// MIT License
///
/// Copyright (c) 2025 Moritz Laupichler <moritz.laupichler@kit.edu>
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

#include <Common/Constants.h>
#include <KARRI/Tools/CommandLine/CommandLineParser.h>
#include <KARRI/DataStructures/Graph/Graph.h>
#include <KARRI/DataStructures/Graph/Attributes/CarEdgeToPsgEdgeAttribute.h>
#include <KARRI/DataStructures/Utilities/OriginDestination.h>
#include <KARRI/DataStructures/Graph/Attributes/EdgeIdAttribute.h>
#include <KARRI/DataStructures/Graph/Attributes/EdgeTailAttribute.h>
#include <KARRI/DataStructures/Graph/Attributes/PsgEdgeToCarEdgeAttribute.h>
#include <KARRI/DataStructures/Geometry/LatLng.h>
#include <ULTRA/DataStructures/Graph/Graph.h>
#include <ULTRA/DataStructures/RAPTOR/Data.h>
#include <ULTRA/DataStructures/Queries/Queries.h>

inline void printUsage() {
    std::cout <<
              "Usage: TransformRequestsToLatLng -veh-g <file> -psg-g <file> -r <file> -o <file>\n"
              "Given a request file, the vehicle graph, and the passenger graph, transforms the origin and destination "
              "locations (given as edge IDs in the vehicle graph) to corresponding edges in the passenger graph, "
              "and then to latitude/longitude of edge heads and writes these coordinates to file.\n"
              "pairs of edges to the output file.\n\n"
              "  -veh-g <file>     vehicle graph in binary format\n"
              "  -psg-g <file>     passenger graph in binary format\n"
              "  -r <file>         requests file in CSV format\n"
              "  -add-time-offset <int>  time offset to add to all request times (in seconds).\n"
              "  -sub-time-offset <int>  time offset to subtract from all request times (in seconds).\n"
              "  -transfer-graph <file>  transfer graph in ULTRA format to match the requests.\n"
              "  -station-mapping <file>  file which maps the station to edge-ids in the given passenger road graph.\n"
              "  -o <file>         place output in <file>\n"
              "  -help             display this help and exit\n";
}

int main(int argc, char *argv[]) {
    try {
        CommandLineParser clp(argc, argv);
        if (clp.isSet("help")) {
            printUsage();
            return EXIT_SUCCESS;
        }

        // Parse the command-line options.
        const auto vehicleGraphFileName = clp.getValue<std::string>("veh-g");
        const auto passengerGraphFileName = clp.getValue<std::string>("psg-g");
        const auto requestsFileName = clp.getValue<std::string>("r");
        const auto addTimeOffset = clp.getValue<int>("add-time-offset", 0);
        const auto subTimeOffset = clp.getValue<int>("sub-time-offset", 0);
        const int timeOffset = addTimeOffset - subTimeOffset;
        std::cout << "Using total time offset of " << timeOffset << " seconds.\n";
        const auto transferGraphFileName = clp.getValue<std::string>("transfer-graph");
        const auto stationMappingFileName = clp.getValue<std::string>("station-mapping");
        auto outputFileName = clp.getValue<std::string>("o");
        if (!endsWith(outputFileName, ".csv"))
            outputFileName += ".csv";


        // Read the source network from file.
        std::cout << "Reading vehicle network from file... " << std::flush;
        using VehicleGraph = KaRRiStaticGraph<VertexAttrs<>, EdgeAttrs<CarEdgeToPsgEdgeAttribute>>;
        std::ifstream vehicleGraphFile(vehicleGraphFileName, std::ios::binary);
        if (!vehicleGraphFile.good())
            throw std::invalid_argument("file not found -- '" + vehicleGraphFileName + "'");
        VehicleGraph vehicleInputGraph(vehicleGraphFile);
        vehicleGraphFile.close();
        std::cout << "done.\n";

        std::cout << "Reading passenger network from file... " << std::flush;
        using PassengerGraph = KaRRiStaticGraph<VertexAttrs<LatLngAttribute>, EdgeAttrs<>>;
        std::ifstream passengerGraphFile(passengerGraphFileName, std::ios::binary);
        if (!passengerGraphFile.good())
            throw std::invalid_argument("file not found -- '" + passengerGraphFileName + "'");
        PassengerGraph psgInputGraph(passengerGraphFile);
        passengerGraphFile.close();
        std::cout << "done.\n";

        // Read the station mapping file
        std::cout << "Reading station mapping from file... " << std::flush;
        std::unordered_map<int, int> stations;
        int edgeId;
        int stationId = 0;
        io::CSVReader<1> stationMappingFileReader(stationMappingFileName);

        stationMappingFileReader.read_header(io::ignore_no_column, "initial_location");

        while (stationMappingFileReader.read_row(edgeId)) {
            if (edgeId < 0) {
                throw std::invalid_argument("invalid edge id for a station-- '" + std::to_string(edgeId) + "'");
            }
            stations.insert({edgeId, stationId});
            stationId++;
        }
        std::cout << "done.\n";

        // Read the request data from file.
        std::cout << "Reading request data from file... " << std::flush;
        std::vector<std::tuple<int, int, int, LatLng, LatLng, int>> odPairs;
        int origin, destination, reqTime;
        io::CSVReader<3, io::trim_chars<' '>> reqFileReader(requestsFileName);
        reqFileReader.read_header(io::ignore_extra_column, "origin", "destination", "req_time");

        while (reqFileReader.read_row(origin, destination, reqTime)) {
            int stationId = INVALID_ID;
            // checks if destination is a station
            if (stations.contains(destination)) {
                stationId = stations[destination];
            }

            const int oPsgEdge = vehicleInputGraph.toPsgEdge(origin);
            const int dPsgEdge = vehicleInputGraph.toPsgEdge(destination);
            const auto oLatLng = psgInputGraph.latLng(psgInputGraph.edgeHead(oPsgEdge));
            const auto dLatLng = psgInputGraph.latLng(psgInputGraph.edgeHead(dPsgEdge));
            odPairs.push_back({origin, destination, reqTime + timeOffset, oLatLng, dLatLng, stationId});
        }
        std::cout << "done.\n";

        const TransferGraph graph(transferGraphFileName);

        ULTRAGraph::printInfo(graph);
        graph.printAnalysis();

        Geometry::Rectangle boundingBox = Geometry::Rectangle::BoundingBox(graph[Coordinates]);
        Geometry::GeoMetricAproximation metric = Geometry::GeoMetricAproximation::ComputeCorrection(boundingBox.center());
        CoordinateTree<Geometry::GeoMetricAproximation> ct(metric, graph[Coordinates]);
        std::vector<VertexQuery> queries;
        std::vector<int> distances;

        std::ofstream out(outputFileName);
        out << "origin,destination,req_time,source,target\n";

        for (const auto& od : odPairs) {
            const auto pointOrigin = Geometry::Point(Construct::LatLong, get<3>(od).latInDeg(), get<3>(od).lngInDeg());
            const auto pointDestination = Geometry::Point(Construct::LatLong, get<4>(od).latInDeg(), get<4>(od).lngInDeg());
            const Vertex originVertex = ct.getNearestNeighbor(pointOrigin);
            const double originDistance = Geometry::geoDistanceInCM(pointOrigin, graph.get(Coordinates, originVertex));
            const Vertex destinationVertex = ct.getNearestNeighbor(pointDestination);
            const double destinationDistance = Geometry::geoDistanceInCM(pointDestination, graph.get(Coordinates, destinationVertex));
            distances.push_back(originDistance);
            distances.push_back(destinationDistance);
            const auto originEdgeId = get<0>(od);
            const auto destinationEdgeId = get<1>(od);
            const int requestTime = get<2>(od);
            const int stationId = get<5>(od);
            const auto originVertexId = originVertex.value();
            const auto destinationVertexId = stationId == INVALID_ID ? destinationVertex.value() : stationId;
            out << originEdgeId << "," << destinationEdgeId << "," << requestTime << "," << originVertexId << "," << destinationVertexId << "\n";
        }

        out.close();

        // Print statistics on distances
        std::sort(distances.begin(), distances.end());
        const size_t numDistances = distances.size();
        std::cout << "Statistics on distances from request points to nearest graph vertices (in meters):" << std::endl;
        std::cout << "  Min: " << distances.front() / 100.0 << std::endl;
        std::cout << "  10th percentile: " << distances[numDistances / 10] / 100.0 << std::endl;
        std::cout << "  Median: " << distances[numDistances / 2] / 100.0 << std::endl;
        std::cout << "  90th percentile: " << distances[(numDistances * 9) / 10] / 100.0 << std::endl;
        std::cout << "  Max: " << distances.back() / 100.0 << std::endl;
        

    } catch (std::exception &e) {
        std::cerr << argv[0] << ": " << e.what() << '\n';
        std::cerr << "Try '" << argv[0] << " -help' for more information.\n";
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}