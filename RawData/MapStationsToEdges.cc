/// ******************************************************************************
/// MIT License
///
/// Copyright (c) 2026 Moritz Laupichler <moritz.laupichler@kit.edu>
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

// Utility that maps a set of stations given by latitude/longitude to the closest edges in a
// vehicle and a pedestrian road network (both in KaRRi binary format).
//
// For each input station the output contains
//   - the sequential edge ID in the vehicle network,
//   - the sequential edge ID in the pedestrian network (-1 if the station lies outside the
//     pedestrian network boundary), and
//   - the walking time (in seconds) from the vehicle edge to the actual station.
//
// If the station lies OUTSIDE the pedestrian network boundary (given as an Osmosis .poly file):
//   - The vehicle edge is the geodesically closest edge in the vehicle network (mapping done
//     as in TransformLocations for lat-lng input and edge-id output, i.e. closest vertex +
//     incident edge).
//   - The walking time is approximated from the great-circle distance between the station and
//     the head of the vehicle edge, assuming a walking speed of 5 km/h.
//   - The pedestrian edge is invalid (-1).
//
// If the station lies INSIDE the pedestrian network boundary:
//   - The pedestrian edge is the closest edge in the pedestrian network (closest vertex +
//     incident edge).
//   - The vehicle edge is found by a reverse Dijkstra search in the pedestrian network rooted
//     at the tail of the pedestrian edge (with offset = travel time of the pedestrian edge).
//     For every settled vertex v and every incident edge e (in the reverse graph) we look up
//     e' = forwardPsgGraph.toCarEdge(reversePsgGraph.edgeId(e)). The first valid e' encountered
//     becomes the vehicle edge of the station, and the distance the search assigned to v is
//     the walking time of the station.
//   - The pedestrian network is expected to be a single strongly connected component that
//     contains at least one edge with a valid mapping to the vehicle network. If no such edge
//     is found the program aborts with an error.

#include <cstdlib>
#include <cmath>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <numeric>
#include <string>
#include <vector>

#include <csv.h>

#include "Common/Constants.h"
#include "KARRI/Tools/CommandLine/CommandLineParser.h"
#include "KARRI/Tools/CommandLine/ProgressBar.h"
#include "KARRI/DataStructures/Geometry/LatLng.h"
#include "KARRI/DataStructures/Geometry/Area.h"
#include "KARRI/DataStructures/Geometry/Point.h"
#include "KARRI/DataStructures/Geometry/Rectangle.h"
#include "KARRI/DataStructures/Geometry/KDTree.h"
#include "KARRI/DataStructures/Graph/Graph.h"
#include "KARRI/DataStructures/Graph/Attributes/LatLngAttribute.h"
#include "KARRI/DataStructures/Graph/Attributes/OsmNodeIdAttribute.h"
#include "KARRI/DataStructures/Graph/Attributes/EdgeIdAttribute.h"
#include "KARRI/DataStructures/Graph/Attributes/EdgeTailAttribute.h"
#include "KARRI/DataStructures/Graph/Attributes/TravelTimeAttribute.h"
#include "KARRI/DataStructures/Graph/Attributes/PsgEdgeToCarEdgeAttribute.h"
#include "KARRI/Algorithms/Dijkstra/Dijkstra.h"
#include "KARRI/DataStructures/Graph/Attributes/PsgVertexToCarVertexAttribute.h"
#include "KARRI/DataStructures/Labels/ParentInfo.h"
#include "KARRI/DataStructures/Labels/BasicLabelSet.h"
#include "LocationMapper/LatLngToTargetVertexMapper.h"
#include "LocationMapper/Utils.h"

inline void printUsage() {
    std::cout <<
              "Usage: MapStationsToEdges -veh-g <file> -psg-g <file> -b <file> -s <file> -o <file>\n"
              "Maps stations given by latitude/longitude to the closest edges in a vehicle and a\n"
              "pedestrian road network.\n"
              "  -veh-g <file>      vehicle network in binary format\n"
              "  -psg-g <file>      pedestrian network in binary format\n"
              "  -b <file>          boundary polygon of the pedestrian network (Osmosis .poly format)\n"
              "  -s <file>          input CSV file containing one station per row\n"
              "  -s-col-name <name>      name of the lat/lng column in the input CSV (dflt: 'latlon').\n"
              "                         Format of each entry: '(<latitude>|<longitude>)'\n"
              "  -o <file>          write output CSV 'veh_edge,psg_edge,walking_time' to <file>\n"
              "  -help             display this help and exit\n";
}

using VehicleGraph = KaRRiStaticGraph<VertexAttrs<LatLngAttribute, OsmNodeIdAttribute>,
        EdgeAttrs<EdgeIdAttribute>>;
using PsgGraph = KaRRiStaticGraph<VertexAttrs<LatLngAttribute, OsmNodeIdAttribute, PsgVertexToCarVertexAttribute>,
        EdgeAttrs<EdgeIdAttribute, EdgeTailAttribute, PsgEdgeToCarEdgeAttribute, TravelTimeAttribute>>;

namespace {
    // Stopping criterion for the reverse Dijkstra search in the pedestrian network. Stops as soon as a
    // settled vertex has a valid mapping to a vertex in the vehicle network.
    struct StopAtFirstMappedCarVertex {
        const PsgGraph &revPsgGraph;   // graph the search runs on
        const PsgGraph &fwdPsgGraph;   // forward pedestrian graph (holds toCarEdge attribute)
        int *resultCarVertex;          // out: sequential vehicle vertex ID (INVALID_VERTEX if none)
        int *resultWalkingTime;        // out: walking time in tenths of seconds

        template<typename DistLabelT, typename DistLabelContT>
        bool operator()(const int v, DistLabelT &distToV, const DistLabelContT &) const {
            const int carVertex = fwdPsgGraph.toCarVertex(v);
            if (carVertex != PsgVertexToCarVertexAttribute::defaultValue()) {
                *resultCarVertex = carVertex;
                *resultWalkingTime = distToV[0];
                return true;
            }
            return false;
        }
    };
}

// Picks an edge incident to vertex v (an edge whose head is v, found via
// the reverse graph). Returns the sequential edge ID or
// INVALID_EDGE if v has no incident edge at all.
template<typename GraphT>
static int pickIncidentEdge(const GraphT &revGraph, const int v) {
    FORALL_INCIDENT_EDGES(revGraph, v, e) {
        return revGraph.edgeId(e);
    }
    return INVALID_EDGE;
}

// Reads a graph from a binary file, assigning sequential edge IDs (graph.edgeId(e) == e afterwards).
// origIdToSeqId maps the original edge IDs found in the file to the sequential IDs.
template<typename GraphT>
static GraphT readGraphWithSeqEdgeIds(const std::string &fileName, std::vector<int32_t> &origIdToSeqId) {
    std::ifstream file(fileName, std::ios::binary);
    if (!file.good())
        throw std::invalid_argument("file not found -- '" + fileName + "'");
    GraphT graph(file);
    file.close();

    origIdToSeqId.clear();
    if (graph.numEdges() > 0 && graph.edgeId(0) == INVALID_ID) {
        origIdToSeqId.assign(graph.numEdges(), INVALID_ID);
        std::iota(origIdToSeqId.begin(), origIdToSeqId.end(), 0);
        FORALL_VALID_EDGES(graph, v, e) {
                graph.edgeId(e) = e;
            }
    } else {
        FORALL_VALID_EDGES(graph, v, e) {
                if (graph.edgeId(e) >= static_cast<int>(origIdToSeqId.size())) {
                    const auto numToInsert = graph.edgeId(e) + 1 - origIdToSeqId.size();
                    origIdToSeqId.insert(origIdToSeqId.end(), numToInsert, INVALID_ID);
                }
                origIdToSeqId[graph.edgeId(e)] = e;
                graph.edgeId(e) = e;
            }
    }
    return graph;
}

int main(int argc, char *argv[]) {
    try {
        CommandLineParser clp(argc, argv);
        if (clp.isSet("help")) {
            printUsage();
            return EXIT_SUCCESS;
        }

        auto vehicleGraphFileName = clp.getValue<std::string>("veh-g");
        if (!endsWith(vehicleGraphFileName, ".gr.bin"))
            vehicleGraphFileName += ".gr.bin";
        auto psgGraphFileName = clp.getValue<std::string>("psg-g");
        if (!endsWith(psgGraphFileName, ".gr.bin"))
            psgGraphFileName += ".gr.bin";
        const auto boundaryFileName = clp.getValue<std::string>("b");
        const auto stationsFileName = clp.getValue<std::string>("s");
        const auto stationsColName = clp.getValue<std::string>("s-col-name", "latlon");
        auto outputFileName = clp.getValue<std::string>("o");
        if (!endsWith(outputFileName, ".csv"))
            outputFileName += ".csv";

        if (vehicleGraphFileName.empty() || psgGraphFileName.empty() || boundaryFileName.empty() ||
            stationsFileName.empty() || outputFileName.empty())
            throw std::invalid_argument("-veh-g, -psg-g, -b, -s and -o must all be set.");

        // Read the vehicle network.
        std::cout << "Reading vehicle network from file... " << std::flush;
        std::vector<int32_t> vehOrigIdToSeqId;
        auto vehicleGraph = readGraphWithSeqEdgeIds<VehicleGraph>(vehicleGraphFileName, vehOrigIdToSeqId);
        const VehicleGraph revVehicleGraph = vehicleGraph.getReverseGraph();
        std::cout << "done. (|V| = " << vehicleGraph.numVertices() << ", |E| = " << vehicleGraph.numEdges() << ")\n";

        // Read the pedestrian network.
        std::cout << "Reading pedestrian network from file... " << std::flush;
        std::vector<int32_t> psgOrigIdToSeqId;
        auto psgGraph = readGraphWithSeqEdgeIds<PsgGraph>(psgGraphFileName, psgOrigIdToSeqId);
        // The pedestrian graph stores toCarEdge as original vehicle edge IDs. Remap them to the
        // sequential IDs of the vehicle graph (mirrors what RunPTaxi does when loading the graphs).
        FORALL_VALID_EDGES(psgGraph, v, e) {
                psgGraph.edgeTail(e) = v;
                const int carEdgeOrigId = psgGraph.toCarEdge(e);
                if (carEdgeOrigId != PsgEdgeToCarEdgeAttribute::defaultValue()) {
                    if (carEdgeOrigId < 0 || carEdgeOrigId >= static_cast<int>(vehOrigIdToSeqId.size()) ||
                        vehOrigIdToSeqId[carEdgeOrigId] == INVALID_ID)
                        throw std::invalid_argument(
                                "Pedestrian edge maps to vehicle edge ID " + std::to_string(carEdgeOrigId) +
                                " which does not exist in the vehicle network.");
                    psgGraph.toCarEdge(e) = vehOrigIdToSeqId[carEdgeOrigId];
                }
            }
        const PsgGraph revPsgGraph = psgGraph.getReverseGraph();
        std::cout << "done. (|V| = " << psgGraph.numVertices() << ", |E| = " << psgGraph.numEdges() << ")\n";

        // Read the pedestrian network boundary.
        std::cout << "Reading pedestrian network boundary from file... " << std::flush;
        Area boundary;
        boundary.importFromOsmPolyFile(boundaryFileName);
        const auto boundaryBox = boundary.boundingBox();
        std::cout << "done.\n";

        // Read the input stations.
        std::cout << "Reading stations from '" << stationsFileName << "' (column '" << stationsColName << "')... "
                  << std::flush;
        std::vector<LatLng> stations;
        {
            io::CSVReader<1, io::trim_chars<' '>, io::no_quote_escape<','>> stationsReader(stationsFileName);
            stationsReader.read_header(io::ignore_extra_column, stationsColName);
            std::string latLngStr;
            while (stationsReader.read_row(latLngStr)) {
                stations.push_back(transform_locations_input::parseLatLngString(latLngStr));
            }
        }
        std::cout << "done. (" << stations.size() << " stations)\n";

        // Build the coordinate-to-vertex mappers.
        LatLngToTargetVertexMapper<VehicleGraph, AlwaysEligible, NullLogger>
                vehVertexMapper(vehicleGraph, std::numeric_limits<double>::max(), {}, nullptr);
        LatLngToTargetVertexMapper<PsgGraph, AlwaysEligible, NullLogger>
                psgVertexMapper(psgGraph, std::numeric_limits<double>::max(), {}, nullptr);

        // Reverse Dijkstra search in the pedestrian network for the inside-boundary case.
        int searchResultCarVertex = INVALID_VERTEX;
        int searchResultWalkingTime = INFTY;
        using StationSearch = KaRRiDijkstra<PsgGraph, TravelTimeAttribute,
                BasicLabelSet<0, ParentInfo::NO_PARENT_INFO>, StopAtFirstMappedCarVertex>;
        StationSearch search(revPsgGraph,
                             StopAtFirstMappedCarVertex{revPsgGraph, psgGraph, &searchResultCarVertex,
                                                      &searchResultWalkingTime});

        // Walking speed of 5 km/h in m/s.
        constexpr double WALKING_SPEED_MPS = 5000.0 / 3600.0;

        std::cout << "Mapping stations... " << std::endl;
        std::ofstream out(outputFileName);
        if (!out.good())
            throw std::invalid_argument("file cannot be opened -- '" + outputFileName + "'");
        out << "veh_edge,psg_edge,walking_time\n";

        int numInside = 0;
        int numOutside = 0;
        KaRRiProgressBar progressBar(stations.size());
        for (int i = 0; i < stations.size(); ++i) {
            unused(i);
            const auto &station = stations[i];

            const Point stationPoint(station.longitude(), station.latitude());
            const bool inside = boundaryBox.contains(stationPoint) && boundary.contains(stationPoint);

            int vehEdge = INVALID_EDGE;
            int psgEdge = INVALID_EDGE;
            int walkingTimeSeconds = 0;

            if (!inside) {
                ++numOutside;
                const int vehVertex = vehVertexMapper.mapToTargetVertex(station);
                if (vehVertex == INVALID_VERTEX)
                    throw std::invalid_argument("Could not map station to a vertex in the vehicle network.");
                vehEdge = pickIncidentEdge(revVehicleGraph, vehVertex);
                if (vehEdge == INVALID_EDGE)
                    throw std::invalid_argument("Vehicle network vertex has no incident edge.");

                const auto headLatLng = vehicleGraph.latLng(vehicleGraph.edgeHead(vehEdge));
                const double distInMeters = station.getGreatCircleDistanceTo(headLatLng);
                walkingTimeSeconds = static_cast<int>(std::lround(distInMeters / WALKING_SPEED_MPS));
            } else {
                ++numInside;
                const int psgVertex = psgVertexMapper.mapToTargetVertex(station);
                if (psgVertex == INVALID_VERTEX)
                    throw std::invalid_argument("Could not map station to a vertex in the pedestrian network.");
                psgEdge = pickIncidentEdge(revPsgGraph, psgVertex);
                if (psgEdge == INVALID_EDGE)
                    throw std::invalid_argument("Pedestrian network vertex has no incident edge.");

                if (psgGraph.toCarEdge(psgEdge) != PsgEdgeToCarEdgeAttribute::defaultValue()) {
                    // If the chosen psg edge is also vehicle accessible, choose it as the vehicle edge, too (with a walking time of 0).
                    vehEdge = psgGraph.toCarEdge(psgEdge);
                    walkingTimeSeconds = 0;
                } else {
                    // Otherwise, search for the closest vehicle-accessible edge from which a traveler can walk to the
                    // station (which is assumed to be at psgVertex).
                    searchResultCarVertex = INVALID_VERTEX;
                    searchResultWalkingTime = INFTY;
                    search.run(psgVertex);

                    if (searchResultCarVertex == INVALID_VERTEX) {
                        const auto edgeLatLng = psgGraph.latLng(psgGraph.edgeHead(psgEdge));
                        std::cerr << argv[0] << ": no vehicle-accessible vertex reachable by walking from pedestrian edge "
                                  << psgEdge << " (head at "
                                  << std::fixed << std::setprecision(7)
                                  << edgeLatLng.latInDeg() << "|" << edgeLatLng.lngInDeg() << ").\n";
                        return EXIT_FAILURE;
                    }

                    vehEdge = pickIncidentEdge(revVehicleGraph, searchResultCarVertex);
                    // Walking time is stored internally in tenths of seconds.
                    walkingTimeSeconds = static_cast<int>(std::lround(searchResultWalkingTime / 10.0));
                }
            }

            out << vehEdge << ',' << psgEdge << ',' << walkingTimeSeconds << '\n';
            ++progressBar;
        }
        out.close();
        std::cout << "\ndone. (" << numInside << " inside boundary, " << numOutside << " outside boundary)\n";
        std::cout << "Output written to '" << outputFileName << "'.\n";

    } catch (std::exception &e) {
        std::cerr << argv[0] << ": " << e.what() << '\n';
        std::cerr << "Try '" << argv[0] << " -help' for more information.\n";
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
