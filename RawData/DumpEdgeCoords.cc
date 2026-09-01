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

// Small utility that maps a list of road-network edge IDs to the latitude/longitude
// coordinates of the respective edge head vertices (the point representation KaRRi/PaRRot
// uses for a location that is given as an edge). This is the conversion needed to turn
// PaRRot output/input locations (edges) into WGS84 coordinates.
//
// Input is a CSV file with one column holding integer edge IDs. Edge IDs may be given
// either as "original" IDs (the IDs found in the demand/request input files, matching the
// graph's EdgeIdAttribute) or as "sequential" IDs (0..|E|-1, the internal IDs that appear
// in PaRRot output files such as legstats.csv). Select via -id-type.
//
// Output is a CSV "edge,lat,lng" with one row per distinct input edge ID (degrees, WGS84).

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <numeric>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

#include "Common/Constants.h"
#include "KARRI/DataStructures/Geometry/LatLng.h"
#include "KARRI/DataStructures/Graph/Graph.h"
#include "KARRI/DataStructures/Graph/Attributes/LatLngAttribute.h"
#include "KARRI/DataStructures/Graph/Attributes/EdgeIdAttribute.h"
#include "KARRI/DataStructures/Graph/Attributes/EdgeTailAttribute.h"
#include "KARRI/Tools/CommandLine/CommandLineParser.h"

inline void printUsage() {
    std::cout <<
              "Usage: DumpEdgeCoords -g <file> -l <file> -o <file> [-col <name>] [-id-type <orig|seq>]\n"
              "Maps road-network edge IDs to the lat/lng of their head vertex.\n"
              "  -g <file>          road network in binary format (needs LatLng + EdgeId attributes).\n"
              "  -l <file>          input CSV file containing a column of integer edge IDs.\n"
              "  -col <name>        name of the edge-ID column in the input CSV (dflt: 'edge').\n"
              "  -id-type <type>    'orig' (IDs as in demand input files, dflt) or 'seq' (0..|E|-1,\n"
              "                     internal IDs as used in PaRRot output files like legstats.csv).\n"
              "  -o <file>          write output CSV 'edge,lat,lng' to <file>.\n"
              "  -help             display this help and exit\n";
}

int main(int argc, char *argv[]) {
    try {
        CommandLineParser clp(argc, argv);
        if (clp.isSet("help")) {
            printUsage();
            return EXIT_SUCCESS;
        }

        auto graphFileName = clp.getValue<std::string>("g");
        if (!endsWith(graphFileName, ".gr.bin"))
            graphFileName += ".gr.bin";
        auto listFileName = clp.getValue<std::string>("l");
        const auto colName = clp.getValue<std::string>("col", "edge");
        const auto idType = clp.getValue<std::string>("id-type", "orig");
        auto outputFileName = clp.getValue<std::string>("o");
        if (!endsWith(outputFileName, ".csv"))
            outputFileName += ".csv";

        if (idType != "orig" && idType != "seq")
            throw std::invalid_argument("-id-type must be 'orig' or 'seq' -- '" + idType + "'");
        const bool useSeqIds = (idType == "seq");

        // Read the road network.
        std::cout << "Reading road network from file... " << std::flush;
        using InputGraph = KaRRiStaticGraph<VertexAttrs<LatLngAttribute>, EdgeAttrs<EdgeIdAttribute, EdgeTailAttribute>>;
        std::ifstream graphFile(graphFileName, std::ios::binary);
        if (!graphFile.good())
            throw std::invalid_argument("file not found -- '" + graphFileName + "'");
        InputGraph graph(graphFile);
        graphFile.close();

        // Build a map from original edge ID to sequential edge ID (mirrors the logic used by
        // the other PaRRot RawData tools). After this loop graph.edgeId(e) == e (sequential).
        std::vector<int32_t> origIdToSeqId;
        if (graph.numEdges() > 0 && graph.edgeId(0) == INVALID_ID) {
            origIdToSeqId.assign(graph.numEdges(), INVALID_ID);
            std::iota(origIdToSeqId.begin(), origIdToSeqId.end(), 0);
            FORALL_VALID_EDGES(graph, v, e) {
                    graph.edgeId(e) = e;
                    graph.edgeTail(e) = v;
                }
        } else {
            FORALL_VALID_EDGES(graph, v, e) {
                    if (graph.edgeId(e) >= static_cast<int>(origIdToSeqId.size())) {
                        const auto numToInsert = graph.edgeId(e) + 1 - origIdToSeqId.size();
                        origIdToSeqId.insert(origIdToSeqId.end(), numToInsert, INVALID_ID);
                    }
                    origIdToSeqId[graph.edgeId(e)] = e;
                    graph.edgeId(e) = e;
                    graph.edgeTail(e) = v;
                }
        }
        std::cout << "done. (|V| = " << graph.numVertices() << ", |E| = " << graph.numEdges() << ")\n";

        // Read the list of edge IDs from the input CSV.
        std::cout << "Reading edge IDs from '" << listFileName << "' (column '" << colName << "')... " << std::flush;
        std::ifstream listFile(listFileName);
        if (!listFile.good())
            throw std::invalid_argument("file not found -- '" + listFileName + "'");

        std::string headerLine;
        if (!std::getline(listFile, headerLine))
            throw std::invalid_argument("input list file is empty -- '" + listFileName + "'");
        // Strip potential trailing carriage return.
        if (!headerLine.empty() && headerLine.back() == '\r')
            headerLine.pop_back();

        int colIdx = -1;
        {
            std::stringstream hs(headerLine);
            std::string field;
            int idx = 0;
            while (std::getline(hs, field, ',')) {
                // trim whitespace
                const auto b = field.find_first_not_of(" \t");
                const auto e = field.find_last_not_of(" \t");
                if (b != std::string::npos)
                    field = field.substr(b, e - b + 1);
                if (field == colName)
                    colIdx = idx;
                ++idx;
            }
        }
        if (colIdx < 0)
            throw std::invalid_argument("column '" + colName + "' not found in header of '" + listFileName + "'");

        std::vector<int> edgeIds;
        std::unordered_set<int> seen;
        std::string line;
        while (std::getline(listFile, line)) {
            if (line.empty())
                continue;
            if (line.back() == '\r')
                line.pop_back();
            std::stringstream ls(line);
            std::string field;
            int idx = 0;
            bool found = false;
            while (std::getline(ls, field, ',')) {
                if (idx == colIdx) {
                    found = true;
                    break;
                }
                ++idx;
            }
            if (!found || field.empty())
                continue;
            const int id = std::stoi(field);
            if (seen.insert(id).second)
                edgeIds.push_back(id);
        }
        listFile.close();
        std::cout << "done. (" << edgeIds.size() << " distinct edge IDs)\n";

        // Resolve and write output.
        std::cout << "Writing coordinates to '" << outputFileName << "'... " << std::flush;
        std::ofstream out(outputFileName);
        if (!out.good())
            throw std::invalid_argument("file cannot be opened -- '" + outputFileName + "'");
        out << "edge,lat,lng\n";
        out << std::fixed << std::setprecision(7);

        int numInvalid = 0;
        for (const int inId: edgeIds) {
            int seqId = inId;
            if (!useSeqIds) {
                if (inId < 0 || inId >= static_cast<int>(origIdToSeqId.size()) || origIdToSeqId[inId] == INVALID_ID) {
                    ++numInvalid;
                    continue;
                }
                seqId = origIdToSeqId[inId];
            } else if (inId < 0 || inId >= graph.numEdges()) {
                ++numInvalid;
                continue;
            }
            const auto ll = graph.latLng(graph.edgeHead(seqId));
            out << inId << ',' << ll.latInDeg() << ',' << ll.lngInDeg() << '\n';
        }
        out.close();
        std::cout << "done.";
        if (numInvalid > 0)
            std::cout << " (" << numInvalid << " edge IDs could not be resolved and were skipped)";
        std::cout << std::endl;

    } catch (std::exception &e) {
        std::cerr << argv[0] << ": " << e.what() << '\n';
        std::cerr << "Try '" << argv[0] << " -help' for more information.\n";
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
