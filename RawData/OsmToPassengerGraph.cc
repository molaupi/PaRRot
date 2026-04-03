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

#include <iostream>

#include "Common/Constants.h"
#include "KARRI/Algorithms/GraphTraversal/StronglyConnectedComponents.h"
#include "KARRI/DataStructures/Graph/Graph.h"
#include "KARRI/DataStructures/Graph/Import/PedestrianOsmImporter.h"
#include "KARRI/DataStructures/Graph/Import/CyclistOsmImporter.h"
#include "KARRI/DataStructures/Graph/Attributes/CoordinateAttribute.h"
#include "KARRI/DataStructures/Graph/Attributes/LatLngAttribute.h"
#include "KARRI/DataStructures/Graph/Attributes/SequentialVertexIdAttribute.h"
#include "KARRI/DataStructures/Graph/Attributes/VertexIdAttribute.h"
#include "KARRI/DataStructures/Graph/Attributes/OsmRoadCategoryAttribute.h"
#include "KARRI/DataStructures/Graph/Attributes/TravelTimeAttribute.h"
#include "KARRI/DataStructures/Graph/Attributes/PsgEdgeToCarEdgeAttribute.h"
#include "KARRI/DataStructures/Graph/Attributes/OsmNodeIdAttribute.h"
#include "KARRI/Tools/CommandLine/CommandLineParser.h"


inline void printUsage() {
    std::cout <<
            "Usage: OsmToPassengerGraph -i <file> -o <file>\n"
            "This program reads a graph from an OSM PBF source file and converts it into a binary graph representing \n"
            "a network for passengers moving on their own (walking or cycling).\n"
            "  -psg-mode <mode>       mode of transportation of the passenger\n"
            "                             possible values: pedestrian (default), cyclist\n"
            "  -no-scc                Do not extract largest strongly connected component from graph.\n"
            "  -i <file>              input graph in OSM PBF format without file extension\n"
            "  -o <file>              passenger graph output file without file extension\n"
            "  -help                  display this help and exit\n";
}

using PsgVertexAttributes = VertexAttrs<
    CoordinateAttribute,
    LatLngAttribute,
    SequentialVertexIdAttribute,
    VertexIdAttribute,
    OsmNodeIdAttribute
>;
using PsgEdgeAttributes = EdgeAttrs<
    PsgEdgeToCarEdgeAttribute,
    TravelTimeAttribute,
    OsmRoadCategoryAttribute
>;
using PsgGraphT = KaRRiStaticGraph<PsgVertexAttributes, PsgEdgeAttributes>;


template<typename PsgOsmImporterT>
void generateGraph(const CommandLineParser &clp, const IsRoadAccessibleByCategory &isPsgAccessible) {
    unused(isPsgAccessible);

    const auto infile = clp.getValue<std::string>("i");

    std::cout << "Constructing passenger graph...\n" << std::flush;

    std::cout << "\tReading the input file..." << std::flush;
    auto psgImporter = std::make_unique<PsgOsmImporterT>(isPsgAccessible);
    auto psgGraph = PsgGraphT::makeGraphUsingImporterRef(infile, *psgImporter);
    std::cout << "\t done." << std::endl;

    if (!clp.isSet("no-scc")) {
        std::cout << "\tComputing strongly connected components..." << std::flush;
        StronglyConnectedComponents psgScc;
        psgScc.run(psgGraph);
        std::cout << "\t done." << std::endl;

        std::cout << "\tExtracting the largest SCC..." << std::flush;
        auto psgGraphToSccEdgeMap = std::make_unique<std::vector<int> >(psgGraph.numEdges(), INVALID_ID);
        psgGraph.extractVertexInducedSubgraph(psgScc.getLargestSccAsBitmask(), *psgGraphToSccEdgeMap);
        std::cout << "\t done." << std::endl;
    }

    std::cout << "Write the passenger graph output file..." << std::flush;
    auto outfile = clp.getValue<std::string>("o");
    if (!endsWith(outfile, ".gr.bin")) outfile += ".gr.bin";
    std::ofstream out(outfile, std::ios::binary);
    if (!out.good())
        throw std::invalid_argument("file cannot be opened -- '" + outfile + ".gr.bin'");
    psgGraph.writeTo(out);
    std::cout << " done." << std::endl;
}


int main(int argc, char *argv[]) {
    try {
        CommandLineParser clp(argc, argv);
        if (clp.isSet("help")) {
            printUsage();
            return EXIT_SUCCESS;
        }

        if (!clp.isSet("i"))
            throw std::invalid_argument(
                "input file not given -- use '-i <file>' to specify the input file in OSM PBF format without file extension");
        if (!clp.isSet("o"))
            throw std::invalid_argument(
                "output file not given -- use '-o <file>' to specify the output file for the passenger graph without file extension");

        const auto psgModeStr = clp.getValue<std::string>("psg-mode");
        if (psgModeStr == "pedestrian") {
            generateGraph<PedestrianOsmImporter>(clp, defaultIsPedestrianAccessible);
        } else if (psgModeStr == "cyclist") {
            generateGraph<CyclistOsmImporter>(clp, defaultIsCyclistAccessible);
        } else {
            throw std::invalid_argument("passenger mode of transport not known -- " + psgModeStr);
        }
    } catch (std::invalid_argument &e) {
        std::cerr << argv[0] << ": " << e.what() << std::endl;
        std::cerr << "Try '" << argv[0] << " -help' for more information." << std::endl;
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
