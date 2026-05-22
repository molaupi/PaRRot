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
#include <fstream>
#include <iomanip>
#include <random>

#include "Common/Constants.h"
#include "KARRI/Algorithms/KaRRi/BaseObjects/Request.h"
#include "KARRI/DataStructures/Geometry/LatLng.h"
#include "KARRI/DataStructures/Containers/BitVector.h"
#include "KARRI/DataStructures/Graph/Attributes/LatLngAttribute.h"
#include "KARRI/DataStructures/Graph/Graph.h"
#include "KARRI/DataStructures/Graph/Attributes/EdgeIdAttribute.h"
#include "KARRI/DataStructures/Graph/Attributes/EdgeTailAttribute.h"
#include "KARRI/DataStructures/Graph/Attributes/OsmRoadCategoryAttribute.h"
#include "KARRI/Tools/CommandLine/CommandLineParser.h"

inline void printUsage() {
    std::cout <<
            "Usage: KarriRequestsToParrotRawFormat -g <file> -r <file> -o <file>\n"
            "Outputs all request origin and destination edges in a request file as longitude/latitude coordinates like needed for PaRRot raw format.\n"
            "  -g <file>         input road network in binary format.\n"
            "  -r <file>         input requests in CSV format.\n"
            "  -csv-in-LOUD-format    if set, assumes that input files are in the format used by LOUD.\n"
            "  -o <file>         place GeoJSON in <file>\n"
            "  -help             display this help and exit\n";
}

int main(int argc, char *argv[]) {
    try {
        CommandLineParser clp(argc, argv);
        if (clp.isSet("help")) {
            printUsage();
            return EXIT_SUCCESS;
        }

        auto inputGraphFileName = clp.getValue<std::string>("g");
        if (!endsWith(inputGraphFileName, ".gr.bin"))
            inputGraphFileName += ".gr.bin";
        auto requestFileName = clp.getValue<std::string>("r");
        if (!endsWith(requestFileName, ".csv"))
            requestFileName += ".csv";
        const bool csvFilesInLoudFormat = clp.isSet("csv-in-LOUD-format");
        auto outputFileName = clp.getValue<std::string>("o");
        if (!endsWith(outputFileName, ".csv"))
            outputFileName += ".csv";


        // Read the source network from file.
        std::cout << "Reading source network from file... " << std::flush;
        using InputGraph = KaRRiStaticGraph<VertexAttrs<LatLngAttribute>, EdgeAttrs<EdgeIdAttribute, EdgeTailAttribute,
            OsmRoadCategoryAttribute> >;
        std::ifstream inputGraphFile(inputGraphFileName, std::ios::binary);
        if (!inputGraphFile.good())
            throw std::invalid_argument("file not found -- '" + inputGraphFileName + "'");
        InputGraph inputGraph(inputGraphFile);
        inputGraphFile.close();
        std::vector<int32_t> origIdToSeqId;
        if (inputGraph.numEdges() > 0 && inputGraph.edgeId(0) == INVALID_ID) {
            origIdToSeqId.assign(inputGraph.numEdges(), INVALID_ID);
            std::iota(origIdToSeqId.begin(), origIdToSeqId.end(), 0);
            FORALL_VALID_EDGES(inputGraph, v, e) {
                assert(inputGraph.edgeId(e) == INVALID_ID);
                inputGraph.edgeId(e) = e;
                inputGraph.edgeTail(e) = v;
            }
        } else {
            FORALL_VALID_EDGES(inputGraph, v, e) {
                assert(inputGraph.edgeId(e) != INVALID_ID);
                if (inputGraph.edgeId(e) >= origIdToSeqId.size()) {
                    const auto numElementsToBeInserted = inputGraph.edgeId(e) + 1 - origIdToSeqId.size();
                    origIdToSeqId.insert(origIdToSeqId.end(), numElementsToBeInserted, INVALID_ID);
                }
                assert(origIdToSeqId[inputGraph.edgeId(e)] == INVALID_ID);
                origIdToSeqId[inputGraph.edgeId(e)] = e;
                inputGraph.edgeId(e) = e;
                inputGraph.edgeTail(e) = v;
            }
        }
        std::cout << "done.\n";

        // Read the request data from file.
        std::cout << "Reading request data from file... " << std::flush;
        std::vector<karri::Request> requests;
        int origin, destination, requestTime;
        io::CSVReader<3, io::trim_chars<' '> > reqFileReader(requestFileName);

        if (csvFilesInLoudFormat) {
            reqFileReader.read_header(io::ignore_no_column, "pickup_spot", "dropoff_spot", "min_dep_time");
        } else {
            reqFileReader.read_header(io::ignore_no_column, "origin", "destination", "req_time");
        }

        while (reqFileReader.read_row(origin, destination, requestTime)) {
            if (origin < 0 || origin >= origIdToSeqId.size() || origIdToSeqId[origin] == INVALID_ID)
                throw std::invalid_argument("invalid location -- '" + std::to_string(origin) + "'");
            if (destination < 0 || destination >= origIdToSeqId.size() ||
                origIdToSeqId[destination] == INVALID_ID)
                throw std::invalid_argument("invalid location -- '" + std::to_string(destination) + "'");
            const auto originSeqId = origIdToSeqId[origin];
            const auto destSeqId = origIdToSeqId[destination];
            const int requestId = static_cast<int>(requests.size());
            assert(inputGraph.edgeTail(originSeqId) != EdgeTailAttribute::defaultValue());
            assert(inputGraph.edgeTail(destSeqId) != EdgeTailAttribute::defaultValue());
            requests.push_back({requestId, originSeqId, destSeqId, requestTime * 10});
        }
        std::cout << "done.\n";


        std::cout << "Writing coordinates to output file... " << std::flush;
        // Open the output file and write coordinates.
        std::ofstream outputFile(outputFileName);
        if (!outputFile.good())
            throw std::invalid_argument("file cannot be opened -- '" + outputFileName + "'");

        outputFile << "lon_orig,lat_orig,lon_dest,lat_dest,req_time\n";
        for (const auto &req: requests) {
            const auto &oLatLng = inputGraph.latLng(inputGraph.edgeHead(req.origin));
            const auto &dLatLng = inputGraph.latLng(inputGraph.edgeHead(req.destination));
            outputFile << std::setprecision(8) << oLatLng.lngInDeg() << ','
                    << std::setprecision(8) << oLatLng.latInDeg() << ','
                    << std::setprecision(8) << dLatLng.lngInDeg() << ','
                    << std::setprecision(8) << dLatLng.latInDeg() << ','
                    << req.requestTime << '\n';
        }

        std::cout << " done." << std::endl;
    } catch (std::exception &e) {
        std::cerr << argv[0] << ": " << e.what() << '\n';
        std::cerr << "Try '" << argv[0] << " -help' for more information.\n";
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
