#include <csv.h>

#include <cassert>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <random>

#include <ULTRA/DataStructures/Queries/Queries.h>
#include <ULTRA/DataStructures/RAPTOR/Data.h>
#include <ULTRA/Algorithms/RAPTOR/ULTRARAPTOR.h>
#include <ULTRA/Algorithms/CH/CH.h>
#include <KARRI/Tools/CommandLine/CommandLineParser.h>


inline void printULTRAUsage()
{
    std::cout << 
    "Usage: RunULTRARAPTOR [options]\n"
    "Runs ULTRARAPTOR queries.\n"
    "  -raptor <file>         RAPTOR input file with transit data\n"
    "  -ch <file>             CH data for transferring\n"
    "  -source <vertex>       Source vertex ID\n"
    "  -target <vertex>       Target vertex ID\n"
    "  -departure <time>      Departure time in seconds\n"
    "  -help                  Show usage help text.\n";
}

int runULTRARAPTOR(int argc, char* argv[])
{
    using namespace karri;
    try {
        CommandLineParser clp(argc, argv);
        if (clp.isSet("help")) {
            printULTRAUsage();
            return EXIT_SUCCESS;
        }

        // Parse ULTRARAPTOR specific arguments
        const auto raptorDataFile = clp.getValue<std::string>("raptor");
        const auto chDataFile = clp.getValue<std::string>("ch");
        const int sourceVertex = clp.getValue<int>("source");
        const int targetVertex = clp.getValue<int>("target");
        const int departureTime = clp.getValue<int>("departure");

        // Load the RAPTOR data
        std::cout << "Loading RAPTOR data..." << std::endl;
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(raptorDataFile);
        raptorData.useImplicitDepartureBufferTimes();
        raptorData.printInfo();
        
        // Load the CH data
        std::cout << "Loading CH data..." << std::endl;
        CH::CH ch(chDataFile);
        
        // Create ULTRARAPTOR algorithm
        RAPTOR::ULTRARAPTOR<RAPTOR::AggregateProfiler, false> algorithm(raptorData, ch);
        
        // Run a single query
        std::cout << "Running query from vertex " << sourceVertex << " to vertex " << targetVertex 
                  << " at departure time " << departureTime << std::endl;
        
        Vertex source(sourceVertex);
        Vertex target(targetVertex);
        
        algorithm.run(source, departureTime, target);
        
        // Print results
        const auto& journeys = algorithm.getJourneys();
        std::cout << "Found " << journeys.size() << " journeys" << std::endl;
        
        // Print profiler statistics
        algorithm.getProfiler().printStatistics();
        
        return EXIT_SUCCESS;

    } catch (std::exception& e) {
        std::cerr << "ULTRARAPTOR error: " << e.what() << '\n';
        std::cerr << "Try '" << argv[0] << " -help' for more information.\n";
        return EXIT_FAILURE;
    }
}