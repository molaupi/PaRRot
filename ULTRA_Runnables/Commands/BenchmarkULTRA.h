#pragma once

#include <string>
#include <vector>
#include <iostream>

#include "../../include/ULTRA/Shell/Shell.h"

using namespace Shell;

#include "../../include/ULTRA/Algorithms/CSA/CSA.h"
#include "../../include/ULTRA/Algorithms/CSA/DijkstraCSA.h"
#include "../../include/ULTRA/Algorithms/CSA/HLCSA.h"
#include "../../include/ULTRA/Algorithms/CSA/ULTRACSA.h"
#include "../../include/ULTRA/Algorithms/RAPTOR/HLRAPTOR.h"
#include "../../include/ULTRA/Algorithms/RAPTOR/DijkstraRAPTOR.h"

#include "../../include/ULTRA/Algorithms/RAPTOR/InitialTransfers.h"
#include "../../include/ULTRA/Algorithms/RAPTOR/RAPTOR.h"
#include "../../include/ULTRA/Algorithms/RAPTOR/ULTRARAPTOR.h"
#include "../../include/ULTRA/Algorithms/TripBased/Query/Query.h"
#include "../../include/ULTRA/Algorithms/TripBased/Query/TransitiveQuery.h"

#include "../../include/ULTRA/DataStructures/Queries/Queries.h"
#include "../../include/ULTRA/DataStructures/CSA/Data.h"
#include "../../include/ULTRA/DataStructures/RAPTOR/Data.h"
#include "../../include/ULTRA/DataStructures/TripBased/Data.h"

class RunTransitiveCSAQueries : public ParameterizedCommand {

public:
    RunTransitiveCSAQueries(BasicShell &shell) :
            ParameterizedCommand(shell, "runTransitiveCSAQueries",
                                 "Runs the given number of random transitive CSA queries.") {
        addParameter("CSA input file");
        addParameter("Number of queries");
        addParameter("Target pruning?");
    }

    virtual void execute() noexcept {
        CSA::Data csaData = CSA::Data::FromBinary(getParameter("CSA input file"));
        csaData.sortConnectionsAscending();
        csaData.printInfo();
        CSA::CSA<true, CSA::AggregateProfiler> algorithm(csaData);

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<StopQuery> queries = generateRandomStopQueries(csaData.numberOfStops(), n);

        const bool targetPruning = getParameter<bool>("Target pruning?");

        for (const StopQuery &query: queries) {
            algorithm.run(query.source, query.departureTime, targetPruning ? query.target : noStop);
        }
        algorithm.getProfiler().printStatistics();
    }
};

class RunDijkstraCSAQueries : public ParameterizedCommand {

public:
    RunDijkstraCSAQueries(BasicShell &shell) :
            ParameterizedCommand(shell, "runDijkstraCSAQueries",
                                 "Runs the given number of random Dijkstra-CSA queries.") {
        addParameter("CSA input file");
        addParameter("CH data");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        CSA::Data csaData = CSA::Data::FromBinary(getParameter("CSA input file"));
        csaData.sortConnectionsAscending();
        csaData.printInfo();
        ULTRACH::CH ch(getParameter("CH data"));
        CSA::DijkstraCSA<RAPTOR::CoreCHInitialTransfers, true, CSA::AggregateProfiler> algorithm(csaData, ch);

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(ch.numVertices(), n);

        for (const VertexQuery &query: queries) {
            algorithm.run(query.source, query.departureTime, query.target);
        }
        algorithm.getProfiler().printStatistics();
    }
};

class RunHLCSAQueries : public ParameterizedCommand {

public:
    RunHLCSAQueries(BasicShell &shell) :
            ParameterizedCommand(shell, "runHLCSAQueries", "Runs the given number of random HL-CSA queries.") {
        addParameter("CSA input file");
        addParameter("Out-hub file");
        addParameter("In-hub file");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        CSA::Data csaData = CSA::Data::FromBinary(getParameter("CSA input file"));
        csaData.sortConnectionsAscending();
        csaData.printInfo();
        const TransferGraph outHubs(getParameter("Out-hub file"));
        const TransferGraph inHubs(getParameter("In-hub file"));
        CSA::HLCSA<CSA::AggregateProfiler> algorithm(csaData, outHubs, inHubs);

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(inHubs.numVertices(), n);

        for (const VertexQuery &query: queries) {
            algorithm.run(query.source, query.departureTime, query.target);
        }
        algorithm.getProfiler().printStatistics();
    }
};

class RunULTRACSAQueries : public ParameterizedCommand {

public:
    RunULTRACSAQueries(BasicShell &shell) :
            ParameterizedCommand(shell, "runULTRACSAQueries", "Runs the given number of random ULTRA-CSA queries.") {
        addParameter("CSA input file");
        addParameter("CH data");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        CSA::Data csaData = CSA::Data::FromBinary(getParameter("CSA input file"));
        csaData.sortConnectionsAscending();
        csaData.printInfo();
        ULTRACH::CH ch(getParameter("CH data"));
        CSA::ULTRACSA<true, CSA::AggregateProfiler> algorithm(csaData, ch);

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(ch.numVertices(), n);

        for (const VertexQuery &query: queries) {
            algorithm.run(query.source, query.departureTime, query.target);
        }
        algorithm.getProfiler().printStatistics();
    }
};

class RunTransitiveRAPTORQueries : public ParameterizedCommand {

public:
    RunTransitiveRAPTORQueries(BasicShell &shell) :
            ParameterizedCommand(shell, "runTransitiveRAPTORQueries",
                                 "Runs the given number of random transitive RAPTOR queries.") {
        addParameter("RAPTOR input file");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
        raptorData.useImplicitDepartureBufferTimes();
        raptorData.printInfo();
        RAPTOR::RAPTOR<true, RAPTOR::AggregateProfiler, true, false> algorithm(raptorData);

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<StopQuery> queries = generateRandomStopQueries(raptorData.numberOfStops(), n);

        double numJourneys = 0;
        for (const StopQuery &query: queries) {
            algorithm.run(query.source, query.departureTime, query.target);
            numJourneys += algorithm.getJourneys().size();
        }
        algorithm.getProfiler().printStatistics();
        std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys / n) << std::endl;

    }
};

class RunDijkstraRAPTORQueries : public ParameterizedCommand {

public:
    RunDijkstraRAPTORQueries(BasicShell &shell) :
            ParameterizedCommand(shell, "runDijkstraRAPTORQueries",
                                 "Runs the given number of random Dijkstra RAPTOR queries.") {
        addParameter("RAPTOR input file");
        addParameter("CH data");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
        raptorData.useImplicitDepartureBufferTimes();
        raptorData.printInfo();
        ULTRACH::CH ch(getParameter("CH data"));
        RAPTOR::DijkstraRAPTOR<RAPTOR::CoreCHInitialTransfers, RAPTOR::AggregateProfiler, true, false> algorithm(
                raptorData, ch);

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(ch.numVertices(), n);

        double numJourneys = 0;
        for (const VertexQuery &query: queries) {
            algorithm.run(query.source, query.departureTime, query.target);
            numJourneys += algorithm.getJourneys().size();
        }
        algorithm.getProfiler().printStatistics();
        std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys / n) << std::endl;
    }
};

class RunHLRAPTORQueries : public ParameterizedCommand {

public:
    RunHLRAPTORQueries(BasicShell &shell) :
            ParameterizedCommand(shell, "runHLRAPTORQueries", "Runs the given number of random HL-RAPTOR queries.") {
        addParameter("RAPTOR input file");
        addParameter("Out-hub file");
        addParameter("In-hub file");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
        raptorData.useImplicitDepartureBufferTimes();
        raptorData.printInfo();
        const TransferGraph outHubs(getParameter("Out-hub file"));
        const TransferGraph inHubs(getParameter("In-hub file"));
        RAPTOR::HLRAPTOR<RAPTOR::AggregateProfiler> algorithm(raptorData, outHubs, inHubs);

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(inHubs.numVertices(), n);

        double numJourneys = 0;
        for (const VertexQuery &query: queries) {
            algorithm.run(query.source, query.departureTime, query.target);
            numJourneys += algorithm.getJourneys().size();
        }
        algorithm.getProfiler().printStatistics();
        std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys / n) << std::endl;
    }
};

class RunULTRARAPTORQueries : public ParameterizedCommand {

public:
    RunULTRARAPTORQueries(BasicShell &shell) :
            ParameterizedCommand(shell, "runULTRARAPTORQueries",
                                 "Runs the given number of random ULTRA-RAPTOR queries.") {
        addParameter("RAPTOR input file");
        addParameter("CH data");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
        raptorData.useImplicitDepartureBufferTimes();
        raptorData.printInfo();
        ULTRACH::CH ch(getParameter("CH data"));
        RAPTOR::ULTRARAPTOR<RAPTOR::AggregateProfiler, false> algorithm(raptorData, ch);

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(ch.numVertices(), n);

        double numJourneys = 0;
        for (const VertexQuery &query: queries) {
            algorithm.run(query.source, query.departureTime, query.target);
            numJourneys += algorithm.getJourneys().size();
        }
        algorithm.getProfiler().printStatistics();
        std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys / n) << std::endl;
    }
};

class RunULTRARAPTORWithGivenQueries : public ParameterizedCommand {

public:
    RunULTRARAPTORWithGivenQueries(BasicShell &shell) :
            ParameterizedCommand(shell, "runULTRARAPTORWithGivenQueries", "Runs ULTRA-RAPTOR for the given queries.") {
        addParameter("RAPTOR input file");
        addParameter("CH data");
        addParameter("ULTRA requests");
        addParameter("Journey output file");
        addParameter("Prevent Walking", "true", {"true", "false"});
    }

    virtual void execute() noexcept {
        const std::string ultraRequestsFileName = getParameter("ULTRA requests");
        std::vector<VertexQuery> queries;

        static constexpr IO::IgnoreColumn ReadMode = IO::IGNORE_NO_COLUMN;
        IO::CSVReader<5, IO::TrimChars<>, IO::DoubleQuoteEscape<',','"'>> in(ultraRequestsFileName);
        in.readHeader(ReadMode, "origin", "destination", "req_time", "source", "target");
        int origin = 0;
        int destination = 0;
        int requestTime = 0;
        int source = 0;
        int target = 0;

        while (in.readRow(origin, destination, requestTime, source, target)) {
            const VertexQuery query(Vertex(source), Vertex(target), requestTime);
            queries.push_back(query);
        }
        

        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
        raptorData.useImplicitDepartureBufferTimes();
        raptorData.printInfo();
        ULTRACH::CH ch(getParameter("CH data"));

        const bool preventWalking = getParameter<bool>("Prevent Walking");
        const std::string outFileName = getParameter("Journey output file");
        if (preventWalking) {
            RAPTOR::ULTRARAPTOR<RAPTOR::AggregateProfiler, true> algorithm(raptorData, ch);
            runQueriesAndWriteJourneyStats(algorithm, queries, outFileName);
        } else {
            RAPTOR::ULTRARAPTOR<RAPTOR::AggregateProfiler, false> algorithm(raptorData, ch);
            runQueriesAndWriteJourneyStats(algorithm, queries, outFileName);
        }
    }

private:

    template<typename Algorithm>
    void runQueriesAndWriteJourneyStats(Algorithm &algorithm, const std::vector<VertexQuery> &queries,
                                        const std::string &outFileName) noexcept {
        const size_t n = queries.size();
        std::cout << "Running queries ..." << std::flush;
        ULTRAProgressBar progressBar(n);
        progressBar.SetDotOutputStep(1);
        progressBar.SetPercentOutputStep(5);
        double numJourneys = 0;
        std::ofstream out(outFileName);
        if (!out.good()) {
            std::cerr << "Could not open output file " << outFileName << " for writing journeys.";
            return;
        }
        out << "request_id,departure_time,arrival_time,num_trips,"
               "accegr_transfer_time,intermediate_transfer_time,wait_time,in_vehicle_time\n";
        size_t requestId = 0;
        for (const VertexQuery &query: queries) {
            algorithm.run(query.source, query.departureTime, query.target);
            numJourneys += algorithm.getJourneys().size();
            const auto journeys = algorithm.getJourneys();
            const auto arrivals = algorithm.getArrivals();
            Assert(journeys.size() == arrivals.size(), "Number of journeys and arrivals do not match.");
            for (size_t i = 0; i < journeys.size(); ++i) {
                const RAPTOR::Journey &journey = journeys[i];
                const auto &arrival = arrivals[i];
                const int accEgrTransferTime = RAPTOR::initialTransferTime(journey);
                const int intermediateTransferTime = RAPTOR::intermediateTransferTime(journey);
                int inVehicleTime = 0;
                for (const auto &leg: journey) {
                    if (leg.usesRoute) {
                        inVehicleTime += leg.arrivalTime - leg.departureTime;
                    }
                }
                const int waitTime = arrival.arrivalTime - query.departureTime - inVehicleTime - accEgrTransferTime -
                                     intermediateTransferTime;
                out << requestId << "," << query.departureTime << "," << arrival.arrivalTime << ","
                    << arrival.numberOfTrips << "," << accEgrTransferTime << ","
                    << intermediateTransferTime << "," << waitTime << "," << inVehicleTime << "\n";

//                // print coordinates of path for debug
//                const auto path = RAPTOR::journeyToPath(journey);
//                std::cout << "Path for request " << requestId << ", journey" << i << ": ";
//                for (size_t j = 1; j < path.size() - 1; ++j) {
//                    const auto coord = transferGraph.get(Coordinates, path[j]);
//                    std::cout << "(" << path[j].value() << "," << coord.latitude << "," << coord.longitude << ")";
//                    if (j + 1 < path.size() - 1) {
//                        std::cout << ", ";
//                    } else {
//                        std::cout << std::endl;
//                    }
//                }

            }
            ++requestId;
            ++progressBar;
        }
        out.close();
        std::cout << " done." << std::endl;
        algorithm.getProfiler().printStatistics();
        std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys / n) << std::endl;
    }
};

class RunTransitiveTBQueries : public ParameterizedCommand {

public:
    RunTransitiveTBQueries(BasicShell &shell) :
            ParameterizedCommand(shell, "runTransitiveTBQueries",
                                 "Runs the given number of random transitive TB queries.") {
        addParameter("Trip-Based input file");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        TripBased::Data tripBasedData(getParameter("Trip-Based input file"));
        tripBasedData.printInfo();
        TripBased::TransitiveQuery<TripBased::AggregateProfiler> algorithm(tripBasedData);

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<StopQuery> queries = generateRandomStopQueries(tripBasedData.numberOfStops(), n);

        double numJourneys = 0;
        for (const StopQuery &query: queries) {
            algorithm.run(query.source, query.departureTime, query.target);
            numJourneys += algorithm.getJourneys().size();
        }
        algorithm.getProfiler().printStatistics();
        std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys / n) << std::endl;
    }
};

class RunULTRATBQueries : public ParameterizedCommand {

public:
    RunULTRATBQueries(BasicShell &shell) :
            ParameterizedCommand(shell, "runULTRATBQueries", "Runs the given number of random ULTRA-TB queries.") {
        addParameter("Trip-Based input file");
        addParameter("CH data");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        TripBased::Data tripBasedData(getParameter("Trip-Based input file"));
        tripBasedData.printInfo();
        ULTRACH::CH ch(getParameter("CH data"));
        TripBased::Query<TripBased::AggregateProfiler> algorithm(tripBasedData, ch);

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(ch.numVertices(), n);

        double numJourneys = 0;
        for (const VertexQuery &query: queries) {
            algorithm.run(query.source, query.departureTime, query.target);
            numJourneys += algorithm.getJourneys().size();
        }
        algorithm.getProfiler().printStatistics();
        std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys / n) << std::endl;
    }
};
