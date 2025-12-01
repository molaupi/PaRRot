#pragma once

#include <string>
#include <vector>
#include <iostream>

#include "../../include/ULTRA/Shell/Shell.h"
using namespace Shell;
    
#include "../../include/ULTRA/Algorithms/RAPTOR/Bounded/BoundedMcRAPTOR.h"
#include "../../include/ULTRA/Algorithms/RAPTOR/ULTRABounded/UBMHydRA.h"
#include "../../include/ULTRA/Algorithms/RAPTOR/ULTRABounded/UBMRAPTOR.h"
#include "../../include/ULTRA/Algorithms/RAPTOR/InitialTransfers.h"
#include "../../include/ULTRA/Algorithms/RAPTOR/McRAPTOR.h"
#include "../../include/ULTRA/Algorithms/RAPTOR/MCR.h"
#include "../../include/ULTRA/Algorithms/RAPTOR/ULTRAMcRAPTOR.h"
#include "../../include/ULTRA/Algorithms/TripBased/BoundedMcQuery/BoundedMcQuery.h"
#include "../../include/ULTRA/Algorithms/TripBased/Query/McQuery.h"
#include "../../include/ULTRA/DataStructures/Queries/Queries.h"
#include "../../include/ULTRA/DataStructures/CSA/Data.h"
#include "../../include/ULTRA/DataStructures/RAPTOR/Data.h"
#include "../../include/ULTRA/DataStructures/TripBased/Data.h"
class RunTransitiveMcRAPTORQueries : public ParameterizedCommand {

public:
    RunTransitiveMcRAPTORQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runTransitiveMcRAPTORQueries", "Runs the given number of random transitive McRAPTOR queries.") {
        addParameter("RAPTOR input file");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
        raptorData.useImplicitDepartureBufferTimes();
        raptorData.printInfo();
        RAPTOR::McRAPTOR<true, true, RAPTOR::AggregateProfiler> algorithm(raptorData);

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<StopQuery> queries = generateRandomStopQueries(raptorData.numberOfStops(), n);

        double numJourneys = 0;
        for (const StopQuery& query : queries) {
            algorithm.run(query.source, query.departureTime, query.target);
            numJourneys += algorithm.getJourneys().size();
        }
        algorithm.getProfiler().printStatistics();
        std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys/n) << std::endl;
    }
};

class RunMCRQueries : public ParameterizedCommand {

public:
    RunMCRQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runMCRQueries", "Runs the given number of random MCR queries.") {
        addParameter("RAPTOR input file");
        addParameter("CH data");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
        raptorData.useImplicitDepartureBufferTimes();
        raptorData.printInfo();
        ULTRACH::CH ch(getParameter("CH data"));
        RAPTOR::MCR<true, RAPTOR::AggregateProfiler> algorithm(raptorData, ch);

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(ch.numVertices(), n);

        double numJourneys = 0;
        for (const VertexQuery& query : queries) {
            algorithm.run(query.source, query.departureTime, query.target);
            numJourneys += algorithm.getJourneys().size();
        }
        algorithm.getProfiler().printStatistics();
        std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys/n) << std::endl;
    }
};

class RunULTRAMcRAPTORQueries : public ParameterizedCommand {

public:
    RunULTRAMcRAPTORQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runULTRAMcRAPTORQueries", "Runs the given number of random ULTRA-McRAPTOR queries.") {
        addParameter("RAPTOR input file");
        addParameter("CH data");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
        raptorData.useImplicitDepartureBufferTimes();
        raptorData.printInfo();
        ULTRACH::CH ch(getParameter("CH data"));
        RAPTOR::ULTRAMcRAPTOR<RAPTOR::AggregateProfiler> algorithm(raptorData, ch);

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(ch.numVertices(), n);

        double numJourneys = 0;
        for (const VertexQuery& query : queries) {
            algorithm.run(query.source, query.departureTime, query.target);
            numJourneys += algorithm.getJourneys().size();
        }
        algorithm.getProfiler().printStatistics();
        std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys/n) << std::endl;
    }
};

class RunULTRAMcTBQueries : public ParameterizedCommand {

public:
    RunULTRAMcTBQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runULTRAMcTBQueries", "Runs the given number of random ULTRA-McTB queries.") {
        addParameter("Trip-Based input file");
        addParameter("CH data");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        TripBased::Data tripBasedData(getParameter("Trip-Based input file"));
        tripBasedData.printInfo();
        ULTRACH::CH ch(getParameter("CH data"));
        TripBased::McQuery<TripBased::AggregateProfiler> algorithm(tripBasedData, ch);

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(ch.numVertices(), n);

        double numJourneys = 0;
        for (const VertexQuery& query : queries) {
            algorithm.run(query.source, query.departureTime, query.target);
            numJourneys += algorithm.getJourneys().size();
        }
        algorithm.getProfiler().printStatistics();
        std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys/n) << std::endl;
    }
};

class RunTransitiveBoundedMcRAPTORQueries : public ParameterizedCommand {

public:
    RunTransitiveBoundedMcRAPTORQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runTransitiveBoundedMcRAPTORQueries", "Runs the given number of random transitive Bounded McRAPTOR queries.") {
        addParameter("RAPTOR input file");
        addParameter("Number of queries");
        addParameter("Arrival slack");
        addParameter("Trip slack");
    }

    virtual void execute() noexcept {
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
        raptorData.useImplicitDepartureBufferTimes();
        raptorData.printInfo();
        const RAPTOR::Data reverseData = raptorData.reverseNetwork();
        RAPTOR::BoundedMcRAPTOR<RAPTOR::AggregateProfiler> algorithm(raptorData, reverseData);

        const double arrivalSlack = getParameter<double>("Arrival slack");
        const double tripSlack = getParameter<double>("Trip slack");

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<StopQuery> queries = generateRandomStopQueries(raptorData.numberOfStops(), n);

        double numJourneys = 0;
        for (const StopQuery& query : queries) {
            algorithm.run(query.source, query.departureTime, query.target, arrivalSlack, tripSlack);
            numJourneys += algorithm.getJourneys().size();
        }
        algorithm.getProfiler().printStatistics();
        std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys/n) << std::endl;
    }
};

class RunUBMRAPTORQueries : public ParameterizedCommand {

public:
    RunUBMRAPTORQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runUBMRAPTORQueries", "Runs the given number of random UBM-RAPTOR queries.") {
        addParameter("RAPTOR input file");
        addParameter("CH data");
        addParameter("Number of queries");
        addParameter("Arrival slack");
        addParameter("Trip slack");
    }

    virtual void execute() noexcept {
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
        raptorData.useImplicitDepartureBufferTimes();
        raptorData.printInfo();
        const RAPTOR::Data reverseData = raptorData.reverseNetwork();
        ULTRACH::CH ch(getParameter("CH data"));
        RAPTOR::UBMRAPTOR<RAPTOR::AggregateProfiler> algorithm(raptorData, reverseData, ch);

        const double arrivalSlack = getParameter<double>("Arrival slack");
        const double tripSlack = getParameter<double>("Trip slack");

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(ch.numVertices(), n);

        double numJourneys = 0;
        for (const VertexQuery& query : queries) {
            algorithm.run(query.source, query.departureTime, query.target, arrivalSlack, tripSlack);
            numJourneys += algorithm.getJourneys().size();
        }
        algorithm.getProfiler().printStatistics();
        std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys/n) << std::endl;
    }
};


class RunUBMRAPTORWithGivenQueries : public ParameterizedCommand {

public:
    RunUBMRAPTORWithGivenQueries(BasicShell& shell) :
            ParameterizedCommand(shell, "runUBMRAPTORWithGivenQueries", "Runs the UBM-RAPTOR with the given queries.") {
        addParameter("RAPTOR input file");
        addParameter("CH data");
        addParameter("Arrival slack");
        addParameter("Trip slack");
        addParameter(
                "Queries"); //CSV file output by TransformKaRRiRequestsToULTRAQueries, header: "source,target,departure_time"
        addParameter("Journey output file");
    }

    virtual void execute() noexcept {
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
        raptorData.useImplicitDepartureBufferTimes();
        raptorData.printInfo();
        const RAPTOR::Data reverseData = raptorData.reverseNetwork();
        ULTRACH::CH ch(getParameter("CH data"));
        RAPTOR::UBMRAPTOR<RAPTOR::AggregateProfiler> algorithm(raptorData, reverseData, ch);

        const double arrivalSlack = getParameter<double>("Arrival slack");
        const double tripSlack = getParameter<double>("Trip slack");

        std::cout << "Reading queries..." << std::flush;

        const std::string queriesFile = getParameter("Queries");
        std::vector<VertexQuery> queries;
        static constexpr IO::IgnoreColumn ReadMode = IO::IGNORE_NO_COLUMN;
        IO::CSVReader<3, IO::TrimChars<>, IO::DoubleQuoteEscape<',', '"'>> in(queriesFile);
        in.readHeader(ReadMode, "source", "target", "departure_time");
        int source, target, departureTime;
        while (in.readRow(source, target, departureTime)) {
            queries.emplace_back(Vertex(source), Vertex(target), departureTime);
        }
        std::cout << " done." << std::endl;

        const size_t n = queries.size();
        std::cout << "Running queries ..." << std::flush;
        ULTRAProgressBar progressBar(n);
        progressBar.SetDotOutputStep(1);
        progressBar.SetPercentOutputStep(5);
        double numJourneys = 0;
        const std::string outFileName = getParameter("Journey output file");
        std::ofstream out(outFileName);
        if (!out.good()) {
            std::cerr << "Could not open output file " << outFileName << " for writing journeys.";
            return;
        }
        out << "request_id,departure_time,arrival_time,num_trips,"
               "accegr_transfer_time,intermediate_transfer_time,wait_time,in_vehicle_time\n";
        size_t requestId = 0;
        for (const VertexQuery &query: queries) {
            algorithm.run(query.source, query.departureTime, query.target, arrivalSlack, tripSlack);
            numJourneys += algorithm.getJourneys().size();
            const auto journeys = algorithm.getJourneys();
            const auto paretoLabels = algorithm.getResults();
            Assert(journeys.size() == paretoLabels.size(), "Number of journeys and paretoLabels do not match.");
            for (size_t i = 0; i < journeys.size(); ++i) {
                const RAPTOR::Journey &journey = journeys[i];
                const auto &label = paretoLabels[i];
                const int accEgrTransferTime = RAPTOR::initialTransferTime(journey);
                const int intermediateTransferTime = RAPTOR::intermediateTransferTime(journey);
                Assert(accEgrTransferTime + intermediateTransferTime == label.walkingDistance, "Transfer times from journey do not match walking distance from label.");
                int inVehicleTime = 0;
                for (const auto &leg: journey) {
                    if (leg.usesRoute) {
                        inVehicleTime += leg.arrivalTime - leg.departureTime;
                    }
                }
                const int waitTime = label.arrivalTime - query.departureTime - inVehicleTime - accEgrTransferTime -
                                     intermediateTransferTime;
                out << requestId << "," << query.departureTime << "," << label.arrivalTime << ","
                    << label.numberOfTrips << "," << accEgrTransferTime << ","
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

class RunUBMTBQueries : public ParameterizedCommand {

public:
    RunUBMTBQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runUBMTBQueries", "Runs the given number of random UBM-TB queries.") {
        addParameter("Trip-Based input file");
        addParameter("Bounded forward Trip-Based input file");
        addParameter("Bounded backward Trip-Based input file");
        addParameter("CH data");
        addParameter("Number of queries");
        addParameter("Arrival slack");
        addParameter("Trip slack");
    }

    virtual void execute() noexcept {
        TripBased::Data tripBasedData(getParameter("Trip-Based input file"));
        tripBasedData.printInfo();
        TripBased::Data forwardBoundedData(getParameter("Bounded forward Trip-Based input file"));
        forwardBoundedData.printInfo();
        TripBased::Data backwardBoundedData(getParameter("Bounded backward Trip-Based input file"));
        backwardBoundedData.printInfo();
        ULTRACH::CH ch(getParameter("CH data"));
        TripBased::BoundedMcQuery<TripBased::AggregateProfiler> algorithm(tripBasedData, forwardBoundedData, backwardBoundedData, ch);

        const double arrivalSlack = getParameter<double>("Arrival slack");
        const double tripSlack = getParameter<double>("Trip slack");

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(ch.numVertices(), n);

        double numJourneys = 0;
        for (const VertexQuery& query : queries) {
            algorithm.run(query.source, query.departureTime, query.target, arrivalSlack, tripSlack);
            numJourneys += algorithm.getJourneys().size();
        }
        algorithm.getProfiler().printStatistics();
        std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys/n) << std::endl;
    }
};

class RunUBMHydRAQueries : public ParameterizedCommand {

public:
    RunUBMHydRAQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runUBMHydRAQueries", "Runs the given number of random UBM-HydRA queries.") {
        addParameter("Trip-Based input file");
        addParameter("Bounded forward Trip-Based input file");
        addParameter("Bounded backward Trip-Based input file");
        addParameter("CH data");
        addParameter("Number of queries");
        addParameter("Arrival slack");
        addParameter("Trip slack");
    }

    virtual void execute() noexcept {
        const TripBased::Data tripBasedData(getParameter("Trip-Based input file"));
        tripBasedData.printInfo();
        const TripBased::Data forwardBoundedData(getParameter("Bounded forward Trip-Based input file"));
        forwardBoundedData.printInfo();
        const TripBased::Data backwardBoundedData(getParameter("Bounded backward Trip-Based input file"));
        backwardBoundedData.printInfo();
        const ULTRACH::CH ch(getParameter("CH data"));

        RAPTOR::UBMHydRA<RAPTOR::AggregateProfiler> algorithm(tripBasedData, forwardBoundedData, backwardBoundedData, ch);

        const double arrivalSlack = getParameter<double>("Arrival slack");
        const double tripSlack = getParameter<double>("Trip slack");

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(ch.numVertices(), n);

        double numJourneys = 0;
        for (const VertexQuery& query : queries) {
            algorithm.run(query.source, query.departureTime, query.target, arrivalSlack, tripSlack);
            numJourneys += algorithm.getJourneys().size();
        }
        algorithm.getProfiler().printStatistics();
        std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys/n) << std::endl;
    }
};

class ComputeTransferTimeSavings : public ParameterizedCommand {

public:
    ComputeTransferTimeSavings(BasicShell& shell) :
        ParameterizedCommand(shell, "computeTransferTimeSavings", "Computes the savings in transfer time of a 3-criteria (bounded) Pareto set compared to a 2-criteria one.") {
        addParameter("RAPTOR input file");
        addParameter("CH data");
        addParameter("Number of queries");
        addParameter("Output file");
    }

    virtual void execute() noexcept {
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
        raptorData.useImplicitDepartureBufferTimes();
        raptorData.printInfo();
        const RAPTOR::Data reverseData = raptorData.reverseNetwork();
        ULTRACH::CH ch(getParameter("CH data"));
        RAPTOR::UBMRAPTOR<RAPTOR::AggregateProfiler> algorithm(raptorData, reverseData, ch);

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(ch.numVertices(), n);

        std::ofstream outputFile(getParameter("Output file"));
        outputFile << std::setprecision(10);
        outputFile << "ArrivalSlack";
        for (const double tripSlack : tripSlacks) {
            const int slackAsInt = tripSlack * 100 - 100;
            for (const double threshold : thresholds) {
                const int thresholdAsInt = threshold * 100;
                outputFile << "\tTripSlack" << slackAsInt << "Savings" << thresholdAsInt;
            }
        }
        outputFile << "\n";
        outputFile.flush();

        for (const double arrivalSlack : arrivalSlacks) {
            outputFile << arrivalSlack;
            for (const double tripSlack : tripSlacks) {
                std::cout << "Arrival slack: " << arrivalSlack << ", trip slack: " << tripSlack << std::endl;
                std::vector<double> transferTimeSavings;
                for (const VertexQuery& query : queries) {
                    algorithm.run(query.source, query.departureTime, query.target, arrivalSlack, tripSlack);
                    const std::vector<RAPTOR::WalkingParetoLabel> fullLabels = algorithm.getResults();
                    const std::vector<RAPTOR::ArrivalLabel>& anchorLabels = algorithm.getAnchorLabels();
                    RAPTOR::WalkingParetoLabel bestLabel;
                    RAPTOR::WalkingParetoLabel bestAnchorLabel;
                    for (const RAPTOR::WalkingParetoLabel& label : fullLabels) {
                        if (label.walkingDistance <= bestLabel.walkingDistance) {
                            bestLabel = label;
                        }
                        if (label.walkingDistance <= bestAnchorLabel.walkingDistance && isAnchorLabel(label, anchorLabels)) {
                            bestAnchorLabel = label;
                        }
                    }
                    if (bestAnchorLabel.walkingDistance == 0) {
                        transferTimeSavings.emplace_back(0);
                    } else {
                        transferTimeSavings.emplace_back((bestAnchorLabel.walkingDistance - bestLabel.walkingDistance)/static_cast<double>(bestAnchorLabel.walkingDistance));
                    }
                }
                std::sort(transferTimeSavings.begin(), transferTimeSavings.end(), [&](const double a, const double b) {
                    return a > b;
                });
                size_t j = 0;
                std::vector<size_t> savingsCount(thresholds.size(), 0);
                for (const double s : transferTimeSavings) {
                    while (s < thresholds[j]) {
                        j++;
                        if (j == thresholds.size()) break;
                    }
                    if (j == thresholds.size()) break;
                    savingsCount[j]++;
                }
                for (const size_t c : savingsCount) {
                    const double ratio = c/static_cast<double>(transferTimeSavings.size());
                    outputFile << "\t" << ratio;
                }

            }
            outputFile << "\n";
            outputFile.flush();
        }
    }

private:
    std::vector<double> thresholds { 0.75, 0.5, 0.25 };
    std::vector<double> arrivalSlacks { 1, 1.1, 1.2, 1.3, 1.4, 1.5 };
    std::vector<double> tripSlacks { 1, 1.25, 1.5 };

    inline bool isAnchorLabel(const RAPTOR::WalkingParetoLabel& label, const std::vector<RAPTOR::ArrivalLabel>& anchorLabels) const noexcept {
        for (const RAPTOR::ArrivalLabel& anchorLabel : anchorLabels) {
            if (label.arrivalTime != anchorLabel.arrivalTime) continue;
            if (label.numberOfTrips != anchorLabel.numberOfTrips) continue;
            return true;
        }
        return false;
    }
};
