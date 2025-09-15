#pragma once

#include <string>

#include "../../include/ULTRA/DataStructures/CSA/Data.h"
#include "../../include/ULTRA/DataStructures/GTFS/Data.h"
#include "../../include/ULTRA/DataStructures/Graph/Graph.h"
#include "../../include/ULTRA/DataStructures/Intermediate/Data.h"
#include "../../include/ULTRA/DataStructures/RAPTOR/Data.h"
#include "../../include/ULTRA/DataStructures/RAPTOR/MultimodalData.h"
#include "../../include/ULTRA/DataStructures/TripBased/MultimodalData.h"
#include "../../include/ULTRA/Shell/Shell.h"

using namespace Shell;

class ParseGTFS : public ParameterizedCommand {
public:
    ParseGTFS(BasicShell& shell)
        : ParameterizedCommand(shell, "parseGTFS",
            "Parses raw GTFS data from the given directory "
            "and converts it to a binary representation.")
    {
        addParameter("Input directory");
        addParameter("Output file");
    }

    virtual void execute() noexcept
    {
        const std::string gtfsDirectory = getParameter("Input directory");
        const std::string outputFile = getParameter("Output file");

        GTFS::Data data = GTFS::Data::FromGTFS(gtfsDirectory);
        data.printInfo();
        data.serialize(outputFile);
    }
};

class GTFSToIntermediate : public ParameterizedCommand {
public:
    GTFSToIntermediate(BasicShell& shell)
        : ParameterizedCommand(
            shell, "gtfsToIntermediate",
            "Converts binary GTFS data to the intermediate network format.")
    {
        addParameter("Input directory");
        addParameter("First day");
        addParameter("Last day");
        addParameter("Use days of operation?");
        addParameter("Use frequencies?");
        addParameter("Output file");
    }

    virtual void execute() noexcept
    {
        const std::string gtfsDirectory = getParameter("Input directory");
        const std::string outputFile = getParameter("Output file");
        const int firstDay = stringToDay(getParameter("First day"));
        const int lastDay = stringToDay(getParameter("Last day"));
        const bool useDaysOfOperation = getParameter<bool>("Use days of operation?");
        const bool useFrequencies = getParameter<bool>("Use frequencies?");

        GTFS::Data gtfs = GTFS::Data::FromBinary(gtfsDirectory);
        gtfs.printInfo();
        Intermediate::Data inter = Intermediate::Data::FromGTFS(
            gtfs, firstDay, lastDay, !useDaysOfOperation, !useFrequencies);
        inter.printInfo();
        inter.serialize(outputFile);
    }
};

class IntermediateToCSA : public ParameterizedCommand {
public:
    IntermediateToCSA(BasicShell& shell)
        : ParameterizedCommand(
            shell, "intermediateToCSA",
            "Converts binary intermediate data to CSA network format.")
    {
        addParameter("Input file");
        addParameter("Output file");
    }

    virtual void execute() noexcept
    {
        const std::string inputFile = getParameter("Input file");
        const std::string outputFile = getParameter("Output file");

        Intermediate::Data inter = Intermediate::Data::FromBinary(inputFile);
        inter.printInfo();
        CSA::Data data = CSA::Data::FromIntermediate(inter);
        data.printInfo();
        data.serialize(outputFile);
    }
};

class IntermediateToRAPTOR : public ParameterizedCommand {
public:
    IntermediateToRAPTOR(BasicShell& shell)
        : ParameterizedCommand(
            shell, "intermediateToRAPTOR",
            "Converts binary intermediate data to RAPTOR network format.")
    {
        addParameter("Input file");
        addParameter("Output file");
        addParameter("Route type", "FIFO",
            { "Geographic", "FIFO", "Offset", "Frequency" });
    }

    virtual void execute() noexcept
    {
        const std::string inputFile = getParameter("Input file");
        const std::string outputFile = getParameter("Output file");
        const std::string routeTypeString = getParameter("Route type");
        int routeType;
        if (routeTypeString == "Geographic") {
            routeType = 0;
        } else if (routeTypeString == "FIFO") {
            routeType = 1;
        } else if (routeTypeString == "Offset") {
            routeType = 2;
        } else {
            routeType = 3;
        }

        Intermediate::Data inter = Intermediate::Data::FromBinary(inputFile);
        inter.printInfo();
        RAPTOR::Data data = RAPTOR::Data::FromIntermediate(inter, routeType);
        data.printInfo();
        ULTRAGraph::printInfo(data.transferGraph);
        data.transferGraph.printAnalysis();
        data.serialize(outputFile);
    }
};

class BuildMultimodalRAPTORData : public ParameterizedCommand {
public:
    BuildMultimodalRAPTORData(BasicShell& shell)
        : ParameterizedCommand(
            shell, "buildMultimodalRAPTORData",
            "Builds multimodal RAPTOR data based on RAPTOR data.")
    {
        addParameter("RAPTOR data");
        addParameter("Output file");
    }

    virtual void execute() noexcept
    {
        const RAPTOR::Data raptorData(getParameter("RAPTOR data"));
        raptorData.printInfo();
        const RAPTOR::MultimodalData multimodalData(raptorData);
        multimodalData.printInfo();
        multimodalData.serialize(getParameter("Output file"));
    }
};

class AddModeToMultimodalRAPTORData : public ParameterizedCommand {
public:
    AddModeToMultimodalRAPTORData(BasicShell& shell)
        : ParameterizedCommand(shell, "addModeToMultimodalRAPTORData",
            "Adds a transfer graph for the specified mode to "
            "multimodal RAPTOR data.")
    {
        addParameter("Multimodal RAPTOR data");
        addParameter("Transfer graph");
        addParameter("Mode");
        addParameter("Output file");
    }

    virtual void execute() noexcept
    {
        RAPTOR::MultimodalData multimodalData(
            getParameter("Multimodal RAPTOR data"));
        multimodalData.printInfo();
        RAPTOR::TransferGraph graph;
        graph.readBinary(getParameter("Transfer graph"));
        const size_t mode = RAPTOR::getTransferModeFromName(getParameter("Mode"));
        multimodalData.addTransferGraph(mode, graph);
        multimodalData.printInfo();
        multimodalData.serialize(getParameter("Output file"));
    }
};

class BuildMultimodalTripBasedData : public ParameterizedCommand {
public:
    BuildMultimodalTripBasedData(BasicShell& shell)
        : ParameterizedCommand(
            shell, "buildMultimodalTripBasedData",
            "Builds multimodal Trip-Based data based on Trip-Based data.")
    {
        addParameter("Trip-Based data");
        addParameter("Output file");
    }

    virtual void execute() noexcept
    {
        const TripBased::Data tripBasedData(getParameter("Trip-Based data"));
        tripBasedData.printInfo();
        const TripBased::MultimodalData multimodalData(tripBasedData);
        multimodalData.printInfo();
        multimodalData.serialize(getParameter("Output file"));
    }
};

class AddModeToMultimodalTripBasedData : public ParameterizedCommand {
public:
    AddModeToMultimodalTripBasedData(BasicShell& shell)
        : ParameterizedCommand(shell, "addModeToMultimodalTripBasedData",
            "Adds a transfer graph for the specified mode to "
            "multimodal Trip-Based data.")
    {
        addParameter("Multimodal Trip-Based data");
        addParameter("Transfer graph");
        addParameter("Mode");
        addParameter("Output file");
    }

    virtual void execute() noexcept
    {
        TripBased::MultimodalData multimodalData(
            getParameter("Multimodal Trip-Based data"));
        multimodalData.printInfo();
        TransferGraph graph;
        graph.readBinary(getParameter("Transfer graph"));
        const size_t mode = RAPTOR::getTransferModeFromName(getParameter("Mode"));
        multimodalData.addTransferGraph(mode, graph);
        multimodalData.printInfo();
        multimodalData.serialize(getParameter("Output file"));
    }
};

class LoadDimacsGraph : public ParameterizedCommand {
public:
    LoadDimacsGraph(BasicShell& shell)
        : ParameterizedCommand(
            shell, "loadDimacsGraph",
            "Converts DIMACS graph data to our transfer graph format.")
    {
        addParameter("Input file");
        addParameter("Output file");
        addParameter("Graph type", "dynamic", { "static", "dynamic" });
        addParameter("Coordinate factor", "0.000001");
    }

    virtual void execute() noexcept
    {
        std::string graphType = getParameter("Graph type");
        if (graphType == "static") {
            load<TransferGraph>();
        } else {
            load<DynamicTransferGraph>();
        }
    }

private:
    template <typename GRAPH_TYPE>
    inline void load() const noexcept
    {
        DimacsGraphWithCoordinates dimacs;
        dimacs.fromDimacs<true>(getParameter("Input file"),
            getParameter<double>("Coordinate factor"));
        ULTRAGraph::printInfo(dimacs);
        dimacs.printAnalysis();
        GRAPH_TYPE graph;
        ULTRAGraph::move(std::move(dimacs), graph);
        ULTRAGraph::printInfo(graph);
        graph.printAnalysis();
        graph.writeBinary(getParameter("Output file"));
    }
};

class RAPTORToCSV : public ParameterizedCommand {
public:
    RAPTORToCSV(BasicShell& shell)
        : ParameterizedCommand(
            shell, "raptorToCSV",
            "Write the given RAPTOR into CSV files.")
    {
        addParameter("Input file");
        addParameter("Output csv file");
    }

    virtual void execute() noexcept
    {
        const std::string inputFile = getParameter("Input file");
        const std::string outputFile = getParameter("Output csv file");

        RAPTOR::Data raptor(inputFile);
        raptor.writeCSV(outputFile);
    }
};

class RandomVertexQuery : public ParameterizedCommand {
public:
    RandomVertexQuery(BasicShell& shell)
        : ParameterizedCommand(
            shell, "randomVertexQueries",
            "Generate random vertex queries and write them into CSV file.")
    {
        addParameter("RAPTOR input file");
        addParameter("CH data");
        addParameter("Number of queries");
        addParameter("Output csv file");
    }

    virtual void execute() noexcept
    {
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
        raptorData.useImplicitDepartureBufferTimes();
        ULTRACH::CH ch(getParameter("CH data"));

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(ch.numVertices(), n);
        const auto& coordinates = raptorData.getCoordinates();
        std::cout << "Coordinates of RAPTOR data, size of " << coordinates.size() << "\n";

        const std::string outputFile = getParameter("Output csv file");
        std::ofstream file(outputFile);
        Assert(file);
        Assert(file.is_open());
        file << "sourceId,targetId,origin,destination\n";
        for (const auto& query : queries) {
            file << query.source.value() << "," 
                 << query.target.value() << ",(" 
                 << coordinates[query.source].latitude << "|" << coordinates[query.source].longitude << "),(" 
                 << coordinates[query.target].latitude << "|" << coordinates[query.target].longitude << ")\n";
        }
        file.close();

    }
};
