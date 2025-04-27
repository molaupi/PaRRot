#pragma once

#include <csv.h>

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "../../../KARRI/Algorithms/CH/CH.h"
#include "../../Algorithms/CH/Preprocessing/CHData.h"
#include "../../DataStructures/Attributes/AttributeNames.h"
#include "../../DataStructures/Geometry/Point.h"
#include "../../DataStructures/Graph/Classes/GraphWrapper.h"
#include "../../DataStructures/Graph/Graph.h"
#include "../../DataStructures/RAPTOR/Data.h"
#include "../../DataStructures/RideRAPTOR/Data.h"
#include "../../Shell/Shell.h"
// TODO adapt correct includes
/* #include "../../../KARRI/Algorithms/KaRRi/AssignmentFinder.h" */
/* #include "../../../KARRI/Algorithms/KaRRi/BaseObjects/Request.h" */
/* #include "../../../KARRI/Algorithms/KaRRi/BaseObjects/Vehicle.h" */
/* #include "../../../KARRI/Algorithms/KaRRi/CHEnvironment.h" */
/* #include "../../../KARRI/Algorithms/KaRRi/PbnsAssignments/VehicleLocator.h" */

#include "../../Algorithms/RAPTOR/Profiler.h"
#include "../../Algorithms/RAPTOR/RAPTOR.h"
/* #include "../../Algorithms/RideRAPTOR/RideRAPTOR.h" */
/* #include "../../Helpers/ConstructorTags.h" */
#include "../../../KARRI/Tools/Constants.h"
#include "../../../KARRI/Tools/Logging/LogManager.h"
#include "../../../KARRI/Tools/StringHelpers.h"

using namespace Shell;

inline constexpr int TIMEINTERVAL = 86400;
inline constexpr auto RIDESHARING_FILENAME = "../Data/StuttgartCity/RideData/veh100_req200_serv28800_alpha1.1/rideData";
inline constexpr auto ROADNETWORK_FILENAME = "../Data/StuttgartCity/Car/Extended/roadGraph";
inline constexpr auto ROADNETWORKCH_FILENAME = "../Data/StuttgartCity/Car/CH/ch";
inline constexpr auto RAPTOR_FILENAME = "../Data/StuttgartCity/Raptor/raptor.binary";
inline constexpr auto WALKINGCH_FILENAME = "../Data/StuttgartCity/Walking/CoreCH/ch";
inline constexpr auto DISTANCEMATRIX_FILENAME = "../Data/StuttgartCity/RideData/distanceMatrix";

inline const Geometry::Rectangle boundingBox = Geometry::Rectangle::BoundingBox(
    Geometry::Point(Construct::LatLong, 48.945354, 8.924070),
    Geometry::Point(Construct::LatLong, 48.574399, 9.482074));

struct RideStopQuery {
    RideStopQuery(const Edge source, const Edge target, const int departureTime)
        : source(source)
        , target(target)
        , departureTime(departureTime)
    {
    }

    Edge source;
    Edge target;
    int departureTime;
};

inline std::vector<RideStopQuery> generateRandomRideStopQueries(
    const size_t numStops, const size_t numQueries) noexcept
{
    std::mt19937 randomGenerator(42);
    std::uniform_int_distribution<> stopDistribution(0, numStops - 1);
    std::uniform_int_distribution<> timeDistribution(0, TIMEINTERVAL);
    std::vector<RideStopQuery> queries;
    for (size_t i = 0; i < numQueries; i++) {
        queries.emplace_back(Edge(stopDistribution(randomGenerator)),
            Edge(stopDistribution(randomGenerator)),
            timeDistribution(randomGenerator));
    }
    return queries;
}

inline std::vector<RideStopQuery> generateRandomRideStopQueries(
    const size_t numStops, const size_t numQueries,
    RAPTOR::Data raptorData) noexcept
{
    if (numStops != raptorData.numberOfStops()) {
        return generateRandomRideStopQueries(numStops, numQueries);
    }

    RAPTOR::RAPTOR<true, RAPTOR::NoProfiler, false, false, false> raptor(
        raptorData);
    std::mt19937 randomGenerator(42);
    std::uniform_int_distribution<> stopDistribution(0, numStops - 1);
    std::uniform_int_distribution<> timeDistribution(0, TIMEINTERVAL);
    std::vector<RideStopQuery> queries;
    int i = 0;
    while (i < numQueries) {
        RideStopQuery query(Edge(stopDistribution(randomGenerator)),
            Edge(stopDistribution(randomGenerator)),
            timeDistribution(randomGenerator));
        raptor.run(StopId(query.source), query.departureTime, StopId(query.target));

        if (raptor.getJourneys().size() == 0) {
            std::cout << "skipped" << std::endl;
            continue;
        }
        queries.emplace_back(query);
        i++;
    }
    return queries;
}

struct StopQuery {
    StopQuery(const StopId source, const StopId target, const int departureTime)
        : source(source)
        , target(target)
        , departureTime(departureTime)
    {
    }

    StopId source;
    StopId target;
    int departureTime;
};

inline std::vector<StopQuery> generateRandomStopQueries(
    const size_t numStops, const size_t numQueries) noexcept
{
    std::mt19937 randomGenerator(42);
    std::uniform_int_distribution<> stopDistribution(0, numStops - 1);
    std::uniform_int_distribution<> timeDistribution(0, (24 * 60 * 60) - 1);
    std::vector<StopQuery> queries;
    for (size_t i = 0; i < numQueries; i++) {
        queries.emplace_back(StopId(stopDistribution(randomGenerator)),
            StopId(stopDistribution(randomGenerator)),
            timeDistribution(randomGenerator));
    }
    return queries;
}

inline std::vector<Loud::Vehicle> generateVehicles(
    const int numberOfEdges, const int numberOfVehicles = 50,
    const int timeInterval = TIMEINTERVAL, const int servTime = 20000,
    const int seed = 0)
{
    std::mt19937 randomGenerator(1 + seed);
    std::uniform_int_distribution<> stopDistribution(0, numberOfEdges - 1);
    std::vector<Loud::Vehicle> fleet;
    int loc, capacity, startServTime, endServTime;
    for (int i = 0; i < numberOfVehicles; i++) {
        loc = stopDistribution(randomGenerator);
        capacity = 4; // Number between 1 and 4
        startServTime = (std::max(timeInterval - servTime, 0) / (numberOfVehicles - 1)) * i; //(timeInterval / numberOfVehicles) * i;
        endServTime = startServTime + servTime;

        fleet.push_back({ i, loc, capacity, startServTime, endServTime });
    }

    return fleet;
}

inline std::vector<Loud::Request> generateRequests(
    const int numberOfEdges, const int numberOfRequests = 300,
    const int timeInterval = TIMEINTERVAL, const int seed = 0)
{
    assert(numberOfEdges > 2 * numberOfRequests);
    std::mt19937 randomGenerator(2 + seed);
    std::uniform_int_distribution<> stopDistribution(0, numberOfEdges - 1);
    std::uniform_int_distribution<> timeDistribution(0, timeInterval);
    std::vector<Loud::Request> requests;
    int pickup, dropoff, minDepTime;
    for (int i = 0; i < numberOfRequests; i++) {
        pickup = stopDistribution(randomGenerator);
        dropoff = stopDistribution(randomGenerator);
        minDepTime = timeDistribution(randomGenerator);

        requests.push_back({ pickup, dropoff, minDepTime });
    }

    return requests;
}

inline std::vector<Loud::Vehicle> readVehicleDataFromFile(
    const std::string vehicleFileName)
{
    std::vector<Loud::Vehicle> fleet;
    int loc, capacity, startServTime, endServTime;
    io::CSVReader<4, io::trim_chars<>> vehFileReader(vehicleFileName + ".vehicles.csv");
    vehFileReader.read_header(io::ignore_no_column, "initial_location",
        "seating_capacity", "start_service_time",
        "end_service_time");
    while (vehFileReader.read_row(loc, capacity, startServTime, endServTime)) {
        const int id = fleet.size();
        fleet.push_back({ id, loc, capacity, startServTime, endServTime });
    }

    return fleet;
}

inline void writeVehiclesToFile(std::vector<Loud::Vehicle> fleet,
    const std::string outputFileName)
{
    std::ofstream outputFile(outputFileName);
    if (!outputFile.good())
        throw std::invalid_argument("file cannot be opened -- '" + outputFileName + "'");
    outputFile << "initial_location,seating_capacity,start_service_time,end_"
                  "service_time\n";

    for (const auto vehicle : fleet) {
        outputFile << vehicle.initialLocation << ',' << vehicle.seatingCapacity
                   << ',' << vehicle.startServiceTime << ','
                   << vehicle.endServiceTime << '\n';
    }
}

class RunRideRaptor : public ParameterizedCommand {
public:
    using InputGraph = RIDERAPTOR::RoadGraph;
    using CHProvider = RIDERAPTOR::CHProvider;
    using Dispatcher = RIDERAPTOR::Dispatcher;

    RunRideRaptor(BasicShell& shell)
        : ParameterizedCommand(shell, "runRideRaptor", "Runs ride raptor")
    {
        addParameter("raptor network file", RAPTOR_FILENAME);
        addParameter("road network file", ROADNETWORK_FILENAME);
        addParameter("hierarchy file", ROADNETWORKCH_FILENAME);
        addParameter("walking CH file", WALKINGCH_FILENAME);
        addParameter("distance matrix file", DISTANCEMATRIX_FILENAME);
        addParameter("ridesharing file", RIDESHARING_FILENAME);

        // Needed for Raptor Query
        addParameter("Use Ride Filter", "true");
        addParameter("Use Distance Approximation", "true");
        addParameter("Use Distance Matrix", "true");
        addParameter("Source");
        addParameter("Target");
        addParameter("Departure time");
    }

    virtual void execute() noexcept
    {
        const std::string raptorFileName = getParameter("raptor network file");
        const std::string networkFileName = getParameter("road network file");
        const std::string hierarchyFileName = getParameter("hierarchy file");
        const std::string walkingCHFileName = getParameter("walking CH file");
        const std::string distanceMatrixFileName = getParameter("distance matrix file");
        const std::string ridesharingFileName = getParameter("ridesharing file");

        // Load Data
        std::cout << "Load graph data... \n"
                  << std::flush;
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(raptorFileName);

        InputGraph inputGraph(networkFileName);

        std::unique_ptr<CHProvider> chProvider;
        Loud::CH ch(hierarchyFileName);
        chProvider.reset(new CHProvider(inputGraph, std::move(ch)));

        ULTRACH::CH walkingCH(getParameter("walking CH file"));

        std::cout << "\nLoad ridesharing data... \n"
                  << std::flush;
        std::vector<Loud::Vehicle> fleet = readVehicleDataFromFile(ridesharingFileName);
        RIDERAPTOR::RideRaptorParameter parameter(ridesharingFileName);
        Dispatcher disp(inputGraph, fleet, *chProvider, parameter.stopTime,
            parameter.alpha, parameter.beta, parameter.maxIdleTime);

        RIDERAPTOR::DistanceMatrix distanceMatrix(distanceMatrixFileName);
        RIDERAPTOR::Data data(ridesharingFileName, raptorData, disp, walkingCH,
            parameter.maxWalkTime, distanceMatrix);

        std::cout << "\nExecute rideRaptor... \n"
                  << std::flush;
        run(data);
    }

private:
    inline void run(RIDERAPTOR::Data& data) const noexcept
    {
        RAPTOR::RideOptimizationFlags flags;
        flags.UseLeewayPruning = getParameter<bool>("Use Ride Filter");
        flags.UseTimePruning = getParameter<bool>("Use Ride Filter");
        flags.UseApproximation = getParameter<bool>("Use Distance Approximation");
        flags.UseDistanceMatrix = getParameter<bool>("Use Distance Matrix");

        RAPTOR::RideRAPTOR<true, RAPTOR::RideProfiler> rideRaptor(data, flags);
        const Edge source = getParameter<Edge>("Source");
        const Edge target = getParameter<Edge>("Target");
        const int departureTime = getParameter<int>("Departure time");

        rideRaptor.run(source, departureTime, target);
        for (const RIDERAPTOR::Journey& journey : rideRaptor.getJourneys()) {
            std::cout << journey << std::endl;
            std::cout << data.journeyToText(journey) << std::endl;
        }
    }
};

class RunPreprocessing : public ParameterizedCommand {
public:
    using InputGraph = RIDERAPTOR::RoadGraph;
    using CHProvider = RIDERAPTOR::CHProvider;
    using Dispatcher = RIDERAPTOR::Dispatcher;

    RunPreprocessing(BasicShell& shell)
        : ParameterizedCommand(shell, "runPreprocessing",
            "Runs the given requests with LOUD.")
    {
        addParameter("raptor network file", RAPTOR_FILENAME);
        addParameter("road network file", ROADNETWORK_FILENAME);
        addParameter("hierarchy file", ROADNETWORKCH_FILENAME);
        addParameter("output folder", "../Data/StuttgartCity/RideData");

        // model Parameters
        addParameter("stop time", "100");
        addParameter("alpha");
        addParameter("beta", "300");
        addParameter("max idle time", "300");
        addParameter("max walk time", "600");

        // Vehicle/Request
        addParameter("vehicle number");
        addParameter("inserted request number");
        addParameter("max request number");
        addParameter("time interval", "86400");
        addParameter("service time", "28800");
    }

    virtual void execute() noexcept
    {
        const std::string networkFileName = getParameter("road network file");
        const std::string raptorFileName = getParameter("raptor network file");
        const std::string hierarchyFileName = getParameter("hierarchy file");

        RIDERAPTOR::RideRaptorParameter parameter;

        parameter.stopTime = getParameter<int>("stop time");
        parameter.alpha = getParameter<double>("alpha");
        parameter.beta = getParameter<int>("beta");
        parameter.maxIdleTime = getParameter<int>("max idle time");
        parameter.maxWalkTime = getParameter<int>("max walk time");

        const auto numberOfVehicles = getParameter<int>("vehicle number");
        const auto numberOfRequests = getParameter<int>("max request number");
        const auto numberOfInsertedRequests = getParameter<int>("inserted request number");
        const auto timeInterval = getParameter<int>("time interval");
        const auto servTime = getParameter<int>("service time");
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(raptorFileName);
        InputGraph inputGraph(networkFileName);

        std::unique_ptr<CHProvider> chProvider;
        if (hierarchyFileName.empty()) {
            std::cout << "Building CH... " << std::flush;
            chProvider.reset(new CHProvider(inputGraph));
        } else {
            Loud::CH ch(hierarchyFileName);
            chProvider.reset(new CHProvider(inputGraph, std::move(ch)));
        }
        std::cout << "done.\n";

        std::cout << "\nGenerating Vehicles/Requests... " << std::flush;
        std::vector<Loud::Vehicle> fleet = generateVehicles(
            inputGraph.numEdges(), numberOfVehicles, timeInterval, servTime);
        std::vector<Loud::Request> requests = generateRequests(inputGraph.numEdges(), numberOfRequests, timeInterval);
        std::cout << "done.\n";

        Dispatcher disp(inputGraph, fleet, *chProvider, parameter.stopTime,
            parameter.alpha, parameter.beta, parameter.maxIdleTime);
        RIDERAPTOR::Data data(raptorData, disp);

        std::cout << "\nExecute Requests... " << std::flush;
        int counter = 0;
        int insertionCount = 0;

        for (const auto& req : requests) {
            auto ins = disp.findBestInsertionWithStopData(req);

            counter++;
            if (ins.sourceStop != 0 && ins.targetStop != 0) {
                insertionCount++;
            }

            if (counter % 50 == 0) {
                std::cout << "\nRequest No. " << counter << "(" << insertionCount << ")"
                          << std::flush;
            }

            if (insertionCount == numberOfInsertedRequests) {
                break;
            }
        }
        std::cout << "\ndone." << std::flush;

        std::cout << "\nBuild RideTransferGraph... \n"
                  << std::flush;
        data.buildRideTransferGraph();
        std::cout << "done.\n"
                  << std::flush;
        data.rideTransferGraph.printAnalysis();
        // data.print();

        std::cout << "\nWrite to output file... " << std::flush;
        const std::string identifier = "veh" + std::to_string(numberOfVehicles) + "_req" + std::to_string(insertionCount) + "_serv" + std::to_string(servTime) + "_alpha" + std::to_string(parameter.alpha).substr(0, 3);
        const std::string outputFileName = getParameter("output folder") + "/" + identifier + "/rideData";

        data.writeRideDataTo(outputFileName);
        writeVehiclesToFile(fleet, outputFileName + ".vehicles.csv");
        parameter.writeTo(outputFileName);

        std::vector<std::string> names = { "stopTime", "alpha", "beta", "idletime",
            "maxwalktime" };
        std::vector<double> values = {
            double(parameter.stopTime), parameter.alpha, double(parameter.beta),
            double(parameter.maxIdleTime), double(parameter.maxWalkTime)
        };
        data.getProfiler().writeToFile(
            getParameter("output folder") + "/" + identifier + ".csv", names,
            values);
        std::cout << "done.\n"
                  << std::flush;
    }
};

class RunRideRaptorQueries : public ParameterizedCommand {
public:
    using InputGraph = RIDERAPTOR::RoadGraph;
    using CHProvider = RIDERAPTOR::CHProvider;
    using Dispatcher = RIDERAPTOR::Dispatcher;

    RunRideRaptorQueries(BasicShell& shell)
        : ParameterizedCommand(shell, "runRideRaptorQueries",
            "Runs multiple ride raptor queries")
    {
        addParameter("raptor network file", RAPTOR_FILENAME);
        addParameter("road network file", ROADNETWORK_FILENAME);
        addParameter("hierarchy file", ROADNETWORKCH_FILENAME);
        addParameter("walking CH file", WALKINGCH_FILENAME);
        addParameter("distance matrix file", DISTANCEMATRIX_FILENAME);
        addParameter("ridesharing file", RIDESHARING_FILENAME);

        // Needed for Raptor Query
        addParameter("Use Time Pruning", "true");
        addParameter("Use Leeway Pruning", "true");
        addParameter("Use Distance Approximation", "true");
        addParameter("Use Distance Matrix", "true");
        addParameter("number of queries");
        addParameter("restrict to stops?");
    }

    virtual void execute() noexcept
    {
        const std::string networkFileName = getParameter("road network file");
        const std::string raptorFileName = getParameter("raptor network file");
        const std::string hierarchyFileName = getParameter("hierarchy file");
        const std::string walkingCHFileName = getParameter("walking CH file");
        const std::string ridesharingFileName = getParameter("ridesharing file");
        const std::string distanceMatrixFileName = getParameter("distance matrix file");

        // Load Data
        std::cout << "Load graph data... \n"
                  << std::flush;
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(raptorFileName);

        InputGraph inputGraph(networkFileName);

        std::unique_ptr<CHProvider> chProvider;
        Loud::CH ch(hierarchyFileName);
        chProvider.reset(new CHProvider(inputGraph, std::move(ch)));

        ULTRACH::CH walkingCH(getParameter("walking CH file"));

        std::cout << "\nLoad ridesharing data... \n"
                  << std::flush;
        std::vector<Loud::Vehicle> fleet = readVehicleDataFromFile(ridesharingFileName);
        RIDERAPTOR::RideRaptorParameter parameter(ridesharingFileName);
        Dispatcher disp(inputGraph, fleet, *chProvider, parameter.stopTime,
            parameter.alpha, parameter.beta, parameter.maxIdleTime);

        RIDERAPTOR::DistanceMatrix distanceMatrix(distanceMatrixFileName);
        RIDERAPTOR::Data data(ridesharingFileName, raptorData, disp, walkingCH,
            parameter.maxWalkTime, distanceMatrix);

        std::cout << "\nExecute rideRaptor Queries... \n"
                  << std::flush;
        run(data);
    }

private:
    inline void run(RIDERAPTOR::Data& data) const noexcept
    {
        RAPTOR::RideOptimizationFlags flags;
        flags.UseLeewayPruning = getParameter<bool>("Use Leeway Pruning");
        flags.UseTimePruning = getParameter<bool>("Use Time Pruning");
        flags.UseApproximation = getParameter<bool>("Use Distance Approximation");
        flags.UseDistanceMatrix = getParameter<bool>("Use Distance Matrix");

        RAPTOR::RideRAPTOR<true, RAPTOR::AggregateRideProfiler> algorithm(data,
            flags);

        const size_t n = getParameter<size_t>("number of queries");
        const bool restrictToStops = getParameter<bool>("restrict to stops?");
        const size_t locationRange = restrictToStops
            ? data.raptorData.numberOfStops()
            : data.disp.inputGraph.numEdges();
        const std::vector<RideStopQuery> queries = generateRandomRideStopQueries(locationRange, n, data.raptorData);

        double numJourneys = 0;
        double numRideJourneys = 0;
        double numInitalRideLegs = 0;
        double numFinalRideLegs = 0;
        for (const RideStopQuery& query : queries) {
            algorithm.run(query.source, query.departureTime, query.target);
            numJourneys += algorithm.getJourneys().size();
            for (const auto& journey : algorithm.getJourneys()) {
                numRideJourneys += usesRide(journey);
                numInitalRideLegs += usesInitialRide(journey);
                numFinalRideLegs += usesFinalRide(journey);
            }
        }
        algorithm.getProfiler().printStatistics();
        std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys / n)
                  << std::endl;
        std::cout << "Avg. ride journeys: "
                  << String::prettyDouble(numRideJourneys / n) << std::endl;
        std::cout << "Share of ride journeys: "
                  << String::prettyDouble((numRideJourneys / numJourneys) * 100)
                  << "%" << std::endl;

        std::vector<std::string> names = {
            "NumQueries", "Journeys", "RideJourneys", "InitialRides",
            "FinalRides", "InitialWalks", "FinalWalks"
        };
        std::vector<double> values = { double(queries.size()),
            numJourneys / n,
            numRideJourneys / n,
            numInitalRideLegs / n,
            numFinalRideLegs / n,
            (numJourneys - numInitalRideLegs) / n,
            (numJourneys - numFinalRideLegs) / n };

        algorithm.getProfiler().writeToFile(
            getParameter("ridesharing file") + ".queries" + std::to_string(n) + "_flags" + std::to_string(int(flags.UseTimePruning)) + std::to_string(int(flags.UseLeewayPruning)) + std::to_string(int(flags.UseApproximation)) + std::to_string(int(flags.UseDistanceMatrix)) + std::to_string(int(restrictToStops)) + ".csv",
            names, values);
    }
};

class BuildRideTransferGraph : public ParameterizedCommand {
public:
    using InputGraph = RIDERAPTOR::RoadGraph;
    using CHProvider = RIDERAPTOR::CHProvider;
    using Dispatcher = RIDERAPTOR::Dispatcher;

    BuildRideTransferGraph(BasicShell& shell)
        : ParameterizedCommand(shell, "buildRideTransferGraph",
            "Builds ride transfer graph")
    {
        addParameter("raptor network file", RAPTOR_FILENAME);
        addParameter("road network file", ROADNETWORK_FILENAME);
        addParameter("hierarchy file", ROADNETWORKCH_FILENAME);
        addParameter("walking CH file", WALKINGCH_FILENAME);
        addParameter("ridesharing file");
    }

    virtual void execute() noexcept
    {
        const std::string networkFileName = getParameter("road network file");
        const std::string raptorFileName = getParameter("raptor network file");
        const std::string hierarchyFileName = getParameter("hierarchy file");
        const std::string walkingCHFileName = getParameter("walking CH file");
        const std::string ridesharingFileName = getParameter("ridesharing file");

        // Load Data
        std::cout << "Load graph data... \n"
                  << std::flush;
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(raptorFileName);

        InputGraph inputGraph(networkFileName);

        std::unique_ptr<CHProvider> chProvider;
        Loud::CH ch(hierarchyFileName);
        chProvider.reset(new CHProvider(inputGraph, std::move(ch)));

        ULTRACH::CH walkingCH(getParameter("walking CH file"));

        std::cout << "\nLoad ridesharing data... \n"
                  << std::flush;
        std::vector<Loud::Vehicle> fleet = readVehicleDataFromFile(ridesharingFileName);
        RIDERAPTOR::RideRaptorParameter parameter(ridesharingFileName);
        Dispatcher disp(inputGraph, fleet, *chProvider, parameter.stopTime,
            parameter.alpha, parameter.beta, parameter.maxIdleTime);

        RIDERAPTOR::Data data(ridesharingFileName, raptorData, disp, walkingCH,
            parameter.maxWalkTime);

        std::cout << "Before: " << data.rideTransferGraph.numVertices() << ","
                  << data.rideTransferGraph.numEdges() << std::endl;

        std::cout << "\nBuild RideTransferGraph... \n"
                  << std::flush;
        data.buildRideTransferGraph();
        std::cout << "done.\n"
                  << std::flush;

        // data.print();

        std::cout << "\nWrite to output file... " << std::flush;
        data.writeRideDataTo(ridesharingFileName);

        std::vector<std::string> names = { "stopTime", "alpha", "beta", "idletime",
            "maxwalktime" };
        std::vector<double> values = {
            double(parameter.stopTime), parameter.alpha, double(parameter.beta),
            double(parameter.maxIdleTime), double(parameter.maxWalkTime)
        };
        data.getProfiler().writeToFile(ridesharingFileName + ".preprocessing.csv",
            names, values);
        std::cout << "done.\n"
                  << std::flush;
    }
};

class BuildDistanceMatrix : public ParameterizedCommand {
public:
    BuildDistanceMatrix(BasicShell& shell)
        : ParameterizedCommand(shell, "buildDistanceMatrix",
            "Builds matrix with distances")
    {
        addParameter("raptor network file");
        addParameter("ch file");
        addParameter("output file");
    }

    virtual void execute() noexcept
    {
        const std::string raptorFileName = getParameter("raptor network file");
        const std::string hierarchyFileName = getParameter("ch file");
        const std::string outputFileName = getParameter("output file");

        std::cout << "Load graph data... \n"
                  << std::flush;
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(raptorFileName);

        raptorData.printInfo();

        ULTRACH::CH ch(hierarchyFileName);

        ULTRATimer totalTimer;
        totalTimer.restart();
        RIDERAPTOR::DistanceMatrix matrix(raptorData.numberOfStops());
        RIDERAPTOR::fillDistanceMatrix(matrix, raptorData, ch);
        const auto totalTime = totalTimer.elapsedMicroseconds();
        std::cout << "Total time: " << String::musToString(totalTime) << std::endl;
        matrix.writeTo(outputFileName);
    }
};

class BuildLoudCH : public ParameterizedCommand {
public:
    using InputGraph = GraphWrapper<LoudGraph>;

    BuildLoudCH(BasicShell& shell)
        : ParameterizedCommand(shell, "buildLoudCH",
            "Generates CH and stores in File")
    {
        addParameter("network file");
        addParameter("output ch file");
    }

    virtual void execute() noexcept
    {
        const auto networkFileName = getParameter("network file");
        const auto outputCHFileName = getParameter("output ch file");

        TransferGraph network(networkFileName);
        InputGraph inputGraph(network);

        Loud::CH ch;
        ch.preprocess<ImplementationDetail::TravelTimeType>(inputGraph);
        ch.writeTo(outputCHFileName);
    }
};

class ExtendGraph : public ParameterizedCommand {
public:
    using InputGraph = RIDERAPTOR::RoadGraph;

    ExtendGraph(BasicShell& shell)
        : ParameterizedCommand(shell, "extendGraph",
            "Extends the Graph with additional Stations")
    {
        addParameter("raptor file");
        addParameter("output file");
        addParameter("RoadGraph?");
    }

    virtual void execute() noexcept
    {
        const auto raptorFileName = getParameter("raptor file");
        const auto outputFileName = getParameter("output file");

        const auto writeToRoadGraph = getParameter<bool>("RoadGraph?");

        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(raptorFileName);

        TransferGraph networkExtended = RIDERAPTOR::getOverheadGraph(raptorData, raptorData.transferGraph);

        if (writeToRoadGraph) {
            InputGraph inputGraph(networkExtended);
            inputGraph.writeBinary(outputFileName);
        } else {
            raptorData.transferGraph = networkExtended;
            raptorData.serialize(outputFileName);
        }
    }
};

class RunMultiplePreprocessing : public ParameterizedCommand {
public:
    using InputGraph = RIDERAPTOR::RoadGraph;
    using CHProvider = RIDERAPTOR::CHProvider;
    using Dispatcher = RIDERAPTOR::Dispatcher;

    RunMultiplePreprocessing(BasicShell& shell)
        : ParameterizedCommand(
            shell, "runMultiplePreprocessings",
            "Runs multiple different preprocessing and writes to csv")
    {
        addParameter("raptor network file", RAPTOR_FILENAME);
        addParameter("road network file", ROADNETWORK_FILENAME);
        addParameter("hierarchy file", ROADNETWORKCH_FILENAME);
        addParameter("output file");

        // model Parameters
        addParameter("stop time", "100");
        addParameter("alpha");
        addParameter("beta", "300");
        addParameter("max idle time", "300");
        addParameter("max walk time", "600");

        // Vehicle/Request
        addParameter("iterations");
        addParameter("vehicle number");
        addParameter("route length interval");
        addParameter("max request number");
        addParameter("time interval", "86400");
        addParameter("service time", "28800");
    }

    virtual void execute() noexcept
    {
        const std::string networkFileName = getParameter("road network file");
        const std::string raptorFileName = getParameter("raptor network file");
        const std::string hierarchyFileName = getParameter("hierarchy file");
        const std::string outputFileName = getParameter("output file");

        RIDERAPTOR::RideRaptorParameter parameter;

        parameter.stopTime = getParameter<int>("stop time");
        parameter.alpha = getParameter<double>("alpha");
        parameter.beta = getParameter<int>("beta");
        parameter.maxIdleTime = getParameter<int>("max idle time");
        parameter.maxWalkTime = getParameter<int>("max walk time");

        const auto numberOfVehicles = getParameter<int>("vehicle number");
        const auto numberOfRequests = getParameter<int>("max request number");
        const auto numIterations = getParameter<int>("iterations");
        const auto routeLengthInterval = getParameter<double>("route length interval");
        const auto insertedRequestInterval = int((numberOfVehicles * routeLengthInterval) / 2);
        const auto timeInterval = getParameter<int>("time interval");
        const auto servTime = getParameter<int>("service time");
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(raptorFileName);
        InputGraph inputGraph(networkFileName);

        std::unique_ptr<CHProvider> chProvider;
        if (hierarchyFileName.empty()) {
            std::cout << "Building CH... " << std::flush;
            chProvider.reset(new CHProvider(inputGraph));
        } else {
            Loud::CH ch(hierarchyFileName);
            chProvider.reset(new CHProvider(inputGraph, std::move(ch)));
        }
        std::cout << "done.\n";

        for (int i = 0; i < numIterations; i++) {
            std::cout << "\nGenerating Vehicles/Requests... " << std::flush;
            std::vector<Loud::Vehicle> fleet = generateVehicles(
                inputGraph.numEdges(), numberOfVehicles, timeInterval, servTime, i);
            std::vector<Loud::Request> requests = generateRequests(
                inputGraph.numEdges(), numberOfRequests, timeInterval, i);
            std::cout << "done.\n";

            Dispatcher disp(inputGraph, fleet, *chProvider, parameter.stopTime,
                parameter.alpha, parameter.beta, parameter.maxIdleTime);

            std::ofstream outputFile(outputFileName + "_veh" + std::to_string(numberOfVehicles) + "_alpha" + std::to_string(parameter.alpha).substr(0, 3) + "_" + std::to_string(i) + ".csv");

            std::cout << "\nExecute Requests... " << std::flush;
            int counter = 0;
            bool buildRideTransfer = false;
            int insertionCount = 0;

            buildRideTransferGraph(raptorData, disp, outputFile, parameter, 0, true);

            for (const auto& req : requests) {
                auto ins = disp.findBestInsertionWithStopData(req);

                counter++;
                if (ins.sourceStop != 0 && ins.targetStop != 0) {
                    insertionCount++;
                    buildRideTransfer = true;
                }

                if (counter % 50 == 0) {
                    std::cout << "\nRequest No. " << counter << "(" << insertionCount
                              << ")" << std::flush;
                }

                if (buildRideTransfer && (insertionCount % insertedRequestInterval == 0)) {
                    buildRideTransferGraph(raptorData, disp, outputFile, parameter,
                        insertionCount, false);
                    buildRideTransfer = false;
                }
            }
            buildRideTransferGraph(raptorData, disp, outputFile, parameter,
                insertionCount);
        }

        std::cout << "\ndone." << std::flush;
    }

    inline void buildRideTransferGraph(RAPTOR::Data& raptorData, Dispatcher& disp,
        std::ofstream& outputFile,
        RIDERAPTOR::RideRaptorParameter parameter,
        const int numberOfRequests,
        const bool writeHeader = false)
    {
        // RIDERAPTOR::Data data(raptorData, disp);
        // data.buildRideTransferGraph();

        if (writeHeader) {
            outputFile << "route length,average leeway, max leeway\n";
            // std::vector<std::string> names = {"requests","stopTime", "alpha",
            // "beta", "idletime", "maxwalktime"};
            // data.getProfiler().writeHeaderToFile(outputFile, names);
        }
        outputFile << double(numberOfRequests * 2 + disp.fleet.size()) / double(disp.fleet.size())
                   << ",";
        disp.computeLeeways(outputFile);
        // std::vector<double> values =
        // {double(numberOfRequests),double(parameter.stopTime), parameter.alpha,
        // double(parameter.beta), double(parameter.maxIdleTime),
        // double(parameter.maxWalkTime)};
        // data.getProfiler().writeValuesToFile(outputFile, values);
    }
};

class RunQueriesOnMultiplePreprocessings : public ParameterizedCommand {
public:
    using InputGraph = RIDERAPTOR::RoadGraph;
    using CHProvider = RIDERAPTOR::CHProvider;
    using Dispatcher = RIDERAPTOR::Dispatcher;

    RunQueriesOnMultiplePreprocessings(BasicShell& shell)
        : ParameterizedCommand(
            shell, "runQueriesOnMultiplePreprocessing",
            "Runs queries on different preprocessings and writes to csv")
    {
        addParameter("raptor network file", RAPTOR_FILENAME);
        addParameter("road network file", ROADNETWORK_FILENAME);
        addParameter("hierarchy file", ROADNETWORKCH_FILENAME);
        addParameter("walking CH file", WALKINGCH_FILENAME);
        addParameter("distance matrix file", DISTANCEMATRIX_FILENAME);
        addParameter("output file");

        // model Parameters
        addParameter("stop time", "100");
        addParameter("alpha");
        addParameter("beta", "300");
        addParameter("max idle time", "300");
        addParameter("max walk time", "600");
        addParameter("time interval", "86400");
        addParameter("service time", "28800");

        // Preprocessing
        addParameter("vehicle number");
        addParameter("route length interval");
        addParameter("max request number");

        // Queries
        addParameter("Use Time Pruning", "true");
        addParameter("Use Leeway Pruning", "true");
        addParameter("Use Distance Approximation", "true");
        addParameter("Use Distance Matrix", "true");
        addParameter("number of queries");
        addParameter("restrict to stops?");
    }

    virtual void execute() noexcept
    {
        const std::string networkFileName = getParameter("road network file");
        const std::string raptorFileName = getParameter("raptor network file");
        const std::string hierarchyFileName = getParameter("hierarchy file");
        const std::string walkingCHFileName = getParameter("walking CH file");
        const std::string distanceMatrixFileName = getParameter("distance matrix file");
        const std::string outputFileName = getParameter("output file");

        RIDERAPTOR::RideRaptorParameter parameter;
        parameter.stopTime = getParameter<int>("stop time");
        parameter.alpha = getParameter<double>("alpha");
        parameter.beta = getParameter<int>("beta");
        parameter.maxIdleTime = getParameter<int>("max idle time");
        parameter.maxWalkTime = getParameter<int>("max walk time");

        RAPTOR::RideOptimizationFlags flags;
        flags.UseLeewayPruning = getParameter<bool>("Use Leeway Pruning");
        flags.UseTimePruning = getParameter<bool>("Use Time Pruning");
        flags.UseApproximation = getParameter<bool>("Use Distance Approximation");
        flags.UseDistanceMatrix = getParameter<bool>("Use Distance Matrix");

        const auto numberOfVehicles = getParameter<int>("vehicle number");
        const auto numberOfRequests = getParameter<int>("max request number");
        const auto routeLengthInterval = getParameter<double>("route length interval");
        const auto insertedRequestInterval = int((numberOfVehicles * routeLengthInterval) / 2);
        const auto timeInterval = getParameter<int>("time interval");
        const auto servTime = getParameter<int>("service time");
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(raptorFileName);
        InputGraph inputGraph(networkFileName);

        std::unique_ptr<CHProvider> chProvider;
        if (hierarchyFileName.empty()) {
            std::cout << "Building CH... " << std::flush;
            chProvider.reset(new CHProvider(inputGraph));
        } else {
            Loud::CH ch(hierarchyFileName);
            chProvider.reset(new CHProvider(inputGraph, std::move(ch)));
        }

        ULTRACH::CH walkingCH(getParameter("walking CH file"));
        std::cout << "done.\n";

        std::cout << "\nGenerating Vehicles/Requests/Queries... " << std::flush;
        std::vector<Loud::Vehicle> fleet = generateVehicles(
            inputGraph.numEdges(), numberOfVehicles, timeInterval, servTime);
        std::vector<Loud::Request> requests = generateRequests(inputGraph.numEdges(), numberOfRequests, timeInterval);
        const size_t n = getParameter<size_t>("number of queries");
        const bool restrictToStops = getParameter<bool>("restrict to stops?");
        const size_t locationRange = restrictToStops ? raptorData.numberOfStops() : inputGraph.numEdges();
        const std::vector<RideStopQuery> queries = generateRandomRideStopQueries(locationRange, n);
        std::cout << "done.\n";

        Dispatcher disp(inputGraph, fleet, *chProvider, parameter.stopTime,
            parameter.alpha, parameter.beta, parameter.maxIdleTime);
        RIDERAPTOR::DistanceMatrix distanceMatrix(distanceMatrixFileName);
        RIDERAPTOR::Data data(raptorData, disp, walkingCH, parameter.maxWalkTime,
            distanceMatrix);

        std::ofstream outputFile(
            outputFileName + "_veh" + std::to_string(numberOfVehicles) + "_alpha" + std::to_string(parameter.alpha).substr(0, 3) + "_flags" + std::to_string(int(flags.UseTimePruning)) + std::to_string(int(flags.UseLeewayPruning)) + std::to_string(int(flags.UseApproximation)) + std::to_string(int(flags.UseDistanceMatrix)) + std::to_string(int(restrictToStops)) + ".csv");

        std::cout << "\nExecute Requests... " << std::flush;
        int counter = 0;
        bool buildRideTransfer = false;
        int insertionCount = 0;

        runQueries(data, outputFile, flags, insertionCount, queries, true);

        for (const auto& req : requests) {
            auto ins = disp.findBestInsertionWithStopData(req);

            counter++;
            if (ins.sourceStop != 0 && ins.targetStop != 0) {
                insertionCount++;
                buildRideTransfer = true;
            }

            if (counter % 50 == 0) {
                std::cout << "\nRequest No. " << counter << "(" << insertionCount << ")"
                          << std::flush;
            }

            if (buildRideTransfer && (insertionCount % insertedRequestInterval == 0)) {
                runQueries(data, outputFile, flags, insertionCount, queries, false);
                buildRideTransfer = false;
            }
        }
        runQueries(data, outputFile, flags, insertionCount, queries);
        std::cout << "\ndone." << std::flush;
    }

    inline void runQueries(RIDERAPTOR::Data& data, std::ofstream& outputFile,
        RAPTOR::RideOptimizationFlags& flags,
        const int numberOfRequests,
        const std::vector<RideStopQuery>& queries,
        const bool writeHeader = false)
    {
        data.buildRideTransferGraph();

        RAPTOR::RideRAPTOR<true, RAPTOR::AggregateRideProfiler> algorithm(data,
            flags);

        double numJourneys = 0;
        double numRideJourneys = 0;
        for (const RideStopQuery& query : queries) {
            algorithm.run(query.source, query.departureTime, query.target);
            numJourneys += algorithm.getJourneys().size();
            for (const auto& journey : algorithm.getJourneys()) {
                numRideJourneys += usesRide(journey);
            }
        }

        if (writeHeader) {
            std::vector<std::string> names = {
                "Vehicles", "VehicleStops", "NumQueries", "Journeys", "RideJourneys"
            };
            algorithm.getProfiler().writeHeaderToFile(outputFile, names);
        }

        std::vector<double> values = {
            double(data.disp.fleet.size()),
            double(data.disp.fleet.size() + 2 * numberOfRequests),
            double(queries.size()), numJourneys / double(queries.size()),
            numRideJourneys / double(queries.size())
        };
        algorithm.getProfiler().writeValuesToFile(outputFile, values);
    }
};

class RunQueriesForEarliestArrival : public ParameterizedCommand {
public:
    using InputGraph = RIDERAPTOR::RoadGraph;
    using CHProvider = RIDERAPTOR::CHProvider;
    using Dispatcher = RIDERAPTOR::Dispatcher;

    RunQueriesForEarliestArrival(BasicShell& shell)
        : ParameterizedCommand(
            shell, "runQueriesForEarliestArrival",
            "Runs multiple queries and writes earliest arrivals to File")
    {
        addParameter("raptor network file", RAPTOR_FILENAME);
        addParameter("road network file", ROADNETWORK_FILENAME);
        addParameter("hierarchy file", ROADNETWORKCH_FILENAME);
        addParameter("walking CH file", WALKINGCH_FILENAME);
        addParameter("distance matrix file", DISTANCEMATRIX_FILENAME);
        addParameter("ridesharing file", RIDESHARING_FILENAME);
        addParameter("output file");
        addParameter("id");

        // Needed for Raptor Query
        addParameter("number of queries");
        addParameter("time interval");
        addParameter("restrict to stops?", "true");
    }

    virtual void execute() noexcept
    {
        const std::string networkFileName = getParameter("road network file");
        const std::string raptorFileName = getParameter("raptor network file");
        const std::string hierarchyFileName = getParameter("hierarchy file");
        const std::string walkingCHFileName = getParameter("walking CH file");
        const std::string ridesharingFileName = getParameter("ridesharing file");
        const std::string distanceMatrixFileName = getParameter("distance matrix file");

        // Load Data
        std::cout << "Load graph data... \n"
                  << std::flush;
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(raptorFileName);

        InputGraph inputGraph(networkFileName);

        std::unique_ptr<CHProvider> chProvider;
        Loud::CH ch(hierarchyFileName);
        chProvider.reset(new CHProvider(inputGraph, std::move(ch)));

        ULTRACH::CH walkingCH(getParameter("walking CH file"));

        std::cout << "\nLoad ridesharing data... \n"
                  << std::flush;
        std::vector<Loud::Vehicle> fleet = readVehicleDataFromFile(ridesharingFileName);
        RIDERAPTOR::RideRaptorParameter parameter(ridesharingFileName);
        Dispatcher disp(inputGraph, fleet, *chProvider, parameter.stopTime,
            parameter.alpha, parameter.beta, parameter.maxIdleTime);

        RIDERAPTOR::DistanceMatrix distanceMatrix(distanceMatrixFileName);
        RIDERAPTOR::Data data(ridesharingFileName, raptorData, disp, walkingCH,
            parameter.maxWalkTime, distanceMatrix);

        std::cout << "\nExecute rideRaptor Queries... \n"
                  << std::flush;
        run(data);
    }

private:
    inline void run(RIDERAPTOR::Data& data) const noexcept
    {
        RAPTOR::RideOptimizationFlags flags;
        flags.UseLeewayPruning = true;
        flags.UseTimePruning = true;
        flags.UseApproximation = true;
        flags.UseDistanceMatrix = true;

        RAPTOR::RideRAPTOR<true, RAPTOR::AggregateRideProfiler> algorithm(data,
            flags);
        RAPTOR::RAPTOR<true, RAPTOR::AggregateProfiler, true, false> raptor(
            data.raptorData);

        size_t n = getParameter<size_t>("number of queries");
        const bool restrictToStops = getParameter<bool>("restrict to stops?");
        const size_t locationRange = restrictToStops
            ? data.raptorData.numberOfStops()
            : data.disp.inputGraph.numEdges();
        const std::vector<RideStopQuery> queries = generateRandomRideStopQueries(locationRange, n);

        std::ofstream outputFile(getParameter("output file"));
        outputFile << "id,time,journey time,journeys,ride journeys\n";

        const int interval = getParameter<int>("time interval");

        for (int depTime = 0; depTime <= 86400; depTime += interval) {
            double numJourneys = 0;
            double numRaptorJourneys = 0;
            double numRideJourneys = 0;
            double rideRaptorTripTime = 0;
            double raptorTripTime = 0;
            for (const RideStopQuery& query : queries) {
                algorithm.run(query.source, depTime, query.target);
                raptor.run(StopId(query.source), depTime, StopId(query.target));
                numJourneys += algorithm.getJourneys().size();
                numRaptorJourneys += raptor.getJourneys().size();
                numRideJourneys += usesRide(algorithm.getEarliestJourney(StopId(query.target)));

                if (raptor.getJourneys().size() == 0) {
                    continue;
                }

                rideRaptorTripTime += (algorithm.getEarliestArrivalTime(StopId(query.target)) - depTime);
                raptorTripTime += (raptor.getEarliestArrivalTime(StopId(query.target)) - depTime);
            }

            outputFile << getParameter("id") << "," << depTime << ","
                       << rideRaptorTripTime / n << "," << numJourneys / n << ","
                       << numRideJourneys / n << '\n';
            // outputFile << "raptor" << "," << depTime << "," << (raptorTripTime / n)
            // << "," << numRaptorJourneys / n << '\n';
        }
    }
};

class RunRAPTORQueries : public ParameterizedCommand {
public:
    RunRAPTORQueries(BasicShell& shell)
        : ParameterizedCommand(shell, "runRAPTORQueries",
            "Runs the given number of random transitive "
            "RAPTOR queries and writes to csv")
    {
        addParameter("RAPTOR input file");
        addParameter("Number of queries");
        addParameter("output file");
    }

    virtual void execute() noexcept
    {
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
        raptorData.printInfo();
        RAPTOR::RAPTOR<true, RAPTOR::AggregateRideProfiler, false, false> algorithm(
            raptorData);

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<StopQuery> queries = generateRandomStopQueries(raptorData.numberOfStops(), n);

        double numJourneys = 0;
        for (const StopQuery& query : queries) {
            algorithm.run(query.source, query.departureTime, query.target);
            numJourneys += algorithm.getJourneys().size();
        }
        algorithm.getProfiler().printStatistics();
        std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys / n)
                  << std::endl;

        std::vector<std::string> names = { "NumQueries", "Journeys", "RideJourneys" };
        std::vector<double> values = { double(queries.size()), numJourneys / n, 0 };
        algorithm.getProfiler().writeToFile(getParameter("output file"), names,
            values);
    }
};
