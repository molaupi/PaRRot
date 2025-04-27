#include <iostream>
#include <random>
#include <string>
#include <vector>

#include "../../include/ULTRA/Algorithms/CH/CH.h"
#include "../../include/ULTRA/Algorithms/CH/Preprocessing/BidirectionalWitnessSearch.h"
#include "../../include/ULTRA/Algorithms/CH/Preprocessing/CHBuilder.h"
#include "../../include/ULTRA/DataStructures/CSA/Data.h"
#include "../../include/ULTRA/DataStructures/Intermediate/Data.h"
#include "../../include/ULTRA/DataStructures/RAPTOR/Data.h"
#include "../../include/ULTRA/Helpers/MultiThreading.h"
#include "../../include/ULTRA/Shell/Shell.h"
using namespace Shell;

inline constexpr int ShortcutWeight = 1024;
inline constexpr int DegreeWeight = 0;
inline constexpr int UnidirectionalPopLimit = 500;
inline constexpr int BidirectionalPopLimit = 200;

template <typename PROFILER>
using UnidirectionalWitnessSearch = ULTRACH::WitnessSearch<CHCoreGraph, PROFILER, UnidirectionalPopLimit>;
template <typename PROFILER>
using BidirectionalWitnessSearch = ULTRACH::BidirectionalWitnessSearch<CHCoreGraph, PROFILER,
    BidirectionalPopLimit>;

template <typename WITNESS_SEARCH>
using GreedyKey = ULTRACH::GreedyKey<WITNESS_SEARCH>;
template <typename WITNESS_SEARCH>
using PartialKey = ULTRACH::PartialKey<WITNESS_SEARCH, GreedyKey<WITNESS_SEARCH>>;
using StopCriterion = ULTRACH::NoStopCriterion;

template <typename CH_BUILDER>
inline ULTRACH::CH finalizeCH(CH_BUILDER&& chBuilder,
    const std::string& orderOutputFile,
    const std::string& chOutputFile) noexcept
{
    chBuilder.copyCoreToCH();
    Order order;
    for (const Vertex vertex : chBuilder.getOrder()) {
        order.emplace_back(vertex);
    }
    order.serialize(orderOutputFile);
    std::cout << "Obtaining CH" << std::endl;
    ULTRACH::CH ch(std::move(chBuilder));
    ch.writeBinary(chOutputFile);
    std::cout << std::endl;
    return ch;
}

template <typename PROFILER, typename WITNESS_SEARCH, typename GRAPH,
    typename KEY_FUNCTION, typename STOP_CRITERION = StopCriterion>
inline ULTRACH::CH buildCH(
    GRAPH& originalGraph, const std::string& orderOutputFile,
    const std::string& chOutputFile, const KEY_FUNCTION& keyFunction,
    const STOP_CRITERION& stopCriterion = StopCriterion()) noexcept
{
    TravelTimeGraph graph;
    ULTRAGraph::copy(originalGraph, graph);
    ULTRAGraph::printInfo(graph);
    ULTRACH::Builder<PROFILER, WITNESS_SEARCH, KEY_FUNCTION, STOP_CRITERION, false,
        false>
        chBuilder(std::move(graph), graph[TravelTime], keyFunction,
            stopCriterion);
    chBuilder.run();
    return finalizeCH(chBuilder, orderOutputFile, chOutputFile);
}

class BuildCH : public ParameterizedCommand {
public:
    BuildCH(BasicShell& shell)
        : ParameterizedCommand(
            shell, "buildCH",
            "Computes a CH with greedy key for the input graph.")
    {
        addParameter("Graph binary");
        addParameter("Order output file");
        addParameter("CH output file");
        addParameter("Use full profiler?", "true");
        addParameter("Witness search type", "bidirectional",
            { "normal", "bidirectional" });
        addParameter("Level weight", "256");
    }

    virtual void execute() noexcept
    {
        if (getParameter<bool>("Use full profiler?")) {
            chooseWitnessSearch<ULTRACH::FullProfiler>();
        } else {
            chooseWitnessSearch<ULTRACH::TimeProfiler>();
        }
    }

private:
    template <typename PROFILER>
    inline void chooseWitnessSearch() const noexcept
    {
        if (getParameter("Witness search type") == "normal") {
            build<PROFILER, UnidirectionalWitnessSearch<PROFILER>>();
        } else {
            build<PROFILER, BidirectionalWitnessSearch<PROFILER>>();
        }
    }

    template <typename PROFILER, typename WITNESS_SEARCH>
    inline void build() const noexcept
    {
        TransferGraph graph(getParameter("Graph binary"));
        GreedyKey<WITNESS_SEARCH> keyFunction(
            ShortcutWeight, getParameter<int>("Level weight"), DegreeWeight);
        buildCH<PROFILER, WITNESS_SEARCH>(graph, getParameter("Order output file"),
            getParameter("CH output file"),
            keyFunction);
    }
};

class BuildCoreCH : public ParameterizedCommand {
public:
    BuildCoreCH(BasicShell& shell)
        : ParameterizedCommand(shell, "buildCoreCH",
            "Computes a core-CH for the input network, where "
            "all stops are kept uncontracted.")
    {
        addParameter("Network input file");
        addParameter("Order output file");
        addParameter("CH output file");
        addParameter("Network output file");
        addParameter("Max core degree", "14");
        addParameter("Network type", "raptor", { "intermediate", "csa", "raptor" });
        addParameter("Use full profiler?", "true");
        addParameter("Witness search type", "bidirectional",
            { "normal", "bidirectional" });
        addParameter("Level weight", "256");
    }

    virtual void execute() noexcept
    {
        if (getParameter<bool>("Use full profiler?")) {
            return chooseWitnessSearch<ULTRACH::FullProfiler>();
        } else {
            return chooseWitnessSearch<ULTRACH::TimeProfiler>();
        }
    }

private:
    template <typename PROFILER>
    inline void chooseWitnessSearch() noexcept
    {
        const std::string witnessSearchType = getParameter("Witness search type");
        if (witnessSearchType == "normal") {
            chooseNetworkType<PROFILER, UnidirectionalWitnessSearch<PROFILER>>();
        } else {
            chooseNetworkType<PROFILER, BidirectionalWitnessSearch<PROFILER>>();
        }
    }

    template <typename PROFILER, typename WITNESS_SEARCH>
    inline void chooseNetworkType() noexcept
    {
        const std::string networkType = getParameter("Network type");
        if (networkType == "raptor") {
            build<RAPTOR::Data, PROFILER, WITNESS_SEARCH>();
        } else if (networkType == "csa") {
            build<CSA::Data, PROFILER, WITNESS_SEARCH>();
        } else {
            build<Intermediate::Data, PROFILER, WITNESS_SEARCH>();
        }
    }

    template <typename NETWORK_TYPE, typename PROFILER, typename WITNESS_SEARCH>
    inline void build() const noexcept
    {
        NETWORK_TYPE data(getParameter("Network input file"));
        data.printInfo();

        std::vector<bool> contractable(data.numberOfStops(), false);
        contractable.resize(data.transferGraph.numVertices(), true);

        const double maxCoreDegree = getParameter<double>("Max core degree");
        std::cout << "Min. core size: " << String::prettyInt(data.numberOfStops())
                  << std::endl;
        std::cout << "Max. core degree: " << String::prettyInt(maxCoreDegree)
                  << std::endl;
        GreedyKey<WITNESS_SEARCH> greedyKey(
            ShortcutWeight, getParameter<int>("Level weight"), DegreeWeight);
        PartialKey<WITNESS_SEARCH> keyFunction(
            contractable, data.transferGraph.numVertices(), greedyKey);
        ULTRACH::CoreCriterion stopCriterion(data.numberOfStops(), maxCoreDegree);
        const ULTRACH::CH ch = buildCH<PROFILER, WITNESS_SEARCH>(
            data.transferGraph, getParameter("Order output file"),
            getParameter("CH output file"), keyFunction, stopCriterion);

        Intermediate::TransferGraph resultGraph;
        resultGraph.addVertices(data.transferGraph.numVertices());
        resultGraph[Coordinates] = data.transferGraph[Coordinates];
        for (const Vertex vertex : resultGraph.vertices()) {
            if (ch.isCoreVertex(vertex)) {
                for (const Edge edge : ch.forward.edgesFrom(vertex)) {
                    resultGraph.addEdge(vertex, ch.forward.get(ToVertex, edge))
                        .set(TravelTime, ch.forward.get(Weight, edge));
                }
            }
        }
        ULTRAGraph::move(std::move(resultGraph), data.transferGraph);
        data.serialize(getParameter("Network output file"));
    }
};
