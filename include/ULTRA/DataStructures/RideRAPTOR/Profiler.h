#pragma once

#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "../../Helpers/Timer.h"
#include "Data.h"
namespace RIDERAPTOR {

inline constexpr static int MetricWidth = 13;
inline constexpr static int TimeWidth = 15;

typedef enum {
    PHASE_BUILDTRANSFERGRAPH,
    PHASE_BCHSEARCHES,
    PHASE_SORT_EDGES,
    NUM_PHASES
} Phase;

constexpr const char* PhaseNames[] = {
    "Build_Graph",
    "BCH_Searches",
    "Sort_Edges",
};

typedef enum {
    METRIC_NEARBYVEHICLEROUTES,
    METRIC_VEHICLES,
    METRIC_STATIONS,
    METRIC_BCHSEARCHES,
    METRIC_VEHICLESTOPS,
    METRIC_VERTICES,
    METRIC_EDGES,
    METRIC_PICKUPEDGES,
    METRIC_DROPOFFEDGES,
    METRIC_NUMBER_OF_INSERTIONS,
    METRIC_MAX_STATIONS_PER_VEHICLE,
    NUM_METRICS
} Metric;

constexpr const char* MetricNames[] = {
    "Nearby_VehicleRoutes",
    "Vehicles",
    "Stations",
    "BCH_Number",
    "Vehicle_Stops",
    "Vertices",
    "Edges",
    "Pickup_Edges",
    "Dropoff_Edges",
    "Number_Insertions",
    "Max_Stations_Vehicle",
};

struct TimeData {
    TimeData()
        : metricValue(NUM_METRICS, 0)
        , phaseTime(NUM_PHASES, 0.0)
        , totalTime(0.0)
    {
    }

    inline TimeData& operator+=(const TimeData& other) noexcept
    {
        for (size_t metric = 0; metric < NUM_METRICS; metric++) {
            metricValue[metric] += other.metricValue[metric];
        }
        for (size_t phase = 0; phase < NUM_PHASES; phase++) {
            phaseTime[phase] += other.phaseTime[phase];
        }
        totalTime += other.totalTime;
        return *this;
    }

    inline TimeData& operator/=(size_t numQueries) noexcept
    {
        for (long long& metric : metricValue) {
            metric /= numQueries;
        }
        for (double& phase : phaseTime) {
            phase /= numQueries;
        }
        totalTime /= numQueries;
        return *this;
    }

    inline static int metricLength(const Metric metric) noexcept
    {
        return std::max((int)std::string(MetricNames[metric]).length() + 1,
            MetricWidth);
    }

    inline static int phaseLength(const Phase phase) noexcept
    {
        return std::max((int)std::string(PhaseNames[phase]).length() + 1,
            TimeWidth);
    }

    inline static void printHeader(const std::vector<Metric>& metrics,
        const std::vector<Phase>& phases) noexcept
    {
        std::cout << std::endl
                  << "Statistics:" << std::endl
                  << std::setw(8) << "Round";
        for (const Metric metric : metrics) {
            std::cout << std::setw(metricLength(metric)) << MetricNames[metric];
        }
        for (const Phase phase : phases) {
            std::cout << std::setw(phaseLength(phase)) << PhaseNames[phase];
        }
        std::cout << std::setw(TimeWidth) << "Total" << std::endl;
    }

    inline void print(const std::vector<Metric>& metrics,
        const std::vector<Phase>& phases,
        const std::string& name) const noexcept
    {
        std::cout << std::setw(8) << name;
        for (const Metric metric : metrics) {
            std::cout << std::setw(metricLength(metric))
                      << String::prettyInt(metricValue[metric]);
        }
        for (const Phase phase : phases) {
            std::cout << std::setw(phaseLength(phase) + 1)
                      << String::musToString(phaseTime[phase]);
        }
        std::cout << std::setw(TimeWidth + 1) << String::musToString(totalTime)
                  << std::endl;
    }

    inline void printValues() const noexcept
    {
        double timePerBCH = phaseTime[PHASE_BCHSEARCHES] / metricValue[METRIC_BCHSEARCHES];
        double stationsPerVehicle = double(metricValue[METRIC_NEARBYVEHICLEROUTES]) / double(metricValue[METRIC_VEHICLES]);
        double stationsPerVehicleStop = double(metricValue[METRIC_NEARBYVEHICLEROUTES]) / double(metricValue[METRIC_VEHICLESTOPS] - metricValue[METRIC_VEHICLES]);
        double vehiclesPerStation = double(metricValue[METRIC_NEARBYVEHICLEROUTES]) / double(metricValue[METRIC_STATIONS]);

        double pickupEdgesPerStation = double(metricValue[METRIC_PICKUPEDGES]) / double(metricValue[METRIC_STATIONS]);
        double dropoffEdgesPerStation = double(metricValue[METRIC_DROPOFFEDGES]) / double(metricValue[METRIC_STATIONS]);
        double pickupEdgesPerVehicleStop = double(metricValue[METRIC_PICKUPEDGES]) / double(metricValue[METRIC_VEHICLESTOPS]);
        double dropoffEdgesPerVehicleStop = double(metricValue[METRIC_DROPOFFEDGES]) / double(metricValue[METRIC_VEHICLESTOPS]);

        std::cout << "Stations/Vehicle: "
                  << String::prettyDouble(stationsPerVehicle) << std::endl;
        std::cout << "Stations/ValidVehicleStop: "
                  << String::prettyDouble(stationsPerVehicleStop) << std::endl;
        std::cout << "Vehicles/Station: "
                  << String::prettyDouble(vehiclesPerStation) << std::endl;
        std::cout << "Time/BCH: " << String::musToString(timePerBCH) << std::endl;

        std::cout << "Pickupedges/Station: "
                  << String::prettyDouble(pickupEdgesPerStation) << std::endl;
        std::cout << "Dropoffedges/Station: "
                  << String::prettyDouble(dropoffEdgesPerStation) << std::endl;
        std::cout << "Pickupedges/VehicleStop: "
                  << String::prettyDouble(pickupEdgesPerVehicleStop) << std::endl;
        std::cout << "Dropoffedges/VehicleStop: "
                  << String::prettyDouble(dropoffEdgesPerVehicleStop) << std::endl;
    }

    inline void writeHeaderToFile(
        std::ofstream& outputFile, const std::vector<Metric>& metrics,
        const std::vector<Phase>& phases,
        std::vector<std::string>& additionalNames) const noexcept
    {
        additionalNames.push_back("Avg_Stations_Ellipse");
        additionalNames.push_back("Avg_Ellipses_Station");

        for (const auto& name : additionalNames) {
            outputFile << name << ',';
        }
        for (const Metric& metric : metrics) {
            outputFile << MetricNames[metric] << ',';
        }
        for (const Phase& phase : phases) {
            outputFile << PhaseNames[phase] << ',';
        }
        outputFile << "total_time" << '\n';
    }

    inline void writeValuesToFile(
        std::ofstream& outputFile, const std::vector<Metric>& metrics,
        const std::vector<Phase>& phases,
        std::vector<double>& additionalValues) const noexcept
    {
        additionalValues.push_back(double(metricValue[METRIC_NEARBYVEHICLEROUTES]) / double(metricValue[METRIC_VEHICLESTOPS] - metricValue[METRIC_VEHICLES]));
        additionalValues.push_back(double(metricValue[METRIC_NEARBYVEHICLEROUTES]) / double(metricValue[METRIC_STATIONS]));

        for (const auto value : additionalValues) {
            outputFile << value << ',';
        }
        for (const Metric metric : metrics) {
            outputFile << metricValue[metric] << ',';
        }
        for (const Phase phase : phases) {
            outputFile << int(phaseTime[phase]) << ',';
        }
        outputFile << int(totalTime) << '\n';
    }

    inline void writeToFile(
        std::ofstream& outputFile, const std::vector<Metric>& metrics,
        const std::vector<Phase>& phases,
        std::vector<std::string>& additionalNames,
        std::vector<double>& additionalValues) const noexcept
    {
        writeHeaderToFile(outputFile, metrics, phases, additionalNames);
        writeValuesToFile(outputFile, metrics, phases, additionalValues);
    }

    std::vector<long long> metricValue;
    std::vector<double> phaseTime;
    double totalTime;
};

class NoProfiler {
public:
    inline void registerPhases(
        const std::initializer_list<Phase>&) const noexcept { }
    inline void registerMetrics(
        const std::initializer_list<Metric>&) const noexcept { }

    inline void initialize() const noexcept { }

    inline void start() const noexcept { }
    inline void done() const noexcept { }

    inline void startPhase() const noexcept { }
    inline void donePhase(const Phase) const noexcept { }

    inline void countMetric(const Metric) const noexcept { }
};

class Profiler : public NoProfiler {
public:
    Profiler()
        : totalTime(0.0)
        , timer(NUM_PHASES)
    {
    }

    inline void registerPhases(
        const std::initializer_list<Phase>& phaseList) noexcept
    {
        for (const Phase phase : phaseList) {
            phases.push_back(phase);
        }
    }

    inline void registerMetrics(
        const std::initializer_list<Metric>& metricList) noexcept
    {
        for (const Metric metric : metricList) {
            metrics.push_back(metric);
        }
    }

    inline void initialize() noexcept { totalTime = 0.0; }

    inline void start() noexcept
    {
        initialize();
        totalTimer.restart();
    }

    inline void done() noexcept
    {
        totalTime += totalTimer.elapsedMicroseconds();
        data.totalTime = totalTime;
        printStatistics();
    }

    inline void startPhase(const Phase phase) noexcept { timer[phase].restart(); }

    inline void donePhase(const Phase phase) noexcept
    {
        data.phaseTime[phase] += timer[phase].elapsedMicroseconds();
    }

    inline void countMetric(const Metric metric, const int value = 1) noexcept
    {
        data.metricValue[metric] += value;
    }

    inline double getTotalTime() const noexcept { return totalTime; }

    inline void writeToFile(std::string outputFileName,
        std::vector<std::string>& additionalNames,
        std::vector<double>& additionalValues) const
    {
        std::ofstream outputFile(outputFileName);
        if (!outputFile.good())
            throw std::invalid_argument("file cannot be opened -- '" + outputFileName + "'");

        data.writeToFile(outputFile, metrics, phases, additionalNames,
            additionalValues);
    }

    inline void writeHeaderToFile(
        std::ofstream& outputFile,
        std::vector<std::string>& additionalNames) const
    {
        data.writeHeaderToFile(outputFile, metrics, phases, additionalNames);
    }

    inline void writeValuesToFile(std::ofstream& outputFile,
        std::vector<double>& additionalValues) const
    {
        data.writeValuesToFile(outputFile, metrics, phases, additionalValues);
    }

    inline void printStatistics() const noexcept
    {
        TimeData::printHeader(metrics, phases);
        data.print(metrics, phases, "total");
        data.printValues();
        std::cout << "Total time: " << String::musToString(totalTime) << std::endl;
    }

private:
    Timer totalTimer;
    double totalTime;
    Timer phaseTimer;
    std::vector<Timer> timer;
    std::vector<Phase> phases;
    std::vector<Metric> metrics;

public:
    TimeData data;
};

} // namespace RIDERAPTOR
