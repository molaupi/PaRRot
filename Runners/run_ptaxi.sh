#!/bin/bash

# Script to run PTaxi on compute servers.
# Usage: sbatch --partition=<server-name> run_ptaxi.sh <source-dir> <instance-name> <output-base-dir> [timeout]
# Example: sbatch --partition=backus run_ptaxi.sh /nfs/home/hnguyen/PdF/PARROT Berlin-1pct /nfs/home/hnguyen/PdF/PARROT/Outputs 500m
#   - <server-name> : name of the server to run the script on, e.g., dijkstra, backus,..
#   - <source-dir> : absolute path to the lowest folder of your repository, so probably something like /nfs/home/hnguyen/KaRRi/karri/
#   - <instance-name> : either Berlin-1pct or Berlin-10pct
#   - <output-base-dir> : absolute path to a freely chosen output folder, e.g., /nfs/home/hnguyen/PdF/PARROT/Outputs/ (the folder must exist before the call)
#   - [timeout]: optional timeout for each run in seconds

# Read input parameters

sourceDir=$1
instanceName=$2
outputBaseDir=$3
timeout=$4
passengerMode=pedestrian

# Check, if output directory exists
if ! [ -d "$outputBaseDir" ]; then
	echo "Output directory ${outputBaseDir} does not exist."
	exit 1
fi

# Set timeout of 300 minutes, if not given
if [ -z ${timeout} ]; then 
	timeout="300m"
fi
echo "Using timeout of ${timeout}."

# Run PTaxi:
name=${instanceName}_${passengerMode}
vehName=${name}_veh
psgName=${name}_psg

# Paths for inputs
parrotInputDir=$sourceDir/Networks/Berlin
karriInputDir=$parrotInputDir/KARRI
vehGraph=$karriInputDir/Graphs/${vehName}.gr.bin
psgGraph=$karriInputDir/Graphs/${psgName}.gr.bin
vehicles=$karriInputDir/Vehicles/${name}.csv
requests=$karriInputDir/Requests/${name}.csv
vehCh=$karriInputDir/CHs/${vehName}_time.ch.bin
psgCh=$karriInputDir/CHs/${psgName}_time.ch.bin

raptor=$parrotInputDir/ULTRA/raptor-shortcuts.binary
stationMapping=$parrotInputDir/Preprocessing/PT/stations.mapped.csv
bucketGraph=$parrotInputDir/Preprocessing/PT/bucket
stationBuckets=$parrotInputDir/Preprocessing/Taxi/stations
ptCh=$parrotInputDir/ULTRA/CH

# Create concrete output directory, whose name consists of instanceName + current timestamp.
currentTime=$(date "+%Y.%m.%d-%H.%M")
outputDir=$outputBaseDir/${instanceName}_${currentTime}
mkdir -p $outputDir

# Build PTaxi
binaryDir=$sourceDir/Build/Release

# Remove build directory to ensure clean build
echo "Cleaning build directory..."
rm -rf $binaryDir

# Fix clock skew by touching all source files to current time
echo "Fixing potential clock skew issues..."
find $sourceDir -name "*.cpp" -o -name "*.hpp" -o -name "*.h" -o -name "*.c" -o -name "CMakeLists.txt" -o -name "*.cmake" | xargs touch

echo "Building PTaxi..."
cmake -DCMAKE_BUILD_TYPE=Release -DKASSERT_ASSERTION_LEVEL=1 -S $sourceDir -B $binaryDir

# Fix clock skew in the build directory as well
if [ -d "$binaryDir" ]; then
    echo "Fixing potential clock skew in build directory..."
    find $binaryDir -exec touch {} +
fi

cmake --build $binaryDir --target PTaxi -j

# Check if binary was built successfully
if [ ! -f "$binaryDir/Runners/PTaxi" ]; then
    echo "Error: PTaxi binary not found after build. Build may have failed."
    exit 1
fi

echo "Build completed successfully."

timeout $timeout $binaryDir/Runners/PTaxi -veh-g $vehGraph -psg-g $psgGraph -v $vehicles -r $requests -veh-h $vehCh -psg-h $psgCh -o $outputDir/ptaxi -raptor-data $raptor -station-mapping $stationMapping -bucket-graph $bucketGraph -station-buckets $stationBuckets -ch $ptCh

# Remark:
# The prefix "timeout $timeout" aborts the process as soon as the timeout is reached, which is specified in the variable $timeout.
# $timeout is by default specified in seconds, but other units are also possible, for example, "90m" is 90 minutes.
# See also man timeout.