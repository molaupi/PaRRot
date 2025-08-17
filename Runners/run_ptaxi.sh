#!/bin/bash

# Script to run PTaxi on compute servers.
# Usage: sbatch --partition=<server-name> run_ptaxi.sh <source-dir> <instance-name> <output-base-dir> [timeout]
# Example: sbatch --partition=dijkstra run_ptaxi.sh /nfs/home/hnguyen/PdF/PARROT/ Berlin-1pct /nfs/home/hnguyen/PdF/PARROT/Outputs/ 120m
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

# Set timeout of 90 minutes, if not given
if [ -z ${timeout} ]; then 
	timeout="90m"
fi
echo "Using timeout of ${timeout}."

# Run PTaxi:
name=${instanceName}_${passengerMode}
vehName=${name}_veh
psgName=${name}_psg

# Hard-coded paths for inputs
karriInputDir=/nfs/work/karri/Inputs
vehGraph=$karriInputDir/Graphs/${vehName}.gr.bin
psgGraph=$karriInputDir/Graphs/${psgName}.gr.bin
vehicles=$karriInputDir/Vehicles/${name}.csv
requests=$karriInputDir/Requests/${name}.csv
vehCh=$karriInputDir/CHs/${vehName}_time.ch.bin
psgCh=$karriInputDir/CHs/${psgName}_time.ch.bin

parrotInputDir=/nfs/home/hnguyen/PdF/PARROT/Networks/Berlin
raptor=$parrotInputDir/ULTRA/raptor.binary
stationMapping=$parrotInputDir/Preprocessing/PT/stations.mapped.csv
bucketGraph=$parrotInputDir/Preprocessing/PT/bucket
stationBuckets=$parrotInputDir/Preprocessing/Taxi/stations
ptPsgCh=$parrotInputDir/Preprocessing/PT/psgCh

# Create concrete output directory, whose name consists of instanceName + current timestamp.
currentTime=$(date "+%Y.%m.%d-%H:%M")
outputDir=$outputBaseDir/${instanceName}_${currentTime}
mkdir -p $outputDir

# Build PTaxi
binaryDir=$sourceDir/Build/Release
cmake -DCMAKE_BUILD_TYPE=Release -S $sourceDir -B $binaryDir
cmake --build $binaryDir --target PTaxi


# Let PTaxi run 5 times
for i in {1..5}
do

# Run PTaxi with ID to differentiate between 5 runs
run_id=PTaxi_intermediate_$i
timeout $timeout $binaryDir/Runners/PTaxi -veh-g $vehGraph -psg-g $psgGraph -v $vehicles -r $requests -veh-h $vehCh -psg-h $psgCh -o $outputDir/$run_id -raptor-data $raptor -station-mapping $stationMapping -bucket-graph $bucketGraph --station-buckets $stationBuckets -psg-ch $ptPsgCh


done

# Remark:
# The prefix "timeout $timeout" aborts the process as soon as the timeout is reached, which is specified in the variable $timeout.
# $timeout is by default specified in seconds, but other units are also possible, for example, "90m" is 90 minutes.
# See also man timeout.
#
# The prefix "taskset 0x1" pins the process to the first processor. This means the operating system cannot freely move the process between processors; 
# instead, the process will remain on a single processor. This helps to get clean time measurements.