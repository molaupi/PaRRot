#!/bin/bash
# Extracts OSM data for smaller network from larger surrounding network

baseDir=$1 # Base dir
surroundingNetworkName=$2 # Name of surrounding network (Germany, Europe, ...). Expects file ${baseDir}/Inputs/RawData/OSM/${surroundingNetworkName}_Highways.osm.pbf
networkName=$3 # Name of the smaller network
boundariesDir=$4 # Expects boundary files for ${networkName}_Inner.poly and ${networkName}_Outer.poly at $boundariesDir/
osmium=$5 # Path to osmium tool (if not given, will expect that osmium is installed and callable anywhere)

if [ -z ${osmium} ]; then
	osmium=osmium # If user did not specify location of osmium executable, assume it is installed on default PATH
fi

osmDir=${baseDir}/Inputs/RawData/OSM

# 1. Create full network for full area (can be used to extract pedestrian graph for public transport transfers in full area)
$osmium extract -p $boundariesDir/${networkName}_Outer.poly -o $osmDir/${networkName}_FullOuter.osm.pbf $osmDir/${surroundingNetworkName}_Highways.osm.pbf


# 2. Create network with full inner area and filtered outer area without pedestrian paths (can be used for KaRRi/PTaxi routing for requests in inner area)

## For inner area take all roads and pedestrian pathways
$osmium extract -p $boundariesDir/${networkName}_Inner.poly -o $osmDir/${networkName}_FullInner.osm.pbf $osmDir/${surroundingNetworkName}_Highways.osm.pbf

## For outer area take only roads
$osmium extract -p $boundariesDir/${networkName}_Outer.poly -o $osmDir/${networkName}_VehicleOuter.osm.pbf $osmDir/${surroundingNetworkName}_VehicleHighways.osm.pbf

## Merge to a shared network
$osmium merge -o $osmDir/${networkName}.osm.pbf $osmDir/${networkName}_FullInner.osm.pbf $osmDir/${networkName}_VehicleOuter.osm.pbf
rm $osmDir/${networkName}_FullInner.osm.pbf $osmDir/${networkName}_VehicleOuter.osm.pbf