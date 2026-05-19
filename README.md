# PaRRot
PaRRot (Public Transit Ride-Pooling Router) is a journey planner for multimodal journeys incorporating public 
transportation (PT) and ride-pooling (RP).
It is based on the multimodal PT routing framework [ULTRA](https://github.com/kit-algo/ULTRA) and the RP dispatcher
[KaRRi](https://github.com/molaupi/karri).

## Prerequisites

To build PaRRot, you need to have some CMake and Python installed. On Debian and its derivatives
(such as Ubuntu) the `apt-get` tool can be used:

```
$ sudo apt-get install cmake
$ sudo apt-get install python3 python3-pip; pip3 install -r RawData/python_requirements.txt
```

Next, you need to clone the libraries in the `External` subdirectory and build the `RoutingKit` library. To do so,
type the following commands at the top-level directory of the framework:

```
$ git submodule update --init
$ make -C External/RoutingKit lib/libroutingkit.so
```

## Input Data
The input data for PaRRot entails a PT network, a road network, and a pedestrian network of the observation area.

### Public Transport Network
For the PT network, obtain schedule data in GTFS format.
Build ULTRA using CMake.
Follow the steps for constructing RAPTOR input and an ULTRAMcRAPTOR transfer shortcut graph in ULTRA.

### Road Network and Pedestrian Path Network
KaRRi constructs a pair of road network and pedestrian path network based on OpenStreetMap input data.
Follow the according steps for KaRRi.

### Station Mapping
Create a mapping of the stations of the PT network into the road and pedestrian networks.
For this, use the `raptorToCSV` command in the ULTRA `Network` executable, which transforms a ULTRA network in RAPTOR
format to CSV format.
The output contains a file for stations that includes the coordinates of each station.
Use the `TransformLocations` to map these coordinates to edges in the road network.

### Station Buckets
The preprocessing of PaRRot requires computing BCH buckets for every station in the PT network.
This needs to be done both on the road network and on the pedestrian network.
Use `BuildStaticBuckets` for this.
This executable computes the buckets and writes them to disk.
They can then be read when running PaRRot.

## Running PaRRot
To run PaRRot, build `PTaxi` using CMake and execute with the prepared input data.


