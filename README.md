# PaRRot
PaRRot (Public Transit Ride-Pooling Router) is a journey planner for multimodal journeys incorporating public 
transportation (PT) and ride-pooling (RP).
It is based on the multimodal PT routing framework [ULTRA](https://github.com/kit-algo/ULTRA) and the RP dispatcher
[KaRRi](https://github.com/molaupi/karri).

## Prerequisites

To build PaRRot, you need to have CMake and Python installed. On Debian and its derivatives
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
The script `ULTRA_Runnables/BuildInputData.sh` constructs the required input data based on OpenStreetMap (OSM) 
data for the road and pedestrian networks, and GTFS data for the PT network.
The build script uses a configuration file, in which you can specify the OSM and GTFS source files, 
boundary polygons for the observation area and other parameters.
An example configuration file for the city of Berlin is provided in `ULTRA_Runnables/config_Berlin.txt`.
The required OSM source file for Germany can be downloaded from [Geofabrik](https://download.geofabrik.de/europe/germany.html).
GTFS data is available through many public transport authorities.
For Germany, consider [OpenData ÖPNV](https://www.opendata-oepnv.de/).
The `Inputs` subdirectory provides boundary polygons for Berlin (separate for an inner area = city limits and an outer area = city limits + margin).
Boundary polygons need to be in [Osmosis polygon format](https://wiki.openstreetmap.org/wiki/Osmosis/Polygon_Filter_File_Format).
Additionally, the `Inputs` subdirectory specifies example demand data for Berlin, given as origin-destination pairs (in WGS84 degrees latitude/longitude) with request times (offset from midnight on a weekday in tenths of seconds).
If you want to use your own demand data, you can specify it in a separate file in the same format.

To run the build script, you need to have the [`osmium` tool](https://osmcode.org/osmium-tool/) installed.
On many Linux distributions, it can be installed using the package manager.
Otherwise, manually download and build it from the source code.
In this case, you can specify the path to the `osmium` binary in the configuration file.

To run the build script, execute the following command at the top-level directory of the framework:

```
$ bash ULTRA_Runnables/BuildInputData.sh ULTRA_Runnables/config_Berlin.txt
```

## Running PaRRot
To run PaRRot, build `PTaxi` using CMake and execute with the prepared input data.

For instance, when using the provided configuration for Berlin, the parameters of `PTaxi` should be set as follows:
```
PTaxi \
-veh-g Inputs/KARRI/Graphs/Berlin_veh.gr.bin \
-psg-g Inputs/KARRI/Graphs/Berlin_psg.gr.bin \
-v Inputs/KARRI/Vehicles/Berlin_vehicles.csv \
-r Inputs/KARRI/Requests/Berlin.csv \
-veh-h Inputs/KARRI/CHs/Berlin_veh_time.ch.bin \
-psg-h Inputs/KARRI/CHs/Berlin_psg_time.ch.bin \
-raptor-data Inputs/ULTRA/Berlin/raptor.binary \
-station-mapping Inputs/PaRRot/Berlin_station_mapping.csv \
-station-buckets Inputs/PaRRot/Berlin_stations_veh \
-psg-station-buckets Inputs/PaRRot/Berlin_stations_psg \
-o <path to output directory>
```

