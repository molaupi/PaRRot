# PTaxi
 ULTRA + KARRI

## GTFS Reading and CSV
Run `downloadBerlinGTFS.sh` to download the current Berlin GTFS instance from VBB (Verkehrsverbund Berlin-Brandenburg) and save it to `Networks/Berlin/GTFS/`.
In `ULTRA_Runnables`: build the executables with `make NetworkRelease`.
Run the executable `./Network` and within the interactive shell `runScript BuildBerlinNetwork.script`. This creates the RAPTOR binaries and the CSV files.
With the python script `python3 prepare_stops.py --csv Networks/Berlin/CSV/`, the file `modified_stops.csv` will be created, which can be read by KaRRi to calculate the edge ids of the PT stations.

In KaRRi: run the Transform Locations executable to get the mapped stations with the following command (note that paths are relative)
`./RawData/TransformLocations -tar-g Networks/Berlin/KARRI/Graphs/Berlin-1pct_pedestrian_veh.gr.bin -v Networks/Berlin/CSV/modified_stops.csv -l-col-name latlon -in-repr lat-lng -out-repr edge-id -psg -o Networks/Berlin/Preprocessing/PT/stations.mapped`

## Build Core CH and normal CH
In `ULTRA_Runnables`: build the executables with `make ULTRARelease`.
Run the executable `./ULTRA`.
Within the interactive shell:
- Run `buildCH ../Networks/Berlin/ULTRA/raptor.binary.graph ../Networks/Berlin/ULTRA/chOrder ../Networks/Berlin/ULTRA/CH`
- Run `buildCoreCH ../Networks/Berlin/ULTRA/raptor.binary ../Networks/Berlin/ULTRA/coreCHOrder ../Networks/Berlin/ULTRA/coreCH ../Networks/Berlin/ULTRA/raptor.binary`
As a result, you will obtain the CH files required to run the ULTRARAPTOR algorithm, while the Core CH is automatically incorporated into the raptor.binary.


## Prerequisites

To build KaRRi, you need to have some tools and libraries installed. On Debian and its derivatives
(such as Ubuntu) the `apt-get` tool can be used:

```
$ sudo apt-get install build-essential
$ sudo apt-get install cmake
$ sudo apt-get install libboost-all-dev
$ sudo apt-get install python3 python3-pip; pip3 install -r RawData/python_requirements.txt
$ sudo apt-get install sqlite3 libsqlite3-dev
$ sudo apt-get install zlib1g-dev
```

Next, you need to clone the libraries in the `External` subdirectory and build the `RoutingKit` library. To do so,
type the following commands at the top-level directory of the framework:

```
$ git submodule update --init
$ make -C External/RoutingKit lib/libroutingkit.so
```

## Build Static Buckets for PTaxi
You must run the executables BuildStaticBuckets to generate required input data for PTaxi.
These includes:
- Bucket graph for ULTRA
- Converted passenger graph for ULTRA
- Station buckets for KaRRi