# PTaxi
 ULTRA + KARRI

## GTFS Reading and CSV
Run `downloadBerlinGTFS.sh` to download the current Berlin GTFS instance from VBB (Verkehrsverbund Berlin-Brandenburg) and save it to `Networks/Berlin/GTFS/`.
In `ULTRA_Runnables`: build the executables with `make NetworkRelease`.
Run the executable `./Network` and within the interactive shell `runScript BuildBerlinNetwork.script`. This creates the RAPTOR binaries and the CSV files.
With the python script `python3 prepare_csv.py --mode modify_stops --csv_directory Networks/Berlin/CSV/1pct`, the file `modified_stops.csv` will be created, which can be read by KaRRi to calculate the edge ids of the PT stations.

In KaRRi: run the Transform Locations executable to get the mapped stations with the following command (note that paths are relative)
`./RawData/TransformLocations -tar-g Networks/Berlin/KARRI/Graphs/Berlin-1pct_pedestrian_veh.gr.bin -v Networks/Berlin/CSV/1pct/modified_stops.csv -l-col-name latlon -in-repr lat-lng -out-repr edge-id -psg -o Networks/Berlin/Preprocessing/PT/Berlin-1pct_stations.mapped`

## Build CH, build core CH and compute shortcuts for ULTRA
In `ULTRA_Runnables`: build the executables with `make ULTRARelease`.
Run the executable `./ULTRA`.
Within the interactive shell:
- Run `buildCH ../Networks/Berlin/ULTRA/Berlin-1pct_raptor.binary.graph ../Networks/Berlin/ULTRA/Berlin-1pct_chOrder ../Networks/Berlin/ULTRA/Berlin-1pct_CH`
- Run `buildCoreCH ../Networks/Berlin/ULTRA/Berlin-1pct_raptor.binary ../Networks/Berlin/ULTRA/Berlin-1pct_coreCHOrder ../Networks/Berlin/ULTRA/Berlin-1pct_coreCH ../Networks/Berlin/ULTRA/Berlin-1pct_raptor-core.binary`
- Run `computeStopToStopShortcuts ../Networks/Berlin/ULTRA/Berlin-1pct_raptor-core.binary ../Networks/Berlin/ULTRA/Berlin-1pct_raptor-shortcuts.binary 0`
As a result, you will obtain the CH files and raptor binary required to run the ULTRARAPTOR algorithm.
`runULTRARAPTORQueries ../Networks/Berlin/ULTRA/Berlin-1pct_raptor-shortcuts.binary ../Networks/Berlin/ULTRA/Berlin-1pct_CH 10`

## Transform KaRRi Requests to LatLng to run in ULTRA

In KaRRi: 
- Run `./RawData/TransformRequestsToLatLng` to map the requests from edge_id to latitude and longtitude.

In ULTRA: run `runULTRARAPTORWithGivenQueries /home/nghalinh2711/PARROT/Networks/Berlin/ULTRA/Berlin-1pct_raptor-shortcuts.binary /home/nghalinh2711/PARROT/Networks/Berlin/ULTRA/Berlin-1pct_CH /home/nghalinh2711/PARROT/Networks/Berlin/ULTRA/Berlin-1pct_raptor.binary.graph /home/nghalinh2711/PARROT/Networks/Berlin/KARRI/Requests/Berlin-1pct_pedestrian_latlng.csv /home/nghalinh2711/PARROT/Networks/Berlin/Outputs/Berlin-1pct_journeys.csv false` to run ULTRARAPTOR algorithm with KaRRi requests.

With the python script `python3 prepare_csv.py --mode merge_files --karri_requests Networks/Berlin/KARRI/Requests/Berlin-1pct_pedestrian.csv --ultra_requests Networks/Berlin/KARRI/Requests/Berlin-1pct_ULTRA_requests.csv --output Networks/Berlin/KARRI/Requests/Berlin-1pct_merged.csv` will be created, which is a combined request file, including valid edge ids and vertex ids for KaRRi and ULTRA, respectively.

## Build Static Buckets for PTaxi
You must run the executables BuildStaticBuckets to generate required input data for PTaxi.
These includes:
- Bucket graph for ULTRA
- Station buckets for KaRRi
`BuildStaticBuckets -veh-g ../../../Networks/Berlin/KARRI/Graphs/Berlin-1pct_pedestrian_veh.gr.bin -psg-g ../../../Networks/Berlin/KARRI/Graphs/Berlin-1pct_pedestrian_psg.gr.bin -veh-h ../../../Networks/Berlin/KARRI/CHs/Berlin-1pct_pedestrian_veh_time.ch.bin -psg-h ../../../Networks/Berlin/KARRI/CHs/Berlin-1pct_pedestrian_psg_time.ch.bin -raptor-data ../../../Networks/Berlin/ULTRA/raptor-shortcuts.binary -station-mapping ../../../Networks/Berlin/Preprocessing/PT/stations.mapped.csv -ch ../../../Networks/Berlin/ULTRA/CH -o-bucket-graph ../../../Networks/Berlin/Preprocessing/PT/bucket -o-station-buckets ../../../Networks/Berlin/Preprocessing/Taxi/stations`


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
