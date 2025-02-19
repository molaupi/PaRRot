# PARROT
 ULTRA + KARRI

`./KARRI/RawData/TransformLocations -tar-g KARRI/Publications/KaRRi/Inputs/Graphs/Berlin-1pct_pedestrian_veh.gr.bin -v Networks/Berlin/csv/modified_stops.csv -l-col-name latlon -in-repr lat-lng -out-repr edge-id -o Networks/Berlin/stations.mapped`

## GTFS Reading and CSV
Einfach in `ULTRA/Runnables` die Network executable bauen mit `make NetworkRelease`. Dann `downloadBerlinGTFS.sh` ausführen. Das downloaded die aktuelle VBB GTFS Instanz (VBB = Verkehrsverbund Berlin-Brandenburg) und speichert es in `Networks/Berlin/GTFS/`. Dann kann man `./Network` ausführen, und einfach `runScript GTFSToRAPTOR.script` ausführen. Das erzeugt die RAPTOR binaries und die CSV Dateien.
Mit dem python script `python3 prepare_stops.py --csv Networks/Berlin/CSV/` wird dann die von KARRI lesbare `modified_stops.csv` erzeugt.


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