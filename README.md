# PARROT
 ULTRA + KARRI

`./KARRI/RawData/TransformLocations -tar-g KARRI/Publications/KaRRi/Inputs/Graphs/Berlin-1pct_pedestrian_veh.gr.bin -v Networks/Berlin/csv/modified_stops.csv -l-col-name latlon -in-repr lat-lng -out-repr edge-id -o Networks/Berlin/stations.mapped`

## GTFS Reading and CSV
Einfach in `ULTRA/Runnables` die Network executable bauen mit `make NetworkRelease`. Dann `downloadBerlinGTFS.sh` ausführen. Das downloaded die aktuelle VBB GTFS Instanz (VBB = Verkehrsverbund Berlin-Brandenburg) und speichert es in `Networks/Berlin/GTFS/`. Dann kann man `./Network` ausführen, und einfach `runScript GTFSToRAPTOR.script` ausführen. Das erzeugt die RAPTOR binaries und die CSV Dateien.
Mit dem python script `python3 prepare_stops.py --csv Networks/Berlin/CSV/` wird dann die von KARRI lesbare `modified_stops.csv` erzeugt.
