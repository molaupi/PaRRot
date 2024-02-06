#!/usr/bin/bash

DIR=../Networks/Berlin/

./rideRAPTOR \
    -veh-g ${DIR}KARRI/Graphs/Berlin-1pct_pedestrian_veh.gr.bin \
    -psg-g ${DIR}KARRI/Graphs/Berlin-1pct_pedestrian_psg.gr.bin \
    -r ${DIR}KARRI/Requests/Berlin-1pct_pedestrian.csv \
    -v ${DIR}KARRI/Vehicles/Berlin-1pct_pedestrian.csv \
    -veh-h ${DIR}KARRI/CHs/Berlin-1pct_pedestrian_veh_time.ch.bin \
    -psg-h ${DIR}KARRI/CHs/Berlin-1pct_pedestrian_psg_time.ch.bin \
    -o ${DIR}RideRAPTOR/ \
    -station-mapping ${DIR}ULTRA/stations.mapped.csv \
    -raptor-data ${DIR}ULTRA/raptor.binary
