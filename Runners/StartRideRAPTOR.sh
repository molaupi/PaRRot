#!/usr/bin/bash
DIR=../Networks/Berlin/

# echo "Starting the RideRAPTOR script with the following arguments:"
# echo "    -veh-g ${DIR}KARRI/Graphs/Berlin-1pct_pedestrian_veh.gr.bin"
# echo "    -psg-g ${DIR}KARRI/Graphs/Berlin-1pct_pedestrian_psg.gr.bin"
# echo "    -r ${DIR}KARRI/Requests/Berlin-1pct_pedestrian.csv"
# echo "    -v ${DIR}KARRI/Vehicles/Berlin-1pct_pedestrian.csv"
# echo "    -veh-h ${DIR}KARRI/CHs/Berlin-1pct_pedestrian_veh_time.ch.bin"
# echo "    -psg-h ${DIR}KARRI/CHs/Berlin-1pct_pedestrian_psg_time.ch.bin"
# echo "    -o ${DIR}RideRAPTOR/"
# echo "    -station-mapping ${DIR}ULTRA/stations.mapped.csv"
# echo "    -raptor-data ${DIR}ULTRA/raptor.binary"
# echo "    -num-queries 10"
# echo "    -num-initial-req 10000"
 
# echo "**** Lets go ****"

./rideRAPTOR \
    -veh-g ${DIR}KARRI/Graphs/Berlin-1pct_pedestrian_veh.gr.bin \
    -psg-g ${DIR}KARRI/Graphs/Berlin-1pct_pedestrian_psg.gr.bin \
    -r ${DIR}KARRI/Requests/Berlin-1pct_pedestrian.csv \
    -v ${DIR}KARRI/Vehicles/Berlin-1pct_pedestrian.csv \
    -veh-h ${DIR}KARRI/CHs/Berlin-1pct_pedestrian_veh_time.ch.bin \
    -psg-h ${DIR}KARRI/CHs/Berlin-1pct_pedestrian_psg_time.ch.bin \
    -o ${DIR}RideRAPTOR/ \
    -station-mapping ${DIR}ULTRA/stations.mapped.csv \
    -raptor-data ${DIR}ULTRA/raptor.binary \
    -num-queries 1000 \
    -num-initial-req 5000
