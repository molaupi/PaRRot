#!/bin/bash

mkdir -p ../../Networks/Berlin/GTFS/

wget https://www.vbb.de/fileadmin/user_upload/VBB/Dokumente/API-Datensaetze/gtfs-mastscharf/GTFS.zip -O gtfs.zip
unzip gtfs.zip -d ../../Networks/Berlin/GTFS/
rm gtfs.zip
