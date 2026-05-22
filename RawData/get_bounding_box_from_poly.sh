#!/usr/bin/env bash

polygon_file="$1"
awk '
BEGIN {
    lon_min=180; lon_max=-180;
    lat_min=90; lat_max=-90;
}
  NF==2 {                     # only lines with two numbers
    lon=$1; lat=$2;
    if (lon < lon_min) lon_min=lon;
    if (lon > lon_max) lon_max=lon;
    if (lat < lat_min) lat_min=lat;
    if (lat > lat_max) lat_max=lat;
  }
  END {print lon_min, lon_max, lat_min, lat_max}
' "${polygon_file}"