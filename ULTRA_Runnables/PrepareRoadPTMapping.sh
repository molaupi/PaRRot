#!/bin/bash

config_file=$1

# Check if config file exists
if [ ! -f "${config_file}" ]; then
    echo "Config file not found: ${config_file}"
    exit 1
fi

# Read config file to load variables
declare -A cfg
while IFS='=' read -r key value || [[ -n $key ]]; do
    # Skip empty lines and comment lines
    [[ -z "$key" || "$key" =~ ^[[:space:]]*# ]] && continue

    # Trim surrounding whitespace (xargs is POSIX‑compatible)
    key=$(echo "$key" | xargs)
    value=$(echo "$value" | xargs)

    cfg["$key"]="$value"
done < "$config_file"

# Check if required variables are set
[ -z "${cfg[base_input_dir]}" ] && echo "base_input_dir is not set in the config file." && exit 1
[ -z "${cfg[instance_name]}" ] && echo "instance_name is not set in the config file." && exit 1


parrot_inputs_dir="${cfg[base_input_dir]}/PaRRot/"
mkdir -p "${parrot_inputs_dir}"

# Output locations of PT stations
echo "Getting locations of PT station."
cmake -S "${cfg[parrot_source_dir]}" -B "${cfg[parrot_source_dir]}"/Build/Release -DCMAKE_BUILD_TYPE=Release
cmake --build "${cfg[parrot_source_dir]}"/Build/Release --target Network -j
"${cfg[parrot_source_dir]}"/Build/Release/ULTRA_Runnables/Network getRAPTORStopCoordinates \
  "${cfg[base_input_dir]}"/ULTRA/"${cfg[instance_name]}"/raptor.binary \
  "${parrot_inputs_dir}"/"${cfg[instance_name]}"_raptor_stop_locations.csv

# Map station locations into road network
echo "Mapping station locations into road network."
cmake -S "${cfg[parrot_source_dir]}" -B "${cfg[parrot_source_dir]}"/Build/Release -DCMAKE_BUILD_TYPE=Release
cmake --build "${cfg[parrot_source_dir]}"/Build/Release --target TransformLocations -j
"${cfg[parrot_source_dir]}"/Build/Release/RawData/TransformLocations \
  -psg \
  -v "${parrot_inputs_dir}"/"${cfg[instance_name]}"_raptor_stop_locations.csv \
  -l-col-name location \
  -in-repr lat-lng \
  -out-repr edge-id \
  -tar-g "${cfg[base_input_dir]}"/KaRRi/Graphs/"${cfg[instance_name]}"_veh.gr.bin \
  -o "${parrot_inputs_dir}"/"${cfg[instance_name]}"_station_mapping.csv

# Remove temporary file with RAPTOR stop locations and vertexmatches file
rm "${parrot_inputs_dir}"/"${cfg[instance_name]}"_raptor_stop_locations.csv
rm "${parrot_inputs_dir}"/"${cfg[instance_name]}"_station_mapping.vertexmatches.csv

# Build static station buckets in vehicle and pedestrian network
echo "Building static station buckets in vehicle and pedestrian network."
cmake -S "${cfg[parrot_source_dir]}" -B "${cfg[parrot_source_dir]}"/Build/Release -DCMAKE_BUILD_TYPE=Release
cmake --build "${cfg[parrot_source_dir]}"/Build/Release --target BuildStaticBuckets -j
"${cfg[parrot_source_dir]}"/Build/Release/RawData/BuildStaticBuckets \
  -veh-g "${cfg[base_input_dir]}"/KaRRi/Graphs/"${cfg[instance_name]}"_veh.gr.bin \
  -psg-g "${cfg[base_input_dir]}"/KaRRi/Graphs/"${cfg[instance_name]}"_psg.gr.bin \
  -veh-h "${cfg[base_input_dir]}"/KaRRi/CHs/"${cfg[instance_name]}"_veh_time.ch.bin \
  -psg-h "${cfg[base_input_dir]}"/KaRRi/CHs/"${cfg[instance_name]}"_psg_time.ch.bin \
  -station-mapping "${parrot_inputs_dir}"/"${cfg[instance_name]}"_station_mapping.csv \
  -o-station-buckets "${parrot_inputs_dir}"/"${cfg[instance_name]}"_stations_veh.bucket.bin \
  -o-psg-station-buckets "${parrot_inputs_dir}"/"${cfg[instance_name]}"_stations_psg.bucket.bin


