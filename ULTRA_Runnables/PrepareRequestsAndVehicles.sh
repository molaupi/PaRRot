#!/bin/bash

set -euo pipefail

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
[ -z "${cfg[parrot_source_dir]}" ] && echo "parrot_source_dir is not set in the config file." && exit 1
[ -z "${cfg[demand_source]}" ] && echo "demand_source is not set in the config file." && exit 1
[ -z "${cfg[num_vehicles]}" ] && echo "num_vehicles is not set in the config file." && exit 1
[ -z "${cfg[vehicle_capacity]}" ] && echo "vehicle_capacity is not set in the config file." && exit 1
[ -z "${cfg[boundary_inner]}" ] && echo "boundary_inner is not set in the config file." && exit 1
[ -z "${cfg[first_day]}" ] && echo "first_day is not set in the config file." && exit 1
[ -z "${cfg[last_day]}" ] && echo "last_day is not set in the config file." && exit 1

graphs_dir="${cfg[base_input_dir]}/KaRRi/Graphs/"

# Map requests from demand source into network
echo "Mapping requests from demand source into network."
requests_dir="${cfg[base_input_dir]}/KaRRi/Requests/"
mkdir -p "${requests_dir}"

bash "${cfg[parrot_source_dir]}"/RawData/demand_source_to_transform_locations_format.sh "${cfg[demand_source]}" "${requests_dir}"/tmp_transform_locations_format.csv
cmake -S "${cfg[parrot_source_dir]}" -B "${cfg[parrot_source_dir]}"/Build/Release -DCMAKE_BUILD_TYPE=Release
cmake --build "${cfg[parrot_source_dir]}"/Build/Release --target TransformLocations -j
"${cfg[parrot_source_dir]}"/Build/Release/RawData/TransformLocations \
  -p "${requests_dir}"/tmp_transform_locations_format.csv \
  -psg \
  -in-repr lat-lng \
  -out-repr edge-id \
  -tar-g "${graphs_dir}"/"${cfg[instance_name]}"_veh.gr.bin \
  -o "${requests_dir}"/"${cfg[instance_name]}"_od_edges.csv
bash "${cfg[parrot_source_dir]}"/RawData/add_req_time_to_origin_destination.sh "${requests_dir}"/"${cfg[instance_name]}"_od_edges.csv "${cfg[demand_source]}" "${requests_dir}"/"${cfg[instance_name]}".csv
rm "${requests_dir}"/tmp_transform_locations_format.csv
rm "${requests_dir}"/"${cfg[instance_name]}"_od_edges.csv
rm "${requests_dir}"/"${cfg[instance_name]}"_od_edges.vertexmatches.csv

# Generate vehicles file
echo "Generating vehicles at random initial locations."
vehicles_dir="${cfg[base_input_dir]}/KaRRi/Vehicles/"
num_seconds_obs_period=$(bash "${cfg[parrot_source_dir]}"/RawData/days_interval_to_num_seconds.sh "${cfg[first_day]}" "${cfg[last_day]}")
mkdir -p "${vehicles_dir}"
cmake -S "${cfg[parrot_source_dir]}" -B "${cfg[parrot_source_dir]}"/Build/Release -DCMAKE_BUILD_TYPE=Release
cmake --build "${cfg[parrot_source_dir]}"/Build/Release --target GenerateRandomVehicles -j
"${cfg[parrot_source_dir]}"/Build/Release/RawData/GenerateRandomVehicles \
  -g "${graphs_dir}"/"${cfg[instance_name]}"_veh.gr.bin \
  -n "${cfg[num_vehicles]}" \
  -c "${cfg[vehicle_capacity]}" \
  -a "${cfg[boundary_inner]}" \
  -start 0 \
  -end "${num_seconds_obs_period}" \
  -o "${vehicles_dir}"/"${cfg[instance_name]}"_vehicles.csv

