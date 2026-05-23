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
[ -z "${cfg[parrot_source_dir]}" ] && echo "parrot_source_dir is not set in the config file." && exit 1
[ -z "${cfg[base_input_dir]}" ] && echo "base_input_dir is not set in the config file." && exit 1
[ -z "${cfg[instance_name]}" ] && echo "instance_name is not set in the config file." && exit 1
[ -z "${cfg[gtfs_source]}" ] && echo "gtfs_source is not set in the config file." && exit 1
[ -z "${cfg[osm_source]}" ] && echo "osm_source is not set in the config file." && exit 1
[ -z "${cfg[boundary_inner]}" ] && echo "boundary_inner is not set in the config file." && exit 1
[ -z "${cfg[boundary_outer]}" ] && echo "boundary_outer is not set in the config file." && exit 1
[ -z "${cfg[demand_source]}" ] && echo "demand_source is not set in the config file." && exit 1
[ -z "${cfg[first_day]}" ] && echo "last_day is not set in the config file." && exit 1
[ -z "${cfg[num_vehicles]}" ] && echo "num_vehicles is not set in the config file." && exit 1
[ -z "${cfg[vehicle_capacity]}" ] && echo "vehicle_capacity is not set in the config file." && exit 1


bash "${cfg[parrot_source_dir]}"/ULTRA_Runnables/ProcessRawOSM.sh "${config_file}"
bash "${cfg[parrot_source_dir]}"/ULTRA_Runnables/BuildRoadNetworks.sh "${config_file}"
bash "${cfg[parrot_source_dir]}"/ULTRA_Runnables/PrepareRequestsAndVehicles.sh "${config_file}"
bash "${cfg[parrot_source_dir]}"/ULTRA_Runnables/ProcessRawGTFS.sh "${config_file}"
bash "${cfg[parrot_source_dir]}"/ULTRA_Runnables/PrepareRAPTORData.sh "${config_file}"
bash "${cfg[parrot_source_dir]}"/ULTRA_Runnables/BuildTransferShortcutGraph.sh "${config_file}"
bash "${cfg[parrot_source_dir]}"/ULTRA_Runnables/PrepareRoadPTMapping.sh "${config_file}"