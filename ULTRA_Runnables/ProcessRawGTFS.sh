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
[ -z "${cfg[gtfs_source]}" ] && echo "gtfs_source is not set in the config file." && exit 1
[ -z "${cfg[base_input_dir]}" ] && echo "base_input_dir is not set in the config file." && exit 1
[ -z "${cfg[first_day]}" ] && echo "first_day is not set in the config file." && exit 1
[ -z "${cfg[last_day]}" ] && echo "last_day is not set in the config file." && exit 1

# Create output directory if it doesn't exist
gtfs_name=$(basename "${cfg[gtfs_source]}")
gtfs_dir="${cfg[base_input_dir]}/RawData/GTFS/"
mkdir -p "${gtfs_dir}"

# Parse GTFS data
echo "Parsing GTFS data from ${cfg[gtfs_source]}."
cmake -S "${cfg[parrot_source_dir]}" -B "${cfg[parrot_source_dir]}"/Build/Release -DCMAKE_BUILD_TYPE=Release
cmake --build "${cfg[parrot_source_dir]}"/Build/Release --target Network -j
"${cfg[parrot_source_dir]}"/Build/Release/ULTRA_Runnables/Network parseGTFS \
  "${cfg[gtfs_source]}" \
  "${gtfs_dir}"/"${gtfs_name}".binary

# Convert GTFS data to intermediate format with specified time interval
echo "Converting GTFS data to intermediate format for time interval ${cfg[first_day]} to ${cfg[last_day]}."
cmake --build "${cfg[parrot_source_dir]}"/Build/Release --target Network -j
"${cfg[parrot_source_dir]}"/Build/Release/ULTRA_Runnables/Network gtfsToIntermediate \
  "${gtfs_dir}"/"${gtfs_name}".binary \
  "${cfg[first_day]}" \
  "${cfg[last_day]}" \
  true true \
  "${gtfs_dir}"/"${gtfs_name}"_intermediate.binary

rm "${gtfs_dir}"/"${gtfs_name}".binary



