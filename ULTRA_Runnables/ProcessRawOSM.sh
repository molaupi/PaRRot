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
[ -z "${cfg[osm_source]}" ] && echo "osm_source is not set in the config file." && exit 1
[ -z "${cfg[boundary_outer]}" ] && echo "boundary_outer is not set in the config file." && exit 1
[ -z "${cfg[base_input_dir]}" ] && echo "base_input_dir is not set in the config file." && exit 1
[ -z "${cfg[instance_name]}" ] && echo "instance_name is not set in the config file." && exit 1

# Set osmfilter executable path
if [ -z "${cfg[osmfilter_executable]}" ]; then
  cfg[osmfilter_executable]=osmfilter # If user did not specify location of osmium executable, assume it is installed on default PATH
fi
if [ -z "${cfg[osmconvert_executable]}" ]; then
  cfg[osmconvert_executable]=osmconvert # If user did not specify location of osmium executable, assume it is installed on default PATH
fi

# Create output directory if it doesn't exist
osm_dir="${cfg[base_input_dir]}/RawData/OSM/"
mkdir -p "${osm_dir}"

# Extract outer area from OSM source and convert to .o5m format
echo "Extracting outer area from OSM source."
${cfg[osmconvert_executable]} "${cfg[osm_source]}" -B="${cfg[boundary_outer]}" -o="${osm_dir}"/"${cfg[instance_name]}"_base.o5m

# Filter OSM source for required highway types
echo "Filtering OSM data to required ways."
${cfg[osmfilter_executable]} "${osm_dir}"/"${cfg[instance_name]}"_base.o5m --parameter-file="${cfg[parrot_source_dir]}"/ULTRA_Runnables/osmfilter_parameters.txt -o="${osm_dir}"/"${cfg[instance_name]}"_base_filtered.o5m

# Convert back to .osm.pbf and clean up
echo "Cleaning up OSM temporary data."
${cfg[osmconvert_executable]} "${osm_dir}"/"${cfg[instance_name]}"_base_filtered.o5m -o="${osm_dir}"/"${cfg[instance_name]}".osm.pbf
rm "${osm_dir}"/"${cfg[instance_name]}"_base.o5m
rm "${osm_dir}"/"${cfg[instance_name]}"_base_filtered.o5m

