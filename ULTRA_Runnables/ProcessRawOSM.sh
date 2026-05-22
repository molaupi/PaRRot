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
[ -z "${cfg[osm_source]}" ] && echo "osm_source is not set in the config file." && exit 1
[ -z "${cfg[boundary_outer]}" ] && echo "boundary_outer is not set in the config file." && exit 1
[ -z "${cfg[base_input_dir]}" ] && echo "base_input_dir is not set in the config file." && exit 1
[ -z "${cfg[instance_name]}" ] && echo "instance_name is not set in the config file." && exit 1

# Set osmium executable path
if [ -z "${cfg[osmium_executable]}" ]; then
  cfg[osmium_executable]=osmium # If user did not specify location of osmium executable, assume it is installed on default PATH
fi

# Create output directory if it doesn't exist
osm_dir="${cfg[base_input_dir]}/RawData/OSM/"
mkdir -p "${osm_dir}"

# Filter OSM source for required highway types
${cfg[osmium_executable]} tags-filter -o "${osm_dir}"/"${cfg[instance_name]}"_base_filtered.osm.pbf "${cfg[osm_source]}" w/highway=motorway,motorway_link,trunk,trunk_link,primary,primary_link,secondary,secondary_link,tertiary,tertiary_link,unclassified,residential,living_street,service,pedestrian,track,footway,bridleway,cycleway,steps,path #,corridor
${cfg[osmium_executable]} sort -o "${osm_dir}"/"${cfg[instance_name]}"_base_filtered_sorted.osm.pbf "${osm_dir}"/"${cfg[instance_name]}"_base_filtered.osm.pbf
mv -f "${osm_dir}"/"${cfg[instance_name]}"_base_filtered_sorted.osm.pbf "${osm_dir}"/"${cfg[instance_name]}"_base_filtered.osm.pbf

# Extract outer area from filtered OSM source
${cfg[osmium_executable]} extract -p "${cfg[boundary_outer]}" -o "${osm_dir}"/"${cfg[instance_name]}".osm.pbf "${osm_dir}"/"${cfg[instance_name]}"_base_filtered.osm.pbf

# Clean up intermediate file
rm "${osm_dir}"/"${cfg[instance_name]}"_base_filtered.osm.pbf

