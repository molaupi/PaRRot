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
[ -z "${cfg[boundary_inner]}" ] && echo "boundary_inner is not set in the config file." && exit 1
[ -z "${cfg[boundary_outer]}" ] && echo "boundary_outer is not set in the config file." && exit 1
[ -z "${cfg[base_input_dir]}" ] && echo "base_input_dir is not set in the config file." && exit 1
[ -z "${cfg[instance_name]}" ] && echo "instance_name is not set in the config file." && exit 1

# If user did not specify location of osmium executable in config, assume it is installed on default PATH
if [[ ! -v cfg[osmium_executable] ]] || [[ -z "${cfg[osmium_executable]}" ]]; then
    cfg[osmium_executable]="osmium"
fi

# Create output directory if it doesn't exist
osm_dir="${cfg[base_input_dir]}/RawData/OSM/"
mkdir -p "${osm_dir}"


# Extract network containing all roads, streets and pathways accessible to motor traffic and/or pedestrian traffic from OSM source
echo "Filtering OSM source to relevant highways (this may take some time)."
${cfg[osmium_executable]} tags-filter -o "${osm_dir}"/tmp_osm_source_AllHighways.osm.pbf "${cfg[osm_source]}" w/highway=motorway,motorway_link,trunk,trunk_link,primary,primary_link,secondary,secondary_link,tertiary,tertiary_link,unclassified,residential,living_street,service,pedestrian,track,footway,bridleway,cycleway,steps,path #,corridor
${cfg[osmium_executable]} sort -o "${osm_dir}"/tmp_osm_source_AllHighways_Sorted.osm.pbf "${osm_dir}"/tmp_osm_source_AllHighways.osm.pbf
mv -f "${osm_dir}"/tmp_osm_source_AllHighways_Sorted.osm.pbf "${osm_dir}"/tmp_osm_source_AllHighways.osm.pbf

# Extract network containing only roads meant for taxi vehicles from OSM source
${cfg[osmium_executable]} tags-filter -o "${osm_dir}"/tmp_osm_source_VehicleHighways.osm.pbf "${osm_dir}"/tmp_osm_source_AllHighways.osm.pbf w/highway=motorway,motorway_link,trunk,trunk_link,primary,primary_link,secondary,secondary_link,tertiary,tertiary_link,unclassified,residential,living_street,service

# Extract network containing only ways accessible for pedestrians.
${cfg[osmium_executable]} tags-filter -o "${osm_dir}"/tmp_osm_source_PedestrianHighways.osm.pbf "${osm_dir}"/tmp_osm_source_AllHighways.osm.pbf w/highway=tertiary,tertiary_link,unclassified,residential,living_street,service,pedestrian,track,footway,bridleway,cycleway,steps,path #,corridor


echo "Extracting OSM networks for area of operation."
# Create pedestrian network for full area (can be used to extract pedestrian graph for public transport transfers in full area)
${cfg[osmium_executable]} extract -p "${cfg[boundary_outer]}" -o "${osm_dir}"/"${cfg[instance_name]}"_PedestrianOuter.osm.pbf "${osm_dir}"/tmp_osm_source_PedestrianHighways.osm.pbf

# Create network with full inner area and filtered outer area without pedestrian paths (can be used for RP routing for requests in inner area)

## For inner area take all roads and pedestrian pathways
${cfg[osmium_executable]} extract -p "${cfg[boundary_inner]}" -o "${osm_dir}"/"${cfg[instance_name]}"_tmp_FullInner.osm.pbf "${osm_dir}"/tmp_osm_source_AllHighways.osm.pbf

## For outer area take only roads
${cfg[osmium_executable]} extract -p "${cfg[boundary_outer]}" -o "${osm_dir}"/"${cfg[instance_name]}"_tmp_VehicleOuter.osm.pbf "${osm_dir}"/tmp_osm_source_VehicleHighways.osm.pbf

## Merge to a shared network
${cfg[osmium_executable]} merge -o "${osm_dir}"/"${cfg[instance_name]}".osm.pbf "${osm_dir}"/"${cfg[instance_name]}"_tmp_FullInner.osm.pbf "${osm_dir}"/"${cfg[instance_name]}"_tmp_VehicleOuter.osm.pbf

# Cleanup
rm "${osm_dir}"/"${cfg[instance_name]}"_tmp_FullInner.osm.pbf "${osm_dir}"/"${cfg[instance_name]}"_tmp_VehicleOuter.osm.pbf
rm "${osm_dir}"/tmp_osm_source_AllHighways.osm.pbf "${osm_dir}"/tmp_osm_source_VehicleHighways.osm.pbf "${osm_dir}"/tmp_osm_source_PedestrianHighways.osm.pbf

## Extract outer area from OSM source and convert to .o5m format
#echo "Extracting outer area from OSM source."
#${cfg[osmconvert_executable]} "${cfg[osm_source]}" -B="${cfg[boundary_outer]}" -o="${osm_dir}"/"${cfg[instance_name]}"_base.o5m
#
## Filter OSM source for required highway types
#echo "Filtering OSM data to required ways."
#${cfg[osmfilter_executable]} "${osm_dir}"/"${cfg[instance_name]}"_base.o5m --parameter-file="${cfg[parrot_source_dir]}"/ULTRA_Runnables/osmfilter_parameters.txt -o="${osm_dir}"/"${cfg[instance_name]}"_base_filtered.o5m
#
## Convert back to .osm.pbf and clean up
#echo "Cleaning up OSM temporary data."
#${cfg[osmconvert_executable]} "${osm_dir}"/"${cfg[instance_name]}"_base_filtered.o5m -o="${osm_dir}"/"${cfg[instance_name]}".osm.pbf
#rm "${osm_dir}"/"${cfg[instance_name]}"_base.o5m
#rm "${osm_dir}"/"${cfg[instance_name]}"_base_filtered.o5m

