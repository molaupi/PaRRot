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
[ -z "${cfg[boundary_outer]}" ] && echo "boundary_inner is not set in the config file." && exit 1


gtfs_name=$(basename "${cfg[gtfs_source]}")
gtfs_dir="${cfg[base_input_dir]}/RawData/GTFS/"

# Create output directory if it doesn't exist
pt_dir="${cfg[base_input_dir]}"/ULTRA/"${cfg[instance_name]}"/
mkdir -p "${pt_dir}"

# shellcheck disable=SC2046
read -r lon_min lon_max lat_min lat_max <<< $(bash "${cfg[parrot_source_dir]}"/RawData/get_bounding_box_from_poly.sh "${cfg[boundary_outer]}")

# Apply bounding box to intermediate of whole GTFS
echo "Applying bounding box to GTFS."
cmake --build "${cfg[parrot_source_dir]}"/Build/Release --target Network -j
"${cfg[parrot_source_dir]}"/Build/Release/ULTRA_Runnables/Network applyCustomBoundingBox \
  "${gtfs_dir}"/"${gtfs_name}"_intermediate.binary \
  "${lon_min}" "${lon_max}" "${lat_min}" "${lat_max}" \
  "${pt_dir}"/intermediate.binary

"${cfg[parrot_source_dir]}"/Build/Release/ULTRA_Runnables/Network reduceGraph \
  "${pt_dir}"/intermediate.binary \
  "${pt_dir}"/intermediate.binary

"${cfg[parrot_source_dir]}"/Build/Release/ULTRA_Runnables/Network reduceToMaximumConnectedComponent \
  "${pt_dir}"/intermediate.binary \
  "${pt_dir}"/intermediate.binary

"${cfg[parrot_source_dir]}"/Build/Release/ULTRA_Runnables/Network reduceGraph \
  "${pt_dir}"/intermediate.binary \
  "${pt_dir}"/intermediate.binary

"${cfg[parrot_source_dir]}"/Build/Release/ULTRA_Runnables/Network makeOneHopTransfers \
  "${pt_dir}"/intermediate.binary \
  86400 \
  "${pt_dir}"/intermediate.binary \
  true

# Load underlying pedestrian network from DIMACS format
echo "Loading underlying pedestrian network for transfer graph."
cmake --build "${cfg[parrot_source_dir]}"/Build/Release --target Network -j
"${cfg[parrot_source_dir]}"/Build/Release/ULTRA_Runnables/Network loadDimacsGraph \
  "${cfg[base_input_dir]}"/KaRRi/Graphs/DIMACS/"${cfg[instance_name]}"_psg \
  "${pt_dir}"/underlying_psg_graph.binary

# Add underlying pedestrian graph as transfer graph.
echo "Adding underlying pedestrian graph as transfer graph."
cmake --build "${cfg[parrot_source_dir]}"/Build/Release --target Network -j
"${cfg[parrot_source_dir]}"/Build/Release/ULTRA_Runnables/Network addGraph \
  "${pt_dir}"/intermediate.binary \
  "${pt_dir}"/underlying_psg_graph.binary \
  "${pt_dir}"/intermediate.binary

"${cfg[parrot_source_dir]}"/Build/Release/ULTRA_Runnables/Network reduceToMaximumConnectedComponent \
  "${pt_dir}"/intermediate.binary \
  "${pt_dir}"/intermediate.binary

"${cfg[parrot_source_dir]}"/Build/Release/ULTRA_Runnables/Network applyMaxTransferSpeed \
  "${pt_dir}"/intermediate.binary \
  5.0 \
  "${pt_dir}"/intermediate.binary

"${cfg[parrot_source_dir]}"/Build/Release/ULTRA_Runnables/Network reduceGraph \
  "${pt_dir}"/intermediate.binary \
  "${pt_dir}"/intermediate.binary

# Convert intermediate to RAPTOR format
echo "Converting intermediate format to RAPTOR format."
cmake --build "${cfg[parrot_source_dir]}"/Build/Release --target Network -j
"${cfg[parrot_source_dir]}"/Build/Release/ULTRA_Runnables/Network intermediateToRAPTOR \
  "${pt_dir}"/intermediate.binary \
  "${pt_dir}"/raptor.binary

# Clean up underlying graph
rm "${pt_dir}"/underlying_psg_graph.binary*