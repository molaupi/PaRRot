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
[ -z "${cfg[parrot_source_dir]}" ] && echo "parrot_source_dir is not set in the config file." && exit 1

osm_dir="${cfg[base_input_dir]}/RawData/OSM/"

# Create output directory if it doesn't exist
graphs_dir="${cfg[base_input_dir]}/KaRRi/Graphs/"
mkdir -p "${graphs_dir}"

# Build and run OsmToCarAndPassengerGraph executable
echo "Extracting KaRRi road and pedestrian networks from OSM data."
cmake -S "${cfg[parrot_source_dir]}" -B "${cfg[parrot_source_dir]}"/Build/Release -DCMAKE_BUILD_TYPE=Release
cmake --build "${cfg[parrot_source_dir]}"/Build/Release --target OsmToCarAndPassengerGraph -j
"${cfg[parrot_source_dir]}"/Build/Release/RawData/OsmToCarAndPassengerGraph \
  -a lat_lng osm_node_id length travel_time \
  -psg-mode pedestrian -no-union-nodes -no-veh-on-service \
  -i "${osm_dir}"/"${cfg[instance_name]}".osm.pbf \
  -co "${graphs_dir}"/"${cfg[instance_name]}"_veh.gr.bin \
  -po "${graphs_dir}"/"${cfg[instance_name]}"_psg.gr.bin


# Convert graph to DIMACS format for use as transfer graph in ULTRA
echo "Converting pedestrian graph to DIMACS format for use as transfer graph in ULTRA."
cmake -S "${cfg[parrot_source_dir]}" -B "${cfg[parrot_source_dir]}"/Build/Release -DCMAKE_BUILD_TYPE=Release
cmake --build "${cfg[parrot_source_dir]}"/Build/Release --target ConvertGraph -j
mkdir -p "${graphs_dir}"/DIMACS/
"${cfg[parrot_source_dir]}"/Build/Release/RawData/ConvertGraph \
  -s binary -d dimacs \
  -i "${graphs_dir}"/"${cfg[instance_name]}"_psg \
  -o "${graphs_dir}"/DIMACS/"${cfg[instance_name]}"_psg

# Divide weights by 10 to get travel times in seconds (instead of deciseconds) for use in ULTRA
mv "${graphs_dir}"/DIMACS/"${cfg[instance_name]}"_psg.gr "${graphs_dir}"/DIMACS/"${cfg[instance_name]}"_psg_tenths_of_seconds.gr
bash "${cfg[parrot_source_dir]}"/RawData/divide_weights_by_10_in_dimacs.sh "${graphs_dir}"/DIMACS/"${cfg[instance_name]}"_psg_tenths_of_seconds.gr "${graphs_dir}"/DIMACS/"${cfg[instance_name]}"_psg.gr
rm "${graphs_dir}"/DIMACS/"${cfg[instance_name]}"_psg_tenths_of_seconds.gr


# Construct contraction hierarchy for vehicle and pedestrian networks
chs_dir="${cfg[base_input_dir]}/KaRRi/CHs/"
mkdir -p "${chs_dir}"
echo "Constructing contraction hierarchy for vehicle network."
cmake -S "${cfg[parrot_source_dir]}" -B "${cfg[parrot_source_dir]}"/Build/Release -DCMAKE_BUILD_TYPE=Release
cmake --build "${cfg[parrot_source_dir]}"/Build/Release --target RunP2PAlgo -j
"${cfg[parrot_source_dir]}"/Build/Release/RawData/RunP2PAlgo \
  -g "${graphs_dir}"/"${cfg[instance_name]}"_veh.gr.bin \
  -a CH -o "${chs_dir}"/"${cfg[instance_name]}"_veh_time.ch.bin

echo "Constructing contraction hierarchy for pedestrian network."
"${cfg[parrot_source_dir]}"/Build/Release/RawData/RunP2PAlgo \
  -g "${graphs_dir}"/"${cfg[instance_name]}"_psg.gr.bin \
  -a CH -o "${chs_dir}"/"${cfg[instance_name]}"_psg_time.ch.bin

