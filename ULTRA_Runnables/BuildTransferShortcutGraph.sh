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
[ -z "${cfg[boundary_outer]}" ] && echo "boundary_inner is not set in the config file." && exit 1


pt_dir="${cfg[base_input_dir]}"/ULTRA/"${cfg[instance_name]}"/

# Compute Core CH
echo "Computing Core CH for stop-to-stop transfer graph."
corech_dir="${pt_dir}"/CoreCH/
mkdir -p "${corech_dir}"
cmake -S "${cfg[parrot_source_dir]}" -B "${cfg[parrot_source_dir]}"/Build/Release -DCMAKE_BUILD_TYPE=Release
cmake --build "${cfg[parrot_source_dir]}"/Build/Release --target ULTRA -j
"${cfg[parrot_source_dir]}"/Build/Release/ULTRA_Runnables/ULTRA buildCoreCH \
  "${pt_dir}"/raptor.binary \
  "${corech_dir}"/raptor.binary.corech.order \
  "${corech_dir}"/raptor.binary.corech \
  "${corech_dir}"/raptor.binary

# Compute more-criteria stop-to-stop transfer shortcuts
echo "Computing more-criteria stop-to-stop transfer shortcuts. This can take hours for large instances."

if [ -z "${cfg[num_threads]}" ]; then
  echo "num_threads is not set in the config file. Using max available threads."
  cfg[num_threads]="max" # If user did not specify number of threads, use max available
fi

shortcuts_dir="${pt_dir}"/ComputeMcStopToStopShortcuts/
mkdir -p "${shortcuts_dir}"
cmake --build "${cfg[parrot_source_dir]}"/Build/Release --target ULTRA -j
"${cfg[parrot_source_dir]}"/Build/Release/ULTRA_Runnables/ULTRA computeMcStopToStopShortcuts \
    "${corech_dir}"/raptor.binary \
    "${shortcuts_dir}"/raptor.binary \
    0 3600 true false "${cfg[num_threads]}"

# Cleanup: result of computeMcStopToStopShortcuts is finished input data
mv -f "${shortcuts_dir}"/raptor.binary "${pt_dir}"/raptor.binary
rm -r "${corech_dir}" "${shortcuts_dir}"