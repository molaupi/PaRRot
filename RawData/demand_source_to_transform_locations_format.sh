#!/usr/bin/env bash

# ------------------------------------------------------------
# transform_requests.sh
#   Convert lon/lat columns to "(lat|lon)" format,
#   rename the header, and write directly to a given output file.
#   Usage: ./transform_requests.sh <input.csv> <output.csv>
# ------------------------------------------------------------

set -euo pipefail

# ---------- Argument validation ----------
if [[ $# -ne 2 ]]; then
    echo "Usage: $0 <input.csv> <output.csv>" >&2
    exit 1
fi

input_file="$1"
output_file="$2"

# Warn if the output file already exists
if [[ -e "$output_file" ]]; then
    echo "Warning: \"$output_file\" exists and will be overwritten." >&2
fi

# ---------- Transformation ----------
awk -F',' -v OFS=',' '
NR==1 {
    # Replace the original header with the new one
    print "origin","destination","req_time"
    next
}
{
    # $1=lon_orig  $2=lat_orig  $3=lon_dest  $4=lat_dest  $5=req_time
    origin      = "(" $2 "|" $1 ")"
    destination = "(" $4 "|" $3 ")"
    print origin, destination, $5
}
' "$input_file" > "$output_file"