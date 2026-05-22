#!/usr/bin/env bash
# ------------------------------------------------------------
# seconds_between_days.sh
#   Compute the number of seconds in the inclusive date span
#   given by two arguments: start_date end_date (format YYYYMMDD).
#   Usage: ./seconds_between_days.sh 20230101 20230110
# ------------------------------------------------------------

set -euo pipefail

# ---------- Argument check ----------
if [[ $# -ne 2 ]]; then
    echo "Usage: $0 <start_YYYYMMDD> <end_YYYYMMDD>" >&2
    exit 1
fi

start_raw="$1"
end_raw="$2"

# ---------- Helper: convert YYYYMMDD to epoch (midnight UTC) ----------
date_to_epoch() {
    local d="$1"
    # GNU date understands YYYYMMDD directly with -d
    date -u -d "${d}" +%s
}

# If you are on macOS with GNU coreutils installed, replace `date` by `gdate`:
# date_to_epoch() { local d="$1"; gdate -u -d "${d}" +%s; }

start_epoch=$(date_to_epoch "$start_raw")
end_epoch=$(date_to_epoch "$end_raw")

# ---------- Compute days (inclusive) ----------
SECONDS_PER_DAY=86400

# Difference in whole days (end – start). Positive result expected.
diff_days=$(( (end_epoch - start_epoch) / SECONDS_PER_DAY + 1 ))

# ---------- Compute total seconds ----------
total_seconds=$(( diff_days * SECONDS_PER_DAY ))

echo "$total_seconds"