#!/usr/bin/env bash
# ------------------------------------------------------------
# merge_origin_reqtime.sh
#   Merge two CSVs row‑wise:
#   * file 1 : columns   origin,destination
#   * file 2 : column    req_time  (may appear anywhere)
#   Output : origin,destination,req_time
#   Usage: ./merge_origin_reqtime.sh <file1.csv> <file2.csv> <output.csv>
# ------------------------------------------------------------

set -euo pipefail

# ---------- Argument check ----------
if [[ $# -ne 3 ]]; then
    echo "Usage: $0 <file1.csv> <file2.csv> <output.csv>" >&2
    exit 1
fi

file1="$1"   # contains origin,destination
file2="$2"   # contains req_time + other columns
outfile="$3"

# Optional warning if output already exists
if [[ -e "$outfile" ]]; then
    echo "Warning: \"$outfile\" will be overwritten." >&2
fi

awk -F',' -v OFS=',' '
# ---------- Process file2 first (NR == FNR) ----------
NR==FNR {
    if (NR==1) {                     # header line of file2
        for (i=1; i<=NF; i++) {
            if ($i == "req_time") {
                rt_col = i
                break
            }
        }
        if (!rt_col) {
            print "Error: column \"req_time\" not found in", FILENAME > "/dev/stderr"
            exit 1
        }
    } else {
        rt_vals[NR] = $rt_col         # store req_time for each row
    }
    next
}
# ---------- Now process file1 ----------
FNR==1 {
    # write the new header
    print "origin","destination","req_time"
    next
}
{
    # $1 = origin, $2 = destination in file1
    # The corresponding req_time is rt_vals[FNR] (same row number)
    print $1,$2,rt_vals[FNR]
}
' "$file2" "$file1" > "$outfile"