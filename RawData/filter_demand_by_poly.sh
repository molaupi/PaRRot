#!/usr/bin/env bash

# ------------------------------------------------------------
# filter_demand_by_poly.sh
#   Keep only those rows of a demand CSV file whose origin and
#   destination both lie within the area given by an osmosis
#   .poly file. Rows are copied verbatim.
#   Input columns : lon_orig, lat_orig, lon_dest, lat_dest (+ any others)
#   Usage: ./filter_demand_by_poly.sh <input.csv> <area.poly> <output.csv>
# ------------------------------------------------------------

set -euo pipefail

# ---------- Argument validation ----------
if [[ $# -ne 3 ]]; then
    echo "Usage: $0 <input.csv> <area.poly> <output.csv>" >&2
    exit 1
fi

input_file="$1"
polygon_file="$2"
output_file="$3"

# Warn if the output file already exists
if [[ -e "$output_file" ]]; then
    echo "Warning: \"$output_file\" exists and will be overwritten." >&2
fi

# ---------- Filtering ----------
# Point-in-polygon test via ray casting (even-odd rule, so holes marked with '!' are handled
# implicitly). To avoid testing every point against every polygon edge, edges are bucketed
# into horizontal latitude bands and a point is only tested against the edges of its band.
awk -v num_bands=2048 '
# ---------- Process polygon file first (NR == FNR) ----------
NR==FNR {
    if (FNR==1) next                          # name of polygon file
    if (NF==0) next
    if (!in_ring) {
        if ($1=="END") next                   # end of file
        in_ring=1; ring_start=num_verts; next # section header
    }
    if ($1=="END") {                          # end of section: close ring
        add_edge(num_verts-1, ring_start)
        in_ring=0; next
    }
    vx[num_verts]=$1+0; vy[num_verts]=$2+0
    if (num_verts > ring_start) add_edge(num_verts-1, num_verts)
    if (num_verts==0 || vx[num_verts] < lon_min) lon_min=vx[num_verts]
    if (num_verts==0 || vx[num_verts] > lon_max) lon_max=vx[num_verts]
    if (num_verts==0 || vy[num_verts] < lat_min) lat_min=vy[num_verts]
    if (num_verts==0 || vy[num_verts] > lat_max) lat_max=vy[num_verts]
    num_verts++
    next
}

function add_edge(i, j) {
    ex1[num_edges]=vx[i]; ey1[num_edges]=vy[i]
    ex2[num_edges]=vx[j]; ey2[num_edges]=vy[j]
    num_edges++
}

function band_of(y,    b) {
    b=int((y-lat_min)/band_height)
    if (b<0) b=0
    if (b>=num_bands) b=num_bands-1
    return b
}

function build_bands(    e, b, lo, hi) {
    band_height=(lat_max-lat_min)/num_bands
    if (band_height<=0) band_height=1
    for (e=0; e<num_edges; e++) {
        lo=band_of(ey1[e] < ey2[e] ? ey1[e] : ey2[e])
        hi=band_of(ey1[e] < ey2[e] ? ey2[e] : ey1[e])
        for (b=lo; b<=hi; b++) band_edges[b, band_size[b]++]=e
    }
}

function inside(px, py,    b, k, e, c) {
    if (px<lon_min || px>lon_max || py<lat_min || py>lat_max) return 0
    b=band_of(py); c=0
    for (k=0; k<band_size[b]; k++) {
        e=band_edges[b, k]
        if ((ey1[e]>py) != (ey2[e]>py) &&
            px < (ex2[e]-ex1[e])*(py-ey1[e])/(ey2[e]-ey1[e])+ex1[e])
            c=!c
    }
    return c
}

# ---------- Now process demand file ----------
FNR==1 {
    if (num_edges==0) {
        print "Error: no polygon found in", ARGV[1] > "/dev/stderr"
        exit 1
    }
    build_bands()
    n=split($0, h, ",")
    for (i=1; i<=n; i++) {
        sub(/\r$/, "", h[i])
        col[h[i]]=i
    }
    if (!("lon_orig" in col) || !("lat_orig" in col) || !("lon_dest" in col) || !("lat_dest" in col)) {
        print "Error: columns lon_orig, lat_orig, lon_dest, lat_dest required in", FILENAME > "/dev/stderr"
        exit 1
    }
    print
    next
}
{
    split($0, f, ",")
    num_rows++
    if (inside(f[col["lon_orig"]]+0, f[col["lat_orig"]]+0) &&
        inside(f[col["lon_dest"]]+0, f[col["lat_dest"]]+0)) {
        print
        num_kept++
    }
}
END {
    if (num_rows>0)
        printf "Kept %d of %d requests with origin and destination inside the area.\n", num_kept, num_rows > "/dev/stderr"
}
' "$polygon_file" "$input_file" > "$output_file"
