#!/bin/bash

# Usage: ./divide_weights.sh input.dimacs output.dimacs

if [ $# -ne 2 ]; then
    echo "Usage: $0 <input_dimacs_file> <output_dimacs_file>"
    exit 1
fi

input="$1"
output="$2"

awk '
  # For edge lines (a u v w): divide weight by 10 and round
  /^a / {
    w = $4 / 10;
    # round to nearest integer (handles negatives correctly)
    if (w < 0) {
      $4 = int(w - 0.5);
    } else {
      $4 = int(w + 0.5);
    }
    print;
    next;
  }
  # Otherwise print unchanged
  { print }
' "$input" > "$output"
