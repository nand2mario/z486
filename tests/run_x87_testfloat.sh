#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 3 ]]; then
    echo "usage: $0 TESTFLOAT_GEN VERILATED_TESTBENCH VECTOR_COUNT" >&2
    exit 2
fi

generator=$1
testbench=$2
vector_count=$3

if [[ ! -x $generator ]]; then
    echo "TestFloat generator not found: $generator" >&2
    echo "Build Berkeley TestFloat 3e or set TESTFLOAT_GEN=<path>." >&2
    exit 1
fi

tmpdir=$(mktemp -d)
trap 'rm -rf "$tmpdir"' EXIT

operations=(add sub mul div sqrt)
round_options=(rnear_even rmin rmax rminMag)

for round in "${!round_options[@]}"; do
    for op in "${!operations[@]}"; do
        name=${operations[$op]}
        vectors="$tmpdir/${name}_${round}.txt"
        "$generator" -seed 1 -level 1 "-${round_options[$round]}" \
            "f64_${name}" | sed -n "1,${vector_count}p" > "$vectors"
        "$testbench" "+VECTORS=$vectors" "+OP=$op" "+ROUND=$round"
    done

    vectors="$tmpdir/round_${round}.txt"
    "$generator" -seed 1 -level 1 -exact \
        "-${round_options[$round]}" f64_roundToInt \
        | sed -n "1,${vector_count}p" > "$vectors"
    "$testbench" "+VECTORS=$vectors" "+OP=5" "+ROUND=$round"

    for bits in 32 64; do
        vectors="$tmpdir/i${bits}_${round}.txt"
        "$generator" -seed 1 -level 1 -exact \
            "-${round_options[$round]}" "f64_to_i${bits}" \
            | sed -n "1,${vector_count}p" > "$vectors"
        op=$((bits == 32 ? 6 : 7))
        "$testbench" "+VECTORS=$vectors" "+OP=$op" "+ROUND=$round"
    done
done
