#!/bin/sh
set -eu

if [ "$#" -ne 5 ]; then
    echo "Uso: sh tests/compare_rimls.sh ejecutable visor rimls.ply referencias tolerancia_xy_m" >&2
    exit 2
fi
viewer=$1
assets=$(cd "$2" && pwd)
ply=$3
reference=$4
tolerance=$5
scratch=$(mktemp -d)
pid=
cleanup() {
    if [ -n "$pid" ]; then
        kill "$pid" 2>/dev/null || :
        wait "$pid" 2>/dev/null || :
    fi
    rm -rf -- "$scratch"
}
trap cleanup EXIT

expected=$(jq -er .ply_sha256 "$reference/routes.json")
actual=$(sha256sum "$ply" | cut -d ' ' -f 1)
if [ "$expected" != "$actual" ]; then
    echo "La huella SHA-256 del PLY no coincide con la referencia" >&2
    exit 1
fi
for name in assets index.html app.js styles.css catalog.json; do
    ln -s "$assets/$name" "$scratch/$name"
done

"$viewer" --root "$scratch" --ply "$ply" --port 0 > "$scratch/server.log" 2>&1 &
pid=$!
url=
for attempt in $(seq 1 250); do
    url=$(sed -n 's/^Abre \(http:.*\)$/\1/p' "$scratch/server.log")
    if [ -n "$url" ]; then break; fi
    if ! kill -0 "$pid" 2>/dev/null; then cat "$scratch/server.log"; exit 1; fi
    sleep .05
done
[ -n "$url" ]

count=$(jq -er '.routes | length' "$reference/routes.json")
[ "$count" -gt 0 ]
failed=0
for i in $(seq 0 $((count - 1))); do
    jq -c ".routes[$i].payload" "$reference/routes.json" > "$scratch/input.json"
    curl --silent --show-error --fail --max-time 120 -H 'Content-Type: application/json' \
        --data-binary "@$scratch/input.json" "$url/api/evaluate" > "$scratch/current.json"
    jq -nc --argjson i "$i" --argjson limit "$tolerance" --slurpfile reference "$reference/metrics.json" \
        --slurpfile current "$scratch/current.json" '
        $reference[0].clean[$i] as $a | $current[0] as $b |
        ($b.frames | map(.position[0:2])) as $xy |
        (if ($xy|length) == 0 or ($xy|length) != ($a.xy|length) then null else
            [range(0; $xy|length) as $k |
                ($xy[$k][0] - $a.xy[$k][0]) as $dx | ($xy[$k][1] - $a.xy[$k][1]) as $dy |
                ($dx*$dx + $dy*$dy | sqrt)] | max end) as $error |
        ($a.status == $b.status and $a.samples == $b.samples and $a.warnings == $b.warnings) as $same |
        {id:$a.id, status:$b.status, samples:$b.samples, same_status_samples_warnings:$same,
         max_xy_m:$error, ok:($same and $error != null and $error <= $limit)}
    ' > "$scratch/comparison.json"
    cat "$scratch/comparison.json"
    if ! jq -e .ok "$scratch/comparison.json" > /dev/null; then failed=1; fi
done
exit "$failed"
