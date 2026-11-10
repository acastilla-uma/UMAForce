#!/bin/sh
set -eu

viewer=$1
assets=$2
curl=$3
jq=$4
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

for name in assets index.html app.js styles.css catalog.json; do
    ln -s "$assets/$name" "$scratch/$name"
done
awk 'BEGIN {
    print "ply\nformat ascii 1.0\nelement vertex 6561\nproperty float x\nproperty float y\nproperty float z\nend_header";
    for (y = -40; y <= 40; ++y) for (x = -40; x <= 40; ++x) print x * .2, y * .2, 0
}' > "$scratch/flat.ply"

"$viewer" --root "$scratch" --port 0 > "$scratch/server.log" 2>&1 &
pid=$!
url=
for attempt in $(seq 1 100); do
    url=$(sed -n 's/^Abre \(http:.*\)$/\1/p' "$scratch/server.log")
    if [ -n "$url" ]; then break; fi
    if ! kill -0 "$pid" 2>/dev/null; then cat "$scratch/server.log"; exit 1; fi
    sleep .05
done
[ -n "$url" ]

get() { "$curl" --silent --show-error --fail --max-time 15 "$url$1"; }
post() {
    "$curl" --silent --show-error --fail --max-time 15 -H 'Content-Type: application/json' --data "$2" "$url$1"
}
upload() {
    "$curl" --silent --show-error --fail --max-time 15 -H 'Content-Type: application/octet-stream' --data-binary "@$2" "$url$1"
}

get / | grep -qi '<html'
get /api/info | "$jq" -e '.cloud == null and (.catalog | length == 3)' > /dev/null
[ "$("$curl" --silent --output /dev/null --write-out '%{http_code}' -H 'Origin: https://example.invalid' "$url/api/info")" = 400 ]
upload /api/load "$scratch/flat.ply" | "$jq" -e '.cloud.count == 6561 and (.points | length == 6561)' > /dev/null
post /api/view '{"center":[0,0,0]}' | "$jq" -e '.total > 0 and .radius == 5' > /dev/null
pose=$(post /api/place '{"x":0,"y":0,"yaw":0,"vehicle":"MAN_5t"}')
echo "$pose" | "$jq" -e '.grounded and (.position | length == 3)' > /dev/null
params=$(echo "$pose" | "$jq" -c '{vehicle:"MAN_5t", x:.position[0], y:.position[1], z:.position[2], roll, pitch, yaw, duration:0.04}')
job=$(post /api/simulation/start "$params" | "$jq" -er .job_id)
for attempt in $(seq 1 150); do
    get "/api/simulation/job?job_id=$job" > "$scratch/result.json"
    status=$("$jq" -er .status "$scratch/result.json")
    if [ "$status" = complete ]; then break; fi
    if [ "$status" = error ]; then cat "$scratch/result.json"; exit 1; fi
    sleep .05
done
"$jq" -e '.status == "complete" and .progress.fraction == 1 and .result.samples == 5 and (.result.frames[0].stability | has("ltr"))' "$scratch/result.json" > /dev/null
get /api/export.csv > "$scratch/original.csv"
grep -q 'contact,p0000' "$scratch/original.csv"
"$jq" -c '{project:{vehicle:"MAN_5t",frames:.result.frames,playback_index:0,csv_available:true}}' "$scratch/result.json" > "$scratch/project.json"
post /api/project/export "@$scratch/project.json" > "$scratch/saved.fusionforce"
upload /api/project/import "$scratch/saved.fusionforce" | "$jq" -e '.project.csv_available and .info.cloud.count == 6561' > /dev/null
get /api/export.csv | cmp - "$scratch/original.csv"

route=$(echo "$pose" | "$jq" -c '{vehicle:"MAN_5t",x:0,y:0,z:(.position[2]+0.2),
    until_destination:true,disable_csv:true,route:[0,2]|map({x:.,y:0,z:0,speed:1,acceleration:1})}')
post /api/evaluate "$route" | "$jq" -e '
    .status == "destination_reached" and .destination_reached and .distance_remaining == 0 and
    .frames[-1].position[0] >= 1.5 and .frames[-1].time < 2' > /dev/null
