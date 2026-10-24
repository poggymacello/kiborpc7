#!/bin/bash
# Run the OFFICIAL local simulator many times and collect result.json files.
# Run inside Ubuntu (WSL2) from the krpc7th-simulator directory, after `bash ./build.sh`.
#
#   bash /path/to/KiboRPC7/tools/batch_local_sim.sh <runs_per_level> "easy normal hard veryhard"
#
# Results: share/batch/<level>_<n>/result.json + user log, then summarize with
#   python3 /path/to/KiboRPC7/tools/analyze_results.py share/batch
set -u
RUNS=${1:-5}
LEVELS=${2:-"easy normal hard veryhard"}
OUT=share/batch
mkdir -p "$OUT"
for lv in $LEVELS; do
  for i in $(seq 1 "$RUNS"); do
    echo "=== level=$lv run=$i ==="
    LEVELS=$lv ENABLE_GRAVITY_MODULE=true docker compose -f docker-compose.yaml up --abort-on-container-exit --exit-code-from krpc > /tmp/krpc_batch.log 2>&1
    docker compose -f docker-compose.yaml down > /dev/null 2>&1
    dst="$OUT/${lv}_${i}"
    mkdir -p "$dst"
    latest=$(readlink -f share/logs/latest)
    cp -r "$latest"/* "$dst"/ 2>/dev/null || true
    cp /tmp/krpc_batch.log "$dst/compose.log"
    if [ -f "$dst/result.json" ]; then
      python3 -c "import json;d=json.load(open('$dst/result.json'));m=d['JudgeReport']['mission'];c=d['JudgeReport']['checkpoint_detail'];print('  duration=%s passed=%d/%d goal=%s' % (m.get('duration_sec'), sum(1 for x in c if x['success']), len(c), d['JudgeReport']['goal']['success']))"
    else
      echo "  no result.json (check $dst/compose.log)"
    fi
  done
done
