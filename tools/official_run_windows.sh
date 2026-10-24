#!/usr/bin/env bash
# One run of the OFFICIAL local simulator from Windows (Git Bash + docker.exe, no WSL integration).
# usage: bash tools/official_run_windows.sh <tarball in submission/> <easy|normal|hard|veryhard> <tag>
#   e.g. bash tools/official_run_windows.sh GaneshaOrbit_v4_fast.tar.gz veryhard veryhard_v4_fast
# - installs the archive exactly as uploaded (extracted inside a Linux container: keeps the symlink + modes)
# - uses docker-compose.port8080.yaml (copy of the official compose file, VNC on host port 8080 instead of
#   80, because WSL's wslrelay may hold port 80); simulation behaviour is unchanged
# - logs host CPU every 10 s during the run (nav-stale faults happen when the host CPU is saturated)
# - copies the result folder (without simulation.bag) to tools/results/official_sim_24okt/<tag>/ and
#   moves simulation.bag to the Windows Recycle Bin
set -u
TAR="$1"; LEVEL="$2"; TAG="$3"
SIM_W='C:\Users\POGGY\krpc\krpc7th-simulator'
SIM=/c/Users/POGGY/krpc/krpc7th-simulator
KIBO=/c/Users/POGGY/Downloads/barengpoggy/KiboRPC7
OUT="$KIBO/tools/results/official_sim_24okt/$TAG"
mkdir -p "$OUT"
cd "$SIM" || exit 1
[ -f docker-compose.port8080.yaml ] || sed 's/            - 80:80/            - 8080:80/' docker-compose.yaml > docker-compose.port8080.yaml

MSYS_NO_PATHCONV=1 docker run --rm -v "$SIM_W\\users:/u" -v 'C:\Users\POGGY\Downloads\barengpoggy\KiboRPC7\submission:/sub:ro' \
  debian:trixie-slim sh -c "rm -rf /u/catkin_ws/src && tar -xzf /sub/$TAR -C /u && md5sum /u/catkin_ws/src/krpc_user/scripts/user_template.py && grep -n '\"MOTION_MODE\"' /u/catkin_ws/src/krpc_user/scripts/user_template.py" \
  | tee "$OUT/install.txt"

before=$(ls share/logs | sort)
powershell.exe -NoProfile -Command "\$n=0; while(\$n -lt 90){ \$v=(Get-Counter '\Processor(_Total)\% Processor Time').CounterSamples.CookedValue; (Get-Date -Format 'HH:mm:ss') + ' cpu% ' + [math]::Round(\$v,0) | Out-File -Append -Encoding ascii '$(cygpath -w "$OUT")\host_cpu.log'; Start-Sleep 10; \$n++ }" &
CPU_PID=$!
date '+start %F %T' > "$OUT/run_$LEVEL.log"
MSYS_NO_PATHCONV=1 LEVELS="$LEVEL" docker compose -f docker-compose.port8080.yaml up --abort-on-container-exit --exit-code-from krpc >> "$OUT/run_$LEVEL.log" 2>&1
rc=$?
MSYS_NO_PATHCONV=1 docker compose -f docker-compose.port8080.yaml down >> "$OUT/run_$LEVEL.log" 2>&1
date '+end %F %T' >> "$OUT/run_$LEVEL.log"
kill $CPU_PID 2>/dev/null

new=""
for d in $(ls share/logs | sort); do
  [ "$d" = latest ] && continue
  echo "$before" | grep -qx "$d" || new="$d"
done
echo "compose rc=$rc run dir=$new" | tee -a "$OUT/run_$LEVEL.log"
if [ -n "$new" ]; then
  for f in cheat_sheet.json judge_module_logs user_programs_logs flight_recorder_logs; do
    [ -e "share/logs/$new/$f" ] && cp -r "share/logs/$new/$f" "$OUT/"
  done
  if [ -f "share/logs/$new/simulation.bag" ]; then
    ls -la "share/logs/$new/simulation.bag" >> "$OUT/run_$LEVEL.log"
    BAG_W=$(cygpath -w "$SIM/share/logs/$new/simulation.bag")
    powershell.exe -NoProfile -Command "Add-Type -AssemblyName Microsoft.VisualBasic; [Microsoft.VisualBasic.FileIO.FileSystem]::DeleteFile('$BAG_W','OnlyErrorDialogs','SendToRecycleBin')" \
      && echo "simulation.bag moved to Recycle Bin" | tee -a "$OUT/run_$LEVEL.log"
  fi
fi
