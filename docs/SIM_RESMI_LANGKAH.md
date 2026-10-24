# Simulator lokal resmi: hasil cek 23 Okt 2026 dan langkah menjalankannya

**Update 23 Okt 2026, 09:20 WIB: simulator resmi SUDAH dijalankan di keempat level. v3_fast LULUS semua kriteria di Easy, Normal, Hard, dan Very Hard (2 seed).** Dua run pertama (Normal dan Hard) gagal karena **navigasi simulator mati** saat CPU laptop penuh. Itu bukan bug program (§0f), dan run ulang keduanya lulus. Hasil lengkap ada di §0. Bagian §1–§3 di bawah adalah catatan pagi hari (sebelum Docker menyala) dan langkah lewat Ubuntu. Langkah itu masih benar, tetapi ternyata tidak wajib (lihat §0c).

## 0-bis. v4 di simulator resmi (24 Okt 2026)
**Cara baru (tanpa mematikan Ubuntu WSL):** `bash tools/official_run_windows.sh <arsip> <level> <tag>` dari Git Bash. Bedanya dengan §0c:
- memakai salinan compose resmi `docker-compose.port8080.yaml` (VNC di port **8080**), karena `wslrelay` memegang port 80 dan di Ubuntu sedang berjalan pekerjaan lain. Perilaku simulasi tidak berubah
- mencatat CPU host tiap 10 s (`host_cpu.log`)
- menyalin hasil ke `tools/results/official_sim_24okt/<tag>/`
- memindahkan `simulation.bag` ke Recycle Bin

Cek hasil: `python tools/results/official_sim_24okt/check_runs.py tools/results/official_sim_24okt`.

| Run | Hasil | Sebab / catatan |
|---|---|---|
| `veryhard_v4_fast_run1_overload` (01:08 WIB) | **Tidak sah.** 3/12 Lv3, T = 96,93 s, `Illegal: false` | CPU host 100% sepanjang run (beban lain: belasan proses meep/ngspice di WSL, 14 worker Python, container lain). Real-time factor ±0,15, jadi watchdog instance (600 s waktu dinding sejak "Go") menghentikan misi di detik-sim ±97. Persiapan sebelum misi saja makan ±8 menit. Sejak record pertama `localization_history`, IB2 sudah tergeser (0,16; 0,09; 0,15) dan berputar ±130° dari dock, dengan laju putar jauh di atas kemampuan kipas. Pergeseran sebesar itu tidak mungkin dihasilkan program kita dalam beberapa milidetik sejak misi mulai. FAST *fault* 2× (attitude 127°/94°) → SAFE, 3 checkpoint Lv3, lalu `/ib2_user/stop` dari watchdog → safe stop → 1× `mission_finished`. Kondisi start berputar ini direproduksi offline; v4 final sekarang menahan robot dengan `ctl` KEEP_POSE sampai diam sebelum FAST (`test_start_settle_when_spinning`) |

**Aturan:** run dengan real-time factor rendah (CPU host penuh), `navigation stale`, atau robot yang sudah bergerak/berputar sebelum misi = **fault simulator**. Ulangi saat laptop tidak menjalankan beban berat lain.

## 0. Hasil run simulator resmi (23 Okt 2026)
### 0a. Hasil, 4 level
Program: `submission/GaneshaOrbit_v3_fast.tar.gz` apa adanya. MD5 `user_template.py` di dalam simulator = `f2eb3cb5c56d61ba500536671561d8c7`, `MOTION_MODE` = `fast`. **Tidak ada perubahan kode.**

| Level | `Illegal` | Checkpoint (semua Lv3) | Goal | `duration_sec` | Skor (model rulebook kita*) | Offline (rata-rata 25 seed) | Folder |
|---|---|---|---|---|---|---|---|
| **Easy** | `false` | **6/6** (5 + goal) | `true` | **132,35 s** | 21 × 1,559 = **32,74** | 32,88 | `easy_v3_fast` |
| **Normal** | `false` | **8/8** (7 + goal) | `true` | **155,55 s** | 27 × 1,481 = **40,00** | 40,02 | `normal_v3_fast_run2` |
| **Hard** | `false` | **10/10** (9 + goal) | `true` | **187,80 s** | 33 × 1,374 = **45,34** † | 46,48 | `hard_v3_fast_run2` |
| **Very Hard** (seed 1) | `false` | **12/12** (11 + goal) | `true` | **200,78 s** | 39 × 1,331 = **51,90** | 51,70 (±202 s) | `veryhard_v3_fast` |
| **Very Hard** (seed 2) | `false` | **12/12** (11 + goal) | `true` | **203,42 s** | 39 × 1,322 = **51,56** | 51,70 | `veryhard_v3_fast_seed2` |

\* `judge.py` tidak menghitung skor. Angka skor dihitung dengan `offline_sim.score()`: 3 poin per Lv3, +3 bonus kalau semua checkpoint + goal berhasil, dikali 1 + (300 − T)/300. Skrip ringkasan: `tools/results/official_sim_23okt/check_runs.py`.
† Hard kehilangan ±10 s karena container `krpc-user` (program kita **dan** `krpc_api_node` resmi) membeku ±7,8 s waktu-sim setelah lapor cp 2 (CPU laptop penuh, §0f). Tanpa kejadian itu, T ≈ 178 s dan skor ≈ 46,4, sesuai offline.

**Run yang gagal (disimpan sebagai bukti, bukan bug program):**
| Run | Hasil | Sebab |
|---|---|---|
| `normal_v3_fast` (run 1) | `Illegal: false`, 0/8, goal `false`, 31,63 s → skor 0 | Navigasi simulator mati di t_sim = 34,1 s (21 s setelah mulai), di tengah leg pertama |
| `hard_v3_fast` (run 1) | `Illegal: false`, 8/10 Lv3, goal `false`, 157,69 s → ±35,38 | Navigasi simulator mati di t_sim = 159,4 s, di leg cp 9 (setelah 8 lapor Lv3) |

Kriteria §2f, semuanya terpenuhi di kelima run yang lulus:
- Setiap leg tercatat `leg -> cp N (fast)` dan diikuti `REPORT … (Lv3 held (fast))`. Rute VH: 9 → 8 → 10 → 11 → 5 → 4 → 3 → 2 → 1 → 7 → 6 → 12 (prediksi 208,7 s).
- `switching to SAFE`: **0 kali**. Tidak ada `FAST profile softened`, refine, limit cycle, atau checkpoint yang ditinggalkan. Artinya `/ctl/wrench` diterima dan robot bergerak sesuai profil.
- `SAFE STOP (mission finished)` muncul **tepat 1×**, sesudah `mission_finished sent … (ok=True)`. Tidak ada `mission crashed` atau `deadline watchdog`.
- Catatan: ±1,7 s kemudian muncul satu baris `SAFE STOP (/ib2_user/stop)`. Ini **normal**: `run.sh` resmi (`kill_user`) memanggil `/ib2_user/stop` setelah misi selesai, dan handler kita menjalankan rutin safe stop yang sama sekali lagi (wrench nol + KEEP_POSE). Tidak berbahaya, dan hasil juri sudah final sebelumnya.

### 0b. Kalibrasi waktu
47 leg FAST dari kelima run yang lulus: rata-rata `actual − pred` = **−0,73 s/leg** (Easy −0,62, Normal −0,60, Hard −0,81, VH −0,77 / −0,77). Angka ini tanpa satu leg Hard yang terkena pembekuan container (+10,5 s). Rentangnya −0,3 sampai −1,3 s. Artinya model waktu sedikit **pesimis** (robot tiba lebih cepat dari prediksi).
- `FAST_OVERHEAD_S` **sengaja tidak diubah** (tetap 0,4), jadi v3 tetap menjadi arsip yang sudah teruji. Alasannya: (1) prediksi yang pesimis membuat penjaga tenggat lebih aman; (2) di VH, rute penuh (208,7 s) masih jauh di bawah batas, jadi koreksi ini tidak menambah skor; (3) Web Simulator berjalan di ARM64 dan belum diukur.
- Kalau nanti data Web Simulator juga menunjukkan selisih negatif, nilai yang wajar adalah 0,0 (bukan −0,3). Ubah lewat v4 dan jalankan ulang semua tes.
- Catatan: angka "fast legs: 48/60" yang dicetak `analyze_results.py` terhitung ganda, karena baris yang sama ada di beberapa file log. Rata-ratanya tetap benar.

### 0c. Cara yang dipakai (tanpa WSL integration, tanpa mengubah setting Docker)
1. Docker Desktop sudah menyala (engine 29.6.1). Ubuntu WSL **tidak dipakai**: semua perintah dijalankan dari Windows (Git Bash + `docker.exe`) di folder kerja `C:\Users\POGGY\krpc\krpc7th-simulator`, hasil ekstrak `vendor/krpc7th-simulator.tar.gz`.
2. **Build:** `MSYS_NO_PATHCONV=1 bash ./build.sh`. Total ±28 menit (07:43 → 08:11). Image `krpc/intball-simulator:latest` dan `krpc/krpc-replay:latest`, masing-masing 8,86 GB di disk (2,43 GB terkompresi). Build **gagal 2×** karena DNS host putus sesaat (`Temporary failure resolving 'mirrors.ubuntu.com'` / `'packages.ros.org'`). Jalankan ulang saja: layer yang sudah jadi tersimpan di cache. Log: `tools/results/official_sim_23okt/build_try1.log`, `build_try2.log`, `build.log`.
3. **Pasang program:** arsip berisi symlink absolut `catkin_ws/src/CMakeLists.txt -> /opt/ros/...`, dan symlink itu tidak bisa dibuat oleh `tar` Windows. Solusinya, ekstrak dari dalam container Linux supaya symlink dan mode file tetap persis:
   ```
   docker run --rm -v "C:\Users\POGGY\krpc\krpc7th-simulator\users:/u" -v "<KiboRPC7>\submission:/sub:ro" debian:trixie-slim sh -c "rm -rf /u/catkin_ws/src && tar -xzf /sub/GaneshaOrbit_v3_fast.tar.gz -C /u"
   ```
4. **Port 80:** layanan web di dalam Ubuntu WSL ikut memakai port 80 lewat `wslrelay`, jadi Ubuntu dihentikan dulu (`wsl --terminate Ubuntu`) sebelum menjalankan simulasi.
5. **Run:** `MSYS_NO_PATHCONV=1 LEVELS=veryhard bash ./simulation.sh`. Satu run ±6–7 menit wall-clock (start Gazebo ±1 menit, misi ±3,5 menit, simpan log ±2 menit). `simulation.bag` berukuran ±1,6 GB per run dan **tidak** disalin ke repo.

### 0d. File hasil
`tools/results/official_sim_23okt/`: satu folder per run (`easy_v3_fast`, `normal_v3_fast`, `normal_v3_fast_run2`, `hard_v3_fast`, `hard_v3_fast_run2`, `veryhard_v3_fast`, `veryhard_v3_fast_seed2`). Isinya `judge_module_logs/result.json`, `judge.log`, `game.log`, `system.log`, `user_programs_logs/user_program_output.log`, `cheat_sheet.json`, `flight_recorder_logs/`, dan output konsol `run_*.log`. Ditambah:
- log build
- `nav_loss_evidence.txt` (bukti §0f)
- `bag_nav.py` (analisis rosbag)
- `check_runs.py` (tabel §0a)
- `host_cpu_during_veryhard_seed2.log`

Rosbag (`simulation.bag`, 0,3–1,6 GB per run) tetap di `C:\Users\POGGY\krpc\krpc7th-simulator\share\logs\<timestamp>\`.

### 0e. Belum diuji
Batch banyak seed per level, v3_safe (tidak perlu, karena FAST tidak pernah jatuh ke SAFE di run yang sehat), dan Web Simulator ARM64 (butuh ID/PW).

### 0f. Kenapa Normal dan Hard run 1 gagal: navigasi simulator mati (bukan bug program)
- **Gejala di log program:** `FAST profile softened (fault: navigation stale)` → `switching to SAFE mode … navigation stale (robot did not respond to /ctl/wrench)`. Setelah itu tiap goal `ctl` langsung `PREEMPTED`. Checkpoint sisanya **tidak dilapor** (jadi tidak pernah FAIL), lalu `mission_finished` dan `SAFE STOP`.
- **Bukti dari rosbag:** `/sensor_fusion/navigation` **dan** `/nav/true/navigation` (posisi *ground truth*) berhenti selamanya di detik yang sama, sementara `/clock` dan `/ctl/status` terus jalan. Normal: berhenti di 34,08 s, clock sampai 46,7 s. Hard: berhenti di 159,4 s, clock sampai 171,8 s. Sebelumnya tidak ada jeda > 0,3 s. `system.log` mencatat `ctl`: `Navigation message timed out`.
- **Sebabnya:** plugin Gazebo `nav.cpp` mempublikasikan kedua topik itu dari satu `ros::Timer` one-shot yang dipasang ulang di setiap callback. Kalau timer itu berhenti, navigasi mati total. Tidak ada input dari program kita ke plugin itu.
- **Pemicunya:** CPU laptop penuh. Saat itu CPU laptop ±100% karena beban lain di luar simulator; container `krpc` sendiri hanya memakai ±1,4 core. Real-time factor turun ke 0,4–0,6 (run VH pertama: 0,84). Run ulang di beban yang sama tidak mengalaminya, jadi kejadiannya acak.
- **Reaksi program sudah tepat.** Tanpa navigasi, robot tidak bisa digerakkan oleh siapa pun. Selesai cepat menyimpan poin yang sudah dilapor dan memaksimalkan pengali waktu (Hard run 1 ±35,4).
  - Satu-satunya catatan: akhiran `(robot did not respond to /ctl/wrench)` menyesatkan. Retry me-reset pose awal leg, jadi "tidak bergerak" selalu benar. Ini kosmetik dan tidak diubah.
- **Saran:** jalankan simulator lokal saat laptop tidak sedang dipakai untuk pekerjaan berat lain. Kalau muncul `navigation stale`, cek dulu `/nav/true/navigation` di bag (`bag_nav.py`) sebelum mencurigai kode.

## 1. Hasil cek (23 Okt 2026 pagi, hanya membaca, sebelum §0)
| Hal | Hasil |
|---|---|
| `wsl.exe` | Ada. Default version 2 |
| Distro WSL | `Ubuntu` (Ubuntu 24.04.4 LTS, WSL 2, status *Stopped*), `docker-desktop` (WSL 2, *Stopped*) |
| Docker CLI Windows | `C:\Program Files\Docker\Docker\resources\bin\docker.exe`, versi **29.6.1** |
| Docker daemon | **Mati.** `docker info` gagal: `failed to connect to the docker API at npipe:////./pipe/dockerDesktopLinuxEngine`. Layanan `com.docker.service` = *Stopped* (StartType *Manual*). Proses Docker Desktop tidak berjalan |
| Docker di dalam Ubuntu | `/usr/bin/docker` **tidak ada**. `docker` di Ubuntu menunjuk ke `docker.exe` Windows lewat `/mnt/c/...`. Artinya **WSL integration untuk Ubuntu belum aktif** |
| Folder kerja `~/krpc` di Ubuntu | Belum ada (simulator belum pernah diekstrak/di-build) |
| Sumber daya Ubuntu | Disk ext4 952 GB kosong, RAM 15 GB, 16 thread. Cukup (Manual §2.2.1: RAM ≥8 GB, disk ≥20 GB) |
| `wslpath` | `wslpath 'C:\Users\POGGY\Downloads\barengpoggy\KiboRPC7'` → `/mnt/c/Users/POGGY/Downloads/barengpoggy/KiboRPC7` ✅ (dicek) |

**Blocker, urut:**
1. Docker Desktop belum dinyalakan, jadi daemon mati. Menyalakannya mengubah status sistem, dan itu di luar izin sesi ini.
2. WSL integration Ubuntu belum aktif (Settings → Resources → WSL integration).
3. Image `krpc/intball-simulator:latest` belum ada. `build.sh` mengunduh dari internet:
   - Docker Hub: `dorowu/ubuntu-desktop-lxde-vnc:bionic`. Pull anonim sudah cukup, tidak perlu login. Kalau kena batas pull, `docker login` dengan akun Docker Hub gratis.
   - `packages.ros.org` (ROS Melodic), `raw.githubusercontent.com` (kunci ROS), `packages.osrfoundation.org` (Gazebo 9), PyPI (`pip3 install ...`)
   - `github.com/jaxa/int-ball2_simulator` (branch `melodic`, `git clone`), serta source OpenCV 3.3.1 + contrib dari GitHub, yang **dikompilasi** di dalam build. Bagian ini yang paling lama
4. Tidak ada blocker dari sisi program. v3 lolos semua tes offline (lihat `00-STATUS…` §0).

## 2. Langkah menjalankan v3_fast satu kali (kamu sendiri)
**Batas waktu saran:** kalau build gagal 2× dengan error jaringan yang sama, berhenti dan kirim `~/krpc/build.log`.

### 2a. Nyalakan Docker (Windows)
1. Buka **Docker Desktop** dari Start menu. Tunggu sampai status kiri bawah **Engine running**.
2. **Settings → Resources → WSL integration** → aktifkan **Ubuntu** → *Apply & restart*.
3. Buka terminal **Ubuntu**, lalu cek:
```bash
which docker          # harus /usr/bin/docker (bukan /mnt/c/...)
docker info | head -5 # tidak boleh error
docker run --rm hello-world
```

### 2b. Siapkan simulator + program v3_fast (Ubuntu)
```bash
KIBO="$(wslpath 'C:\Users\POGGY\Downloads\barengpoggy\KiboRPC7')"
mkdir -p ~/krpc && cd ~/krpc
tar -xzf "$KIBO/vendor/krpc7th-simulator.tar.gz"
cd krpc7th-simulator
rm -rf users/catkin_ws/src
tar -xzf "$KIBO/submission/GaneshaOrbit_v3_fast.tar.gz" -C users     # isi arsip = catkin_ws/src/...
md5sum users/catkin_ws/src/krpc_user/scripts/user_template.py
#   harus: f2eb3cb5c56d61ba500536671561d8c7  (v3_fast; tabel MD5 di §3)
grep -n '"MOTION_MODE"' users/catkin_ws/src/krpc_user/scripts/user_template.py   # "fast"
```

### 2c. Build (sekali, lama)
```bash
cd ~/krpc/krpc7th-simulator
bash ./build.sh 2>&1 | tee ~/krpc/build.log
docker images | grep krpc     # krpc/intball-simulator dan krpc/krpc-replay harus ada (±6,3 GB)
```

### 2d. Jalankan satu level
```bash
cd ~/krpc/krpc7th-simulator
LEVELS=veryhard bash ./simulation.sh 2>&1 | tee ~/krpc/run_veryhard.log
```
- Tonton di http://localhost. Simulasi berhenti sendiri setelah misi selesai (atau tekan Ctrl+C).
- Hasil: `share/logs/latest/result.json` dan `share/logs/latest/user_programs_logs/`.

### 2e. Cek hasil (salin-tempel)
```bash
cd ~/krpc/krpc7th-simulator
L=share/logs/latest
grep -rh "\[GaneshaOrbit\]" $L/user_programs_logs | grep -E "leg -> cp [0-9]+ \(fast\)" | head -20
grep -rh "switching to SAFE" $L/user_programs_logs && echo "!! FAST jatuh ke SAFE" || echo "OK: tidak ada switching to SAFE"
grep -rh "SAFE STOP\|mission crashed\|deadline watchdog" $L/user_programs_logs
python3 - <<'EOF'
import json
d = json.load(open("share/logs/latest/result.json"))
jr = d.get("JudgeReport", {})
cps = jr.get("checkpoint_detail", [])
print("Illegal       :", d.get("Illegal"))
print("duration_sec  :", (jr.get("mission") or {}).get("duration_sec"))
print("checkpoints   :", sum(1 for c in cps if c.get("success")), "/", len(cps), "passed;",
      sum(1 for c in cps if c.get("level") == 3), "at Lv3")
print("levels        :", [c.get("level") for c in cps])
print("goal.success  :", (jr.get("goal") or {}).get("success"))
EOF
python3 "$KIBO/tools/analyze_results.py" share/logs/latest
```

### 2f. Kriteria lulus
| Cek | Harus |
|---|---|
| Log program | Ada `leg -> cp N (fast)` untuk tiap checkpoint, disusul `REPORT … (Lv3 held (fast))` |
| Log program | **Tidak ada** `switching to SAFE mode` |
| Log program | `SAFE STOP (mission finished)` muncul **satu kali**, sesudah `mission_finished sent`. **Tidak ada** `mission crashed` atau `deadline watchdog` |
| `result.json` | `Illegal: false`, semua `level == 3`, `goal.success == true` |
| Waktu | Very Hard sekitar 200 s (offline: 202 s). Selisih per leg (`actual − pred`) dimasukkan ke `FAST_OVERHEAD_S` |

Kalau ada yang gagal: simpan `~/krpc/run_veryhard.log` + folder `share/logs/latest/`, lalu jalankan ulang dengan `GaneshaOrbit_v3_safe.tar.gz` (ulangi 2b dengan nama file itu, tanpa build ulang) sebagai pembanding.

## 3. MD5 `user_template.py` di dalam arsip (23 Okt)
| Arsip | MD5 `catkin_ws/src/krpc_user/scripts/user_template.py` |
|---|---|
| `GaneshaOrbit_v3_fast.tar.gz` (MD5 arsip `35eb74af0f26d4d81e54bf0262f2d44b`) | `f2eb3cb5c56d61ba500536671561d8c7` (sama persis dengan `krpc_user/scripts/user_template.py` di repo) |
| `GaneshaOrbit_v3_safe.tar.gz` (MD5 arsip `cf82cad1f052fd29de169b874f2346f6`) | `57dfd629002458eb300e169356565cb3` (repo + `"MOTION_MODE": "safe"`) |
