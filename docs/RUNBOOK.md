# RUNBOOK — Kibo-RPC ke-7 (tim Ganesha Orbit)
Langkah kerja dari nol sampai submit. Semua perintah sudah diuji kecuali yang ditandai **[di laptopmu]**: langkah itu butuh hak admin/WSL2 yang tidak bisa saya jalankan dari sini.

---

## 0. Isi folder `KiboRPC7/`
| Path | Isi |
|---|---|
| `krpc_user/` | **Paket yang dikumpulkan.** `scripts/user_template.py` = program misi. CMakeLists, package.xml, dan launch file **tidak diubah** (sesuai aturan) |
| `submission/GaneshaOrbit_v4_fast.tar.gz` | **Arsip utama untuk upload** (24 Okt; lihat `JURI_90/CARA_KIRIM.md`). v3 + pembagian kipas per leg + *reference governor* + opsi ArUco (mati) |
| `submission/GaneshaOrbit_v4_safe.tar.gz` | Cadangan v4 (mode SAFE, hanya `ctl` bawaan) |
| `submission/GaneshaOrbit_v3_fast.tar.gz` | Versi 23 Okt, **sudah lulus simulator resmi di 4 level**. Mode FAST + pengaman osilasi (v2) + *safe stop* (v3): kalau program crash, macet, atau dihentikan, FAST dimatikan, wrench nol dikirim 3×, `ctl` diminta KEEP_POSE, baru `mission_finished`. Cadangan pertama kalau v4 bermasalah di Web Simulator |
| `submission/GaneshaOrbit_v3_safe.tar.gz` | Cadangan (mode SAFE, hanya `ctl` bawaan), dengan safe stop v3 yang sama |
| `submission/GaneshaOrbit_v2_*.tar.gz`, `_v1_*` | Versi 20 Okt dan 10 Okt, disimpan untuk perbandingan |
| `tools/official_run_windows.sh` | v4: satu run simulator resmi dari Windows (Git Bash + `docker.exe`), VNC di port 8080, log CPU host, bag ke Recycle Bin |
| `tools/offline_sim.py` | Simulator Monte Carlo offline (tanpa Docker) |
| `tools/make_submission.py` | Membuat arsip submit persis seperti `packing.sh` resmi |
| `tools/batch_local_sim.sh` | Menjalankan simulator lokal resmi berkali-kali |
| `tools/analyze_results.py` | Merangkum `result.json` dan mengkalibrasi model waktu |
| `tests/test_all.py` | Verifikasi: evaluator == `judge.py` resmi, generator == resmi, planner optimal, dll. |
| `tests/test_ros_adapter.py` | Menjalankan semua jalur kode ROS (`RosApi`, start/stop, publish wrench, jalur crash, `/ib2_user/stop`, watchdog tenggat) dengan modul ROS tiruan |
| `vendor/` | Bahan resmi JAXA yang diunduh (simulator, sample code) + repo Int-Ball2 (rujukan) |
| `docs/` | Dokumen ini + `STRATEGI_PENILAIAN_JURI.md` |

## 1. Uji cepat di laptop (Windows, tanpa Docker)
```bash
cd KiboRPC7
python tests/test_all.py          # harus diakhiri "ALL 16 TESTS PASSED" (±3 menit)
python tests/test_ros_adapter.py  # harus diakhiri "ALL 5 CHECKS PASSED"
python tools/offline_sim.py --seeds 5 --strategies fast,safe
python tools/offline_sim.py --levels veryhard --seeds 1 --strategies fast --verbose
```
Uji ketahanan (gangguan berat, delay navigasi 0,2 s, kipas 20% lebih lemah, wrench diabaikan):
```bash
python tools/offline_sim.py --seeds 8 --strategies fast,safe --bias 0.005 --noise 0.02 --delay 0.2 --dist 0.003
python tools/offline_sim.py --seeds 8 --strategies fast,safe --fan_scale 0.8
python tools/offline_sim.py --seeds 3 --strategies fast --ignore_user_wrench
```

## 2. Pasang WSL2 + Docker **[di laptopmu]**
> **Update 23 Okt, 08:25:** simulator resmi sudah di-build dan dijalankan **dari Windows `docker.exe`, tanpa WSL integration**. v3_fast lulus di keempat level, semua checkpoint Lv3: Easy 132,35 s, Normal 155,55 s, Hard 187,80 s, VH 200,78 / 203,42 s. Kalau CPU laptop penuh, plugin navigasi simulator bisa mati (`navigation stale`); ulangi run-nya. Cara dan hasil: [`SIM_RESMI_LANGKAH.md`](SIM_RESMI_LANGKAH.md) §0.
>
> **Update 23 Okt (pagi):** di laptop POGGY, Ubuntu 24.04 (WSL versi 2) dan Docker Desktop (CLI 29.6.1) sudah terpasang, jadi langkah 1–3 di bawah dilewati. Yang masih perlu: **nyalakan Docker Desktop** dan aktifkan **Settings → Resources → WSL integration → Ubuntu**. Hasil cek dan langkah lengkap: [`SIM_RESMI_LANGKAH.md`](SIM_RESMI_LANGKAH.md). Path di §3 sekarang dihitung otomatis dengan `wslpath`, tidak perlu diganti manual.

Hasil cek 9 Okt: WSL di laptopmu masih **versi 1** dan Docker belum ada. Ikuti Programming Manual §2.4:
1. PowerShell **sebagai Administrator**: `wsl --install -d Ubuntu-24.04` lalu restart (`shutdown /r /t 0`).
2. Cek dengan `wsl --list --verbose`; kolom VERSION harus `2`. Kalau masih 1: `wsl --set-version Ubuntu-24.04 2`.
3. Di terminal Ubuntu, pasang Docker Engine (perintah lengkap ada di Manual §2.4.2), lalu jalankan `sudo usermod -aG docker $USER`, tutup dan buka lagi terminal, dan cek dengan `docker run hello-world`.
4. Spesifikasi laptopmu cukup: Ryzen 7 6800H 16 thread, RAM 15 GB, disk C kosong 172 GB. Image simulator ±6,3 GB.

## 3. Simulator lokal resmi **[di laptopmu, di Ubuntu WSL2]**
Set dulu variabel `KIBO` ke folder `KiboRPC7`. `wslpath` mengubah path Windows menjadi path WSL, jadi kalau folder dipindah cukup ganti path Windows di baris pertama.
```bash
KIBO="$(wslpath 'C:\Users\POGGY\Downloads\barengpoggy\KiboRPC7')"   # = /mnt/c/Users/POGGY/Downloads/barengpoggy/KiboRPC7
ls -l "$KIBO/vendor/krpc7th-simulator.tar.gz"                           # harus ada
mkdir -p ~/krpc && cd ~/krpc
tar -xzf "$KIBO/vendor/krpc7th-simulator.tar.gz"
cd krpc7th-simulator
# pasang program PERSIS seperti yang di-upload (isi arsip = catkin_ws/src/...)
rm -rf users/catkin_ws/src
tar -xzf "$KIBO/submission/GaneshaOrbit_v3_fast.tar.gz" -C users
md5sum users/catkin_ws/src/krpc_user/scripts/user_template.py          # bandingkan dengan MD5 di SIM_RESMI_LANGKAH.md
bash ./build.sh 2>&1 | tee ~/krpc/build.log                             # sekali saja; lama dan butuh internet
LEVELS=veryhard bash ./simulation.sh 2>&1 | tee ~/krpc/run_veryhard.log # buka http://localhost untuk menonton
```
- Kerjakan di `~/krpc` (filesystem Linux), **jangan** langsung di `/mnt/c/...`. Build dan bind-mount Docker jauh lebih lambat di path Windows.
- Hasil ada di `share/logs/latest/`: `result.json` (juri) dan `user_programs_logs/` (log program kita, cari baris `[GaneshaOrbit]`).
- Batch banyak run + ringkasan (dari folder `~/krpc/krpc7th-simulator`, dengan `KIBO` sudah di-set seperti di atas):
```bash
bash "$KIBO/tools/batch_local_sim.sh" 5 "easy normal hard veryhard"
python3 "$KIBO/tools/analyze_results.py" share/batch
```

### Yang WAJIB dicek di simulator lokal (urut)
1. **Mode FAST benar-benar menggerakkan robot.** Di log harus ada `leg -> cp N (fast)` disusul `REPORT ... (Lv3 held (fast))`, dan **tidak ada** `switching to SAFE mode`.
2. Kalau muncul `switching to SAFE mode`, catat alasannya (`tracking error ...` / `navigation stale`). Misi tetap selesai di mode SAFE, tetapi kirim lognya ke tim untuk dianalisis.
3. `result.json` → `Illegal: false`, semua `checkpoint_detail.level == 3`, `goal.success == true`.
4. v3: baris `SAFE STOP (mission finished)` harus muncul **satu kali**, sesudah `mission_finished sent`. Kalau ada `mission crashed`, `deadline watchdog`, atau `SAFE STOP (mission crashed)`, kirim log lengkapnya ke tim.
5. `analyze_results.py` memberi selisih `actual − predicted`. Tambahkan nilainya ke `FAST_OVERHEAD_S` (FAST) atau `LEG_OVERHEAD_S` (SAFE) di `CONFIG`.

## 4. Web Simulator resmi (setelah ID/PW dari BRIN, simulator dibuka 15 Okt)
1. Login di https://jaxa.krpc.jp/user-auth → **SIMULATION**.
2. Upload `submission/GaneshaOrbit_v3_fast.tar.gz`. Pilih *Simulator Level* (uji keempat level), lalu **START SIMULATION**.
3. **VIEW RESULT** → catat Final Score, Mission Time, Time Bonus, dan level per checkpoint. Lalu **DOWNLOAD LOG FILES** dan jalankan `analyze_results.py` pada folder unduhan.
4. Web Simulator berjalan di **ARM64** (seperti Int-Ball2 asli). Kalau mode FAST bermasalah di sana tetapi lancar di lokal, upload varian `_safe` sebagai pembanding.
5. Buat arsip baru setelah mengubah kode:
```bash
python tools/make_submission.py --name GaneshaOrbit_v5_fast --mode fast   # nomor versi berikutnya; jangan timpa v3/v4
```

## 5. Kenop yang boleh diatur (`CONFIG` di `user_template.py`)
| Kunci | Default | Efek / kapan diubah |
|---|---|---|
| `MOTION_MODE` | `fast` | `safe` = hanya `ctl` bawaan (lebih lambat, paling aman untuk final ISS) |
| `FAST_ETA_T` / `FAST_ETA_R` | 0,55 / 0,40 | Porsi kapasitas kipas untuk translasi/rotasi. Naikkan perlahan (maks. jumlah 0,95) **hanya** setelah lolos uji lokal/Web |
| `FAST_VMAX` | 0,20 m/s | Sama dengan batas profiler JAXA. **Jangan dinaikkan** (alasan keamanan dan review JAXA) |
| `HOLD_S` | 1,0 s | Syarat Rulebook (diam di checkpoint 1 detik). **Jangan dikurangi** |
| `POS_MARGIN` / `ANG_MARGIN` | 0,6 | Lapor hanya bila jarak ≤0,6× batas Lv3 (0,06 m) dan sudut ≤6°. Turunkan kalau ada checkpoint yang dinilai Lv2 |
| `SAFETY_MARGIN_S` | 15 s | Cadangan waktu sebelum batas 5 menit |
| `FAST_OVERHEAD_S` / `LEG_OVERHEAD_S` | 0,4 / 1,0 | Isi dari hasil kalibrasi (§3 langkah 4) |
| `ENABLE_CAMERA_CHECK` | False | Mencatat ID ArUco yang terlihat saat lapor (diagnostik; tidak memengaruhi skor) |
| `SAFE_STOP_ZERO_N` / `SAFE_STOP_ZERO_DT` | 3 / 0,02 s | v3: jumlah dan jarak wrench nol saat *safe stop*. Jangan dikurangi |
| `SAFE_STOP_CTL_WAIT_S` | 1,0 s | v3: batas tunggu server `ctl` dan hasil KEEP_POSE di jalur darurat (crash, watchdog, `/ib2_user/stop`) |
| `DEADLINE_WATCHDOG_S` | 2,0 s | v3: watchdog terpisah memaksa safe stop + `mission_finished` di detik 298 kalau thread misi macet. Di run normal tidak pernah aktif, karena misi sudah selesai paling lambat detik 296 |
| `FAST_ETA_ADAPT` | True | v4: anggaran kipas yang sama (E = `FAST_ETA_T` + `FAST_ETA_R` = 0,95) dibagi **per leg**. Rotasi hanya diberi porsi yang cukup untuk selesai di 90% waktu translasi (`FAST_ROT_LEAD`), sisanya untuk translasi. `False` = pembagian tetap v1–v3 (hasil identik per run dengan v3) |
| `FAST_ETA_T_MAX_FRAC` / `FAST_ETA_R_MIN_FRAC` | 0,79 / 0,21 | v4: batas pembagian, yaitu η_t ≤ 0,75 dan η_r ≥ 0,20 pada E = 0,95. η_r tidak pernah di atas `FAST_ETA_R` (0,40). Sweep: 0,70 → VH −0,6 nominal, +0,4 di kipas 0,8× |
| `FAST_GOV` + `FAST_GOV_*` | True; 0,10/0,06 m, 8/4°, 0,5 /s, s ≥ 0,5 | v4: *reference governor*. Kalau robot tertinggal dari profil, jam profil diperlambat (kecepatan ×s, percepatan ×s²) sampai galat kembali < pita LO. Hanya aktif kalau robot sudah bergerak ≥3 cm/3°, supaya aktuator mati tetap cepat terdeteksi. Di run nominal praktis tidak pernah aktif |
| `START_REST_V` / `START_REST_W_DEG` / `START_SETTLE_MAX_S` | 0,03 m/s / 5°/s / 12 s | v4: kalau robot bergerak/berputar saat start, `ctl` KEEP_POSE dulu sampai diam, baru FAST. Tanpa efek kalau start dalam keadaan diam |
| `ARUCO_ALIGN` | `"off"` | v4: penyelarasan akhir dengan kamera (ArUco 4×4, 2 cm) untuk final ISS. `"log"` = hanya mencatat selisih navigasi vs marker; `"correct"` = checkpoint digeser sebesar selisih itu (maks. 5 cm, minimal 3 deteksi konsisten). **Biarkan `off` di penyisihan**: juri simulator menilai pose navigasi, jadi koreksi kamera tidak bisa menaikkan skor |

## 6. Submit penyisihan
- Batas: **20 Jan 2027 23:59 JST (21:59 WIB)**. Batas pertanyaan 20 Jan 12:00 JST.
- Program dijalankan dulu di layar **"Preliminary"** Web Simulator. Pilih hasil yang statusnya **FINISHED** → **SUBMIT**. Submit ulang tetap bisa lewat **CANCEL** sampai tenggat.
- Hanya hasil berstatus FINISHED yang bisa di-submit, jadi jangan menunggu menit terakhir.

## 7. Pembagian kerja tim
| Peran | Tugas |
|---|---|
| Lead + infra (kamu) | WSL2/Docker, batch run, analisis log, kalibrasi, submit |
| Kontrol | Tuning `FAST_ETA_*`, gain, ambang fault; cek kestabilan dari rekaman (`replay.sh`) |
| Navigasi / CV | `ENABLE_CAMERA_CHECK`, kalibrasi kamera, persiapan final ISS |
| Perencanaan | Cek urutan rute di log; uji kasus tepi (checkpoint di plafon, marker 11 Airlock) |
| QA | Jalankan `tests/test_all.py` dan suite Monte Carlo tiap ada perubahan kode |

## 8. Final ISS (khusus finalis, awal Mar – pertengahan Apr 2027)
- JAXA me-review kode demi keamanan. Siapkan dua versi: **FAST** (sudah teruji di Web Simulator ARM64) dan **SAFE** (100% `ctl` bawaan). Kalau JAXA keberatan dengan kontrol wrench langsung, pakai SAFE.
- Simulator untuk ISS dirilis pertengahan Maret. Ulangi §3–§4 dengan simulator itu.
- Di ISS hanya ada **satu run**: robot yang macet karena program sendiri tidak diulang. Jangan menaikkan kecepatan di atas batas JAXA.
