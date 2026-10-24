# Kibo-RPC ke-7 — Penilaian dari sudut pandang juri, temuan kunci, dan solusi
### Tim Ganesha Orbit · 9 Okt 2026

## 1. Nilai 1–100 dari sudut pandang juri
Di Kibo-RPC, "juri" penyisihan adalah **skor simulator resmi (rata-rata 10 run acak)** yang dihitung `judge.py` JAXA + bonus waktu, lalu diperingkat BRIN. Di final, JAXA menambahkan **review keamanan kode** sebelum program dijalankan di ISS. Karena itu rubrik di bawah meniru apa yang benar-benar diukur:

| Komponen (bobot) | Yang diukur juri |
|---|---|
| **A. Skor simulator (60)** | Rata-rata skor 10 run, dibandingkan batas atas realistis tiap level |
| **B. Keandalan (20)** | Selalu selesai < 5 menit, goal tercapai, tahan gangguan dan pola acak |
| **C. Kepatuhan & kesiapan ISS (20)** | Patuh Rulebook, tanpa celah/eksploitasi, aman di-review JAXA, ada jalur aman |

| Solusi | A (60) | B (20) | C (20) | **Total** | Catatan juri |
|---|---|---|---|---|---|
| Pola sample code resmi (urutan ID, tunggu SUCCESS) | 13 | 4 | 17 | **34** | Normal ke atas hampir selalu melewati 5 menit → skor 0 |
| **Rencana lama saya** (dok. 9 Okt: TSP + koreksi ArUco + `/ctl/command`) | 30 | 12 | 15 | **57** | Arah benar, tetapi belum tahu `ctl` berjalan geser-lalu-putar + "diam 3 s", dan koreksi ArUco tidak dipakai juri |
| **Program baru, mode SAFE** (hanya `ctl` bawaan) | 40 | 20 | 20 | **80** | 100% tepat waktu, tetapi Hard/Very Hard terpaksa melewatkan 1–3 checkpoint |
| **Program baru, mode FAST** (default, dengan fallback) | 55 | 18 | 16 | **89** | Very Hard 12/12 Lv3 dalam ±203 s. −2 di B karena belum divalidasi di simulator resmi; −4 di C karena kontrol wrench langsung perlu lolos review JAXA (varian SAFE sudah siap) |

**Jadi rencana lama tidak cukup untuk menang (57/100).** Program yang sekarang dibangun (89/100) mengalahkan pola umum karena hal-hal yang hanya terlihat dari kode resmi (§2).

Cara menghitung A: batas atas tiap level diasumsikan sebagai semua checkpoint Lv3 + bonus dengan waktu 25% lebih cepat dari mode FAST kita. Rata-rata FAST = **91%** dari batas itu, SAFE = 66%, sample = 22%. Semua angka berasal dari simulasi offline (§5; data mentah di `tools/results/v4/`). Validasi di simulator resmi adalah gerbang berikutnya (RUNBOOK §3–4).

## 2. Temuan kunci dari kode resmi (yang tidak ada di Rulebook)
Sumber: `krpc7th-simulator.tar.gz` (JAXA, 5 Okt 2026) + repo `jaxa/int-ball2_simulator` (branch melodic).

1. **Juri menilai sekali, saat program melapor.** `judge.py` mengevaluasi pose navigasi (`/sensor_fusion/navigation`) tepat ketika `/event/checkpoint_passed` dipanggil. Checkpoint yang sama tidak dinilai dua kali, jadi **lapor terlalu cepat = FAIL permanen**.
2. **Program boleh tahu target dan toleransinya.** `/mission/get_checkpoints` mengirim posisi target, quaternion target, dan pita skor tiap checkpoint (Lv3: ≤0,10 m, centering ≤10°, roll ≤10°). Karena itu evaluasi juri bisa dihitung sendiri **sebelum** lapor. Evaluator kita terbukti identik dengan `judge.py` pada 6.000 pose acak.
3. **Level = syarat terburuk** dari jarak, centering (sumbu kamera ke marker), dan roll di sekitar garis pandang (roll target acak ±180°).
4. **Checkpoint dibangkitkan** 0,5 m dari marker (yaw ±20°, pitch ±15°) di dalam SafeArea. Goal = marker 12 (panel dekat dock), jadi rutenya berupa **loop** yang kembali ke dekat titik start. Jumlah checkpoint: Easy 6 / Normal 8 / Hard 10 / Very Hard 12.
5. **`ctl` bawaan lambat** (`ctl.yaml`, `pos_att_profiler.cpp`):
   - perintah `MOVE_TO_ABSOLUTE_TARGET` dijalankan **berurutan: translasi lalu rotasi**
   - percepatan dibatasi `eta=0.4` (±0,012–0,023 m/s²), v ≤ 0,2 m/s, ω ≤ 25°/s
   - status sukses baru keluar setelah **3 detik diam** di toleransi 0,1 m / **0,5 rad (±29°)**, jauh lebih longgar dari Lv3
   - **preempt** (ganti target di tengah jalan) memicu profil berhenti + 3 detik lagi
6. **Robot ini sangat lemah:** 3,2 kg dengan 8 kipas @0,06 N. Akibatnya pola sample code (tunggu SUCCESS di tiap checkpoint) **melewati 5 menit** di Normal–Very Hard (§5).
7. **`/ctl/wrench` adalah API resmi** (Programming Manual 6.2.2; JAXA punya contoh `test_force_and_torque.py`). `fsm` mengalokasikan wrench dari publisher mana pun ke kipas, dan `ctl` diam saat **STAND_BY**. Jadi translasi + rotasi bisa dijalankan **paralel** dengan profil sendiri.

## 3. Solusi yang dibangun (program `krpc_user/scripts/user_template.py`)
```
mission_start → get_checkpoints → [ctl → STAND_BY] → loop:
    rencana Held-Karp eksak (semua subset; biaya = model waktu fisik, termasuk rotasi)
    → leg FAST (profil paralel, PD + feedforward, guard saturasi kipas)
    → pantau pose: evaluasi identik juri → lapor setelah Lv3 (margin 60%) bertahan 1 s
    → rencanakan ulang dengan waktu nyata; lewati checkpoint hanya bila 5 menit terancam
→ goal (marker 12) terakhir → lapor → mission_finished → serahkan ke ctl (KEEP_POSE)
```
| Fitur | Alasan |
|---|---|
| Evaluator = juri resmi | Tidak pernah melapor saat masih FAIL; target Lv3 dengan margin |
| Lapor setelah 1 s di dalam pita | Mematuhi Rulebook 2.3.1 (diam 1 detik) meski juri simulator tidak menegakkannya |
| Held-Karp eksak (≤11 node, ≈0,2 s) | Urutan rute optimal terhadap waktu tempuh **dan** waktu putar (terbukti == brute force) |
| Model waktu = rumus profiler JAXA | Prediksi waktu akurat → anggaran 5 menit aman; mode SAFE juga memilih "putar dulu" bila lebih cepat |
| FAST: profil paralel di `/ctl/wrench` | Menghapus urutan translasi-lalu-rotasi dan "diam 3 s": ±1,6× lebih cepat |
| Gain PD = gain `ctl` resmi | Stabil terhadap delay navigasi 0,1 s (gain sudah dirancang JAXA) |
| Guard saturasi kipas (≤95%) | Arah gaya tidak terdistorsi |
| Monitor fault → lunakkan profil (maks. 2×) → SAFE | Kipas lebih lemah dari model: profil dilunakkan, FAST tetap jalan. Robot sama sekali tidak merespons wrench: langsung SAFE (±4,5 s). Misi tetap selesai |
| Refine FAST | Bila Lv3 belum bertahan 6 s setelah profil selesai, jalankan profil koreksi pendek dari pose sekarang, tetapi hanya bila tenggat goal tetap aman |
| Penjaga tenggat | Goal selalu dicapai sebelum 5 menit; checkpoint bernilai rendah dilepas bila perlu |
| `/ib2_user/stop` → wrench nol, `on_shutdown` → wrench nol | Syarat keamanan perangkat asli |

**Yang sengaja TIDAK dilakukan** (agar aman di mata juri):
- **Tidak** membaca `cheat_sheet.json`. Platform punya deteksi "Illegal" untuk akses kunci jawaban.
- **Tidak** memakai celah "goal sudah dilewati sebelumnya". Melewati marker 12 di awal lalu `mission_finished` dari jauh diterima `judge.py` lokal, tetapi melanggar Rulebook 2.2.4 (lapor saat tiba di GoalPoint).
- **Tidak** mengubah parameter `ctl` lewat rosparam/`/ctl/update_params`. Itu bukan API peserta.
- **Tidak** menaikkan v_max atau ω_max di atas batas JAXA.

## 4. Kesiapan final ISS
- Varian **SAFE** (100% `ctl` JAXA) siap sebagai jalur aman bila JAXA keberatan dengan kontrol wrench langsung.
- Mode FAST memakai batas kecepatan yang sama dengan JAXA, memeriksa navigasi basi, dan membuat robot berhenti aman.
- Di ISS hanya ada satu run, jadi urutan run (tier) ditentukan skor penyisihan. Skor tinggi = jatah run pasti.

## 5. Bukti: simulasi Monte Carlo offline
Simulator offline (`tools/offline_sim.py`) mereplikasi `ctl_only` (profiler, PD, alokasi 8 kipas, aturan stay/preempt), plant 6-DoF (`sim.yaml`: massa, inersia, geometri kipas, derau kipas), delay navigasi, gangguan aliran udara, bias "gravitasi mikro", generator (identik dengan resmi), dan juri (identik dengan resmi). Skor memakai asumsi poin Lv3=3/Lv2=2/Lv1=1, bonus semua-checkpoint=3, dan pengali waktu 1+(300−T)/300 (maks 2×). Nilai absolutnya asumsi; **perbandingan antarstrategi** yang penting.

**Nominal (10 seed per level):**
| Level | FAST | SAFE | Pola sample |
|---|---|---|---|
| Easy (6) | 128 s · 100% Lv3 · **33,0** | 195 s · 100% · 28,4 | 236 s · 97% · 24,6 |
| Normal (8) | 155 s · 100% · **40,0** | 249 s · 100% · 31,6 | 306 s · hanya 30% tepat waktu · 8,0 |
| Hard (10) | 177 s · 100% · **46,6** | 259 s · 89% · 30,4 | 375 s · 0% tepat waktu · 0 |
| Very Hard (12) | 203 s · 100% · **51,6** | 258 s · 77% · 31,5 | 445 s · 0% tepat waktu · 0 |

**Stres A** (bias 5 mN, derau kipas 0,02 N, delay navigasi 0,2 s, aliran udara 2×; 8 seed): FAST tetap 100% tepat waktu, 100% Lv3, **0 fault** (skor 33,1 / 40,1 / 46,6 / 51,7).

**Stres B** (kipas 20% lebih lemah dari model; 8 seed): FAST tetap **100% tepat waktu, 288/288 checkpoint Lv3, 0 kali jatuh ke SAFE** (skor 32,5 / 39,2 / 46,5 / 49,6; Very Hard terlama 257 s). Sebelum ada pelunakan bertahap dan refine, 13 dari 32 run jatuh ke SAFE.

**Uji gagal paksa** (semua wrench pengguna diabaikan; 3 seed × 4 level): robot tidak merespons → langsung SAFE dalam ±4,5 s. Semua 12 run tetap selesai < 5 menit dan goal tercapai (Easy/Normal 100% Lv3; Hard/Very Hard 83%/75%, setara mode SAFE).

**Verifikasi (`tests/test_all.py`):** evaluator == `judge.py` resmi (6.000 pose) · generator == resmi · planner == brute force · model waktu == profiler · misi end-to-end lulus. `tests/test_ros_adapter.py` menjalankan semua jalur kode ROS (start/stop, publish wrench, layanan misi) dengan modul ROS tiruan. Kompatibel Python 3.6 (dicek dengan `vermin`).

## 6. Risiko yang tersisa & cara menutupnya
| Risiko | Mitigasi |
|---|---|
| Gazebo resmi berbeda dari simulator offline (aliran udara nyata, saturasi `fsm`, ARM64) | Uji di simulator lokal (Docker) dan Web Simulator mulai 15 Okt. Kalibrasi `FAST_OVERHEAD_S` dan `FAST_ETA_*` dengan `analyze_results.py` |
| `ctl` ternyata tidak STAND_BY saat start | Program mengirim `STAND_BY` resmi sebelum FAST. Bila tetap konflik, fault → SAFE |
| Konversi skor/bonus waktu resmi berbeda dari asumsi | Strategi tidak bergantung pada angka itu: urutan prioritas tetap "semua Lv3, secepat mungkin, tidak pernah lewat 5 menit" |
| JAXA menolak kontrol wrench untuk ISS | Pakai varian SAFE untuk final |
