# Ganesha Orbit — program Kibo-RPC ke-7 (Int-Ball2)

Program misi lengkap untuk babak penyisihan Kibo-RPC ke-7, beserta simulator uji, tes verifikasi, dan dokumen strategi.

## Mulai di sini
0. **Skor juri v3 → v4, review keamanan, dan cara kirim (24 Okt 2026)** → [JURI_90/SKOR.md](JURI_90/SKOR.md), [JURI_90/CARA_KIRIM.md](JURI_90/CARA_KIRIM.md)
1. **Status, jawaban formulir BRIN, draf pesan, dan langkahmu berikutnya** → [docs/00-STATUS-dan-PENDAFTARAN-20okt2026.md](docs/00-STATUS-dan-PENDAFTARAN-20okt2026.md) (§0 = 23–24 Okt)
2. **Kenapa solusi ini menang dan berapa nilainya di mata juri** → [docs/STRATEGI_PENILAIAN_JURI.md](docs/STRATEGI_PENILAIAN_JURI.md)
3. **Langkah kerja (setup, uji, submit)** → [docs/RUNBOOK.md](docs/RUNBOOK.md)
4. **File yang di-upload ke Web Simulator** → `submission/GaneshaOrbit_v4_fast.tar.gz` (cadangan berurutan: `_v3_fast` yang sudah lulus simulator resmi di 4 level, lalu `_v4_safe`; `_v2_*` dan `_v1_*` disimpan untuk perbandingan)
5. **Menjalankan simulator lokal resmi (Windows + Docker)** → [docs/SIM_RESMI_LANGKAH.md](docs/SIM_RESMI_LANGKAH.md)

## Ringkasan teknis
- `krpc_user/scripts/user_template.py`: satu file Python 3.6, tanpa dependensi di luar image resmi. Komponennya:
  - evaluator **identik dengan `judge.py` resmi**
  - perencana rute Held-Karp eksak
  - model waktu dari rumus profiler JAXA
  - **mode FAST** (translasi + rotasi paralel lewat API resmi `/ctl/wrench`) dengan fallback otomatis ke **mode SAFE** (`ctl` bawaan)
  - penjaga tenggat 5 menit, ditambah watchdog terpisah (v3)
  - **safe stop** (v3): semua cara misi berakhir (selesai normal, crash, watchdog, `/ib2_user/stop`) memakai satu rutin yang sama. FAST dimatikan, wrench nol dikirim 3×, lalu `ctl` diminta KEEP_POSE
- Simulasi offline, 25 seed per level (20 Okt): FAST menyelesaikan **semua checkpoint di Lv3** pada keempat level (Very Hard ±202 s). Pola sample code resmi melewati batas 5 menit pada level Normal ke atas.
- v2 (20 Okt): pengaman osilasi attitude (gain laju attitude ×0,5 bila Lv3 masuk-keluar tanpa bisa ditahan 1 s). Hanya aktif di uji delay navigasi 0,5 s; hasil nominal identik dengan v1.
- v3 (23 Okt): perbaikan keamanan. Di v2, kalau `Mission.run()` crash, kontroler wrench tetap aktif. Skor Monte Carlo v3 identik dengan v2 di kedelapan skenario (`tools/results/after_v3_23okt/`).
- v4 (24 Okt): (1) anggaran kipas yang sama (0,95) dibagi per leg antara translasi dan rotasi; (2) *reference governor* yang memperlambat jam profil kalau robot tertinggal; (3) opsi penyelarasan ArUco untuk ISS (mati). Monte Carlo, seed sama dengan v3: nominal **33,94 / 41,63 / 48,72 / 54,77** (v3: 32,88 / 40,02 / 46,48 / 51,70), 100% Lv3 dan goal; lebih baik dari v3 di semua uji tekan kecuali *wrench diabaikan* (−0,04 s.d. −0,18). Dengan kedua fitur dimatikan, hasilnya identik per run dengan v3 (100/100). Data: `tools/results/after_v4_24okt/`

## Perintah cepat
```bash
python tests/test_all.py                                   # verifikasi (16 tes)
python tests/test_ros_adapter.py                           # jalur kode ROS (stub)
python tools/offline_sim.py --seeds 5 --strategies fast,safe,baseline
python tools/make_submission.py --name GaneshaOrbit_v5_fast --mode fast   # versi berikutnya; v3 dan v4 sudah ada
```
