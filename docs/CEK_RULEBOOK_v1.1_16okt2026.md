# Cek kode Ganesha Orbit vs Rulebook v1.1 (16 Okt 2026)

## Ringkasan
**Kode tidak perlu diubah.** Rulebook v1.1 (15 Okt) hanya berisi *"Editorial correction"*. Tidak ada angka aturan yang berubah, dan file simulator resmi juga tidak berubah.

## Yang dicek
| Hal | Hasil |
|---|---|
| Diff kata per kata v1.0 (5 Okt) vs v1.1 (15 Okt) | Hanya penomoran langkah 2.2.3, tata letak tabel, nomor halaman, dan glosarium (entri "Astrobee", "GSE", serta catatan internal editor dihapus). **Tidak ada perubahan angka.** |
| Batas waktu 5 menit, termasuk laporan selesai | `T_LIMIT_S = 300` ✅ |
| Tahan 1 detik di checkpoint dengan marker di tengah kamera | `HOLD_S = 1.0` ✅ |
| Multiplier waktu maks 2× | `MAG_MAX = 2.0` ✅ |
| Penyisihan: rata-rata 10 run acak | Disimulasikan dengan 10 seed ✅ |
| 6–12 checkpoint, level Lv1–Lv3, start Dock (0,2; 0; 0) | Sama ✅ |
| `vendor/krpc7th-simulator.tar.gz` | MD5 `c90937a3…` = file di situs saat ini ✅ |
| `vendor/krpc7th-sample-source-beta.tar.gz` | MD5 `fba02f40…` = file di situs saat ini ✅ |
| `tests/test_all.py` dan `tests/test_ros_adapter.py` | Lolos semua ✅ |

## Catatan: quaternion marker di Tabel 2.2.2-2
- Tabel rulebook dan `area_config.json` simulator **tidak sama persis**. Contoh marker 1: rulebook `(-0.707, 0, 0, 0.707)`, config `(-0.5, 0.5, 0.5, 0.5)`.
- Setelah dihitung, **arah hadap marker (sumbu normal) identik untuk ke-12 marker**. Yang berbeda hanya rotasi frame di dalam bidang marker (0°, ±90°, atau 180°), jadi ini beda konvensi penulisan.
- Kode membaca pose marker dari data misi saat runtime, dan simulator offline memakai salinan persis `area_config.json` resmi. Penilai resmi (`judge.py`) juga memakai file itu.
- **Tindakan:** tanyakan di Guidance Session 21 Okt (16:00 WIB) konvensi mana yang dipakai untuk sudut roll checkpoint di web simulator v1.0.

## Simulasi offline (10 seed per level, 16 Okt)
| Level | FAST T [s] | FAST skor | SAFE skor |
|---|---|---|---|
| Easy | 128,0 | 33,04 | 28,36 |
| Normal | 155,3 | 40,02 | 31,57 |
| Hard | 176,6 | 46,57 | 30,38 |
| Very Hard | 202,8 | 51,64 | 31,46 |

FAST: semua run selesai tepat waktu, mencapai goal, dan 100% checkpoint di Lv3.

**Uji tekan di Very Hard (FAST):**
| Gangguan | Skor | Catatan |
|---|---|---|
| Gangguan acak 2,7×, atau bias 3× + acak 3× | 51,64 | Tidak berubah; gangguan terserap penuh oleh kontrol tertutup |
| Delay navigasi 0,5 s | 49,45 | 97% Lv3 |
| Kipas asli 30% lebih lemah | 44,99 | |
| Gangguan ekstrem (bias 15×, acak 13×) | 35,18 | Goal 80%. Ini membuktikan jalur gangguan memang aktif |

## Yang belum bisa dicek dari sini
- **Web Simulator v1.0** (rilis 15 Okt) butuh login, yang baru dikirim POC BRIN setelah tim terdaftar (tenggat **4 Nov 14:59 WIB**). Begitu login didapat:
  1. Unggah `submission/GaneshaOrbit_v1_fast.tar.gz`.
  2. Cek log pertama: apakah `/ctl/wrench` diterima. Kalau mode FAST gagal, kode otomatis jatuh ke SAFE.
  3. Bandingkan waktu dan skor dengan tabel di atas.
- **Beginner Course** (rulebook terpisah, 15 Okt) adalah jalur lain. Kode ini dibuat untuk **Regular Course**, jadi pilih *Regular* di form.
