# JURI_90 — Skor Ganesha Orbit (Kibo-RPC ke-7, babak penyisihan)
Dinilai 23–24 Okt 2026 (malam) dengan protokol `03-MEGA-PROMPT-JURI-90.md`. Lomba ini **dinilai otomatis**, jadi fungsi tujuannya: maksimalkan ekspektasi skor rata-rata 10 run acak, lalu kecilkan variansinya. Syaratnya: tidak pernah kehilangan Lv3 atau melewati tenggat, dan tetap lolos review keamanan JAXA untuk run di ISS.

## 1. Aturan (dibaca ulang 23 Okt malam dari sumber resmi)
| Sumber | Isi (kutipan pendek / ringkas) | Dampak |
|---|---|---|
| Rulebook v1.1 (15 Okt), §2.3.1 + glosarium (`jaxa.krpc.jp/static/2881600683dab1897ef65741461f8618/7thKibo-RPC_Rulebook.pdf`, MD5 file = nama folder) | Score = Pt_all × Mag_clear. Pt_all = jumlah Pt_check per Lv + Pt_goal. Pt_goal hanya kalau **semua** checkpoint dilewati dalam batas waktu dan GoalPoint dicapai. Mag_clear naik dengan sisa waktu, *"A maximum of 2x is assumed"*. Level = syarat terburuk dari posisi, centering, roll; tahan 1 detik | Rumus pengali **tidak** diberikan. Model kita (1 + (300 − T)/300, Lv3/2/1 = 3/2/1, Pt_goal = 3) tetap **asumsi** |
| FAQ resmi (diperbarui 23 Okt) | Rumus skor termasuk Time Bonus **tidak diumumkan**; poin per level tidak publik; `/ctl/wrench` **tidak dilarang**; apa pun yang tidak ada di Programming Manual dianggap dilarang; tidak ada penalti tabrakan/KOZ, tetapi kode berisiko bisa diminta diubah; level penyisihan belum ditentukan | Angka absolut skor kita = model. Keputusan v3 → v4 tidak bergantung pada rumus (§3c). Kekhawatiran lama "wrench boleh atau tidak" sebagian tertutup |
| Rulebook §2.2.5 | 10 run otomatis, kondisi acak dan gangguan berbeda, skor = rata-rata | Ukuran utama = rata-rata; p10 dan min untuk variansi |
| Rulebook §3.1 | Submit sampai **20 Jan 2027**; hanya hasil **FINISHED** di simulator *Preliminary*; SUBMIT/CANCEL bisa diulang sampai tenggat | `CARA_KIRIM.md` |
| Rulebook §4.1 | Finalis: JAXA me-review kode supaya tidak berdampak negatif pada IB2 di orbit, dan bisa meminta perubahan | Checklist keamanan §6 |
| Guidance Session (materi 23 Okt) | *"Depending on the country/region, factors other than the simulation score may also be evaluated."* Web Simulator: 120 run/bulan, 20 hasil tersimpan | **Presentasi nasional mungkin ada** (BRIN belum mengumumkan). Pertanyaan sudah ada di email POC (`00-STATUS` §0f, no. 3) |
| Programming Manual v1.0 §5.13 | ArUco 4×4, ID 1–12, sisi 2 cm, `/camera_main/camera_info`, kamera 0,14 m di depan pusat badan | Dipakai untuk opsi penyelarasan kamera (§4) |

**Peta kriteria → bukti:** skor simulator → `tools/results/after_v4_24okt/compare_v3_v4.md` + simulator resmi (§3b). Keandalan → uji tekan 8 skenario + tes. Kepatuhan/ISS → checklist §6. Dokumentasi → file ini, `CARA_KIRIM.md`, `docs/RUNBOOK.md` §5.

**Bobot rubrik (asumsi, karena tidak diumumkan):** A. Ekspektasi skor simulator 55 · B. Keandalan/variansi 20 · C. Kepatuhan Rulebook + kesiapan review keamanan ISS 15 · D. Dokumentasi dan kesiapan presentasi nasional 10.

## 2. Panel
J1 = insinyur GN&C JAXA/NASA (keamanan, ketahanan, kesiapan ISS) · J2 = tim peringkat atas Kibo-RPC sebelumnya (trik skor yang sah) · J3 = juri nasional BRIN (dokumentasi, presentasi) · J4 = skeptis (kepatuhan aturan, celah sim vs ISS; veto 80).

## 3. Angka
### 3a. Monte Carlo offline, seed sama dengan `after_v3_23okt` (FAST, rata-rata / p10)
| Skenario | Easy v3 → v4 | Normal v3 → v4 | Hard v3 → v4 | Very Hard v3 → v4 |
|---|---|---|---|---|
| **Nominal (25 seed)** | 32,88 → **33,94** / p10 32,32 → 33,35 | 40,02 → **41,63** / 38,91 → 40,80 | 46,48 → **48,72** / 45,50 → 47,74 | 51,70 → **54,77** / 50,92 → 54,15 |
| Waktu misi rata-rata | 130,2 → 115,1 s | 155,3 → 137,4 s | 177,4 → 157,1 s | 202,3 → 178,7 s |
| Stres A (bias, derau, delay 0,2 s, udara 2×) | 32,80 → 33,91 | 39,99 → 41,62 | 46,59 → 48,84 | 51,74 → 54,93 |
| Bias 3× + acak 3× | 32,79 → 33,85 | 39,98 → 41,48 | 46,59 → 48,81 | 51,66 → 54,46 |
| Kipas 20% lebih lemah | 32,25 → 33,28 | 39,08 → 40,36 | 45,78 → 47,91 | 49,76 → 52,67 (p10 44,61 → 48,12) |
| Kipas 30% lebih lemah | – | – | 40,09 → **45,38** (p10 31,18 → 43,37) | 44,78 → **49,87** |
| Delay navigasi 0,5 s | 31,78 → 32,97 | 38,52 → 40,10 | 44,74 → 47,59 | 49,11 → 52,58 |
| Ekstrem (bias 15×, acak 13×) | – | – | – | 32,01 → 33,59, tetapi goal 90% → **85%**, p10 23,76 → 21,28 |
| Wrench diabaikan (FAST mati total → SAFE) | 27,91 → 27,73 | 30,94 → 30,88 | 29,51 → 29,48 | 30,72 → 30,64 |

- Nominal, stres A, bias 3×, kipas 0,8×/0,7×, delay 0,5 s: **100% tepat waktu, 100% goal, 100% Lv3** di v4 (seperti v3), dan **p10 naik di semua level**; min juga naik, kecuali kipas 0,8× Normal (37,39 → 37,38).
- **Identitas:** v4 dengan `FAST_ETA_ADAPT=False, FAST_GOV=False` memberi hasil **identik per run** dengan v3 (100/100 run nominal: T dan skor sama). Jadi semua selisih di atas berasal dari dua fitur baru, bukan dari perubahan lain.
- Tabel lengkap (mean, p10, min, goal, Lv3): `tools/results/after_v4_24okt/compare_v3_v4.md`. Perintah: `COMMANDS.txt`.

### 3b. Simulator resmi lokal (Gazebo, `krpc/intball-simulator:latest`)
| Run | Versi | Hasil | Catatan |
|---|---|---|---|
| Easy / Normal / Hard / VH ×2 (23 Okt) | v3_fast | 6/6, 8/8, 10/10, 12/12, 12/12 Lv3; 132,35 / 155,55 / 187,80 / 200,78 / 203,42 s → 32,74 / 40,00 / 45,34 / 51,90 / 51,56 | `official_sim_23okt/` |
| VH run 1 (24 Okt 01:08) | v4_fast | **Tidak sah (simulator kelebihan beban)**: host CPU 100% sepanjang run (beban lain: 14 worker Python + belasan proses meep/ngspice di WSL + container lain). Real-time factor ±0,15, jadi watchdog instance 600 s (waktu dinding) menghentikan misi di detik-sim 97 dengan 3/12 Lv3. Sejak record pertama `localization_history`, IB2 sudah tergeser 0,15 m dan berputar ±130° dari dock, lalu berputar cepat sebelum kontroler kita bisa berbuat apa-apa. FAST masuk *fault* 2× → SAFE (jalur cadangan bekerja); `/ib2_user/stop` → safe stop → 1× `mission_finished` | `official_sim_24okt/veryhard_v4_fast_run1_overload/` |
| VH dan Hard (menunggu CPU host < 65%) | v4_fast | lihat §3b-bis | `official_sim_24okt/` |

### 3c. Kenapa keputusan v3 → v4 tidak bergantung pada rumus skor yang dirahasiakan
Di setiap run nominal dan uji tekan (kecuali "ekstrem"), v4 mencapai jumlah checkpoint Lv3 yang sama (semua) dan goal yang sama, dengan **waktu lebih singkat**. Jadi v4 lebih baik untuk **setiap** pengali waktu yang monoton turun terhadap waktu, berapa pun poin per level. Pengecualian: skenario ekstrem (goal 85% vs 90%) dan "wrench diabaikan" (−0,04 s.d. −0,18).

## 4. Perubahan v4 (24 Okt)
1. **Pembagian anggaran kipas per leg** (`FAST_ETA_ADAPT`). Anggaran tetap 0,95 dari kapasitas kipas, sama dengan v1–v3. Analisis 180 leg: hanya 2 leg yang dibatasi rotasi, rata-rata translasi 20,2 s vs rotasi 8,8 s, dan percepatan translasi efektif ±0,013 m/s² (profil segitiga, waktu ∝ 1/√a). Karena itu tiap leg memberi rotasi porsi secukupnya (η_r ∈ [0,20; 0,40], selesai ≤90% waktu translasi) dan sisanya ke translasi (η_t ≤ 0,75). Perencana dan profil memakai fungsi yang sama, jadi prediksi = profil yang diterbangkan (tes). Kecepatan maks. tetap 0,20 m/s dan ω maks. 25°/s.
2. **Reference governor** (`FAST_GOV`). Kalau galat pelacakan > 0,10 m atau > 8°, jam profil diperlambat (kecepatan ×s, percepatan ×s², s ≥ 0,5). Pulih ke s = 1 setelah galat < 0,06 m / 4°. Governor hanya aktif kalau robot sudah bergerak ≥3 cm/3° sejak profil mulai, supaya aktuator yang mati tetap cepat terdeteksi dan jatuh ke SAFE. Tanpa governor, η_t 0,75 lebih buruk dari v3 di kipas 0,8× (VH 50,14, p10 46,46, 3–4 fault di Easy). Dengan governor: 52,67, 0 fault.
3. **FAST hanya mulai dari kondisi diam** (`START_REST_*`). Kalau saat start robot bergerak > 3 cm/s atau berputar > 5°/s, `ctl` JAXA diminta KEEP_POSE dulu (maks. 12 s), baru FAST. Ini kondisi yang terlihat di run resmi yang kelebihan beban (§3b). Direproduksi offline: tanpa langkah ini FAST *fault* 2× (galat attitude ±100°), T 161,8 s (skor 30,67); dengan langkah ini tidak ada *fault*, T 139,7 s (32,22). Robot yang start dalam keadaan diam tidak terpengaruh (offline: identik per run).
4. **Penyelarasan akhir ArUco** (`ARUCO_ALIGN`, default `"off"`). Mode `"log"` mencatat selisih posisi marker (kamera + PnP) terhadap peta. Mode `"correct"` menggeser checkpoint sebesar selisih itu: maks. 5 cm, ≥3 deteksi dengan sebaran ≤1 cm, lalu profil pendek. Untuk ISS, aktifkan hanya kalau JAXA menilai lewat pengenalan marker. Dengan `off`, kamera tidak disentuh (tes ROS: tidak ada subscriber kamera). Matematika dan ketiga mode diuji offline dengan kamera tiruan; **belum diuji di Gazebo**.
5. Penyesuaian yang **ditolak** setelah diuji: `FAST_OVERHEAD_S` 0,4 → 0,0 (identik per run: angka ini hanya dipakai prediksi, dan rute penuh selalu muat); margin lapor 0,6 → 0,7/0,8 (+0,04…+0,29 di VH, tetapi ada run yang harus melunakkan profil karena lapor lebih awal dengan robot masih bergerak; margin 0,6 dipertahankan demi cadangan pengenalan marker di ISS); η tetap 0,75/0,20 tanpa governor (rapuh di kipas lemah).
6. Tes: `tests/test_all.py` **16/16** (5 baru: pembagian η, matematika ArUco, mode ArUco off/log/correct, governor, start berputar), `tests/test_ros_adapter.py` **5/5** (ditambah cek ArUco default mati). Tes limit-cycle v2 sekarang memeriksa goal pada v4 dan pengamannya pada gerak v3.
7. Arsip: `GaneshaOrbit_v4_fast.tar.gz` (MD5 `4e005df76ab38584c9805949b46ca00b`) / `_v4_safe.tar.gz` (`dc6147b102660e334e7bf783932a12f7`), 30 entri, mode 755, LF. v1–v3 tidak diubah. `tools/official_run_windows.sh` untuk run resmi dari Windows (VNC di port 8080 karena port 80 dipakai `wslrelay`, CPU host dicatat, bag ke Recycle Bin).

## 5. Skor juri (0–10) v3 → v4
| Kriteria (bobot) | J1 | J2 | J3 | J4 | Rata-rata |
|---|---|---|---|---|---|
| A. Ekspektasi skor (55) | 8 → 9 | 7 → 8 | 8 → 9 | 7 → 8 | 7,50 → 8,50 |
| B. Keandalan/variansi (20) | 9 → 9 | 9 → 9 | 9 → 9 | 8 → 7 | 8,75 → 8,50 |
| C. Kepatuhan + kesiapan ISS (15) | 8 → 8 | 9 → 9 | 8 → 8 | 7 → 8 | 8,00 → 8,25 |
| D. Dokumentasi + presentasi (10) | 7 → 8 | 7 → 7 | 6 → 7 | 7 → 8 | 6,75 → 7,50 |
| **Skor berbobot** | | | | | **77,5 → 83,6** |

Alasan singkat dan "apa yang membuat 10/10":
- **A.** J2: +5,9% di VH dan +3,2% di Easy secara nominal, tanpa melanggar batas kecepatan JAXA. Itu trik yang dipakai tim teratas (memakai seluruh otoritas kipas), tetapi tetap dalam anggaran v3. *10/10 kalau* blending antar-leg (leg berikutnya mulai sebelum robot berhenti) dan margin lapor 0,7 juga terbukti aman di Web Simulator.
- **B.** J4 turun 8 → 7: profil v4 lebih agresif dan **belum** lulus simulator resmi dalam kondisi host sehat. Satu-satunya run resmi v4 tidak sah karena host kelebihan beban. Offline, p10 naik di semua skenario realistis. *10/10 kalau* v4 lulus simulator resmi di 4 level dan Web Simulator ARM64 di ≥2 level.
- **C.** J1: kecepatan maks., ω maks., dan anggaran kipas tidak berubah. Governor hanya bisa memperlambat. Jalur safe stop v3 utuh. J4 naik karena FAQ kini menulis `/ctl/wrench` tidak dilarang. *10/10 kalau* JAXA mengonfirmasi tertulis bahwa kontrol wrench boleh di ISS (email `00-STATUS` §0f no. 5).
- **D.** J3: aturan, angka, dan prosedur kirim sudah terdokumentasi dan bisa direproduksi. Belum ada deck presentasi karena format babak nasional belum diumumkan. *10/10 kalau* BRIN mengumumkan format dan deck 5 menit (masalah → ide → bukti → keamanan) sudah disiapkan.

## 6. Checklist review keamanan (untuk JAXA, final ISS)
| # | Butir | Status | Bukti |
|---|---|---|---|
| 1 | Kecepatan translasi ≤ 0,20 m/s, ω ≤ 25°/s (sama dengan profiler `ctl` JAXA) | ✅ tidak berubah di v4 | `CONFIG` `FAST_VMAX`, `FAST_WMAX` |
| 2 | Gaya kipas per kipas ≤ 95% Fmax (guard saturasi) dan anggaran η_t + η_r ≤ 0,95 | ✅ | `FastController.wrench`, tes `test_eta_split…` |
| 3 | Kalau robot tertinggal profil: perlambat, bukan percepat | ✅ governor v4 (s ∈ [0,5; 1]) | tes `test_reference_governor_weak_fans` |
| 4 | Fault pelacakan (0,25 m / 25° selama 1,5 s), navigasi basi (> 0,5 s), atau aktuator tidak merespons → lunakkan profil, lalu SAFE (`ctl` JAXA) | ✅ | MC "wrench diabaikan": 100% goal |
| 5 | Semua jalan keluar (selesai, crash, watchdog 298 s, `/ib2_user/stop`): FAST off → wrench nol 3× → `ctl` KEEP_POSE → tepat 1× `mission_finished` | ✅ (v3, tidak diubah) | tes crash/watchdog/stop; run resmi 24 Okt: `/ib2_user/stop` → safe stop → finish |
| 6 | Tidak membaca `cheat_sheet.json`, tidak mengubah parameter `ctl`, tidak memakai celah goal | ✅ | `docs/STRATEGI_PENILAIAN_JURI.md` §3; `Illegal: false` di semua run resmi |
| 7 | Tidak ada perintah gerak sebelum `/ib2_user/start` atau setelah finish/stop | ✅ | `Mission._send`, tes stop-before-start |
| 8 | Kamera: default mati; mode koreksi dibatasi 5 cm dan butuh 3 deteksi konsisten | ✅ (belum diuji di Gazebo) | tes ArUco |
| 9 | Varian tanpa `/ctl/wrench` siap | ✅ `GaneshaOrbit_v4_safe` / `v3_safe` | |
| 10 | Mulai dari kondisi berputar/bergerak (terlihat di run resmi yang kelebihan beban) | ✅ v4: `ctl` KEEP_POSE sampai diam (maks. 12 s), baru FAST | tes `test_start_settle_when_spinning` |
| 11 | Perilaku di ARM64 (Web Simulator / Int-Ball2) | ⏳ butuh login BRIN | `CARA_KIRIM.md` §1 |

## 7. Fatal flaw (J4) dan penutupnya
| # | Fatal flaw | Status | Cara menutup |
|---|---|---|---|
| F1 | Rumus pengali waktu dan poin per level dirahasiakan (FAQ), jadi angka skor kita model | **Ditutup untuk keputusan** | Argumen dominansi §3c. Angka absolut dikalibrasi dari *Time Bonus* Web Simulator (`CARA_KIRIM` §2) |
| F2 | v4 belum lulus simulator resmi dengan host sehat; profil lebih agresif dari v3 yang sudah lulus | **Terbuka** | Run VH + Hard otomatis saat CPU host < 65% (§3b). Sampai lulus, v3_fast tetap cadangan pertama. Aturan pilih versi: `CARA_KIRIM` §3 |
| F3 | Kontrol `/ctl/wrench` di ISS | **Sebagian ditutup** | FAQ: tidak dilarang. Konfirmasi tertulis lewat email POC. Varian SAFE siap |
| F4 | Celah sim vs ISS (ARM64, kipas nyata, aliran udara) | **Sebagian** | Uji tekan kipas 0,7–0,8×, delay 0,5 s, bias 3×. Web Simulator ARM64 = aksi manusia |

F2 masih terbuka, tetapi tidak fatal untuk penyisihan, karena keputusan upload memakai aturan berbasis bukti (`CARA_KIRIM` §3) dan v3 tersedia. **Veto J4 tidak berlaku.**

## 8. Sisa gap dan siapa yang menutup
| Gap | Siapa | Kapan |
|---|---|---|
| Run simulator resmi v4 dengan host sehat (VH, Hard, lalu Easy/Normal) | Agen/kamu, saat laptop tidak menjalankan beban lain | Begitu CPU bebas |
| Web Simulator ARM64, 4 level + kalibrasi *Time Bonus* | **Kamu** (butuh login BRIN) | Nov 2026 |
| Format babak nasional dan presentasi | **Kamu** (email POC) | Sebelum 2 Nov |
| Tim ≥3 orang (syarat daftar) | **Kamu** | 30 Okt / 2 Nov |
| Konfirmasi `/ctl/wrench` di ISS | JAXA via POC | Sebelum final |
| Uji Gazebo mode ArUco `log` (validasi matematika kamera) | Anggota peran kamera | Nov–Des |

**Plafon sekarang:** ±85 (A tidak bisa 10 tanpa bukti Web Simulator; D tanpa deck). **Plafon setelah aksi manusia** (v4 lulus resmi + Web Simulator, format nasional diketahui + deck, konfirmasi wrench): ±92.
