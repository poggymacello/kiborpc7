# Status tim Ganesha Orbit + panduan pendaftaran (20 Okt 2026)

Dokumen ini jawaban untuk dua pertanyaan: **"sudah sejauh apa?"** dan **"apa yang harus saya lakukan, urut, dengan tanggal?"**.
Semua jam ditulis dalam **WIB**. JST = WIB + 2 jam.

---

## 0-bis. Update Sabtu 24 Okt 2026 dini hari: v4 (baca ini dulu)
Ringkasan lengkap: [`../JURI_90/SKOR.md`](../JURI_90/SKOR.md) (skor juri, tabel v3 vs v4, checklist keamanan) dan [`../JURI_90/CARA_KIRIM.md`](../JURI_90/CARA_KIRIM.md) (langkah Web Simulator, file mana, tenggat).
- **Rulebook dibaca ulang dari situs resmi.** Rumus pengali waktu **tidak diumumkan**. FAQ (23 Okt) menulis: rumus skor + Time Bonus dirahasiakan, poin per level tidak publik, **`/ctl/wrench` tidak dilarang**. Materi Guidance Session 23 Okt: tiap negara boleh menilai faktor lain selain skor simulator, jadi tanyakan format babak nasional ke POC.
- **v4** = v3 + pembagian anggaran kipas per leg (anggaran 0,95 tetap) + *reference governor* + FAST baru mulai setelah robot diam (kalau robot bergerak/berputar saat start, `ctl` KEEP_POSE dulu) + opsi kamera ArUco (mati). Monte Carlo dengan seed yang sama, nominal: **33,94 / 41,63 / 48,72 / 54,77** (v3: 32,88 / 40,02 / 46,48 / 51,70), p10 juga naik di semua level, tetap 100% Lv3 + goal. Kalau kedua fitur dimatikan, hasilnya identik per run dengan v3.
- Tes: **16/16** + **5/5**. Arsip baru: `submission/GaneshaOrbit_v4_fast.tar.gz` (MD5 `4e005df7…`, `user_template.py` `53c94ff7…`) dan `_v4_safe`. v1–v3 tidak diubah.
- Simulator resmi v4: run VH pertama **tidak sah**. Host CPU 100% karena beban lain di laptop, real-time factor ±0,15, sehingga watchdog 600 s menghentikan misi. Run VH + Hard dijadwalkan otomatis saat CPU < 65% (hasil di `SKOR.md` §3b dan `SIM_RESMI_LANGKAH.md` §0-bis). **Sampai v4 lulus, v3_fast tetap cadangan pertama.**

## 0. Update Jumat 23 Okt 2026
Sisa waktu: **12 hari ke tenggat formulir BRIN (Rab 4 Nov, 14:59 WIB; target kirim Sen 2 Nov)** dan 89 hari ke tenggat submit program (Rab 20 Jan 2027).

### 0a. Yang selesai hari ini
| Hal | Hasil |
|---|---|
| **Bug keamanan diperbaiki (v3)** | Di v2, kalau `Mission.run()` crash, program hanya mengirim `mission_finished`. Mode FAST tetap menyala, kontroler wrench terus menggerakkan kipas, dan `ctl` tidak pernah diminta menahan posisi. Sekarang **semua cara misi berakhir memakai satu rutin `safe_stop()`**: FAST dimatikan → wrench nol dikirim 3× (jarak 20 ms) → `ctl` diminta KEEP_POSE → baru `mission_finished` (tepat satu kali, dijaga lock). Rutin yang sama dipakai oleh `/ib2_user/stop` dan oleh **watchdog tenggat baru**. Watchdog ini thread terpisah: kalau thread misi macet, ia memaksa safe stop + `mission_finished` di detik 298. Saat misi selesai normal, `mission_finished` tetap dikirim lebih dulu karena kejar waktu, lalu safe stop. Kalau misi belum mulai, safe stop tidak mengirim perintah ke `ctl`, supaya robot yang belum kita gerakkan tidak ikut bergerak |
| Tes | `tests/test_all.py`: **11/11 lolos** (6 lama + 5 baru untuk jalur crash). `tests/test_ros_adapter.py`: **5/5 cek lolos** (1 lama + 4 baru: crash, `/ib2_user/stop`, watchdog, stop sebelum start). Satu tes baru mereproduksi bug v2 (kontroler tetap aktif setelah crash) supaya perbaikannya terbukti |
| Monte Carlo offline | 1.020 run, 8 skenario dan seed yang sama dengan 20 Okt. Hasil v3 **identik per run** dengan v2 (§0b). Data: `tools/results/after_v3_23okt/` |
| Arsip submit baru | `submission/GaneshaOrbit_v3_fast.tar.gz` (**utama**) dan `_v3_safe` (cadangan). v1 dan v2 tetap disimpan. Isi arsip sama dengan v2 kecuali `user_template.py` (30 entri, mode 755, LF, `#!/usr/bin/python3`). MD5 `user_template.py`: v3_fast `f2eb3cb5c56d61ba500536671561d8c7`, v3_safe `57dfd629002458eb300e169356565cb3` |
| Simulator resmi lokal | **Sudah dijalankan di 4 level (sampai 09:20 WIB). v3_fast LULUS semuanya.** Semua run: `Illegal: false`, semua checkpoint Lv3, goal sukses (skor = model rulebook kita). Easy 6/6, 132,35 s (32,74). Normal 8/8, 155,55 s (40,00). Hard 10/10, 187,80 s (45,34). Very Hard 12/12, 200,78 s (51,90) dan seed kedua 203,42 s (51,56). Di run yang sehat, FAST tidak pernah jatuh ke SAFE, `SAFE STOP (mission finished)` tepat 1×, tanpa crash/watchdog. Normal dan Hard run 1 gagal (0/8 dan 8/10) karena **plugin navigasi Gazebo mati** saat CPU laptop penuh. Bag membuktikan *ground truth* nav juga berhenti, jadi **bukan bug program**; run ulang lulus. Model waktu pesimis 0,73 s/leg; `FAST_OVERHEAD_S` sengaja tidak diubah. **Tidak ada perubahan kode.** Dijalankan dari Windows `docker.exe`, tanpa WSL integration. Detail: [`SIM_RESMI_LANGKAH.md`](SIM_RESMI_LANGKAH.md) §0. Log: `tools/results/official_sim_23okt/` |
| Dokumen | `RUNBOOK.md` §3: path sekarang `KIBO="$(wslpath 'C:\Users\POGGY\Downloads\barengpoggy\KiboRPC7')"`, tidak lagi `/mnt/c/Users/Ari Azis/...`. File lama `../kibo-rpc-7-2026.md` diberi banner **USANG**, dan fakta yang salah sudah dikoreksi (tenggat 14 Okt, Python 2.7, rencana `/ctl/command`) |

### 0b. Monte Carlo sebelum (v2, 20 Okt) dan sesudah (v3, 23 Okt), FAST
Skor rata-rata FAST per level (E / N / H / VH). Kolom terakhir: berapa run yang identik (T, skor, level per checkpoint, goal, fault) dari semua run skenario itu, termasuk SAFE dan baseline.

| Skenario | v2 (20 Okt) | v3 (23 Okt) | Run identik |
|---|---|---|---|
| Nominal (25 seed) | 32,88 / 40,02 / 46,48 / 51,70 | 32,88 / 40,02 / 46,48 / 51,70 | 300/300 |
| Bias 2,5× + acak 1,7× + delay 0,2 s + udara 2× | 32,80 / 39,99 / 46,59 / 51,74 | 32,80 / 39,99 / 46,59 / 51,74 | 160/160 |
| Bias 3× + acak 3× + udara 3× | 32,79 / 39,98 / 46,59 / 51,66 | 32,79 / 39,98 / 46,59 / 51,66 | 80/80 |
| Kipas 20% lebih lemah | 32,25 / 39,08 / 45,78 / 49,76 | 32,25 / 39,08 / 45,78 / 49,76 | 160/160 |
| Kipas 30% lebih lemah | – / – / 40,09 / 44,78 | – / – / 40,09 / 44,78 | 80/80 |
| Ekstrem (bias 15×, acak 13×) | – / – / – / 32,01 | – / – / – / 32,01 | 40/40 |
| Wrench diabaikan | 27,91 / 30,94 / 29,51 / 30,72 | 27,91 / 30,94 / 29,51 / 30,72 | 40/40 |
| Delay navigasi 0,5 s | 31,78 / 38,52 / 44,74 / 49,11 | 31,78 / 38,52 / 44,74 / 49,11 | 160/160 |

**Kesimpulan: tidak ada regresi.** Total 1.020/1.020 run identik. Perubahan v3 hanya aktif saat crash, macet, atau stop, dan tidak menyentuh jalur normal. Perintah lengkap: `tools/results/after_v3_23okt/COMMANDS.txt`, perbandingan per run: `compare_v2_v3.txt`.

### 0c. Yang hanya bisa kamu lakukan (urut)
| # | Paling lambat | Apa | Bahan |
|---|---|---|---|
| 1 | **Hari ini → Jum 30 Okt** | Rekrut **≥4 adik tingkat** yang masih mahasiswa ITB sampai **Jun 2027**. Target 5 anggota (kamu + 4). Minimal 3 orang supaya bisa daftar | Pesan siap kirim §0d |
| 2 | **Sab 31 Okt** | Pilih **Sub-Leader** (Member 2) di antara mereka. Dia kontak cadangan dan penerus ketua setelah kamu wisuda | Kriteria §0e |
| 3 | **Sen 2 Nov** (tenggat Rab 4 Nov 14:59 WIB) | Isi dan **Submit formulir BRIN** sendiri: https://forms.gle/4keLSMoDKW8AHHC8A | Jawaban per kolom §4. Yang masih kosong ada di §0e |
| 4 | Minggu ini (sebelum Sen 2 Nov) | Kirim **email pertanyaan ke POC BRIN** (roll marker, `/ctl/wrench` di ISS, format penyisihan nasional, tenggat). Kalau sudah terjawab di Guidance Session 21 Okt, catat jawabannya di sini dan hapus pertanyaan itu | Email siap kirim §0f |
| 5 | ~~Nov~~ **Selesai 23 Okt** | ~~Build simulator resmi, lalu jalankan `v3_fast` di Very Hard~~ Lulus di keempat level (VH 2 seed). Opsional: batch banyak seed, saat laptop tidak sedang menjalankan beban berat lain (image sudah ada di Docker Desktop) | [`SIM_RESMI_LANGKAH.md`](SIM_RESMI_LANGKAH.md) §0 |
| 6 | ±3 hari kerja setelah daftar | Cek email (dan spam) untuk ID/PW Web Simulator, lalu jalankan checklist §8 dengan **v3_fast** | §8 |
| 7 | Jum 15 Jan 2027 | Submit di layar *Preliminary* (status FINISHED). Tenggat resmi 20 Jan 21:59 WIB | §8 |

### 0d. Pesan rekrut, siap kirim (WA/Line grup angkatan atau DM)
Ganti satu bagian **[ISI]**, lalu kirim. Kirim juga DM satu per satu ke 6–8 orang yang kamu kenal bisa diandalkan. Grup angkatan saja biasanya sepi balasan.

> Halo, aku Poggy ([ISI: jurusan + angkatan], ITB). Aku lagi bentuk tim ITB untuk **Kibo Robot Programming Challenge ke-7** (JAXA × NASA, di Indonesia lewat BRIN). Tugasnya memprogram robot Int-Ball2 supaya terbang ke beberapa titik di modul Kibo dalam 5 menit. Penyisihan pakai simulator online. **Program pemenang nasional dijalankan di robot asli di ISS.**
>
> Programnya sudah jalan dan sudah lolos lebih dari 1.000 run uji di simulator offline kami, jadi kamu tidak mulai dari nol. Yang dibutuhkan: orang yang mau memegang satu bagian (kontrol gerak, uji simulator resmi, kamera/marker, atau strategi rute) dan menguji di simulator resmi.
>
> Komitmen: ±4–6 jam/minggu dari Nov sampai pertengahan Jan (libur saat UAS 7–22 Des). Submit penyisihan 20 Jan 2027. Kalau jadi finalis, Mar–Apr 2027 merevisi kode untuk ISS.
>
> Syarat: **masih mahasiswa ITB sampai Juni 2027**, bisa Python dasar. Aku lulus April, jadi tim nanti dilanjutkan kalian.
>
> Pendaftaran tutup **Rabu 4 Nov**, jadi aku butuh jawaban paling lambat **Jumat 30 Okt**. Kalau minat, balas "ikut", nanti aku kirim daftar data yang dibutuhkan formulir.

**Pesan lanjutan untuk yang bilang "ikut"** (data ini wajib di formulir BRIN):
> Makasih! Tolong kirim paling lambat Jumat 30 Okt:
> 1. Nama lengkap (sesuai KTM)
> 2. Umur per 4 Nov 2026
> 3. Jurusan + angkatan, dan perkiraan bulan lulus (harus setelah Jun 2027)
> 4. Email aktif
> 5. Kewarganegaraan
> 6. Pernah ikut Kibo-RPC sebelumnya? (ya/tidak)
> 7. Peran yang kamu mau: kontrol / uji simulator / kamera-marker / rute
>
> Catatan: satu orang hanya boleh ada di satu tim Kibo-RPC.

### 0e. Sub-Leader dan kolom formulir yang masih kosong
**Kriteria Sub-Leader (Member 2):** angkatan paling muda yang tetap aktif sampai Jun 2027 (idealnya 2024–2025), responsif di chat, dan bersedia jadi ketua setelah kamu wisuda. Dia juga memegang email cadangan untuk login Web Simulator. Kalau bisa, pilih yang mengambil peran "uji simulator", karena dialah yang paling sering memegang hasil run.

**Kolom formulir (§4) yang masih harus kamu isi sendiri:**
| Kolom | Isi |
|---|---|
| Number of team members | Jumlah sebenarnya (3–8). Target 5 |
| Member 1: Zip code and Address | Kode pos dulu, lalu alamat lengkap (untuk kiriman sertifikat) |
| Member 2 (Sub-Leader) | Nama, email, umur, kampus (Institut Teknologi Bandung), kewarganegaraan, riwayat ikut |
| Member 3–5 | Nama, umur, kampus, kewarganegaraan, riwayat ikut (*This is my first time participating* kalau belum pernah) |
| How did you find out / What media | Pilih sesuai kenyataan |
| Consent | Centang *Yes* **setelah membaca** Terms and Conditions / Entry Description |

Kolom lain sudah terisi di §4 (nama tim, perwakilan mahasiswa, data Member 1, non-EU). Setelah Submit, perubahan apa pun dikirim lewat email ke POC, bukan dengan mengisi formulir lagi.

### 0f. Email pertanyaan ke POC BRIN (siap kirim)
Alamat email POC tampil sebagai gambar di kartu Indonesia di https://jaxa.krpc.jp/ (Happy Rumiris Simanungkalit, Dessi Marlia). Salin dari sana. Pertanyaan teknis ditulis dalam bahasa Inggris supaya BRIN bisa langsung meneruskannya ke JAXA. Email ini menggantikan draf §5c.

> **Subjek:** Pertanyaan pendaftaran dan teknis 7th Kibo-RPC (tim Ganesha Orbit, ITB)
>
> Yth. Ibu Happy Rumiris Simanungkalit dan Ibu Dessi Marlia,
>
> Saya Poggy Macello Gultom dari Institut Teknologi Bandung, perwakilan tim Ganesha Orbit, yang akan mendaftar 7th Kibo-RPC (Regular Course) sebelum tenggat. Mohon bantuan Ibu untuk pertanyaan berikut. Pertanyaan 4–6 bersifat teknis; kalau perlu diteruskan ke JAXA, silakan gunakan teks bahasa Inggris di bawah.
>
> **Pendaftaran dan babak nasional**
> 1. Situs resmi (berita 15 Okt 2026) menulis tenggat pendaftaran 4 Nov 2026 pukul 14:59 "in each country's standard time", tetapi deskripsi formulir Indonesia masih menulis 14 Okt. Apakah tenggat untuk Indonesia 4 Nov 2026 pukul 14:59 WIB?
> 2. Untuk perwakilan mahasiswa, formulir tidak menampilkan pilihan Course. Apakah tim kami otomatis tercatat di Regular Course?
> 3. Bagaimana format babak penyisihan nasional (21 Jan – 17 Feb 2027): daring atau luring, di mana, dan apakah peringkat nasional hanya memakai skor Web Simulator (rata-rata 10 run) atau ada presentasi/penilaian tambahan? Apakah tanggalnya sudah ditetapkan?
>
> **Technical questions for JAXA**
> 4. *Checkpoint roll convention.* The marker quaternions in Rulebook v1.1 Table 2.2.2-2 differ from those in the simulator's `area_config.json` by an in-plane rotation (e.g. marker 1: Rulebook (-0.707, 0, 0, 0.707) vs. config (-0.5, 0.5, 0.5, 0.5)); the marker normals are identical. For the roll-about-line-of-sight condition in Web Simulator v1.0 and in the ISS Final Round, is the reference always `attitude.quaternion` of each checkpoint returned by `/mission/get_checkpoints`?
> 5. *`/ctl/wrench` on the ISS.* Programming Manual §6.3.2 documents `/ctl/wrench` as a user API. Is a program that controls Int-Ball2 directly via `/ctl/wrench` (speed limited to 0.2 m/s, fan force below the hardware limit, automatic fallback to `/ctl/command`, and on stop/exception: zero wrench + KEEP_POSE) allowed in the ISS Final Round Run, or must finalists use `/ctl/command` only? Are there additional limits on force, torque or speed?
> 6. *Stop handling.* On `/ib2_user/stop` our program disables its wrench controller, publishes a zero wrench and sends a `KEEP_POSE` goal on `/ctl/command` before responding. Is this the expected behaviour, or should the user program only cancel its running action?
>
> Terima kasih atas waktu dan bantuannya.
>
> Hormat saya,
> Poggy Macello Gultom · Institut Teknologi Bandung · +62 853-6104-8752 · gultompoggy@gmail.com

---

## 1. Ringkasan singkat
- **Program sudah jadi dan lolos semua tes offline.** Hari ini ada perbaikan kecil (v2) untuk satu kelemahan yang muncul di uji tekan. File baru: `submission/GaneshaOrbit_v2_fast.tar.gz` (utama) dan `_v2_safe` (cadangan). File v1 tetap disimpan.
- **Yang belum ada: pendaftaran, anggota tim, dan uji di simulator resmi.** Ketiganya tergantung kamu, bukan kode.
- **Tenggat pertama yang penting: formulir BRIN, Rabu 4 Nov 2026.** Kirim paling lambat **Senin 2 Nov**.

## 2. Kesiapan (perkiraan, bukan angka resmi)
| Bagian | Bobot | Siap | Alasan |
|---|---|---|---|
| Kode program | 25% | 90% | Semua tes lolos. Belum pernah jalan di simulator resmi (Gazebo/ARM64) |
| Tes + simulasi offline | 10% | 95% | 25 seed × 4 level + 7 uji tekan, sebelum dan sesudah v2 (§6) |
| Validasi simulator resmi (lokal/web) | 20% | 0% | Web Simulator butuh login dari BRIN. Simulator lokal: WSL2 Ubuntu + Docker Desktop sudah terpasang di laptop ini, tapi belum pernah dijalankan |
| Pendaftaran (formulir) | 20% | 30% | Jawaban sudah disiapkan (§4). Belum dikirim |
| Tim (anggota) | 20% | 20% | Baru kamu. Minimal 3 orang untuk daftar, dan **≥3 adik tingkat** supaya tim tetap sah setelah kamu wisuda |
| Dosen pembimbing | 5% | 0% | **Tidak wajib** untuk tim mahasiswa (lihat §3). Tetap disarankan |
| **Total** | | **≈ 42%** | |

## 3. Hasil cek situs resmi (20 Okt, hanya membaca)
| Hal | Hasil |
|---|---|
| Berita baru setelah 15 Okt | **Tidak ada.** Berita terakhir tetap 15 Okt (tenggat diperpanjang, Web Simulator v1.0, Rulebook Beginner) |
| Tenggat pendaftaran | Situs menulis **"Nov 04, 2026 (14:59) in each country's standard time"**. Data di balik situs berisi `2026-11-04T14:59:59Z`, yaitu 23:59 JST = 21:59 WIB. Bacaan paling aman: **4 Nov 14:59 WIB** |
| Formulir BRIN | Masih menerima jawaban. Deskripsinya masih menulis 14 Okt 23:59 JST (belum diperbarui) |
| Rulebook | Masih **v1.1 (15 Okt)**, hash file sama dengan yang dicek 16 Okt |
| Programming Manual | Masih **v1.0 (5 Okt)** |
| Simulator lokal + sample code | Hash sama dengan file di `vendor/` (`c90937a3…`, `fba02f40…`) |
| FAQ | Tidak ada perubahan. Poin penting: (1) saat **submit program** kamu harus masih terdaftar sebagai mahasiswa; (2) **anggota baru tidak bisa ditambah setelah babak penyisihan selesai** |
| Guidance Session ke-2 | **Rabu 21 Okt, 16:00–17:00 WIB**, Zoom. Daftar lewat form Microsoft di berita 15 Okt (kamu yang daftar) |

**Koreksi soal dosen pembimbing.** Di formulir BRIN, halaman *Supervisor's Information* hanya muncul kalau perwakilan memilih *Guardian / Teacher*. Kalau kamu memilih *Team member (Undergraduate to Master student…)*, formulir langsung lompat ke "Number of team members". Guidebook §3.3 juga menulis data guardian/teacher hanya untuk tim siswa SMA ke bawah. Jadi **dosen tidak wajib diisi**. Halaman *Course Selection* juga ikut terlewati, karena Beginner Course hanya untuk tim yang semua anggotanya usia SMP ke bawah. Kemungkinan besar tim kamu otomatis tercatat di **Regular Course** (bisa dikonfirmasi lewat email §5c).

### Bahasa dan runtime program (dicek dari Manual + simulator)
| Hal | Fakta | Program kita |
|---|---|---|
| Bahasa | **Python 3** (default, `user_template.py`) atau **C++14** (`user_template_node`) | Python ✅ |
| Versi | Ubuntu 18.04 / ROS 1 **Melodic**, **Python 3.6.9**, GCC 7.5.0 | Lolos cek sintaks Python 3.6 (`ast.parse(feature_version=(3,6))`) dan `vermin` (minimum 3.0). Tanpa f-string, walrus, atau dataclass ✅ |
| Shebang + baris | `#!/usr/bin/python3`, file dijalankan langsung oleh `roslaunch` | Sama, akhir baris LF, mode file 755 di arsip ✅ |
| Isi arsip upload | `packing.sh` = `tar -zcvf <nama>.tar.gz catkin_ws/src` (berisi `krpc_user`, `krpc_api_server`, `krpc_ib2_msgs`, symlink `CMakeLists.txt`) | `make_submission.py` menyalin persis dari arsip resmi dan hanya mengganti `user_template.py`. 30 entri, struktur sama ✅ |
| Yang tidak boleh diubah | Nama paket, `CMakeLists.txt`, `package.xml`, nama launch file, `krpc_api_server`, `krpc_ib2_msgs` | Tidak diubah ✅ |
| Arsitektur Web Simulator | **ARM64** | Python murni, tidak ada kode yang dikompilasi khusus ✅ |
| Batas waktu | 5 menit sejak undock, **laporan selesai harus masuk sebelum batas** | `T_LIMIT_S = 300`, cadangan 15 s ✅ |
| Penilaian checkpoint | Posisi + marker di tengah kamera + roll sekitar sumbu kamera. Level = yang terburuk. **Ditahan 1 detik** | Evaluator identik dengan `judge.py` (6.000 pose acak), `HOLD_S = 1,0` ✅ |
| Skor | (jumlah poin checkpoint + poin goal) × pengali waktu, **maks. 2×** | Lv3/2/1 = 3/2/1 poin (dari `judge.py`). Poin goal = 3 dan rumus pengali linear adalah **asumsi** kita, karena skor akhir dihitung di server JAXA |
| Jumlah run penyisihan | **10 run otomatis**, kondisi acak dan gangguan berbeda, skor = **rata-rata** | Simulasi offline memakai rata-rata banyak seed ✅ |
| Syarat submit | Hanya hasil berstatus **FINISHED** di layar *Preliminary* yang bisa di-SUBMIT. Bisa CANCEL lalu submit ulang sampai tenggat | Dicatat di checklist §8 |

**Ketidakcocokan yang ditemukan:** tidak ada di kode atau format arsip. Satu-satunya koreksi ada di dokumen: dosen pembimbing ternyata tidak wajib (lihat di atas). Catatan kecil: `docs/RUNBOOK.md` §2–3 dulu memakai path `/mnt/c/Users/Ari Azis/KiboRPC7` *(23 Okt: sudah diganti dengan `wslpath`)*. Langkah "pasang WSL2" bisa dilewati karena Ubuntu (WSL versi 2) dan Docker Desktop sudah ada.

## 4. Jawaban formulir, kolom per kolom
Link: https://forms.gle/4keLSMoDKW8AHHC8A. **Kamu sendiri yang mengisi dan menekan Submit.** Kalau ada yang berubah setelah kirim, jangan isi ulang formulir. Kirim email ke POC BRIN.

| Kolom | Jawaban |
|---|---|
| Email Anda (halaman 1) | gultompoggy@gmail.com |
| Team Name | Ganesha Orbit |
| Representative (pilihan) | **Team member (Undergraduate to Master student, Level University (Age: 17 - 25 yo))** |
| Supervisor's Information | *(halaman ini tidak muncul untuk pilihan di atas)* |
| Course | *(halaman ini tidak muncul; otomatis Regular)*. Kalau ternyata muncul, pilih **Regular Course** |
| Number of team members | **[ISI: jumlah sebenarnya, 3–8]**. Saran: 5 (kamu + 4 adik tingkat) |
| Member 1 (Representative): Name | Poggy Macello Gultom |
| Member 1: Zip code and Address | **[ISI: mulai dengan kode pos, lalu alamat lengkap]** (dipakai untuk kirim sertifikat) |
| Member 1: Phone Number | +62 853-6104-8752 |
| Member 1: E-mail | gultompoggy@gmail.com (boleh dikosongkan kalau sama dengan halaman 1; diisi saja supaya jelas) |
| Member 1: Age | 22 |
| Member 1: School Name | Institut Teknologi Bandung |
| Member 1: Nationality | Indonesia |
| Member 1: Previous participation | This is my first time participating |
| Member 2 (Sub-Leader): Name / E-mail / Age / School / Nationality / Previous | **[ISI]**. Pilih adik tingkat yang paling bisa diandalkan, karena dia jadi kontak cadangan dan **penerus ketua setelah kamu wisuda** |
| Member 3–5: Name / Age / School / Nationality / Previous | **[ISI]** |
| How did you find out about Kibo-RPC? | **[ISI: pilih yang benar]**. Kalau dari situs JAXA: *JAXA website* |
| What media have you seen about Kibo-RPC? | **[ISI: centang yang benar]** |
| Participants from/reside in the EU? | No |
| Consent | Centang *Yes* **setelah kamu membaca** Terms and Conditions / Entry Description |

**Aturan anggota yang wajib diingat**
- Minimal **3 orang** saat daftar. Anggota bisa ditambah nanti lewat email ke POC (nama, kampus, kewarganegaraan, umur, riwayat ikut), tapi **tidak bisa setelah babak penyisihan selesai (17 Feb 2027)**.
- Kamu kehilangan status peserta begitu tidak lagi terdaftar sebagai mahasiswa (sekitar wisuda 22–24 Apr 2027; **[cek ke Direktorat Pendidikan ITB tanggal status mahasiswamu berakhir]**). Run ISS final dijadwalkan April–awal Mei 2027 (situs: Apr, Guidebook: early May).
- Jadi **minimal 3 anggota harus tetap berstatus mahasiswa sampai Juni 2027**. Paling aman 4 adik tingkat, karena kalau satu mundur, tim tetap sah.
- Satu mahasiswa tidak boleh ikut dua tim.

## 5. Draf pesan
Pesan di bawah sengaja singkat. Ganti bagian **[ISI]** dengan fakta, lalu ubah kalimatnya sesuai gayamu.

### 5a. WhatsApp rekrut adik tingkat (angkatan 2023–2025, EL/IF/MS/AE)
> Halo, aku Poggy ([ISI: jurusan + angkatan]). Aku lagi bentuk tim ITB untuk **Kibo Robot Programming Challenge ke-7** (JAXA × NASA, di Indonesia lewat BRIN). Tugasnya memprogram robot Int-Ball2 supaya terbang ke beberapa titik di modul Kibo dalam 5 menit. Penyisihan pakai simulator online. Pemenang nasional programnya **dijalankan di robot asli di ISS**.
>
> Programnya sudah jalan: rute optimal, kontrol gerak sendiri, dan sudah diuji lebih dari 1.000 run di simulator offline buatan kami. Jadi kamu tidak mulai dari nol. Yang dibutuhkan: orang yang mau memegang satu bagian dan menguji di simulator resmi.
>
> Komitmen kira-kira:
> - Nov–pertengahan Jan: ±4–6 jam/minggu (libur saat UAS 7–22 Des). Submit penyisihan 20 Jan.
> - Kalau jadi finalis: Mar–Apr, revisi kode untuk ISS sesuai masukan JAXA.
>
> Pilihan peran:
> - **Kontrol**: tuning gerak dan kestabilan robot (cocok EL/AE/MS)
> - **Uji + simulator**: jalankan simulator resmi (Docker), baca log, bandingkan skor
> - **Navigasi/kamera**: cek marker ArUco dan pose kamera (cocok IF/EL)
> - **Perencanaan rute**: uji kasus sulit dan strategi skip checkpoint (siapa saja yang suka algoritma)
>
> Syarat: masih mahasiswa ITB sampai **Juni 2027**, bisa Python dasar. Aku lulus April, jadi tim nanti dilanjutkan kalian. Pendaftaran tutup **4 Nov**, jadi aku butuh nama lengkap, umur, dan jurusan paling lambat **[ISI: tanggal, mis. Jumat 30 Okt]**. Minat? Balas di sini atau DM ya.

### 5b. Email/WA ke calon dosen pembimbing (opsional)
Sasaran: dosen bidang kontrol/robotika (STEI–Teknik Elektro) atau teknik komputer/sistem tertanam. Contoh tempat bertanya: laboratorium kontrol/robotika di STEI, atau dosen yang membimbing tim robotik ITB. **Jangan sebut nama dosen sebelum kamu cek sendiri bidang dan ketersediaannya.**

> Subjek: Permohonan menjadi pembimbing tim ITB di Kibo Robot Programming Challenge (JAXA)
>
> Yth. Bapak/Ibu [ISI: nama dosen],
>
> Saya Poggy Macello Gultom, mahasiswa [ISI: jurusan, angkatan] ITB. Saya sedang membentuk tim **Ganesha Orbit** untuk Kibo Robot Programming Challenge ke-7, lomba pemrograman robot dari JAXA yang di Indonesia dikelola BRIN. Peserta memprogram robot Int-Ball2 di simulator. Pemenang tiap negara programnya dijalankan di robot asli di ISS.
>
> Program kami sudah berjalan di simulator offline. Kontrol geraknya memakai PD dengan profil translasi dan rotasi paralel, dan ada mode cadangan yang hanya memakai kontroler bawaan JAXA. Kami ingin meminta Bapak/Ibu menjadi dosen pembimbing. Bentuknya: masukan teknis 2–3 kali (Nov–Jan), dan kalau kami lolos ke final, membantu saat review keamanan kode oleh JAXA (Mar–Apr 2027). Dari sisi lomba, data pembimbing tidak wajib untuk tim mahasiswa, jadi tidak ada beban administrasi.
>
> Kalau Bapak/Ibu bersedia, saya bisa menjelaskan program dan jadwalnya dalam 15 menit, di kampus atau daring. Terima kasih atas waktunya.
>
> Hormat saya,
> Poggy Macello Gultom · [ISI: NIM] · +62 853-6104-8752 · gultompoggy@gmail.com

### 5c. Email ke POC BRIN (opsional, untuk konfirmasi tenggat)
Alamat email POC tampil sebagai gambar di kartu Indonesia di https://jaxa.krpc.jp/ (Happy Rumiris Simanungkalit, Dessi Marlia). Salin dari sana.

> Subjek: Konfirmasi tenggat pendaftaran 7th Kibo-RPC Indonesia (tim Ganesha Orbit, ITB)
>
> Yth. Ibu Happy Rumiris Simanungkalit dan Ibu Dessi Marlia,
>
> Saya Poggy Macello Gultom dari Institut Teknologi Bandung, perwakilan tim Ganesha Orbit. Situs resmi Kibo-RPC (berita 15 Okt 2026) menulis tenggat pendaftaran diperpanjang menjadi 4 Nov 2026 pukul 14:59. Deskripsi formulir Indonesia masih menulis 14 Okt 2026. Mohon konfirmasi:
> 1. Apakah perpanjangan sampai 4 Nov berlaku untuk Indonesia, dan pukul berapa dalam WIB?
> 2. Apakah sudah ada tanggal dan format (daring/luring) babak penyisihan nasional 21 Jan–17 Feb 2027?
> 3. Untuk perwakilan mahasiswa, formulir tidak menampilkan pilihan Course. Apakah tim kami otomatis tercatat di Regular Course?
>
> Terima kasih.
> Poggy Macello Gultom · +62 853-6104-8752 · gultompoggy@gmail.com

## 6. Hasil tes dan simulasi (20 Okt)
Perintah: `python tests/test_all.py`, `python tests/test_ros_adapter.py`, `tools/offline_sim.py`. Data mentah: `tools/results/before_20okt/` (v1) dan `tools/results/after_v2_20okt/` (v2).

**Tes:** semua lolos, sebelum dan sesudah perubahan (evaluator == `judge.py` resmi pada 6.000 pose; generator == resmi; planner == brute force; model waktu == profiler; end-to-end OK; adaptor ROS OK). Tes baru: `test_attitude_limit_cycle_guard`.

**Nominal, 25 seed per level (rata-rata skor offline; v1 = v2, angka identik):**
| Level | FAST | SAFE | Pola sample code |
|---|---|---|---|
| Easy | 32,88 (130 s) | 27,99 | 24,43 |
| Normal | 40,02 (155 s) | 31,58 | 8,41 (68% run lewat 5 menit) |
| Hard | 46,48 (177 s) | 30,32 | 0 (selalu lewat 5 menit) |
| Very Hard | 51,70 (202 s) | 31,64 | 0 |
FAST: 100% tepat waktu, 100% goal, 100% checkpoint Lv3 di keempat level.

**Uji tekan, 20 seed per level, FAST:**
| Gangguan | v1 | v2 | Catatan |
|---|---|---|---|
| Bias 2,5× + acak 1,7× + delay 0,2 s + aliran udara 2× | sama | sama | VH 51,74, 100% Lv3 |
| Bias 3× + acak 3× | sama | sama | VH 51,66 |
| Kipas 20% lebih lemah | sama | sama | VH 49,76 |
| Kipas 30% lebih lemah (Hard/VH) | sama | sama | VH 44,78 |
| Gangguan ekstrem (bias 15×, acak 13×) | sama | sama | VH 32,01, goal 90% (karena fault, lalu pindah ke SAFE) |
| Wrench diabaikan (uji gagal total) | sama | sama | Otomatis pindah ke SAFE, goal 100% |
| **Delay navigasi 0,5 s** | E 30,66 · N 37,33 · H 43,79 · VH 48,53; goal 95% di N dan H | **E 31,78 · N 38,52 · H 44,74 · VH 49,11; goal 100%, Lv3 100%** | Ini satu-satunya yang berubah |

**Kelemahan yang ditemukan dan perbaikannya (v2).** Dengan delay navigasi 0,5 s, kontrol attitude FAST berosilasi di checkpoint goal (marker 12). Roll bolak-balik ±30–50°, jadi Lv3 tidak pernah bertahan 1 detik. Program lalu tidak melapor goal padahal waktu masih sisa >130 s (normal seed 16: skor 30,61; hard seed 18: 36,68).
Perbaikan: kalau setelah profil selesai robot **masuk-keluar jendela Lv3 ≥2 kali tanpa bisa bertahan 1 detik**, gain laju attitude diturunkan ×0,5 untuk sisa misi, dan satu refine tambahan diizinkan. Hasilnya dua run tadi naik ke 39,37 dan 44,85. Di kondisi nominal dan uji tekan lain, pengaman ini tidak pernah aktif (angkanya identik sampai dua desimal). Gain yang diturunkan secara permanen sempat dicoba, tapi skor dengan kipas 30% lebih lemah turun (VH min 42,69 → 29,77), jadi tidak dipakai.
Catatan: mode SAFE (kontroler bawaan JAXA) juga kehilangan goal di 5–10% run pada delay 0,5 s. Bagian itu tidak bisa kita ubah tanpa mengganti `ctl`, jadi dibiarkan.

**File submit:**
| File | Isi |
|---|---|
| `submission/GaneshaOrbit_v2_fast.tar.gz` | **Utama.** FAST + pengaman osilasi, fallback otomatis ke SAFE |
| `submission/GaneshaOrbit_v2_safe.tar.gz` | Cadangan, hanya `ctl` bawaan |
| `submission/GaneshaOrbit_v1_fast.tar.gz`, `_v1_safe` | Versi 10 Okt, disimpan untuk perbandingan |

## 7. Tanggal penting
| Tanggal (WIB) | Apa | Catatan |
|---|---|---|
| Rab 21 Okt, 16:00–17:00 | Guidance Session ke-2 (Zoom) | Daftar sendiri lewat form di berita 15 Okt. Tanyakan konvensi roll marker (lihat `CEK_RULEBOOK_v1.1`) dan apakah `/ctl/wrench` boleh dipakai di final ISS |
| Jum 30 Okt | Batas internal: nama adik tingkat terkumpul | |
| **Sen 2 Nov** | **Target kirim formulir** | Cadangan 2 hari |
| **Rab 4 Nov, 14:59** | **Tenggat formulir BRIN** | Bacaan paling aman; mungkin sebenarnya 21:59 WIB |
| ±3 hari kerja setelah kirim | ID/PW datang lewat email | Cek folder spam. Kalau 1 minggu belum ada, hubungi POC |
| Nov | Uji Web Simulator (§8) + simulator lokal | |
| 7–22 Des | **UAS ITB** | Jangan rencanakan kerja tim |
| 23 Des – 15 Jan | Iterasi terakhir | |
| Jum 15 Jan 2027 | **Target internal submit penyisihan** | Pastikan status FINISHED |
| Rab 20 Jan, 10:00 | Batas pertanyaan ke JAXA | 12:00 JST |
| **Rab 20 Jan, 21:59** | **Tenggat submit program** | 23:59 JST |
| 21 Jan – 17 Feb | Babak penyisihan nasional (BRIN) | Tanggal acara BRIN belum diumumkan. Setelah ini anggota tidak bisa ditambah |
| 30–31 Jan | **YLI Forum 3, Jakarta** (tetap) | Submit 20 Jan sudah selesai sebelumnya. Kalau acara BRIN jatuh 30–31 Jan, beri tahu POC lebih awal |
| Awal Mar – pertengahan Apr | Khusus finalis: revisi kode untuk ISS | Simulator ISS rilis pertengahan Maret |
| 22–24 Apr | Wisuda ITB | Kamu keluar dari roster, tim lanjut dengan ≥3 adik tingkat |
| Apr – awal Mei | Run final di ISS | Satu kali run |
| Jun 2027 | Acara final | |

## 8. Checklist Web Simulator (begitu login dari BRIN datang)
1. Login di https://jaxa.krpc.jp/user-auth → **SIMULATION**. Pastikan *Simulator Version* = versi terbaru (v1.0 atau lebih baru), *Course* = Regular.
2. *(24 Okt: ikuti [`../JURI_90/CARA_KIRIM.md`](../JURI_90/CARA_KIRIM.md). File utama `GaneshaOrbit_v4_fast.tar.gz`, pembanding `v3_fast`.)* Upload `submission/GaneshaOrbit_v3_fast.tar.gz` *(23 Okt: v3 menggantikan v2)*, level **Very Hard**, isi Memo "v3_fast VH". START → OK.
3. Kalau status **FAILED** (gagal build/launch): download log, cari baris error pertama. Lalu upload `_v3_safe`. Kalau SAFE juga gagal, masalahnya di arsip atau lingkungan, bukan di mode gerak. Kirim log ke saya.
4. Kalau **Finished**: VIEW RESULT → catat Final Score, Mission Time, Time Bonus, CheckPoints Passed, Goal Reached, level per checkpoint. **DOWNLOAD LOG FILES**.
5. Di log program cari `[GaneshaOrbit]`:
   - `leg -> cp N (fast)` lalu `REPORT … (Lv3 held (fast))` = FAST jalan ✅
   - `switching to SAFE mode` = FAST gagal. Catat alasannya (`tracking error`, `navigation stale`, `did not respond to /ctl/wrench`)
   - `attitude limit cycle` = pengaman v2 aktif. Catat checkpoint mana. Ini tanda delay navigasi di server cukup besar
   - Baris `SUMMARY` + `leg cpN … pred= actual=` untuk kalibrasi
6. Bandingkan dengan tabel §6 (VH: ±202 s, skor ±51,7, 12/12 Lv3). Selisih waktu per leg (`actual − pred`) dimasukkan ke `FAST_OVERHEAD_S` di `CONFIG`. Jalankan `python tools/analyze_results.py <folder unduhan>`.
7. Ulangi untuk Easy, Normal, Hard. Lalu jalankan v2_fast satu kali di Very Hard. Hasilnya seharusnya sama, karena v3 hanya mengubah jalur crash/stop/watchdog. Di log v3 harus ada satu baris `SAFE STOP (mission finished)`, dan tidak boleh ada `mission crashed` atau `deadline watchdog`.
8. Aturan pilih versi:
   - FAST Finished, goal tercapai, tanpa `switching to SAFE` → pakai **v3_fast**
   - FAST sering pindah ke SAFE atau skornya di bawah SAFE → pakai **v3_safe**, kirim log ke saya untuk dianalisis
9. Kalau ada ubahan kode: `python tests/test_all.py` → `python tools/make_submission.py --name GaneshaOrbit_v4_fast --mode fast` (nomor versi berikutnya; v3 sudah ada).

## 9. Yang harus kamu lakukan, urut
1. **Hari ini–21 Okt:** daftar Guidance Session (form Microsoft di berita 15 Okt), ikut 21 Okt 16:00 WIB.
2. **20–30 Okt:** kirim pesan §5a ke adik tingkat. Kumpulkan **≥4 nama** lengkap dengan umur, jurusan, dan angkatan. Pilih satu jadi Sub-Leader.
3. **Opsional, 20–30 Okt:** hubungi calon dosen (§5b). Tidak menghambat pendaftaran.
4. **Opsional:** email POC BRIN (§5c) untuk konfirmasi jam tenggat.
5. **Paling lambat Sen 2 Nov:** isi formulir sesuai §4 (kode pos + alamat, data anggota), baca Terms, lalu Submit.
6. **±5–9 Nov:** cek email (dan spam) untuk ID/PW. Kalau 1 minggu belum datang, email POC.
7. **Setelah login datang:** jalankan checklist §8, kirim hasil + log ke saya.
8. **Nov:** nyalakan Docker Desktop, ikuti [`SIM_RESMI_LANGKAH.md`](SIM_RESMI_LANGKAH.md) / RUNBOOK §3 (23 Okt: path sudah otomatis lewat `wslpath`), dan bagi peran ke anggota (RUNBOOK §7).
9. **Paling lambat 15 Jan 2027:** submit di layar *Preliminary* (status FINISHED). Tenggat resmi 20 Jan 21:59 WIB.
