# CARA_KIRIM — Kibo-RPC ke-7, babak penyisihan (Web Simulator)
Ditulis 24 Okt 2026. Semua jam dalam **WIB** (JST = WIB + 2 jam). **Kamu sendiri yang login, upload, dan menekan SUBMIT.** Agen tidak login ke mana pun.

## 0. Ringkas
| Hal | Isi |
|---|---|
| Portal | https://jaxa.krpc.jp/user-auth/ → **SIMULATION** (login dari POC BRIN, datang ±3 hari kerja setelah formulir dikirim) |
| File utama | `submission/GaneshaOrbit_v4_fast.tar.gz` (MD5 arsip `4e005df76ab38584c9805949b46ca00b`, `user_template.py` `53c94ff7aa3131384b03a0d0724f3eda`) |
| Cadangan 1 | `submission/GaneshaOrbit_v3_fast.tar.gz` (MD5 `35eb74af0f26d4d81e54bf0262f2d44b`), sudah lulus simulator resmi lokal di 4 level |
| Cadangan 2 | `submission/GaneshaOrbit_v4_safe.tar.gz` (MD5 `dc6147b102660e334e7bf783932a12f7`), hanya `ctl` bawaan |
| Target internal | **Jum 15 Jan 2027**: hasil berstatus **FINISHED** di layar *Preliminary* sudah di-**SUBMIT** |
| Tenggat resmi | Rab 20 Jan 2027, 23:59 JST = **21:59 WIB**. Sesudahnya tombol SUBMIT/CANCEL hilang (Rulebook v1.1 §3.1) |
| Kuota | 120 run per bulan; hanya 20 hasil yang disimpan, jadi hapus hasil lama kalau penuh (Guidance Session 23 Okt) |

## 1. Begitu ID/PW datang (Nov 2026)
1. Login di https://jaxa.krpc.jp/user-auth/ → pastikan *My Page* tampil.
2. **SIMULATION** → drag `GaneshaOrbit_v4_fast.tar.gz` ke form.
   - *Simulator Level* = **Very Hard**
   - *Simulator Version* = versi terbaru (v1.0 atau lebih baru)
   - *Memo* = `v4_fast VH 1`
3. **START SIMULATION** → **OK**. Tunggu status **Finished**.
4. **VIEW RESULT** → catat: *Final Score*, *Mission Time*, *Time Bonus*, *CheckPoints Passed*, *Goal Reached*, level per checkpoint. **DOWNLOAD LOG FILES**.
5. Ulangi untuk **Hard**, **Normal**, **Easy** (masing-masing 1 run). Lalu satu run **v3_fast Very Hard** sebagai pembanding. Total 5 run dari kuota 120.

## 2. Cek log (cari `[GaneshaOrbit]`)
| Baris | Arti | Tindakan |
|---|---|---|
| `leg -> cp N (fast) ... eta_t=0.75 eta_r=0.20` diikuti `REPORT ... (Lv3 held (fast))` | v4 berjalan normal | – |
| `governor s=` di baris `FAST profile softened` | Robot sempat tertinggal profil, governor memperlambat. Wajar kalau 0–2 kali per run | Catat checkpoint mana. Kalau muncul di **hampir semua leg**, kipas Web Simulator lebih lemah dari model: coba `v4` dengan `FAST_ETA_T_MAX_FRAC` 0,737 (v5) |
| `switching to SAFE mode` | FAST gagal di leg itu, sisa misi pakai `ctl` | Catat alasannya. Kalau terjadi di ≥1 dari 4 level: pakai **v3_fast** |
| `SAFE STOP (mission finished)` tepat 1×, tanpa `mission crashed` / `deadline watchdog` | Jalur berhenti aman normal | – |
| `SUMMARY` + `leg cpN fast pred= actual=` | Kalibrasi waktu | Jalankan `python tools/analyze_results.py <folder unduhan>` |

**Kalibrasi pengali waktu.** Rumus *Time Bonus* tidak diumumkan (FAQ). Dari VIEW RESULT, bagi *Final Score* dengan jumlah poin checkpoint+goal untuk mendapatkan pengali, lalu bandingkan dengan model kita 1 + (300 − T)/300. Catat di `docs/00-STATUS…` §0.

## 3. Aturan pilih versi (setelah 5 run di §1)
1. v4_fast **Finished** di keempat level, goal tercapai, semua Lv3, tanpa `switching to SAFE` → **submit v4_fast**.
2. v4_fast bermasalah (SAFE, Lv2, goal gagal, atau skor < v3_fast di level yang sama) → **submit v3_fast**, kirim log v4 untuk analisis.
3. v3_fast juga bermasalah → **v4_safe**.
4. Status **FAILED** (build/launch gagal): download log, cari error pertama. Upload `v4_safe`. Kalau ikut gagal, masalahnya di arsip/lingkungan; jangan ubah apa pun sebelum lognya dianalisis.

## 4. Submit (paling lambat Jum 15 Jan 2027)
1. Sebelum babak penyisihan, layar hasil mendapat simulator **"Preliminary"**. Jalankan versi pilihan (§3) di simulator *Preliminary* itu (Rulebook v1.1 §3.1).
2. **RESULTS** → pilih hasil *Preliminary* versi itu yang statusnya **FINISHED** → **SUBMIT**. Tombol berubah jadi **CANCEL**.
3. Screenshot halaman RESULTS (tombol CANCEL terlihat), simpan di `tools/results/web_sim/`.
4. Ganti pilihan: **CANCEL** → SUBMIT hasil lain, selama sebelum tenggat. Jangan lakukan di hari terakhir.
5. Catatan: hanya hasil **FINISHED** yang bisa di-SUBMIT. Kalau hanya ada FAILED, tim tidak ikut penyisihan.

## 5. Checklist H-1 (Kam 14 Jan 2027)
- [ ] Masih terdaftar sebagai mahasiswa (syarat saat submit, FAQ)
- [ ] Login Web Simulator jalan; sisa kuota bulan Jan ≥ 5 run
- [ ] MD5 arsip yang di-upload sama dengan tabel §0 (`md5sum submission/*.tar.gz`)
- [ ] Simulator *Preliminary* sudah tersedia dan versi pilihan sudah **FINISHED** di sana
- [ ] Log run itu: tidak ada `switching to SAFE`, `mission crashed`, `deadline watchdog`
- [ ] Belum ada perubahan kode setelah run terakhir. Kalau ada: `python tests/test_all.py` (16/16) + `python tests/test_ros_adapter.py` (5/5) + arsip baru `v5`, lalu ulangi §1
- [ ] Anggota tim final dikirim ke POC (anggota baru **tidak bisa ditambah** setelah penyisihan selesai, 17 Feb 2027)
- [ ] Format babak nasional (daring/luring, presentasi atau tidak) sudah ditanyakan ke POC BRIN. Guidance Session 23 Okt: *"factors other than the simulation score may also be evaluated"* tergantung negara

## 6. Setelah submit
- Simpan semua log Web Simulator di `tools/results/web_sim/<tanggal>_<level>_<versi>/`.
- Finalis (Mar–Apr 2027): JAXA me-review kode untuk keamanan (Rulebook §4.1). Bawa `JURI_90/SKOR.md` §6 (checklist keamanan) dan siapkan v4_safe kalau JAXA meminta tanpa `/ctl/wrench`.
