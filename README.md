# Kinematika Kaki Robot 2-DoF

## Profil

-   Nama: Muhammad Daffa Izdihar
-   NIM: 25/560183/SV/26399

## Deskripsi

Program ini dibuat untuk menghitung kinematika kaki robot 2-DoF. Forward
Kinematics (FK) digunakan untuk mencari posisi sendi lutut dan ujung
kaki dari sudut kedua servo. Perhitungan FK menggunakan Homogeneous
Transformation Matrix dengan NumPy.

Program juga menyediakan Inverse Kinematics (IK) untuk mencari sudut
servo berdasarkan koordinat target. Sebelum menghitung IK, program
memeriksa apakah target masih berada dalam jangkauan robot.

Panjang segmen yang digunakan:

  Segmen     Panjang
  -------- ---------
  Coxa          0 cm
  Femur        83 cm
  Tibia        99 cm

## Fitur

  Fitur                                            Status
  ------------------------------------------------ -----------
  Forward Kinematics 2-DoF                         Selesai
  Homogeneous Transformation Matrix dengan NumPy   Selesai
  Visualisasi postur dengan Matplotlib             Selesai
  Menyimpan hasil visualisasi ke PNG               Selesai
  Inverse Kinematics                               Selesai
  Pengecekan out of workspace                      Selesai
  Verifikasi hasil IK menggunakan FK               Selesai
  Mathematical Proof                               Terlampir

## Struktur File

``` text
.
├── main.py
├── requirements.txt
├── README.md
└── documentation/
    ├── MathematicalProof.pdf
    ├── fk_40.0_30.0.png
    ├── ik_100.0_100.0.png
    └── testScreenshot.png
```

## Instalasi dan Cara Menjalankan

Pastikan Python sudah terpasang. Install library yang dibutuhkan dengan
perintah:

``` bash
pip install -r requirements.txt
```

Jalankan program:

``` bash
python main.py
```

### Pengujian Forward Kinematics

1.  Pilih menu `1`.
2.  Masukkan sudut Servo 1: `40`.
3.  Masukkan sudut Servo 2: `30`.

Hasil koordinat yang diharapkan:

-   Sendi lutut: sekitar `(63.5817, 53.3514)` cm.
-   End-effector: sekitar `(97.4417, 146.3809)` cm.

Program akan menampilkan hasil perhitungan dan menyimpan gambar postur
kaki sebagai file PNG.

### Pengujian Inverse Kinematics

1.  Pilih menu `2`.
2.  Masukkan target X: `100`.
3.  Masukkan target Y: `100`.

Program akan menghitung sudut servo, lalu memeriksa hasilnya kembali
menggunakan Forward Kinematics. Grafik hasilnya juga disimpan sebagai
file PNG.

## Dokumentasi

Folder `documentation` berisi mathematical proof, hasil visualisasi FK
dan IK, serta screenshot pengujian program.
