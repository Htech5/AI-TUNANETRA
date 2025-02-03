# AI Tongkat Tunetetra

AI Tongkat Tunetetra adalah inovasi tongkat untuk tunanetra yang dilengkapi dengan GPS dan kamera. Proyek ini menggunakan teknologi computer vision untuk mendeteksi objek di sekitar dan memberikan informasi suara kepada pengguna.

## Fitur

- Deteksi objek menggunakan model YOLO.
- Pembacaan data GPS.
- Suara pengumuman objek yang terdeteksi.
- Kontrol GPIO untuk menghidupkan/mematikan perangkat keras.

## Prasyarat

Sebelum memulai, pastikan Anda memiliki perangkat keras dan perangkat lunak berikut:

### Perangkat Keras

- Raspberry Pi (model yang mendukung GPIO)
- Kamera USB
- Modul GPS (misalnya, NEO-6M)
- Speaker atau modul audio

### Perangkat Lunak

- Python 3.x
- PIP (Python Package Installer)

## Instalasi

Ikuti langkah-langkah berikut untuk menginstal semua dependensi yang diperlukan:

1. **Perbarui dan tingkatkan sistem Anda:**

   ```
   bash
   sudo apt update
   sudo apt upgrade
   ```

2. **Instal Python dan PIP:**

   Jika Python belum terinstal, Anda dapat menginstalnya dengan perintah berikut:

   ```
   bash
   sudo apt install python3 python3-pip
   ```

3. **Instal pustaka yang diperlukan:**

   Jalankan perintah berikut untuk menginstal semua pustaka yang diperlukan:

   ```
   bash
   pip3 install opencv-python
   pip3 install ultralytics
   pip3 install gtts
   pip3 install pygame
   pip3 install pyserial
   pip3 install pynmea2
   ```

4. **Instal YOLOv8:**

   Anda perlu mengunduh model YOLOv8. Anda dapat mengunduh model yang telah dilatih (misalnya, `best.pt`) dari repositori resmi YOLO atau melatih model Anda sendiri. Tempatkan file model di direktori yang sesuai, misalnya `/home/pi/best.pt`.

5. **Konfigurasi GPIO:**

   Pastikan Anda telah mengonfigurasi GPIO pada Raspberry Pi Anda. Anda mungkin perlu menginstal pustaka RPi.GPIO jika belum terinstal:

   ```
   bash
   sudo apt install python3-rpi.gpio
   ```

## Penggunaan

1. **Jalankan program:**

   Setelah semua dependensi terinstal, Anda dapat menjalankan program dengan perintah berikut:

   ```
   bash
   python3 your_script_name.py
   ```

   Gantilah `your_script_name.py` dengan nama file Python Anda.

2. **Berinteraksi dengan program:**

   Program akan mulai mendeteksi objek dan memberikan informasi suara. Pastikan kamera dan modul GPS terhubung dengan benar.

## Catatan

- Pastikan Anda menjalankan program dengan hak akses yang sesuai, terutama jika Anda menggunakan GPIO dan perangkat keras lainnya.
- Anda mungkin perlu menyesuaikan pengaturan GPIO dan port serial sesuai dengan perangkat keras yang Anda gunakan.
