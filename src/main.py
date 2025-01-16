import time
import cv2
from ultralytics import YOLO
from gtts import gTTS
import threading
import queue
import RPi.GPIO as GPIO
import os
import pygame  # Untuk audio playback
import serial  # Untuk GPS serial communication

# GPIO setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(23, GPIO.OUT)
GPIO.setup(24, GPIO.OUT)

# GPS Setup
port = '/dev/ttyUSB0'  # Sesuaikan dengan port USB TTL Anda
baudrate = 9600
gps_serial = serial.Serial(port, baudrate, timeout=1)

# Model YOLO setup
model = YOLO('/home/pi/best.pt')
cap = cv2.VideoCapture(0)

speech_queue = queue.Queue()
detection_time = {}


def speak(text):
    """Menyuarakan teks menggunakan gTTS."""
    tts = gTTS(text=text, lang='id')  # Set bahasa ke Indonesia
    filename = "temp_audio.mp3"
    tts.save(filename)
    pygame.mixer.init()
    pygame.mixer.music.load(filename)
    pygame.mixer.music.play()
    while pygame.mixer.music.get_busy():
        time.sleep(1)
    print(f"Suara sudah keluar: {text}")
speak("Mulaiii")  # Menambahkan pengucapan "Mulaiii" di awal program


def get_gps_coordinates():
    """Membaca data NMEA dari GPS dan mengembalikan latitude dan longitude."""
    try:
        line = gps_serial.readline().decode('ascii', errors='ignore').strip()
        if line.startswith('$GNGGA') or line.startswith('$GPRMC'):
            parts = line.split(',')
            if parts[2] and parts[4]:  # Data latitude dan longitude tersedia
                # Latitude
                raw_lat = parts[2]
                lat_dir = parts[3]
                latitude = convert_to_decimal(raw_lat, lat_dir)

                # Longitude
                raw_lon = parts[4]
                lon_dir = parts[5]
                longitude = convert_to_decimal(raw_lon, lon_dir)

                return latitude, longitude
    except Exception as e:
        print(f"Error membaca GPS: {e}")
    return None, None


def convert_to_decimal(raw_value, direction):
    """Mengonversi koordinat NMEA menjadi derajat desimal."""
    if not raw_value or not direction:
        return None
    degrees = float(raw_value[:2])
    minutes = float(raw_value[2:])
    decimal = degrees + (minutes / 60)
    if direction in ['S', 'W']:
        decimal = -decimal
    return decimal


def detect_objects():
    """Fungsi utama untuk deteksi objek dan pembacaan GPS."""

    if not cap.isOpened():
        print("Error: Could not open webcam.")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error reading frame from webcam.")
            break

        results = model(frame, conf=0.6)
        classes = results[0].names
        detected_classes = results[0].boxes.cls.tolist()
        confidences = results[0].boxes.conf.tolist()

        current_detected_classes = set()

        for class_id, confidence in zip(detected_classes, confidences):
            if confidence >= 0.6:
                class_name = classes[int(class_id)]
                current_detected_classes.add(class_name)

                if class_name not in detection_time:
                    detection_time[class_name] = time.time()

                GPIO.output(23, GPIO.HIGH)
                GPIO.output(24, GPIO.HIGH)
                print("GPIO 23 dan 24 dinyalakan.")

        if not current_detected_classes:
            GPIO.output(23, GPIO.LOW)
            GPIO.output(24, GPIO.LOW)
            print("GPIO 23 dan 24 dimatikan.")
        else:
            GPIO.output(23, GPIO.HIGH)
            GPIO.output(24, GPIO.HIGH)

        for class_name in list(detection_time.keys()):
            if class_name in current_detected_classes:
                elapsed_time = time.time() - detection_time[class_name]
                if elapsed_time >= 5:
                    speak(class_name)
                    del detection_time[class_name]

        # GPS Integration
        latitude, longitude = get_gps_coordinates()
        if latitude is not None and longitude is not None:
            gps_message = f"Lokasi Anda adalah, Latitude: {latitude}, Longitude: {longitude}"
            print(gps_message)
            speak(gps_message)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    GPIO.output(23, GPIO.LOW)
    GPIO.output(24, GPIO.LOW)
    print("GPIO 23 dan 24 dimatikan sebelum keluar.")
    cap.release()
    GPIO.cleanup()


detect_objects()
