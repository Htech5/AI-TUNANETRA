import time  
import cv2  
from ultralytics import YOLO  
from gtts import gTTS  
import threading  
import queue  
import RPi.GPIO as GPIO  
import os  
import pygame  # Untuk audio playback  
import serial  
import pynmea2  
import subprocess  
  
# GPIO setup  
GPIO.setmode(GPIO.BCM)  
GPIO.setup(23, GPIO.OUT)  
GPIO.setup(24, GPIO.OUT)  
  
# Model YOLO setup  
model = YOLO('/home/pi/best.pt')  
cap = cv2.VideoCapture(0)  
  
speech_queue = queue.Queue()  
detection_time = {}  
gps_data = None  
  
def speak(text):  
    """Menyuarakan teks menggunakan gTTS."""  
    tts = gTTS(text=text, lang='id')  # Set bahasa ke Indonesia  
    filename = "temp_audio.mp3"  
  
    subprocess.call(["sudo", "chmod", "777", filename])  
    tts.save(filename)  
    pygame.mixer.init()  
    pygame.mixer.music.load(filename)  
    pygame.mixer.music.play()  
    while pygame.mixer.music.get_busy():  
        time.sleep(1)  
    print(f"Suara sudah keluar: {text}")  
    os.remove("temp_audio.mp3")  
  
speak("Mulaiii")  # Menambahkan pengucapan "Mulaiii" di awal program  
  
def read_gps():  
    """Fungsi untuk membaca data GPS."""  
    global gps_data  
    port = "/dev/ttyUSB0"  # Ganti dengan port USB yang sesuai  
    ser = serial.Serial(port, baudrate=9600, timeout=0.5)  
    while True:  
        newdata = ser.readline()  
        if newdata[0:6] == b"$GNRMC":  
            newmsg = pynmea2.parse(newdata.decode('ascii', errors='replace'))  
            lat = newmsg.latitude  
            lng = newmsg.longitude  
            gps_data = f"Latitude={lat} and Longitude={lng}"  
    
            
  
def detect_objects():  
    """Fungsi utama untuk deteksi objek."""  
    if not cap.isOpened():  
        print("Error: Could not open webcam.")  
        speak("kamera tidak terhubung")  
        time.sleep(5)  
        return  
    speak("kamera terhubung")  
    time.sleep(4)  
  
    while True:  
        ret, frame = cap.read()  
        if not ret:  
            print("Error reading frame from webcam.")  
            break  
  
        results = model(frame, conf=0.4)  
        classes = results[0].names  
        detected_classes = results[0].boxes.cls.tolist()  
        confidences = results[0].boxes.conf.tolist()  
  
        current_detected_classes = set()  
  
        for class_id, confidence in zip(detected_classes, confidences):  
            if confidence >= 0.4:  
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
                if elapsed_time >= 2:  
                    speak(class_name)  
#                     if gps_data:  
#                         speak(gps_data)  # Speak the GPS data if available  
                    del detection_time[class_name]  
  
        if cv2.waitKey(1) & 0xFF == ord('q'):  
            break  
  
    GPIO.output(23, GPIO.LOW)  
    GPIO.output(24, GPIO.LOW)  
    print("GPIO 23 dan 24 dimatikan sebelum keluar.")  
    cap.release()  
    GPIO.cleanup()  
  
# Start the GPS reading in a separate thread  
gps_thread = threading.Thread(target=read_gps, daemon=True)  
gps_thread.start()  
  
# Start the object detection  
detect_objects()  

