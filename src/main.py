import serial      
import time      
import string      
import pynmea2      
import cv2      
from ultralytics import YOLO      
from gtts import gTTS      
import threading      
import queue      
import RPi.GPIO as GPIO      
import os      
import pygame  # Make sure to install pygame for audio playback  
  
GPIO.setmode(GPIO.BCM)    
GPIO.setup(23, GPIO.OUT)     
GPIO.setup(24, GPIO.OUT)     
  
model = YOLO('/home/pi/best.pt')      
cap = cv2.VideoCapture(0)     
  
speech_queue = queue.Queue()      
detection_time = {}    
  
def read_gps():    
    port = "/dev/ttyAMA0"      
    ser = serial.Serial(port, baudrate=9600, timeout=0.5)      
    while True:      
        newdata = ser.readline()      
        if newdata[0:6] == b"$GNRMC":      
            newmsg = pynmea2.parse(newdata.decode('ascii', errors='replace'))      
            lat = newmsg.latitude      
            lng = newmsg.longitude      
            gps = f"Latitude={lat} and Longitude={lng}"      
            print(gps)      
  
def speak(text):    
    tts = gTTS(text=text, lang='id')  # Set language to Indonesian  
    filename = "temp_audio.mp3"  
    tts.save(filename)  
    os.system(f"mpg321 {filename}")  # Play the audio file  
    print(f"Suara sudah keluar: {text}")    
  
def detect_objects():    
    if not cap.isOpened():      
        print("Error: Could not open webcam.")      
        return    
    
    desired_width = 640      
    desired_height = 480      
    
    while True:      
        ret, frame = cap.read()      
        if not ret:      
            print("Error reading frame from webcam.")      
            break      
    
        results = model(frame, conf=0.8)      
        classes = results[0].names      
        detected_classes = results[0].boxes.cls.tolist()      
        confidences = results[0].boxes.conf.tolist()      
    
        current_detected_classes = set()      
    
        for class_id, confidence in zip(detected_classes, confidences):      
            if confidence >= 0.8:      
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
                    speak(class_name)  # Call the speak function directly  
                    del detection_time[class_name]      
    
        # Hapus bagian ini untuk tidak menampilkan GUI    
        # resized_frame = cv2.resize(annotated_frame, (desired_width, desired_height))      
        # cv2.imshow('YOLOv8 Detection', resized_frame)      
                      
        if cv2.waitKey(1) & 0xFF == ord('q'):      
            break      
    
    GPIO.output(23, GPIO.LOW)    
    GPIO.output(24, GPIO.LOW)      
    print("GPIO 23 dan 24 dimatikan sebelum keluar.")    
        
    cap.release()      
    GPIO.cleanup()      
  
gps_thread = threading.Thread(target=read_gps)      
gps_thread.start()    
  
detect_objects()  
