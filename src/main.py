import time
import cv2
from ultralytics import YOLO
import threading
import queue
import RPi.GPIO as GPIO
import serial
import signal
import pyttsx3

# GPIO setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(23, GPIO.OUT)
GPIO.setup(24, GPIO.OUT)

# GPS Setup
port = '/dev/ttyUSB0'  # Adjust to your USB TTL port
baudrate = 9600
gps_serial = serial.Serial(port, baudrate, timeout=1)

# YOLO Model setup
model = YOLO('/home/pi/best.pt')
cap = cv2.VideoCapture(0)

# Queues for threading
speech_queue = queue.Queue()

# Signal handler for safe exit
def signal_handler(sig, frame):
    GPIO.cleanup()
    cap.release()
    print("Cleanup done. Exiting...")
    os._exit(0)

signal.signal(signal.SIGINT, signal_handler)

# TTS worker using pyttsx3
def tts_worker():
    """Process speech queue using pyttsx3."""
    engine = pyttsx3.init()
    engine.setProperty('rate', 150)
    engine.setProperty('volume', 1)
    while True:
        text = speech_queue.get()
        if text is None:  # Exit signal
            break
        engine.say(text)
        engine.runAndWait()

# Start TTS thread
tts_thread = threading.Thread(target=tts_worker, daemon=True)
tts_thread.start()

# Add speech to the queue
def speak(text):
    """Add text to the speech queue."""
    speech_queue.put(text)

# GPS worker function
def gps_worker():
    """Continuously read GPS data and print coordinates."""
    while True:
        latitude, longitude = get_gps_coordinates()
        if latitude and longitude:
            gps_message = f"Your location is Latitude: {latitude}, Longitude: {longitude}"
            print(gps_message)
            # Uncomment below if you want to announce the GPS coordinates
            # speak(gps_message)
        time.sleep(2)

def get_gps_coordinates():
    """Read NMEA data from GPS and return latitude and longitude."""
    try:
        line = gps_serial.readline().decode('ascii', errors='ignore').strip()
        if line.startswith('$GNGGA') or line.startswith('$GPRMC'):
            parts = line.split(',')
            if parts[2] and parts[4]:  # Data latitude and longitude available
                raw_lat = parts[2]
                lat_dir = parts[3]
                latitude = convert_to_decimal(raw_lat, lat_dir)

                raw_lon = parts[4]
                lon_dir = parts[5]
                longitude = convert_to_decimal(raw_lon, lon_dir)

                return latitude, longitude
    except Exception as e:
        print(f"Error reading GPS: {e}")
    return None, None

def convert_to_decimal(raw_value, direction):
    """Convert NMEA coordinates to decimal degrees."""
    if not raw_value or not direction:
        return None
    degrees = float(raw_value[:2])
    minutes = float(raw_value[2:])
    decimal = degrees + (minutes / 60)
    if direction in ['S', 'W']:
        decimal = -decimal
    return decimal

# Object detection worker
def detection_worker():
    """Continuously process frames for object detection."""
    if not cap.isOpened():
        speak("Camera not connected")
        print("Error: Could not open webcam.")
        return
    speak("Camera connected")
    print("Starting object detection...")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error reading frame from webcam.")
            break

        # Perform YOLO detection
        results = model(frame, conf=0.4)
        detected_classes = results[0].boxes.cls.tolist()
        confidences = results[0].boxes.conf.tolist()
        class_names = results[0].names

        current_detected_classes = set()

        for class_id, confidence in zip(detected_classes, confidences):
            if confidence >= 0.4:
                class_name = class_names[int(class_id)]
                current_detected_classes.add(class_name)
                print(f"Detected: {class_name} with confidence {confidence:.2f}")
                GPIO.output(23, GPIO.HIGH)
                GPIO.output(24, GPIO.HIGH)

        if not current_detected_classes:
            GPIO.output(23, GPIO.LOW)
            GPIO.output(24, GPIO.LOW)

        # Announce detected objects
        for class_name in current_detected_classes:
            speak(class_name)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    GPIO.output(23, GPIO.LOW)
    GPIO.output(24, GPIO.LOW)
    cap.release()
    GPIO.cleanup()

# Start detection and GPS threads
detection_thread = threading.Thread(target=detection_worker, daemon=True)
gps_thread = threading.Thread(target=gps_worker, daemon=True)

detection_thread.start()
gps_thread.start()

# Main thread waits for other threads to complete
detection_thread.join()
gps_thread.join()
speech_queue.put(None)  # Signal TTS thread to stop
tts_thread.join()
