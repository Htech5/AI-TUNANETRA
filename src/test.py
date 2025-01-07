import cv2
from ultralytics import YOLO
import pyttsx3
import threading
import queue
import time

engine = pyttsx3.init()
voices = engine.getProperty('voices')
for voice in voices:
    if 'indonesia' in voice.name.lower():
        engine.setProperty('voice', voice.id)
        break

model = YOLO('C:/Users/Lenovo/Downloads/AI TUNATERA1/AI TUNATERA/runs/detect/train/weights/best.pt')
cap = cv2.VideoCapture(1)

if not cap.isOpened():
    print("Error: Could not open webcam.")
else:
    speech_queue = queue.Queue()
    detection_time = {}

    def speak():
        while True:
            text = speech_queue.get() 
            if text is None:
                break
            engine.say(text)
            engine.runAndWait()

    speech_thread = threading.Thread(target=speak)
    speech_thread.start()

    desired_width = 640
    desired_height = 480

    while True:
        ret, frame = cap.read()
        
        if not ret:
            print("Error reading frame from webcam.")
            break
        
        results = model(frame, conf=0.8)
        annotated_frame = results[0].plot()
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

        for class_name in list(detection_time.keys()):
            if class_name in current_detected_classes:
                elapsed_time = time.time() - detection_time[class_name]
                if elapsed_time >= 5:
                    speech_queue.put(class_name)
                    del detection_time[class_name]

        resized_frame = cv2.resize(annotated_frame, (desired_width, desired_height))

        cv2.imshow('YOLOv8 Detection', resized_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    speech_queue.put(None)
    speech_thread.join()

    cap.release()
    cv2.destroyAllWindows()
