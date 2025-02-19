import time
import serial
import cv2
import numpy as np
import RPi.GPIO as GPIO
import adafruit_rplidar
from adafruit_ads1x15.analog_in import AnalogIn
from adafruit_ads1x15.ads1115 import ADS1115
import board
import busio
from geopy.distance import geodesic
from ultralytics import YOLO

# GPIO setup
ULTRASONIC_TRIG = 23
ULTRASONIC_ECHO = 24
PROXIMITY_SENSOR = 25
CRANE_CONTROL = 26
BUZZER = 27

GPIO.setmode(GPIO.BCM)
GPIO.setup(ULTRASONIC_TRIG, GPIO.OUT)
GPIO.setup(ULTRASONIC_ECHO, GPIO.IN)
GPIO.setup(PROXIMITY_SENSOR, GPIO.IN)
GPIO.setup(CRANE_CONTROL, GPIO.OUT)
GPIO.setup(BUZZER, GPIO.OUT)

# Adjustable threshold distances
BUZZER_HIGH_INTENSITY = 50  # cm
BUZZER_LOW_INTENSITY = 100  # cm

# Serial setup for GSM module
ser_gsm = serial.Serial("/dev/ttyS0", baudrate=115200, timeout=1)

def get_gps_coordinates():
    ser_gsm.write(b'AT+CGPSINFO\r\n')
    time.sleep(2)
    response = ser_gsm.read(ser_gsm.inWaiting()).decode()
    if "+CGPSINFO:" in response:
        data = response.split(",")
        if len(data) >= 4:
            lat = float(data[1])
            lon = float(data[3])
            return lat, lon
    return None

# LIDAR setup
lidar = adafruit_rplidar.RPLidar(None, '/dev/ttyUSB0', baudrate=115200)

# Webcam setup
cap = cv2.VideoCapture(0)
yolo_model = YOLO("yolov8n.pt")

def detect_obstacles():
    distances = []
    for scan in lidar.iter_scans():
        for (_, angle, distance) in scan:
            if 0 < distance < 5000:  # Objects within 2m
                distances.append(distance)
    return min(distances) if distances else None

def ultrasonic_distance():
    GPIO.output(ULTRASONIC_TRIG, True)
    time.sleep(0.00001)
    GPIO.output(ULTRASONIC_TRIG, False)
    start_time, end_time = time.time(), time.time()
    while GPIO.input(ULTRASONIC_ECHO) == 0:
        start_time = time.time()
    while GPIO.input(ULTRASONIC_ECHO) == 1:
        end_time = time.time()
    return ((end_time - start_time) * 34300) / 2  # cm

def detect_cranes():
    ret, frame = cap.read()
    if not ret:
        return False
    results = yolo_model(frame)
    for result in results:
        if 'crane' in result.names:  # Assuming YOLO model is trained for cranes
            return True
    return False

def stop_crane():
    GPIO.output(CRANE_CONTROL, GPIO.LOW)
    GPIO.output(BUZZER, GPIO.HIGH)

def slow_down_crane():
    GPIO.output(CRANE_CONTROL, GPIO.HIGH)
    GPIO.output(BUZZER, GPIO.LOW)

def buzzer_intensity(distance):
    if distance < BUZZER_HIGH_INTENSITY:
        GPIO.output(BUZZER, GPIO.HIGH)
    elif distance < BUZZER_LOW_INTENSITY:
        GPIO.output(BUZZER, GPIO.LOW)
    else:
        GPIO.output(BUZZER, GPIO.LOW)

try:
    while True:
        lidar_distance = detect_obstacles()
        ultrasonic_dist = ultrasonic_distance()
        proximity_detected = GPIO.input(PROXIMITY_SENSOR)
        crane_nearby = detect_cranes()
        
        if lidar_distance and lidar_distance < 100:  # Less than 1m
            stop_crane()
        elif ultrasonic_dist < 50 or proximity_detected:  # Less than 50cm
            slow_down_crane()
        elif crane_nearby:
            stop_crane()
        
        buzzer_intensity(min(lidar_distance if lidar_distance else 1000, ultrasonic_dist))
        time.sleep(1)
except KeyboardInterrupt:
    print("Shutting down")
    GPIO.cleanup()
    lidar.stop()
    lidar.disconnect()
    cap.release()
    cv2.destroyAllWindows()
