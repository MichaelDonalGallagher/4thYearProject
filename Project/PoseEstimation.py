import time
import cv2
import mediapipe as mp
import numpy as np
import tkinter as tk
from tkinter import ttk
import pygame

mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose
mp_holistic = mp.solutions.holistic
pygame.init()
pygame.mixer.music.load('buzzer.wav')


def calculateAngle(a, b, c):
    a = np.array(a)  ## Start
    b = np.array(b)  ## Mid
    c = np.array(c)  ## End

    radians = np.arctan2(c[1] - b[1], c[0] - b[0]) - np.arctan2(a[1] - b[1], a[0] - b[0])
    angle = np.abs(radians * 180.0 / np.pi)

    if angle > 180.0:
        angle = 360 - angle

    return angle


def switchArm():
    print("Hello")


def runCurl():
    main(mp_pose.PoseLandmark.RIGHT_ELBOW.value,
         mp_pose.PoseLandmark.RIGHT_SHOULDER.value,
         mp_pose.PoseLandmark.RIGHT_HIP.value,
         exercise="Curl")


def runSquat():
    main(mp_pose.PoseLandmark.RIGHT_ANKLE.value,
         mp_pose.PoseLandmark.RIGHT_KNEE.value,
         mp_pose.PoseLandmark.RIGHT_HIP.value,
         exercise="Squat")


def runPushUp():
    main(mp_pose.PoseLandmark.RIGHT_KNEE.value,
         mp_pose.PoseLandmark.RIGHT_HIP.value,
         mp_pose.PoseLandmark.RIGHT_SHOULDER.value,
         exercise="PushUp")


def trackAngle(point1, point2, point3, exercise, mistakeCounter):

    if exercise == "Curl":
        if calculateAngle(point1, point2, point3) > 10:
            mistakeCounter = mistakeCounter + 1
            print(mistakeCounter)
            if mistakeCounter % 5 == 0:
                print("Too wide!")
                print(calculateAngle(point1, point2, point3))
                pygame.mixer.music.play()
        else:
            mistakeCounter = 1
    elif exercise == "Squat":
        if calculateAngle(point1, point2, point3) < 90:
            mistakeCounter = mistakeCounter + 1
            print(mistakeCounter)
            if mistakeCounter % 5 == 0:
                print("Too wide!")
                print(calculateAngle(point1, point2, point3))
                pygame.mixer.music.play()
        else:
            mistakeCounter = 1
    elif exercise == "PushUp":
        if calculateAngle(point1, point2, point3) < 170:
            mistakeCounter = mistakeCounter + 1
            print(mistakeCounter)
            if mistakeCounter % 5 == 0:
                print("Too wide!")
                print(calculateAngle(point1, point2, point3))
                pygame.mixer.music.play()
        else:
            mistakeCounter = 1

    return mistakeCounter


def main(startPoint, midPoint, endPoint, exercise):
    # Take video from webcam
    cap = cv2.VideoCapture(2)
    prevTime = 0
    mistakeCounter = 1

    with mp_pose.Pose(
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5) as pose:
        while cap.isOpened():
            success, image = cap.read()
            if not success:
                print("Ignoring empty camera frame.")
                continue

            # Convert the BGR image to RGB.
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            image.flags.writeable = False

            # Store detection
            results = pose.process(image)

            # Convert image back to BGR
            image.flags.writeable = True
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

            # Extract landmarks
            try:
                landmarks = results.pose_landmarks.landmark
                # print(landmarks[mp_pose.PoseLandmark.RIGHT_ELBOW.value])

                # Extract specific landmarks
                point1 = [landmarks[startPoint].x], [
                    landmarks[startPoint].y]
                point2 = [landmarks[midPoint].x], [
                    landmarks[midPoint].y]
                point3 = [landmarks[endPoint].x], [
                    landmarks[endPoint].y]

                mistakeCounter = trackAngle(point1,point2,point3, exercise, mistakeCounter)

            except:
                pass

            # Rendering detections
            mp_drawing.draw_landmarks(
                image, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)

            # Show FPS count
            currTime = time.time()
            fps = 1 / (currTime - prevTime)
            prevTime = currTime
            cv2.putText(image, f'FPS: {int(fps)}', (20, 70), cv2.FONT_HERSHEY_PLAIN, 3, (0, 196, 255), 2)

            cv2.imshow('BlazePose', image)

            if cv2.waitKey(5) & 0xFF == 27:
                break
    cap.release()


win = tk.Tk()

# Creating buttons
btn1 = ttk.Button(
    win,
    text="Bicep Curl",
    command=runCurl,
)

btn2 = ttk.Button(
    win,
    text="Squat",
    command=runSquat,
)

btn3 = ttk.Button(
    win,
    text="Press Up",
    command=runPushUp,
)

# Adding buttons to screen
btn1.pack()
btn2.pack()
btn3.pack()

win.mainloop()
