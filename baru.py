from collections import deque
from imutils.video import VideoStream
import numpy as np
import argparse
import imutils
import cv2
import time
from time import sleep
import RPi.GPIO as GPIO
import serial

in1 = 13
in2 = 16
in3 = 18
in4 = 37
en1 = 32
en2 = 33

GPIO.setwarnings(False)
servoPIN = 26
GPIO.setmode(GPIO.BCM)
GPIO.setup(servoPIN, GPIO.OUT)
i=45
# GPIO.setmode(GPIO.BOARD)
# GPIO.setup(in1,GPIO.OUT)
# GPIO.setup(in2,GPIO.OUT)
# GPIO.setup(in3,GPIO.OUT)
# GPIO.setup(in4,GPIO.OUT)
# GPIO.setup(en1,GPIO.OUT)
# GPIO.setup(en2,GPIO.OUT)

# p1 = GPIO.PWM(en1,1000)
# p2 = GPIO.PWM(en2,1000)
# p1.start(0)
# p2.start(0)

pwm = GPIO.PWM(servoPIN,50)
pwm.start(0)

# def SetAngle(angle):
#     duty = angle/18+2
#     GPIO.output(servoPIN,True)
#     pwm.ChangeDutyCycle(duty)
#     sleep(1)
#     GPIO.output(servoPIN,False)
#     pwm.ChangeDutyCycle(0)


ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video", help="path to the (optional) video file")
ap.add_argument("-b", "--buffer", type=int, default=64,
    help="max buffer size")
args = vars(ap.parse_args())

orangeLower = (0, 215, 104)
orangeUpper = (18, 255, 255)
pts = deque(maxlen=args["buffer"])

camera = cv2.VideoCapture(0)

try:
    res = serial.Serial('/dev/ttyS0', 9600, timeout=1);
    res.flush();

    while True:
        (grabbed, frame) = camera.read()
        
        if args.get("video") and not grabbed:
            break
        
        frame = imutils.resize(frame, width=400)
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv, orangeLower, orangeUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        center = None

        if len(cnts) > 0:
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)

            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            if radius > 10:
                cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                cv2.circle(frame, center, 5, (0, 0, 255), -1)
                
            pts.appendleft(center)
            cv2.putText(frame, ('x='+str(int(x))), (10,60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (50,200,100), 2, cv2.LINE_AA)
            cv2.putText(frame, ('y='+str(int(y))), (10,80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (50,200,100), 2, cv2.LINE_AA)
            cv2.putText(frame, ('z='+str(int(radius))), (10,100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (50,200,100), 2, cv2.LINE_AA)

            if x>0 and x<40:
                res.write("a".encode('utf-8'))
            if x>41 and x<80:
                res.write("b".encode('utf-8'))
            if x>81 and x<120:
                res.write("c".encode('utf-8'))
            if x>121 and x<160:
                res.write("d".encode('utf-8'))
            if x>161 and x<200:
                res.write("e".encode('utf-8'))
            if x>201 and x<240:
                res.write("f".encode('utf-8'))
            if x>241 and x<280:
                res.write("g".encode('utf-8'))
            if x>281 and x<320:
                res.write("h".encode('utf-8'))
            if x>321 and x<360:
                res.write("i".encode('utf-8'))
            if x>361 and x<400:
                res.write("j".encode('utf-8'))
        for i in range(1, len(pts)):
            
            if pts[i - 1] is None or pts[i] is None:
                continue

            thickness = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
            cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)

        cv2.imshow("Frame", frame)
        
        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            res.close()
            break

    camera.release()
    cv2.destroyAllWindows()

except Keyboardinterrupt:
    p1.stop(0)
    p2.stop(0)
    GPIO.cleanup()





