# -*- coding: utf8 -*-
import cv2
import socket
import numpy
import sys

faceCascade = cv2.CascadeClassifier('C:\opencv\sources\data\haarcascades\haarcascade_frontalface_default.xml')

## TCP 사용
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
## server ip, port
s.connect(('192.168.0.2', 22042))

## webcam 이미지 capture
cam = cv2.VideoCapture(0)

## 이미지 속성 변경 3 = width, 4 = height
cam.set(3, 1280);
cam.set(4, 960);

## 0~100에서 90의 이미지 품질로 설정 (default = 95)
encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]

while True:
    # 비디오의 한 프레임씩 읽는다.
    # 제대로 읽으면 ret = True, 실패면 ret = False, frame에는 읽은 프레임

    ret, frame = cam.read()

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    faces = faceCascade.detectMultiScale(
        gray,
        scaleFactor=1.1,
        minNeighbors=5,
        minSize=(30, 30),
        flags=cv2.CASCADE_SCALE_IMAGE
    )

    # Draw a rectangle around the faces
    for (x, y, w, h) in faces:
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

    # cv2. imencode(ext, img [, params])
    # encode_param의 형식으로 frame을 jpg로 이미지를 인코딩한다.
    result, frame = cv2.imencode('.jpg', frame, encode_param)
    # frame을 String 형태로 변환
    data = numpy.array(frame)
    stringData = data.tostring()

    # 서버에 데이터 전송
    # (str(len(stringData))).encode().ljust(16)
    s.sendall((str(len(stringData))).encode().ljust(16) + stringData)

cam.release()
