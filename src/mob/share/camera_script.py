#!/usr/bin/env python

import io
import cv2
import socket
import picamera
import numpy as np
from flask import Flask, render_template, Response

app = Flask(__name__)

cv2.namedWindow("preview")
vc = cv2.VideoCapture(0) # configure camera to usb
@app.route('/')
def index():

    return render_template('index.html')

def gen():

    if vc.isOpened(): # try to get the first frame
        rval, frame = vc.read()
    else:
        rval = False

    while rval:
        cv2.imshow("preview", frame)
        rval, frame = vc.read()
        cv2.imwrite('t.mjpg', frame)
        yield(b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + open('t.mjpg', 'rb').read() + b'\r\n')
        key = cv2.waitKey(20)
        if key == 27: # exit on ESC
            break
    cv2.destroyWindow("preview")

@app.route('/video_feed')
def video_feed():
    return Response(gen(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':

    app.run(host='192.168.11.12', debug=True, threaded=True)
