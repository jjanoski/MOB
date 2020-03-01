#!/usr/bin/env python
import cv2
import numpy as np
from flask import Flask, render_template, Response

app = Flask(__name__)
cap = cv2.VideoCapture(0)

@app.route('/')
def index():
    return render_template('index.html')

def gen():
    while True:
        read_return_code, frame = cap.read()
        encode_return_code, image_buffer = cv2.imencode('.jpg', frame)
        io_buff = io.BytesIO(image_buffer)
        yield(b'--frame\r\n'
              b'Content-Type: image/jpeg\r\n\r\n' + io_buff.read() + b'\r\n')

@app.route('/video_feed')
def video_feed():
    return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    app.run(host='192.168.11.12', debug=True, threaded=True)
