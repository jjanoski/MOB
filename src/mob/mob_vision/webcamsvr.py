#!/usr/bin/env python3

from flask import Flask, render_template, Response
import cv2
import io

capture = cv2.VideoCapture(0)
print("capture: "+str(capture)+"Type: "+str(type(capture)))
app = Flask(__name__)

@app.route('/')
def index():
    print("index is operating")
    return render_template('index.html')

def gen():
    try:
        if capture.isOpened():
            rval, frame = capture.read()
        else:
            rval = False

        face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
        print("face cas: "+str(face_cascade))
        while rval:
            print("while loop running")
            #ret, frame = capture.read()
            faces = face_cascade.detectMultiScale(frame, 1.3, 5)

            for(x, y, w, h) in faces:
                cv2.rectangle(frame, (x, y),(x+y, y+h), (255, 0, 0), 2)
                roi_color = frame[y:y+h, x:x+w]
        #yield(cv2.imshow("preview", frame))
        cv2.imwrite('t.jpeg', frame)
        #io_buff = io.BytesIO(byteArray)
        #return byteArray
        yield (b'--frame\r\n'
               b'Content-Type: image/jpg\r\n\r\n' + open('t.jpg', 'rb').read() + b'\r\n')
    except Exception as e:
        print("Error:: "+str(e))

@app.route('/video_feed')
def video_feed():             
    return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')


if __name__ == '__main__':
    app.run(host='192.168.11.12', debug=True, threaded=True)
