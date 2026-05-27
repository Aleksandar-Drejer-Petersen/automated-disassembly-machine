from picamera2 import Picamera2
from flask import Flask, Response
import cv2
import time

app = Flask(__name__)

picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"size": (1280, 720)}))
picam2.start()

# AfMode 0 = Manual, LensPosition 4.5
picam2.set_controls({"AfMode": 0, "LensPosition": 4.5})
time.sleep(1)

def generate_frames():
    while True:
        frame = picam2.capture_array()
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        ret, buffer = cv2.imencode('.jpg', frame)
        if not ret:
            continue
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/')
def index():
    return '<html><body style="margin:0"><img src="/video_feed" style="width:100%"></body></html>'

if __name__ == '__main__':
    print("Live feed running at http://172.20.10.2:5000")
    app.run(host='0.0.0.0', port=5000)
