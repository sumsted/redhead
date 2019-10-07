if False:
    print("remote debugging wait")
    import ptvsd
    ptvsd.enable_attach(address=('0.0.0.0', 3000))
    ptvsd.wait_for_attach()

import time
import traceback
import io
from threading import Condition
import picamera
from bottle import route, run, template, static_file, route, get, response, HTTPResponse
from gevent import monkey; monkey.patch_all()

HOST = '0.0.0.0'
PORT = 8099

class StreamingOutput(object):
    def __init__(self):
        print("so init")
        self.buffer = io.BytesIO()
        self.condition = Condition()

    def write(self, buf):
        print("so write")
        if buf.startswith(b'\xff\xd8'):
            # New frame, copy the existing buffer's content and notify all
            # clients it's available
            self.buffer.truncate()
            with self.condition:
                self.frame = self.buffer.getvalue()
                self.condition.notify_all()
            self.buffer.seek(0)
        return self.buffer.write(buf)

@route('/roboteye.mjpg')
def get_roboteye():
    try:
        print("re")
        response.status = 200    
        response.set_header('Age', 0)
        response.set_header('Cache-Control', 'no-cache, private')
        response.set_header('Pragma', 'no-cache')
        response.set_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
        print("re y0")
        yield ""#response
        print("re y1")
        while True:
            with output.condition:
                output.condition.wait()
                frame = output.frame()
                print("re f")

            yield b'--FRAME\r\nContent-Type: image/jpeg\r\nContent-Length:%d'%len(frame)
            print("re y2")
            yield frame
            print("re y3")
            yield b'\r\n'
            print("re y4")
    except Exception as e:
        print("Exception: ", e, traceback.format_exc())

if __name__ == '__main__':
    with picamera.PiCamera(resolution='640x480', framerate=24) as camera:
        output = StreamingOutput()
        camera.start_recording(output, format='mjpeg')
        try:
            run(host=HOST, port=PORT)
        finally:
            camera.stop_recording()