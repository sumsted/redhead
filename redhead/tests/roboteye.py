import time
import io
import picamera
from bottle import route, run, template, static_file, route, get, response, HTTPResponse
from gevent import monkey; monkey.patch_all()

HOST = '0.0.0.0'
PORT = 8099

class StreamingOutput(object):
    def __init__(self):
        self.buffer = io.BytesIO()
        self.condition = Condition()

    def write(self, buf):
        if buf.startswith(b'\xff\xd8'):
            # New frame, copy the existing buffer's content and notify all
            # clients it's available
            self.buffer.truncate()
            with self.condition:
                self.frame = self.buffer.getvalue()
                self.condition.notify_all()
            self.buffer.seek(0)
        return self.buffer.write(buf)

@get('/roboteye.mjpg')
def get_roboteye():
    response.status_code = 200    
    response.set_header('Age', 0)
    response.set_header('Cache-Control', 'no-cache, private')
    response.set_header('Pragma', 'no-cache')
    response.set_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
    yield response
    while True:
        with out.condition:
            out.condition.wait()
            frame = out.frame()
        yield b'--FRAME\r\n'
        image_frame = HTTPResponse(frame)
        image_frame.set_header('Content-Type', 'image/jpeg')
        image_frame.set_header('Content-Length', len(frame))
        yield image_frame
        yield b'\r\n'

if __name__ == '__main__':
    out = StreamingOutput()
    run(host=HOST, port=PORT)
