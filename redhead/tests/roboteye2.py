import io
import picamera
import logging
import socketserver
from threading import Condition
from http import server
from apriltag import Detector, DetectorOptions
from PIL import Image
import traceback
import numpy
import json
import mmap
import contextlib
import uuid

PAGE="""\
<html>
<head>
<title>picamera MJPEG streaming demo</title>
</head>
<body>
<h1>PiCamera MJPEG Streaming Demo</h1>
<img src="stream.mjpg" width="640" height="480" />
</body>
</html>
"""


april_options = DetectorOptions()
april_detector = Detector(april_options)

CLEAR = """
                                                                                                                                                        
                                                                                                                                                        
                                                                                                                                                        
                                                                                                                                                        
                                                                                                                                                        
"""

APRILTAG_SHARED = '../data/apriltag.shared'


def create_map_file():
    with open(APRILTAG_SHARED, 'w') as f:
        f.write(CLEAR)


class StreamingOutput(object):
    def __init__(self):
        self.frame = None
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

class StreamingHandler(server.BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            self.send_response(301)
            self.send_header('Location', '/index.html')
            self.end_headers()
        elif self.path == '/index.html':
            content = PAGE.encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.send_header('Content-Length', len(content))
            self.end_headers()
            self.wfile.write(content)
        elif self.path == '/stream.mjpg':
            self.send_response(200)
            self.send_header('Age', 0)
            self.send_header('Cache-Control', 'no-cache, private')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Content-Type', 'mul   tipart/x-mixed-replace; boundary=FRAME')
            self.end_headers()
            try:
                with open('ultrasonic_sensors.txt', 'r+') as f:
                    with contextlib.closing(mmap.mmap(f.fileno(), 0)) as m:
                        while True:
                            with output.condition:
                                output.condition.wait()
                                frame = output.frame
                            
                            # convert image to gray
                            # orig = cv2.imdecode(frame)
                            # if len(orig.shape) == 3:
                            #     gray = cv2.cvtColor(orig, cv2.COLOR_RGB2GRAY)
                            # else:
                            #     gray = orig

                            pil_image = Image.open(io.BytesIO(frame))
                            orig = numpy.array(pil_image)
                            gray = numpy.array(pil_image.convert('L'))
                            
                            # detect tags
                            detections, dimg = april_detector.detect(gray, return_image=True)

                            # overlay tag border
                            if len(orig.shape) == 3:
                                overlay = orig // 2 + dimg[:, :, None] // 2
                            else:
                                overlay = gray // 2 + dimg // 2

                            # frame_with_overlay = cv2.imencode(overlay)
                            overlay_image = Image.fromarray(overlay)

                            # write to buffer
                            with io.BytesIO() as out_buffer:
                                overlay_image.save(out_buffer, format="JPEG")
                                frame_with_overlay = out_buffer.getvalue()

                            num_detections = len(detections)
                            print("num detections: %d"%num_detections)

                            if num_detections > 0:
                                m.seek(0)  # rewind memory map and clear buffer
                                m.write(str.encode(CLEAR))
                                m.flush()

                                apriltags = {'uuid':uuid.uuid4, 'num_detections':num_detections,'detections':detections}
                                m.seek(0)  # rewind memory map and write data
                                m.write(json.dumps(apriltags))
                                m.flush()

                            # loop through tag detections, print, and overlay image
                            for i, detection in enumerate(detections):
                                print(detection.tostring(indent=2))                    

                            # send frame with overlay
                            self.wfile.write(b'--FRAME\r\n')
                            self.send_header('Content-Type', 'image/jpeg')
                            self.send_header('Content-Length', len(frame_with_overlay))
                            self.end_headers()
                            self.wfile.write(frame_with_overlay)
                            self.wfile.write(b'\r\n')
            except Exception as e:
                print("Exception: ", e, traceback.format_exc())
                logging.warning(
                    'Removed streaming client %s: %s',
                    self.client_address, str(e))
        else:
            self.send_error(404)
            self.end_headers()

class StreamingServer(socketserver.ThreadingMixIn, server.HTTPServer):
    allow_reuse_address = True
    daemon_threads = True


if __name__ == '__main__':
    
    with picamera.PiCamera(resolution='640x480', framerate=24) as camera:
        output = StreamingOutput()
        camera.start_recording(output, format='mjpeg')
        try:
            address = ('', 8000)
            server = StreamingServer(address, StreamingHandler)
            server.serve_forever()
        finally:
            camera.stop_recording()
