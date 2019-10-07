import time
from bottle import route, run, template, static_file, route, get, response
from gevent import monkey; monkey.patch_all()


@route('/stream')
def stream():

    header =  "HTTP/2 200\r\nAge: 0\r\ncache-control: private, no-cache\r\ncontent-type: multipart/x-mixed-replace; boundary=FRAME\r\npragma: no-cache\r\n\r\n"
    yield response(header)
    time.sleep(3)
    body =  "<html><body>hello</body></html>"
    yield body
    # time.sleep(5)
    # yield 'END'

            
if __name__ == '__main__':

    run(host='0.0.0.0', port=8084)
