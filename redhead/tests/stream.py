import time
from bottle import route, run, template, static_file, route, get, response
from gevent import monkey; monkey.patch_all()


@route('/stream')
def stream():
    response.status = 200    
    response.set_header('Age', 0)
    response.set_header('Cache-Control', 'no-cache, private')
    response.set_header('Pragma', 'no-cache')
    response.set_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
    yield "k"
    time.sleep(1)
    body =  "<html><body>hello2asdf</body></html>"
    yield body
    # time.sleep(5)
    # yield 'END'

            
if __name__ == '__main__':

    run(host='0.0.0.0', port=8084)
