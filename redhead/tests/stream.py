import time
from bottle import route, run, template, static_file, route, get, response
from gevent import monkey; monkey.patch_all()


@route('/stream')
def stream():
    yield 'START'
    time.sleep(3)
    yield 'MIDDLE'
    time.sleep(5)
    yield 'END'

            
if __name__ == '__main__':

    run(host='0.0.0.0', port=8084)
