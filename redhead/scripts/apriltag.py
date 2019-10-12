
import json
import mmap
import contextlib
import time


APRILTAG_SHARED = '../data/apriltag.shared'


def start():
    with open(APRILTAG_SHARED, 'r') as f:
        with contextlib.closing(mmap.mmap(f.fileno(), 0, access=mmap.ACCESS_READ)) as m:
            while True:
                m.seek(0)
                # print('\nreading...')
                arpriltag_json = m.readline().decode("utf-8")
                print(apriltag_json)
                try:
                    apriltag = json.loads(sensors_json)
                except Exception as e:
                    print("exception %s"% str(e))
                time.sleep(.25)

if __name__=='__main__':
    start()
