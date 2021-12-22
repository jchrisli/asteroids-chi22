'''
Create a mock-up for robot update messages for the websocket server
'''

import socketio
import math
import time

sio = socketio.Client(logger=True, engineio_logger = True)
#sio.connect('https://videochat-new.herokuapp.com/robots')
sio.connect('http://localhost:5000', namespaces=['/robots'])


frame = 0
update_every = 0.2
cycle = 3.0
frame_cycle = int(cycle / update_every)
increment = math.pi / frame_cycle

w = 320
h = 180
origin1 = [640, 360]
origin2 = [200, 150]
origin3 = [1000, 40]
radius = 40
users1 = ['t1', 't2']
users2 = ['t2']
users3 = []

while True:
    try:
        fake_robot_update = [ 
            {
                'id': 1,
                'x': origin1[0] + radius * math.cos(frame * increment),
                'y': origin1[1] + radius * math.sin(frame * increment),
                'heading': math.pi / 2,
                'users': users1
            },
            {
                'id': 2,
                'x': origin2[0] + radius * math.cos(frame * increment),
                'y': origin2[1] + radius * math.sin(frame * increment),
                'heading': math.pi / 2,
                'users': users2
            },
            {
                'id': 3,
                'x': origin3[0] + radius * math.cos(frame * increment),
                'y': origin3[1] + radius * math.sin(frame * increment),
                'heading': math.pi / 2,
                'users': users3
            }
        ]
        sio.emit('robot-update', fake_robot_update, namespace='/robots')
        frame += 1
        time.sleep(update_every)
    except KeyboardInterrupt:
        sio.disconnect()
        break
