import math
import roslibpy


def print_joint_state(message):
    joint_angles = (str(round(math.degrees(x), 2)) for x in message['position'])
    print('Joint state: ' + ', '.join(joint_angles))


client = roslibpy.Ros(host='localhost', port=9090)
client.run()

listener = roslibpy.Topic(client, '/joint_states', 'sensor_msgs/JointState', throttle_rate=1000)
listener.subscribe(print_joint_state)

try:
    while True:
        pass
except KeyboardInterrupt:
    client.terminate()
