import time

import roslibpy

client = roslibpy.Ros(host='localhost', port=9090)
client.run()

talker = roslibpy.Topic(client, '/chatter', 'std_msgs/String')

i = 0

while client.is_connected:
    message = roslibpy.Message({'data': f'Hello World! {i}'})
    talker.publish(message)
    print(f'Sending message {i}...')
    i += 1
    time.sleep(1)

talker.unadvertise()
client.terminate()
