import roslibpy


def print_message(message):
    print('Heard talking: ' + message['data'])


client = roslibpy.Ros(host='localhost', port=9090)
client.run()

listener = roslibpy.Topic(client, '/chatter', 'std_msgs/String')
listener.subscribe(print_message)

try:
    while True:
        pass
except KeyboardInterrupt:
    client.terminate()
