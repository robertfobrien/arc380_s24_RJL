import roslibpy


def generate_ik_request(x, y, z):
    msg = roslibpy.ServiceRequest({
        'ik_request': {
            'group_name': 'manipulator',
            'robot_state': {
                'joint_state': {
                    'name': ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'],
                    'position': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                }
            },
            'pose_stamped': {
                'header': {
                    'frame_id': 'base_link'
                },
                'pose': {
                    'position': {
                        'x': x,
                        'y': y,
                        'z': z
                    },
                    'orientation': {
                        'x': 0.0,
                        'y': 0.0,
                        'z': 0.0,
                        'w': 1.0
                    }
                }
            }
        }
    })

    return msg


client = roslibpy.Ros(host='localhost', port=9090)
client.run()

service = roslibpy.Service(client, '/compute_ik', 'moveit_msgs/GetPositionIK')
msg = generate_ik_request(0.1, 0.0, 0.5)
request = roslibpy.ServiceRequest(msg)
result = service.call(request)
print(result)

client.terminate()
