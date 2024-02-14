import roslibpy


def generate_ik_request(x, y, z):
    msg = roslibpy.ServiceRequest({
        'ik_request': {
            'group_name': 'manipulator',
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
request = roslibpy.ServiceRequest({
    'ik_request': {
        'group_name': 'manipulator',
        'pose_stamped': {
            'header': {
                'frame_id': 'base_link'
            },
            'pose': {
                'position': {
                    'x': 0.5,
                    'y': 0.0,
                    'z': 0.5
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
result = service.call(request)
print(result)

client.terminate()
