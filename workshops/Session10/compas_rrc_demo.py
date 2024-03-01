import compas.geometry as cg
import compas_rrc as rrc

if __name__ == '__main__':

    # Create Ros Client
    ros = rrc.RosClient()
    ros.run()

    # Create ABB Client
    abb_rrc = rrc.AbbClient(ros, '/rob1-rw6')
    print('Connected.')

    # Set tools
    abb_rrc.send(rrc.SetTool('pen_1'))

    # Set speed [mm/s]
    speed = 30

    # Go to home position (linear joint move)
    home = rrc.RobotJoints([0, 0, 0, 0, 90, 0])
    done = abb_rrc.send_and_wait(rrc.MoveToJoints(home, [], speed, rrc.Zone.FINE))

    # Read current frame positions
    frame = abb_rrc.send_and_wait(rrc.GetFrame())
    print(f'Frame = {frame}')

    # Create a new frame with the same orientation but a different position
    new_frame = cg.Frame(cg.Point(340, 315, 275), frame.xaxis, frame.yaxis)

    # Move robot the new pos
    done = abb_rrc.send_and_wait(rrc.MoveToFrame(new_frame, speed, rrc.Zone.FINE, rrc.Motion.LINEAR))

    # End of Code
    print('Finished')

    # Close client
    ros.close()
    ros.terminate()
