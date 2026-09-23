#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from ur_msgs.srv import SetIO
from ur_msgs.msg import IOStates
import time
import numpy as np
from math import pi
import sys
class JointAngles:
    def __init__(self):
        self.name = ["", "", "", "", "", ""]  #could have also done [""] * 6
        self.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# UR3e home position
# home = np.radians([120, -90, 90, -90, -90, 0])

# Hanoi tower location 
# Q11 = [120*pi/180.0, -56*pi/180.0, 124*pi/180.0, -158*pi/180.0, -90*pi/180.0, 0*pi/180.0]
# Q12 = [120*pi/180.0, -64*pi/180.0, 123*pi/180.0, -148*pi/180.0, -90*pi/180.0, 0*pi/180.0]
# Q13 = [120*pi/180.0, -72*pi/180.0, 120*pi/180.0, -137*pi/180.0, -90*pi/180.0, 0*pi/180.0]

############## Your Code Start Here ##############
"""
TODO: Initialize Q matrix
"""
# qINDEXHEIGHT
home = [153*pi/180.0, -84*pi/180.0, 94*pi/180.0, -104*pi/180.0, -90*pi/180.0, 0*pi/180.0]
# home = [167*pi/180.0, -100.7*pi/180.0, 74.79*pi/180.0, -83.4*pi/180.0, -93.16*pi/180.0, 0*pi/180.0]

# Q11 = [134.2*pi/180.0, -53.2*pi/180.0, 107.7*pi/180.0, -140.6*pi/180.0, -90.3*pi/180.0, 0*pi/180.0]
Q11 = [134.4*pi/180.0, -53.6*pi/180.0, 111.4*pi/180.0, -148.2*pi/180.0, -89.9*pi/180.0, 0*pi/180.0]
# Q12 = [134.2*pi/180.0, -60.46*pi/180.0, 109.5*pi/180.0, -138.4*pi/180.0, -90.1*pi/180.0, 0*pi/180.0]
Q12 = [134.4*pi/180.0, -60*pi/180.0, 110.2*pi/180.0, -140.6*pi/180.0, -89.9*pi/180.0, 0*pi/180.0]
# Q13 = [134.2*pi/180.0, -65.6*pi/180.0, 108.7*pi/180.0, -133.5*pi/180.0, -90*pi/180.0, 0*pi/180.0]
Q13 = [134.4*pi/180.0, -65.9*pi/180.0, 107.7*pi/180.0, -132.2*pi/180.0, -89.9*pi/180.0, 0*pi/180.0]

Q21 = [155.3*pi/180.0, -56.6*pi/180.0, 115.1*pi/180.0, -144.5*pi/180.0, -90*pi/180.0, 0*pi/180.0]
Q22 = [155.3*pi/180.0, -63*pi/180.0, 114.7*pi/180.0, -140*pi/180.0, -89.6*pi/180.0, 0*pi/180.0]
Q23 = [155.5*pi/180.0, -69*pi/180.0, 112*pi/180.0, -133*pi/180.0, -90*pi/180.0, 0*pi/180.0]

# Q31 = [180.3*pi/180.0, -49*pi/180.0, 100.8*pi/180.0, -138.7*pi/180.0, -91.5*pi/180.0, 0*pi/180.0]
# Q31 = [179.4*pi/180.0, -50.3*pi/180.0, 100.8*pi/180.0, -139*pi/180.0, -91.1*pi/180.0, 0*pi/180.0]
Q31 = [180.3*pi/180.0, -50.2*pi/180.0, 105.2*pi/180.0, -148.4*pi/180.0, -91.1*pi/180.0, 0*pi/180.0]
Q32 = [179.6*pi/180.0, -54.9*pi/180.0, 98.1*pi/180.0, -129.7*pi/180.0, -89.6*pi/180.0, 0*pi/180.0]
Q33 = [179.4*pi/180.0, -59.5*pi/180.0, 96*pi/180.0, -123.8*pi/180.0, -88.8*pi/180.0, 0*pi/180.0]

Qt1 = [135.1*pi/180.0, -75.1*pi/180.0, 91.2*pi/180.0, -105.8*pi/180.0, -93.3*pi/180.0, 0*pi/180.0]
Qt2 = [157.6*pi/180.0, -83.6*pi/180.0, 96.5*pi/180.0, -101.5*pi/180.0, -93.2*pi/180.0, 0*pi/180.0]
Qt3 = [178.2*pi/180.0, -76.2*pi/180.0, 92*pi/180.0, -108.9*pi/180.0, -88*pi/180.0, 0*pi/180.0]

# Q = [ [Q11, Q12, Q13], \
#       [Q21, Q22, Q23], \
#       [Q31, Q32, Q33] ]
Q = [ [Q11, Q12, Q13], \
      [Q21, Q22, Q23], \
      [Q31, Q32, Q33] ]
Qts = [ Qt1, Qt2, Qt3 ]
############### Your Code End Here ###############
class UR3e(Node):
    def __init__(self):
        super().__init__('ur3e')

        # Publishers
        self.trajectory_pub = self.create_publisher(JointTrajectory, '/scaled_joint_trajectory_controller/joint_trajectory', 10)

        # Subscribers
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        ############## Your Code Start Here ##############
        # TODO: define a ROS subscriber for gripper input message and corresponding callback function
        # ROS2 gripper input topic: /io_and_status_controller/io_states

        self.io_state_sub = self.create_subscription(IOStates, '/io_and_status_controller/io_states', self.io_state_callback, 10)
        



        ############### Your Code End Here ###############

        # Service clients
        self.io_client = self.create_client(SetIO, '/io_and_status_controller/set_io')
        while not self.io_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('IO service not available, waiting...')

        # State variables
        self.current_joint_state = None
        self.analog_in_0_value = 0
        self.gotone = False
        self.current_JointAngles = JointAngles()
        self.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ] # shoulder_pan_joint is the base rotation joint

    def joint_state_callback(self, msg):
        self.current_joint_state = msg  # Currently only used to check if messages have arrived
        index_inOrder = 0
        for name in self.joint_names:
            index_outofOrder = msg.name.index(name)
            self.current_JointAngles.name[index_inOrder] = name
            self.current_JointAngles.position[index_inOrder] = msg.position[index_outofOrder]
            index_inOrder = index_inOrder + 1 


    def io_state_callback(self, msg):
    ############## Your Code Start Here ##############
        """
        TODO: define a ROS topic callback funtion that 
        receives and stores the state of  the suction cup
        Whenever /io_and_status_controller/io_states 
        publishes this info, this callback function is
        called.
        """
        # self.analog_in_0_value = msg.analog_in_states[0].state
        # if(self.analog_in_0_value < 1.5):
        #     self.gotone = False
        # else:
        #     self.gotone = True
        # pass
        for analog_in_state in msg.analog_in_states:
            if analog_in_state.pin == 0:
                self.analog_in_0_value = analog_in_state.state
                if self.analog_in_0_value < 1.5:
                    self.gotone = False
                else:
                    self.gotone = True
                break

    ############### Your Code End Here ###############

    def set_io(self, pin, state):
        req = SetIO.Request()
        req.fun = 1
        req.pin = pin
        req.state = state
        future = self.io_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


    def move_arm(self, target):
        if self.current_joint_state is None:
            self.get_logger().error("No joint state received!")
            return False

        V_MAX = 1#2.09    # rad/s
        A_MAX = 0.8#2.79   # rad/s^2
        MIN_DURATION = 1
        MAX_DURATION = 8.0

        deltas = []
        for i in range(6):
            deltas.append(abs(self.current_JointAngles.position[i] - target[i]))


        max_delta = max(deltas)
        t_acc = V_MAX / A_MAX
        d_acc = 0.5 * A_MAX * (t_acc ** 2)
        if max_delta > 2 * d_acc:
            # trapezoidal velocity profile
            t_total = 2 * t_acc + (max_delta - 2 * d_acc) / V_MAX
        else:
            # triangular velocity profile
            t_total = 2 * (max_delta / A_MAX) ** 0.5

        duration = max(MIN_DURATION, min(t_total, MAX_DURATION))

        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joint_names

        # Start immediately when the controller receives it
        trajectory_msg.header.stamp.sec = 0
        trajectory_msg.header.stamp.nanosec = 0

        # Anchor point: current measured joint state at t = 0
        p0 = JointTrajectoryPoint()
        p0.positions = self.current_JointAngles.position
        p0.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # starting at rest
        p0.accelerations = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # starting at rest
        p0.time_from_start.sec = 0
        p0.time_from_start.nanosec = 0
        trajectory_msg.points.append(p0)

        # Goal point
        p1 = JointTrajectoryPoint()
        p1.positions = target
        p1.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # end at rest, 2 point trajectory
        p1.accelerations = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] #end at rest.
        p1.time_from_start.sec = int(duration)
        p1.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        trajectory_msg.points.append(p1)

        self.trajectory_pub.publish(trajectory_msg)

        self.get_logger().info(f'Moving to position: {np.degrees(target)}')

        # Wait for movement completion
        start_time = time.time()
        while time.time() - start_time < duration + 2:
            rclpy.spin_once(self, timeout_sec=0.1)

            deltas = []
            for i in range(6):
                deltas.append(abs(self.current_JointAngles.position[i] - target[i]))
            if all(delta < 0.001 for delta in deltas):
                time.sleep(0.25)
                return True
        return False



    def move_block(self, start_tower, start_height, end_tower, end_height):
        global Q
    ############## Your Code Start Here ##############
    # TODO: add code to move block from start tower and height to end tower and height
    ### Hint: Use the Q array to map out your towers by location and "height".
        # error = 0
        # return error
        # AL

        self.move_arm(home)
        self.move_arm(Qts[start_tower])
        self.move_arm(Q[start_tower][start_height])
        self.set_io(0, 1.0)
        # time.sleep(3.0) 

        start_wait = time.time()
        while(time.time() - start_wait < 1.0):
            rclpy.spin_once(self, timeout_sec=0.1)  

        if(self.gotone == False):
            self.get_logger().error(f"Suction cup failed to pick up the block! Value {self.analog_in_0_value}")
            return False

        self.move_arm(home)
        self.move_arm(Qts[end_tower])
        self.move_arm(Q[end_tower][end_height])
        self.set_io(0, 0.0)
        return True
    ############### Your Code End Here ###############


def main(args=None):
    input("Check if the UR3e is in 'Remote' Mode?\n\
    Check if the UR3e is initialized and in 'Normal' state.\n\
    Have you run the ROS2 launch statement?\n\
    If there was an UR3e emergency stop or error, Ctrl-C the ros2 launch and rerun.\n\
    \n\
    Press <Enter> to Continue.")
    rclpy.init(args=args)
    node = UR3e()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    ############## Your Code Start Here ##############
    # TODO: modify the code below so that program can get user input
    loop_count = 0
    # Wait for initial state updates
    while node.current_joint_state is None:
        executor.spin_once(timeout_sec=0.05)
        node.get_logger().info("Waiting for initial state updates...")
        time.sleep(0.5)

    try:
        # Get start tower
        input_string = input("Enter start tower <Either 1 2 3 or 0 to quit> ")
        print("You entered " + input_string + "\n")

        if(int(input_string) == 1):
            start_tower = 0
        elif (int(input_string) == 2):
            start_tower = 1
        elif (int(input_string) == 3):
            start_tower = 2
        elif (int(input_string) == 0):
            print("Quitting... ")
            sys.exit()
        else:
            print("Please just enter the character 1 2 3 or 0 to quit \n\n")

        # Get end tower
        input_string = input("Enter end tower <Either 1 2 3 or 0 to quit> ")
        print("You entered " + input_string + "\n")

        if(int(input_string) == 1):
            end_tower = 0
        elif (int(input_string) == 2):
            end_tower = 1
        elif (int(input_string) == 3):
            end_tower = 2
        elif (int(input_string) == 0):
            print("Quitting... ")
            sys.exit()
        else:
            print("Please just enter the character 1 2 3 or 0 to quit \n\n")

        mid_tower = 3 - start_tower - end_tower

        ############## Your Code Start Here ##############
        # TODO: modify the code so that UR3e can move tower accordingly from user input
        # given start_tower, mid_tower end_tower 
            # node.move_arm(home)

        ret = node.move_block(start_tower, 2, end_tower, 0)
        if not ret:
            node.get_logger().error("Failed to move block from start tower to end tower!")
            return
        ret = node.move_block(start_tower, 1, mid_tower, 0)
        if not ret:
            node.get_logger().error("Failed to move block from start tower to middle tower!")
            return
        ret = node.move_block(end_tower, 0, mid_tower, 1)
        if not ret:
            node.get_logger().error("Failed to move block from end tower to middle tower!")
            return
        ret = node.move_block(start_tower, 0, end_tower, 0)
        if not ret:
            node.get_logger().error("Failed to move block from start tower to end tower!")
            return
        ret = node.move_block(mid_tower, 1, start_tower, 0)
        if not ret:
            node.get_logger().error("Failed to move block from middle tower to start tower!")
            return
        ret = node.move_block(mid_tower, 0, end_tower, 1)
        if not ret:
            node.get_logger().error("Failed to move block from middle tower to end tower!")
            return
        ret = node.move_block(start_tower, 0, end_tower, 2)
        if not ret:
            node.get_logger().error("Failed to move block from start tower to end tower!")
            return

        node.move_arm(home)
            

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
