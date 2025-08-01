import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import argparse
import time
import sys
import os
from pathlib import Path
from xbot_common_interfaces.srv import DynamicLaunch
from xbot_common_interfaces.srv import StringMessage
from std_srvs.srv import Trigger
from xbot_common_interfaces.action import SimpleTrajectory
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState

import time

class WR1ControlDemo(Node):
    def __init__(self):
        super().__init__('wr1_control_demo')
        # Service
        self.dyn_launch_cli = self.create_client(DynamicLaunch, '/dynamic_launch')
        self.ready_cli = self.create_client(Trigger, '/ready_service')
        self.activate_cli = self.create_client(Trigger, '/activate_service')
        self.stop_cli = self.create_client(Trigger, '/stop_launch')
        # Action
        self.traj_client = ActionClient(self, SimpleTrajectory, '/simple_trajectory')

        # State
        self.last_joint_state = None
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )

    def call_service(self, client, req):
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def joint_state_callback(self, msg):
        self.last_joint_state = msg

    def wait_for_joint_state(self, joint_names, timeout=10.0):
        start = time.time()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.last_joint_state:
                # Check to include the required joint names
                if all(j in self.last_joint_state.name for j in joint_names):
                    return self.last_joint_state
            if time.time() - start > timeout:
                self.get_logger().warn('Failure to obtain joint status in a timely manner may result in unpublished data streams')
                while rclpy.ok():
                    rclpy.spin_once(self, timeout_sec=0.1)
                return None
        return None

    def stop_sdk(self):
        # Stop joint service
        self.get_logger().info('Wait stop_launch service...')
        self.stop_cli.wait_for_service()
        self.get_logger().info('Stop joint service...')
        self.call_service(self.stop_cli, Trigger.Request())
        self.get_logger().info('OK')
        time.sleep(1)

    def start_sdk(self):
        # Start joint service
        self.get_logger().info('Wait dynamic_launch service...')
        self.dyn_launch_cli.wait_for_service()
        self.get_logger().info('Start joint service...')
        dyn_req = DynamicLaunch.Request()
        dyn_req.app_name = ''
        dyn_req.sync_control = False
        dyn_req.launch_mode = 'pos'
        self.call_service(self.dyn_launch_cli, dyn_req)

        # Initialize joint module
        self.get_logger().info('Wait ready_launch service...')
        self.ready_cli.wait_for_service()
        self.get_logger().info('Initialize joint module...')
        self.call_service(self.ready_cli, Trigger.Request())

        # Send joint arm lifting trajectory
        self.get_logger().info('Lift your arm...')
        goal = SimpleTrajectory.Goal()
        goal.traj_type = 14
        goal.duration = 6.0
        self.get_logger().info('Waiting to send the arm lift trajectory...')
        self.traj_client.wait_for_server()
        self.get_logger().info('Send arm lift trajectory...')
        send_goal_future = self.traj_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Arm raising Action Goal not accepted')
        else:
            rclpy.spin_until_future_complete(self, goal_handle.get_result_async())
        time.sleep(1)

        # Activate joint module
        self.get_logger().info('Wait activate_launch service...')
        self.activate_cli.wait_for_service()
        self.get_logger().info('Activate joint module...')
        self.call_service(self.activate_cli, Trigger.Request())
        time.sleep(1)

class StartControlDemo(Node):
    def __init__(self):
        super().__init__('start_control_demo')
        self.client = self.create_client(Trigger, '/Start_EE_Retarget')

    def run(self):
        max_wait_time = 10.0
        start_time = time.time()

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
            if time.time() - start_time > max_wait_time:
                self.get_logger().error('Service wait timeout!')
                return 
        request = Trigger.Request()
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f'Service call succeeded: {future.result().success, future.result().message}')
        else:
            self.get_logger().error('Service call failed')

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.client = self.create_client(StringMessage, '/teleoperation/service')

    def run(self, key, value):
        try:

            max_wait_time = 10.0
            start_time = time.time()

            while not self.client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Service not available, waiting...')
                if time.time() - start_time > max_wait_time:
                    self.get_logger().error('Service wait timeout!')
                return 
            json_data = json.dumps({key: value})
            request = StringMessage.Request()
            request.data = json_data

            future = self.client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                self.get_logger().info(f'Service call succeeded: {future.result().result, future.result().message}')
            else:
                self.get_logger().error('Service call failed')
        except Exception as e:
            self.get_logger().error(f"send faild: {e}")

def read_from_file_v2(file_path="key.dat"):
    try:
        with open(file_path, "r", encoding="utf-8") as file:
            first_line = file.readline().strip()

        if not first_line:
            raise ValueError(f"Error: The file {file_path} is empty or does not contain a valid signature.")

        return first_line

    except FileNotFoundError:
        raise FileNotFoundError(f"Error: The file {file_path} does not exist.")
    except Exception as e:
        raise IOError(f"Error: Could not read from file {file_path}. {str(e)}")

def start(minimal_publisher, cmd, verify_path, hand, mocap, camera_type,enable_capture_audio, enable_player_audio):
    if not os.path.exists(verify_path):
        print("verify_path path not exist")
        sys.exit(0)
    path = Path(verify_path)
    if not path.is_dir():
        print("verify_path path not dir")
        sys.exit(0)
    auth = verify_path + "/auth_info.json"
    key = verify_path + "/key.dat"

    if not os.path.exists(auth):
        print("auth path not exist")
        sys.exit(0)
    if not os.path.exists(key):
        print("key path not exist")
        sys.exit(0)
    if hand == None or mocap == None or camera_type == None:
        print("not Parameter hand or mocap or camera_type")
        sys.exit(0)

    with open(auth, "r", encoding="utf-8") as file:
        data = json.load(file)
        auth_str = json.dumps(data)
    verify = {"auth" : auth_str, "key": read_from_file_v2(key), "hand" : hand, "mocap" : mocap, "camera_type" : camera_type}
    if enable_capture_audio:
        verify["enable_capture_audio"] = enable_capture_audio
    if enable_player_audio:
        verify["enable_player_audio"] = enable_player_audio

    minimal_publisher.run(cmd, json.dumps(verify))

def start_any(minimal_publisher, cmd):
    minimal_publisher.run(cmd, "")

def stop(minimal_publisher, cmd):
    minimal_publisher.run(cmd, "")

# one
# Priority activation of SDK functionality
# python pub_client.py --cmd start_sdk

# two
# Start remote operation
# python pub_client.py init_teleop --verify [path] --hand [xhand | lite] --mocap [vr | gamepad] --camera-type [dummy | realsense | stereo] # Path is the authorized file path

# three
# Start control
# python pub_client.py --cmd start_teleop

# four
# Stop remote operation service
# python pub_client.py --cmd stop_teleop

# five
# Stop sdk
# python pub_client.py --cmd stop_sdk

def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser(description="client", allow_abbrev=False)
    parser.add_argument("--cmd", help="start_sdk | init_teleop | start_teleop | stop_teleop | stop_sdk")
    parser.add_argument("--verify", help="verify path")
    parser.add_argument("--hand", help="xhand | lite")
    parser.add_argument("--mocap", help="vr | gamepad")
    parser.add_argument("--camera-type", help="dummy | realsense | stereo")
    parser.add_argument("--enable_capture_audio", help="device name:[default]")
    parser.add_argument("--enable_player_audio", help="device name:[default]")
    
    
    args = parser.parse_args()
    
    minimal_publisher = MinimalPublisher()
    if args.cmd == "start_sdk" :
        node = WR1ControlDemo()
        node.start_sdk()
        node.destroy_node()
        rclpy.shutdown()
        return
    elif args.cmd == "init_teleop" :
        if args.verify is None:
            print("start need verify path")
            return
        start(minimal_publisher,args.cmd, args.verify, args.hand, args.mocap, args.camera_type, args.enable_capture_audio, args.enable_player_audio)
    elif args.cmd == "start_teleop":
        node = StartControlDemo()
        node.run()
        node.destroy_node()
        rclpy.shutdown()
        return
    elif args.cmd == "stop_teleop":
        stop(minimal_publisher, args.cmd)
    elif args.cmd == "stop_sdk":
        node = WR1ControlDemo()
        node.stop_sdk()
        node.destroy_node()
        rclpy.shutdown()
        return
        

if __name__ == '__main__':
    main()