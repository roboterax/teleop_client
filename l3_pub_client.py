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

        # Send joint zero trajectory
        self.get_logger().info('Wait simple_trajectory service...')
        self.traj_client.wait_for_server()
        self.get_logger().info('Reset...')
        goal = SimpleTrajectory.Goal()
        goal.traj_type = 0
        goal.duration = 4.0
        self.get_logger().info('Wait send joint reset trajectory...')
        self.traj_client.wait_for_server()
        self.get_logger().info('Send joint reset trajectory...')
        send_goal_future = self.traj_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Reset Action Goal not accepted')
        else:
            rclpy.spin_until_future_complete(self, goal_handle.get_result_async())
        time.sleep(1)

        # Send joint arm lifting trajectory
        self.get_logger().info('Lift your arm...')
        goal = SimpleTrajectory.Goal()
        goal.traj_type = 2
        goal.duration = 4.0
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

    def run2(self, typeT, data_dict):
        try:

            max_wait_time = 10.0
            start_time = time.time()

            while not self.client.wait_for_service(timeout_sec=10):
                self.get_logger().info('Service not available, waiting...')
                if time.time() - start_time > max_wait_time:
                    self.get_logger().error('Service wait timeout!')
                return 
            json_dict = {"type" : typeT, "message" : json.dumps(data_dict)}
            json_data = json.dumps(json_dict)
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

    def run3(self, cmd, data_dict):
        try:

            max_wait_time = 10.0
            start_time = time.time()

            while not self.client.wait_for_service(timeout_sec=10):
                self.get_logger().info('Service not available, waiting...')
                if time.time() - start_time > max_wait_time:
                    self.get_logger().error('Service wait timeout!')
                return 
            json_dict = {"command" : cmd, "message" : json.dumps(data_dict)}
            json_data = json.dumps(json_dict)
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

def start_webxr(data_dict, camera_type, enable_capture_audio, enable_player_audio):

    if camera_type == None:
        print("not Parameter camera_type")
        sys.exit(0)

    data_dict["camera_type"] = camera_type

    if enable_capture_audio:
        data_dict["enable_capture_audio"] = enable_capture_audio
    if enable_player_audio:
        data_dict["enable_player_audio"] = enable_player_audio



def start_retargetx(data_dict, hand, mocap):
    if hand == None or mocap == None:
        print("not Parameter hand or mocap")
        sys.exit(0)
    data_dict["hand"] = hand
    data_dict["mocap"] = mocap


def start_cmd(minimal_publisher, enable):
    if enable not in ["0", "1"]:
        print("mpc reset mode not support")
        return
    data_dict = {"enable": bool(int(enable))}
    minimal_publisher.run3("enableRL", data_dict)


def start_any(minimal_publisher, typeT, cmd, hand, mocap, camera_type,enable_capture_audio, enable_player_audio, mode):
    data_dict = {"command": cmd}
    if typeT == 'webxr':
        if cmd == "start":
            start_webxr(data_dict, camera_type, enable_capture_audio, enable_player_audio)
            minimal_publisher.run2(typeT, data_dict)
            return
        minimal_publisher.run2(typeT, data_dict)
    elif typeT == "mpc":
        if cmd == "reset":
            if mode is None:
                print("mpc reset need mode")
                return
            if mode not in ["0", "1"]:
                print("mpc reset mode not support")
                return
            data_dict["mode"] = int(mode)
            minimal_publisher.run2(typeT, data_dict)
            return

        minimal_publisher.run2(typeT, data_dict)
        return
    elif typeT == "rl":
        minimal_publisher.run2(typeT, data_dict)
        return
    elif typeT == "retargetx":
        if cmd == "start":
            start_retargetx(data_dict, hand, mocap)
            minimal_publisher.run2(typeT, data_dict)
            return
        minimal_publisher.run2(typeT, data_dict)



# Priority activation of SDK functionality
# python pub_client.py --cmd start_sdk


# Start remote operation
# python pub_client.py --type  webxr --cmd start --camera-type [dummy | realsense | stereo]
# python pub_client.py --type  webxr --cmd query
# python pub_client.py --type  webxr --cmd stop

# python pub_client.py --type  retargetx --cmd start --hand [xhand | lite]
# python pub_client.py --type  retargetx --cmd query
# python pub_client.py --type  retargetx --cmd stop

# python pub_client.py --type  mpc --cmd start
# python pub_client.py --type  mpc --cmd query
# python pub_client.py --type  mpc --cmd stop


# python pub_client.py --type  rl --cmd start
# python pub_client.py --type  rl --cmd query
# python pub_client.py --type  rl --cmd stop

# Start control
# python pub_client.py --cmd start_teleop


# Stop sdk
# python pub_client.py --cmd stop_sdk

def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser(description="client", allow_abbrev=False)
    parser.add_argument("--cmd", help="start_sdk | start_teleop |  stop_sdk | start | stop | reset | query | enable_rl")
    parser.add_argument("--type", help="webxr | retargetx | mpc | rl")
    parser.add_argument("--hand", help="xhand | lite")
    parser.add_argument("--mocap", help="vr | gamepad")
    parser.add_argument("--mode", help="0 | 1")
    parser.add_argument("--camera-type", help="dummy | realsense | stereo")
    parser.add_argument("--enable_capture_audio", help="device name:[default]")
    parser.add_argument("--enable_player_audio", help="device id:[6]")
    
    
    args = parser.parse_args()
    
    minimal_publisher = MinimalPublisher()
    if args.cmd == "start_sdk" :
        node = WR1ControlDemo()
        node.start_sdk()
        node.destroy_node()
        rclpy.shutdown()
        return
    elif args.cmd == "start_teleop":
        node = StartControlDemo()
        node.run()
        node.destroy_node()
        rclpy.shutdown()
        return
    elif args.cmd == "stop_sdk":
        node = WR1ControlDemo()
        node.stop_sdk()
        node.destroy_node()
        rclpy.shutdown()
        return
    elif args.cmd == "enable_rl":
         start_cmd(minimal_publisher, args.mode)
         return
    else :
        start_any(minimal_publisher,args.type, args.cmd, args.hand, args.mocap, args.camera_type, args.enable_capture_audio, args.enable_player_audio, args.mode)
        return
        

if __name__ == '__main__':
    main()