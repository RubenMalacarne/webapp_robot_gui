#!/usr/bin/env python3
#example for ros2
import os
import json
import fcntl
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy


STATE_FILE = "/tmp/joy_state.json"
NUM_AXES   = 8
NUM_BTNS   = 24

def _safe_list(seq, target_len, fill=0.0):
    lst = list(seq)[:target_len]
    lst.extend([fill] * (target_len - len(lst)))
    return lst

def save_state(state):
    with open(STATE_FILE, 'w') as f:
        fcntl.flock(f, fcntl.LOCK_EX)
        json.dump(state, f)
        fcntl.flock(f, fcntl.LOCK_UN)

qos = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=QoSReliabilityPolicy.BEST_EFFORT
)

class JoyListener(Node):
    def __init__(self):        
        super().__init__('joy_state_listener')
        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.get_logger().info("JoyListener - topic subscriber /joy")

        os.makedirs(os.path.dirname(STATE_FILE), exist_ok=True)

        self.joy_state = {
            "axes": [0.0] * NUM_AXES,
            "buttons": [0]  * NUM_BTNS,
        }
        save_state(self.joy_state)  

    def joy_callback(self, msg: Joy) -> None:
        self.joy_state["axes"]    = _safe_list(msg.axes,    NUM_AXES, 0.0)
        self.joy_state["buttons"] = _safe_list(msg.buttons, NUM_BTNS, 0)

        save_state(self.joy_state)

        self.get_logger().info(
            f"axes={self.joy_state['axes'][:4]} btn0={self.joy_state['buttons'][0]}",
            throttle_duration_sec=0.5,
        )

def main():
    rclpy.init()
    node = JoyListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
