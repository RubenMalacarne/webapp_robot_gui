#!/usr/bin/env python3
#example for ros2
import os
import json
import fcntl
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

STATE_FILE = "/tmp/crosshair_state.json"


def save_state(state):
    """Salva lo stato nel file JSON in modo sicuro con file locking."""
    with open(STATE_FILE, 'w') as f:
        fcntl.flock(f, fcntl.LOCK_EX)
        json.dump(state, f)
        fcntl.flock(f, fcntl.LOCK_UN)

class JoyListener(Node):
    def __init__(self):
        super().__init__('crosshair_joy_listener')
        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.get_logger().info("🎮 JoyListener avviato - in ascolto su /joy")

        # Assicura che la directory esista
        os.makedirs(os.path.dirname(STATE_FILE), exist_ok=True)

        self.crosshair_state = {
            "x": 0.0, "y": 0.0,
            "max_range": 500,
            "frozen": False,
            "frozen_x": 0.0,
            "frozen_y": 0.0,
            "robot_x": 0.0,
            "robot_y": 0.0,
            "axis0": 0.0,
            "axis1": 0.0,
            "axis2": 0.0, 
            "axis3": 0.0
        }

    def joy_callback(self, msg):
        state = self.crosshair_state

        if len(msg.axes) >= 6:
            # Attiva/disattiva congelamento mirino con l'asse 4
            if msg.axes[4] == -1 and not state["frozen"]:
                state["frozen"] = True
                state["frozen_x"] = state["x"]
                state["frozen_y"] = state["y"]
                self.get_logger().info("❄️ FREEZE ACTIVATED")
            elif state["frozen"] and msg.axes[4] != -1:
                state["frozen"] = False
                self.get_logger().info("▶️ FREEZE DEACTIVATED")

            # Aggiorna posizione robot con l'asse 5
            if msg.axes[5] == -1:
                state["robot_x"] = state["x"]
                state["robot_y"] = state["y"]
                self.get_logger().info(f"🤖 ROBOT POSITION UPDATED to ({state['robot_x']}, {state['robot_y']})")

        if not state["frozen"]:
            if len(msg.axes) >= 2:
                axis0 = -msg.axes[0]  # Left stick horizontal
                axis1 = -msg.axes[1]  # Left stick vertical
                state["axis0"] = axis0
                state["axis1"] = axis1
                state["x"] = axis0 * state["max_range"]
                state["y"] = axis1 * state["max_range"]

            if len(msg.axes) >= 4:
                axis2 = -msg.axes[2]  # Right stick horizontal
                axis3 = -msg.axes[3]  # Right stick vertical
                state["axis2"] = axis2
                state["axis3"] = axis3
        elif state["frozen"]:
            state["x"] = state["frozen_x"]
            state["y"] = state["frozen_y"]

        save_state(state)

        # Log periodico ogni 20 callback
        self.callback_count = getattr(self, 'callback_count', 0) + 1
        if self.callback_count % 20 == 0:
            status = "FROZEN" if state["frozen"] else "ACTIVE"
            self.get_logger().info(
                f"[{status}] X={state['x']:.1f}, Y={state['y']:.1f} | "
                f"Axis0={state['axis0']:.3f}, Axis1={state['axis1']:.3f}"
            )

def main():
    rclpy.init()
    node = JoyListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
