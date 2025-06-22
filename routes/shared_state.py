import json
import os

STATE_FILE = "/tmp/crosshair_state.json"


def get_state():
    if not os.path.exists(STATE_FILE):
        return {
            "x": 0.0, "y": 0.0,
            "max_range": 500,
            "frozen": False,
            "frozen_x": 0.0,
            "frozen_y": 0.0,
            "robot_x": 0.0,
            "robot_y": 0.0,
            "axis0": 0.0,
            "axis1": 0.0,
        }
    try:
        with open(STATE_FILE, 'r') as f:
            return json.load(f)
    except Exception as e:
        print(f"Errore nella lettura dello stato: {e}")
        return {}

        
def save_shared_state(state):
    with open(STATE_FILE, 'w') as f:
        json.dump(state, f)
        