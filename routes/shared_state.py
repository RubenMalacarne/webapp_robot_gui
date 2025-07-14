import json
import os
import fcntl



class JsonStateStore:

    def __init__(self, filename: str, default_state: dict):
        self.filname = filename
        self.default  = default_state
    
    def read_file(self):
        if os.path.exists(self.filname):
            try:
                with open(self.filname, 'r') as f:
                    fcntl.flock(f, fcntl.LOCK_SH)
                    content = f.read().strip()
                    fcntl.flock(f, fcntl.LOCK_UN)
                    if content:
                        return json.loads(content)
            except Exception as e:
                print(f"[WARN] ERROR DURING READING FILE {self.filname}: {e}")
        return self.default.copy()

    def write_file(self, state: dict) -> None:
        try:
            with open(self.filname, 'w') as f:
                fcntl.flock(f, fcntl.LOCK_EX)
                json.dump(state, f)
                f.flush()
                os.fsync(f.fileno())
                fcntl.flock(f, fcntl.LOCK_UN)
        except Exception as e:
            print(f"[WARN] ERROR DURING WRITING FILE {self.filname}: {e}")
            
    def get(self) -> dict:
            return self.read_file()

    def set(self, **updates) -> dict:
        state = self.get()
        state.update(updates)
        self.write_file(state)
        return state

    def reset(self) -> None:
        self.write_file(self._default.copy())

#--------------------------------------------
# write type of state to send and received
CROSSHAIR_STATE = {
    "x": 0,
    "y": 0,
    "axis0": 0.0,
    "axis1": 0.0,
    "axis2": 0.0,
    "axis3": 0.0,
    "max_range": 500,
    "frozen": False,
    "frozen_x": 0.0,
    "frozen_y": 0.0,
    "robot_x": 0.0,
    "robot_y": 0.0,
}
JOY_STATE = {
    "axes": [0.0] * 8, 
    "buttons": [0] * 24
}

stores = {
    "crosshair": JsonStateStore("/tmp/crosshair_state.json", CROSSHAIR_STATE),
    "joy":       JsonStateStore("/tmp/joy_state.json",       JOY_STATE),
}
#--------------------------------------------

def get_store(name):
    return stores.get(name)