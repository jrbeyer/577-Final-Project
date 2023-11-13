import holoocean
import numpy as np
from pynput import keyboard

# constants
# Environment refresh rate (from Holoocean, Hz)
env_refresh_rate = 1/30
# Control rate (Env ticks per control step, 1 Hz)
control_rate = 30






# Keyboard listener for exiting simulation
pressed_keys = list()
def on_press(key):
    global pressed_keys
    if hasattr(key, 'char'):
        pressed_keys.append(key.char)
        pressed_keys = list(set(pressed_keys))
def on_release(key):
    global pressed_keys
    if hasattr(key, 'char'):
        pressed_keys.remove(key.char)
listener = keyboard.Listener(
    on_press=on_press,
    on_release=on_release)
listener.start()


def test_holoocean():
    # The hovering AUV takes a command for each thruster
    command = np.array([5,5,-5,-5,0,0,0,0])

    with holoocean.make("PierHarbor-Hovering") as env:
        env.set_render_quality(1)
        for i in range(100000):
            if 'q' in pressed_keys:
                break
            state = env.step(command)


# generate control law for the AUV
def control_law() -> np.ndarray:
    command = np.ndarray(8)
    command.fill(0)
    # print(f"New control command: {command}")
    return command



if __name__ == "__main__":
    with holoocean.make("PierHarbor-Hovering") as env:
        env.set_render_quality(1)
        for i in range(100000):
            if 'q' in pressed_keys:
                break
            command = np.ndarray(8)
            command.fill(0)
            state = env.step(command)
            if i % control_rate == 0:
                print(f'State: {state}')
                command = control_law()


