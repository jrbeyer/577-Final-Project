import holoocean
import numpy as np
from pynput import keyboard
from typing import Tuple
# import matplotlib.pyplot as plt


np.set_printoptions(precision=4,suppress=True)

# constants
# Environment refresh rate (from Holoocean, s)
env_refresh_rate = 1/30
# Control rate (s)
control_rate = 1/30

pitch_kp = 0.1
pitch_ki = 0.0025
pitch_integrator = 0
pitch_i_saturation = 0.7
pitch_command_clip = 2

yaw_kp = 1.1
# yaw_ki = 0.0025
yaw_ki = 0.001
yaw_kd = 3
yaw_integrator = 0
yaw_i_saturation = 0.1
yaw_prior_error = 0
yaw_command_clip = 2





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
    command = np.array([5,5,-5,-5,0,0,0,0])
    return command

# test closing yaw loop
# yaw_current: degrees
# yaw_commanded: degrees
# output: 8x1 vector of thruster commands
def test_yaw_control(yaw_current, yaw_commanded) -> Tuple[np.ndarray, float, float, float, float, float]:
    global yaw_integrator
    global yaw_prior_error

    nominal_command = np.ndarray(8)
    nominal_command.fill(0)
    # nominal_command = np.array([0,0,0,0,4,4,4,4])

    unit_rotation_command = np.array([0,0,0,0,-1,1,1,-1])

    # constrain yaw_error to -180 to 180
    yaw_error = yaw_commanded - yaw_current
    if yaw_error > 180:
        yaw_error = yaw_error - 360
    elif yaw_error <= -180:
        yaw_error = yaw_error + 360

    # yaw control law
    yaw_proportional = yaw_kp * yaw_error
    yaw_integrator += yaw_ki * yaw_error * control_rate
    yaw_integrator = np.clip(yaw_integrator, -yaw_i_saturation, yaw_i_saturation)
    yaw_derivative = yaw_kd * (yaw_error - yaw_prior_error) / control_rate
    yaw_command = yaw_proportional + yaw_integrator + yaw_derivative
    yaw_command = np.clip(yaw_command, -yaw_command_clip, yaw_command_clip)

    yaw_prior_error = yaw_error

    # add yaw command to nominal command
    return (nominal_command + (yaw_command * unit_rotation_command), yaw_error, yaw_command, yaw_proportional, yaw_integrator, yaw_derivative)



if __name__ == "__main__":
    with holoocean.make("PierHarbor-Hovering") as env:
        env.set_render_quality(1)
        command = np.ndarray(8)
        command.fill(0)
        yaw_commanded = 180
        for i in range(100000):
            if 'q' in pressed_keys:
                break
            state = env.step(command)

            orientation = state["PoseSensor"][0:3,0:3]
            fx = orientation[0,0]
            fy = orientation[0,1]
            uz = orientation[2,2]
            yaw = np.arctan2(fx, fy)*180/np.pi
            # pitch = np.arccos(-uz)*180/np.pi
            command, yaw_error, yaw_command, yaw_proportional, yaw_integrator, yaw_derivative = test_yaw_control(yaw, yaw_commanded)

            if i % 10 == 0:
                print(f'Yaw: {yaw:.6}')
                print(f'Yaw error: {yaw_error:.6}')
                print(f'Yaw proportional: {yaw_proportional:.6}')
                print(f'Yaw integrator: {yaw_integrator:.6}')
                print(f'Yaw derivative: {yaw_derivative:.6}')
                print(f'Yaw command: {yaw_command:.6}')
                print(f'Command: {command}\n')

            # if i % control_rate == 0:
            #     print(f'State: {state}')
            #     print(f'Pose sensor: \n{state["PoseSensor"]}')
            #     # PoseSensor is a 4x4 array, with the first 3x3 being the rotation matrix, next 3x1 being x,y,z, and bottom row being 0,0,0,1
            #     print(f'Orientation matrix: \n{state["PoseSensor"][0:3,0:3]}')
            #     print(f'Position: \n{state["PoseSensor"][0:3,3]}')
            #     command = control_law()


