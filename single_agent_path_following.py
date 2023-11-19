import holoocean
import numpy as np
from pynput import keyboard
from typing import Tuple
import time
import my_configs as config
from agent_wrapper import agent
# import matplotlib.pyplot as plt


np.set_printoptions(precision=4,suppress=True)

# constants
# Environment refresh rate (from Holoocean, s)
env_refresh_rate = 1/100

control_ticks_per_update = 5
# Control rate (s)
control_rate = env_refresh_rate * control_ticks_per_update


pitch_kp = 0.05
pitch_ki = 0.1
pitch_kd = 0 
# pitch_kp = 1.1
# pitch_ki = 0.001
# pitch_kd = 3
pitch_integrator = 0
pitch_i_saturation = 3
pitch_prior_error = 0
pitch_command_clip = 6

yaw_kp = 1.1
# yaw_ki = 0.0025
yaw_ki = 0.001
yaw_kd = 3
yaw_integrator = 0
yaw_i_saturation = 1
yaw_prior_error = 0
yaw_command_clip = 3


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
    env_cfg = config.get_single_agent_test_cfg()

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

# yaw control law
# yaw_current: degrees
# yaw_commanded: degrees
# output: 8x1 vector of differential thruster commands
def yaw_loop(yaw_current, yaw_desired) -> Tuple[np.ndarray, float, float, float, float, float]:
    global yaw_integrator
    global yaw_prior_error

    nominal_command = np.ndarray(8)
    nominal_command.fill(0)

    unit_rotation_command = np.array([0,0,0,0,1,-1,-1,1])

    # constrain yaw_error to -180 to 180
    yaw_error = yaw_desired - yaw_current
    if yaw_error > 180:
        yaw_error = yaw_error - 360
    elif yaw_error <= -180:
        yaw_error = yaw_error + 360

    # yaw control law
    yaw_proportional = yaw_kp * yaw_error
    yaw_integrator += yaw_ki * yaw_error * control_rate
    yaw_integrator = np.clip(yaw_integrator, -yaw_i_saturation, yaw_i_saturation)
    yaw_derivative = yaw_kd * (yaw_error - yaw_prior_error) / control_rate
    yaw_pid_out = yaw_proportional + yaw_integrator + yaw_derivative
    yaw_pid_out = np.clip(yaw_pid_out, -yaw_command_clip, yaw_command_clip)

    yaw_prior_error = yaw_error

    return (yaw_pid_out * unit_rotation_command, yaw_error, yaw_pid_out, yaw_proportional, yaw_integrator, yaw_derivative)

# pitch control law
# pitch_current: degrees
# pitch_commanded: degrees
# output: 8x1 vector of differential thruster commands
def pitch_loop(pitch_current, pitch_desired) -> Tuple[np.ndarray, float, float, float, float, float]:
    global pitch_integrator
    global pitch_prior_error

    nominal_command = np.ndarray(8)
    nominal_command.fill(0)

    unit_rotation_command = np.array([-1,-1,1,1,0,0,0,0])

    # constrain pitch_error to -180 to 180
    pitch_error = pitch_desired - pitch_current
    if pitch_error > 180:
        pitch_error = pitch_error - 360
    elif pitch_error <= -180:
        pitch_error = pitch_error + 360

    # pitch control law
    pitch_proportional = pitch_kp * pitch_error
    pitch_integrator += pitch_ki * pitch_error * control_rate
    pitch_integrator = np.clip(pitch_integrator, -pitch_i_saturation, pitch_i_saturation)
    pitch_derivative = pitch_kd * (pitch_error - pitch_prior_error) / control_rate
    pitch_pid_out = pitch_proportional + pitch_integrator + pitch_derivative
    pitch_pid_out = np.clip(pitch_pid_out, -pitch_command_clip, pitch_command_clip)

    pitch_prior_error = pitch_error

    return (pitch_pid_out * unit_rotation_command, pitch_error, pitch_pid_out, pitch_proportional, pitch_integrator, pitch_derivative)

# pure pursuit law
# adapted from Pelizer et al. (2017)
# W_i: initial position of line to track
# W_i1: final position of line to track
# position: current position of AUV
# output: (yaw_desired, pitch_desired)
def pure_pursuit(W_i, W_i1, position) -> Tuple[float, float]:
    position = position.reshape(-1,1)
    alpha = np.arctan2(W_i1[1]-W_i[1], W_i1[0]-W_i[0])
    yaw_desired = np.arctan2(W_i1[1]-position[1], W_i1[0]-position[0])*180/np.pi


    Rz = np.array([[np.cos(alpha),  np.sin(alpha),  0],
                   [-np.sin(alpha), np.cos(alpha),  0],
                   [0,              0,              1]])

    W_i1_r = Rz @ W_i1
    position_r = Rz @ position

    pitch_desired = np.arctan2(position_r[2]-W_i1_r[2], W_i1_r[0]-position_r[0])*180/np.pi

    # W_i_r = Rz*W_i
    # beta = np.arctan2(W_i_r[2]-W_i1_r[2], 
    #                   np.sqrt((W_i_r[0]-W_i1_r[0])**2 + (W_i_r[1]-W_i1_r[1])**2))

    # Ry = np.array([[np.cos(beta),   0,              -np.sin(beta)],
    #                [0,              1,              0],
    #                [np.sin(beta),   0,              np.cos(beta)]])

    # e = Ry*Rz*(position - W_i)
    
    return (yaw_desired[0], pitch_desired[0])


VERBOSE = False

if __name__ == "__main__":
    env_cfg = config.get_single_agent_test_cfg()
    # print(f'env_cfg: {env_cfg}')
    # with holoocean.make("PierHarbor-Hovering") as env:
        # print(f'Environment info: \n{env.info()}')
    with holoocean.make(scenario_cfg=env_cfg) as env:
        env.set_render_quality(1)
        nominal_command = np.array([0,0,0,0,9,9,9,9])
        # nominal_command = np.array([0,0,0,0,0,0,0,0])
        command = nominal_command
        yaw_desired = 0
        pitch_desired = 0

        auv0 = agent('auv0', env_refresh_rate, control_ticks_per_update)

        state = env.tick()
        W_i = np.array([5, 5, -5])
        W_i1 = W_i + np.array([20,-20,-4])

        env.draw_box([c for c in W_i1], [1, 1, 1], lifetime=0)
        env.draw_line([c for c in W_i], [c for c in W_i1], lifetime=0)


        print(f'Initial position: {W_i}')
        print(f'Final position: {W_i1}')


        for i in range(100000):
            if '`' in pressed_keys:
                break
            # time.sleep(0.1)

            if i % control_ticks_per_update == 0:
                orientation = state["PoseSensor"][0:3,0:3]
                position = state["PoseSensor"][0:3,3]
                fx = orientation[0,0]
                fy = orientation[1,0]
                fz = orientation[2,0]
                yaw = np.arctan2(fy, fx)*180/np.pi
                pitch = np.arcsin(-fz)*180/np.pi
                auv0.update_pose(state["PoseSensor"])
                yaw_desired, pitch_desired = auv0.pure_pursuit(W_i, W_i1)
                yaw_command, yaw_error, yaw_pid_out, yaw_proportional, yaw_integrator, yaw_derivative = auv0.yaw_loop(yaw_desired)
                pitch_command, pitch_error, pitch_pid_out, pitch_proportional, pitch_integrator, pitch_derivative = auv0.pitch_loop(pitch_desired)
                command = nominal_command + pitch_command + yaw_command

                v = position - W_i
                s = W_i1 - W_i
                u = np.dot(v,s)*s / np.dot(s,s)
                # midpoint = (W_i1-W_i + u)/2 + W_i
                midpoint = 0.3*(W_i1-W_i) + 0.7*u + W_i
                env.draw_box([c for c in midpoint], extent=[1,1,1], color=[0,255,0], lifetime=0.1)

            env.act('auv0', command)
            state = env.tick()

            if i % (5*control_ticks_per_update) == 0:
                print(f'\nIteration: {i}')
                print(f'Command: {command}')
                if VERBOSE:
                    print(f'Orientation matrix: \n{state["PoseSensor"][0:3,0:3]}')
                    # print(f'State: \n{state}')
                    print(f'Position: {position}')
                    print(f'Position error: {position - W_i1}')
                    print(f'Yaw desired: {yaw_desired}')
                    print(f'Yaw: {yaw:.6}')
                    print(f'Yaw error: {yaw_error:.6}')
                    # print(f'Yaw proportional: {yaw_proportional:.6}')
                    # print(f'Yaw integrator: {yaw_integrator:.6}')
                    # print(f'Yaw derivative: {yaw_derivative:.6}')
                    print(f'Yaw PID out: {yaw_pid_out:.6}')
                    print(f'Pitch desired: {pitch_desired}')
                    print(f'Pitch: {pitch:.6}')
                    print(f'Pitch error: {pitch_error:.6}')
                    # print(f'Pitch proportional: {pitch_proportional:.6}')
                    # print(f'Pitch integrator: {pitch_integrator:.6}')
                    # print(f'Pitch derivative: {pitch_derivative:.6}')
                    # print(f'Pitch PID out: {pitch_pid_out:.6}')
                    print(f'Yaw command: {yaw_command}')
                    print(f'Pitch command: {pitch_command}')

