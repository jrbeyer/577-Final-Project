import numpy as np
from typing import Tuple

#   Wrap the agent in an object to make it easier to use
class agent:
    def __init__(self, name, env_refresh_rate, control_ticks_per_update):
        self.name = name

        self.env_refresh_rate = env_refresh_rate
        self.control_ticks_per_update = control_ticks_per_update
        self.control_rate = self.env_refresh_rate * self.control_ticks_per_update

        self.pitch_kp = 0.05
        self.pitch_ki = 0.1
        self.pitch_kd = 0 
        self.pitch_integrator = 0
        self.pitch_i_saturation = 3
        self.pitch_prior_error = 0
        self.pitch_command_clip = 6

        self.yaw_kp = 1.1
        self.yaw_ki = 0.001
        self.yaw_kd = 3
        self.yaw_integrator = 0
        self.yaw_i_saturation = 1
        self.yaw_prior_error = 0
        self.yaw_command_clip = 3

    def update_pose(self, pose):
        self.orientation = pose[0:3,0:3]
        self.position = pose[0:3,3]
        self.fx = self.orientation[0,0]
        self.fy = self.orientation[1,0]
        self.fz = self.orientation[2,0]
        self.yaw = np.arctan2(self.fy, self.fx)*180/np.pi
        self.pitch = np.arcsin(-self.fz)*180/np.pi

    def yaw_loop(self, yaw_desired) -> Tuple[np.ndarray, float, float, float, float, float]:
        nominal_command = np.ndarray(8)
        nominal_command.fill(0)

        unit_rotation_command = np.array([0,0,0,0,1,-1,-1,1])

        # constrain yaw_error to -180 to 180
        yaw_error = yaw_desired - self.yaw
        if yaw_error > 180:
            yaw_error = yaw_error - 360
        elif yaw_error <= -180:
            yaw_error = yaw_error + 360

        # yaw control law
        yaw_proportional = self.yaw_kp * yaw_error
        self.yaw_integrator += self.yaw_ki * yaw_error * self.control_rate
        self.yaw_integrator = np.clip(self.yaw_integrator, -self.yaw_i_saturation, self.yaw_i_saturation)
        yaw_derivative = self.yaw_kd * (yaw_error - self.yaw_prior_error) / self.control_rate
        yaw_pid_out = yaw_proportional + self.yaw_integrator + yaw_derivative
        yaw_pid_out = np.clip(yaw_pid_out, -self.yaw_command_clip, self.yaw_command_clip)

        self.yaw_prior_error = yaw_error

        return (yaw_pid_out * unit_rotation_command, yaw_error, yaw_pid_out, yaw_proportional, self.yaw_integrator, yaw_derivative)
    
    def pitch_loop(self, pitch_desired) -> Tuple[np.ndarray, float, float, float, float, float]:
        nominal_command = np.ndarray(8)
        nominal_command.fill(0)

        unit_rotation_command = np.array([-1,-1,1,1,0,0,0,0])

        # constrain pitch_error to -180 to 180
        pitch_error = pitch_desired - self.pitch
        if pitch_error > 180:
            pitch_error = pitch_error - 360
        elif pitch_error <= -180:
            pitch_error = pitch_error + 360

        # pitch control law
        pitch_proportional = self.pitch_kp * pitch_error
        self.pitch_integrator += self.pitch_ki * pitch_error * self.control_rate
        self.pitch_integrator = np.clip(self.pitch_integrator, -self.pitch_i_saturation, self.pitch_i_saturation)
        pitch_derivative = self.pitch_kd * (pitch_error - self.pitch_prior_error) / self.control_rate
        pitch_pid_out = pitch_proportional + self.pitch_integrator + pitch_derivative
        pitch_pid_out = np.clip(pitch_pid_out, -self.pitch_command_clip, self.pitch_command_clip)

        self.pitch_prior_error = pitch_error

        return (pitch_pid_out * unit_rotation_command, pitch_error, pitch_pid_out, pitch_proportional, self.pitch_integrator, pitch_derivative)
    
    # pure pursuit law
    # adapted from Pelizer et al. (2017)
    # W_i: initial position of line to track
    # W_i1: final position of line to track
    # position: current position of AUV
    # output: (yaw_desired, pitch_desired)
    def pure_pursuit(self, W_i, W_i1) -> Tuple[float, float]:
        position = self.position.reshape(-1,1)
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