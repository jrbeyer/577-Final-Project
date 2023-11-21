import numpy as np
from typing import Tuple, List

#   Wrap the agent in an object to make it easier to use
class agent:
    def __init__(self, name, env_refresh_rate, control_ticks_per_update):
        self.name = name

        self.env_refresh_rate = env_refresh_rate
        self.control_ticks_per_update = control_ticks_per_update
        self.control_rate = self.env_refresh_rate*self.control_ticks_per_update

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

        self.nominal_command = np.array([0,0,0,0,9,9,9,9])
        self.position = np.array([0,0,0])

        self.setpoint = np.array([0,0,0])

    def update_pose(self, pose):
        self.orientation = pose[0:3,0:3]
        self.position = pose[0:3,3]
        self.fx = self.orientation[0,0]
        self.fy = self.orientation[1,0]
        self.fz = self.orientation[2,0]
        self.yaw = np.arctan2(self.fy, self.fx)*180/np.pi
        self.pitch = np.arcsin(-self.fz)*180/np.pi

    def yaw_loop(self, yaw_desired
                 ) -> Tuple[np.ndarray, float, float, float, float, float]:
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
        self.yaw_integrator = np.clip(self.yaw_integrator, 
                                      -self.yaw_i_saturation, 
                                      self.yaw_i_saturation)
        yaw_derivative = self.yaw_kd * (yaw_error - self.yaw_prior_error) / self.control_rate
        yaw_pid_out = yaw_proportional + self.yaw_integrator + yaw_derivative
        yaw_pid_out = np.clip(yaw_pid_out, -self.yaw_command_clip, self.yaw_command_clip)

        self.yaw_prior_error = yaw_error

        return (yaw_pid_out * unit_rotation_command, 
                yaw_error, 
                yaw_pid_out, 
                yaw_proportional, 
                self.yaw_integrator, 
                yaw_derivative)
    
    def pitch_loop(self, pitch_desired
                   ) -> Tuple[np.ndarray, float, float, float, float, float]:
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

        return (pitch_pid_out * unit_rotation_command,
                pitch_error, 
                pitch_pid_out, 
                pitch_proportional, 
                self.pitch_integrator, 
                pitch_derivative)
    
    # pure pursuit law
    # adapted from Pelizer et al. (2017)
    # W_i: initial position of line to track
    # W_i1: final position of line to track
    # position: current position of AUV
    # output: (yaw_desired, pitch_desired)
    # modifies: self.setpoint
    def compute_pure_pursuit_angles(self, W_i, W_i1) -> Tuple[float, float]:
        position = self.position.reshape(-1,1)
        alpha = np.arctan2(W_i1[1]-W_i[1], W_i1[0]-W_i[0])

        v = self.position - W_i
        s = W_i1 - W_i
        u = np.dot(v,s)*s / np.dot(s,s)
        midpoint = 0.3*(W_i1-W_i) + 0.7*u + W_i
        self.setpoint = midpoint
        yaw_desired = np.arctan2(midpoint[1]-position[1], midpoint[0]-position[0])*180/np.pi
        # yaw_desired = np.arctan2(W_i1[1]-position[1], W_i1[0]-position[0])*180/np.pi


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
    
    # Compute control law to be executed for pure pursuit
    # Attenuates speed when reaching goal
    # returns: command vector
    def compute_pure_pursuit_command(self, W_i, W_i1) -> np.ndarray:
        yaw_desired, pitch_desired = self.compute_pure_pursuit_angles(W_i, W_i1)
        yaw_command = self.yaw_loop(yaw_desired)
        pitch_command = self.pitch_loop(pitch_desired)

        attenuated_nominal_command = self.nominal_command
        l2norm = np.linalg.norm(self.position - W_i1)
        l2norm_limit = 4.0
        l2norm_stop_limit = 0.4

        if l2norm < l2norm_limit:
            attenuated_nominal_command = (l2norm/l2norm_limit)*self.nominal_command
        if l2norm < l2norm_stop_limit:
            attenuated_nominal_command = np.zeros(8)

        # print('='*20)
        # print(f'Agent {self.name}')
        # print(f'Position: {self.position}')
        # print(f'Yaw: {self.yaw}')
        # print(f'Yaw desired: {yaw_desired}')
        # print(f'Pitch: {self.pitch}')
        # print(f'Pitch desired: {pitch_desired}')
        # print(f'Yaw command: {yaw_command[0]}')
        # print(f'Pitch command: {pitch_command[0]}')
        # print(f'Final command: {self.nominal_command + yaw_command[0] + pitch_command[0]}')
        # print('='*20)
        return attenuated_nominal_command + yaw_command[0] + pitch_command[0]
    
    # Compute simple control law for turning to view a point
    # point: 3D point to look at
    # returns: command vector
    def compute_turn_control(self, point: np.ndarray) -> np.ndarray:
        yaw_desired = np.arctan2(point[1]-self.position[1], 
                                 point[0]-self.position[0])*180/np.pi
        yaw_command = self.yaw_loop(yaw_desired)
        return yaw_command[0]
    

    # Compute if a point is within the field of view of the agent, assuming
    # a 5 degree blind spot behind the agent (after Berlinger et al. 2021)
    # point: 3D point to check
    # returns: boolean
    def can_see_point(self, point: np.ndarray) -> bool:
        max_angle = 175
        d = point - self.position
        f = self.orientation[:,0]
        relative_angle = abs(np.arccos(np.dot(d, f) 
                                   / np.linalg.norm(d)))*180/np.pi
        # print(f'relative angle: {relative_angle}')
        # print(f'd: {d}')
        # print(f'f: {f}')
        return relative_angle < max_angle
    
    # Compute the next pure pursuit setpoint using Lennard-Jones potential
    # adapted from Berlinger et al. 2021
    # agent_list: list of all agents in the environment (including self)
    # target_distance: target neighbor distance
    # returns: next setpoint
    def compute_lj_setpoint(self, agent_list: List['agent'], target_distance: float) -> np.ndarray:
        a = 12
        b = 6
        # compute the setpoint for the current agent
        setpoint = np.zeros(3)
        for other_agent in agent_list:
            if other_agent.name != self.name: # names are unique
                distance = self.position - other_agent.position
                distance_norm = np.linalg.norm(distance)
                target_norm = target_distance/distance_norm
                unit_distance = distance/distance_norm
                setpoint += (a*(target_norm)**a - 2*b*(target_norm)**b)*unit_distance
        return setpoint / (len(agent_list)-1)