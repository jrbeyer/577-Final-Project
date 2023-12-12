import holoocean
import numpy as np
from pynput import keyboard
from typing import Tuple, List, Dict
import time
import my_configs as config
from agent_wrapper import Agent
from telemetry import Telemetry
import queue

# number of environment ticks between control updates
control_ticks_per_update = 5

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

# Create list of agents, named by their index in the list
def create_agent_list(cfg: Dict) -> List[Agent]:
    agent_list = []
    ticks_per_sec = cfg["ticks_per_sec"]
    for agent_cfg in cfg["agents"]:
        agent_list.append(Agent(name=agent_cfg["agent_name"], 
                                env_refresh_rate=1/ticks_per_sec, 
                                control_ticks_per_update=control_ticks_per_update))
    return agent_list

# Update pose of all agents from the global state
def update_all_agent_poses(agent_list: List[Agent], state) -> None:
    for agent in agent_list:
        agent.update_pose(state[agent.name]['PoseSensor'])


# Execute line following control loop for all agents
# agent_list: list of all agents, as created by create_agent_list()
# lines: list of lines to be followed by each agent
#           lines[i] is a tuple of (start_point, end_point) for the line to be 
#           followed by agent i
# returns: list of controls for each agent
def control_all_agents(agent_list: List[Agent], 
                       lines: List[Tuple[np.ndarray, np.ndarray]]
                       ) -> List[np.ndarray]:
    control_list = []
    for agent, line in zip(agent_list, lines):
        control_list.append(agent.compute_pure_pursuit_command(line[0], line[1]))
    return control_list

# Execute control loop under Lennard-Jones potentials for all agents
# agent_list: list of all agents, as created by create_agent_list()
# returns: list of controls for each agent
def control_all_agents_lj(agent_list: List[Agent], target_distance: float) -> List[np.ndarray]:
    control_list = []
    for agent in agent_list:
        setpoint = agent.compute_lj_setpoint(agent_list, target_distance)
        line = (agent.position, setpoint)
        control_list.append(agent.compute_pure_pursuit_command(line[0], line[1]))
        # if agent.name == 'auv0':
            # print(f'LJ Setpoint:    {setpoint}')
            # print(f'Agent setpoint: {agent.setpoint}')
            # print(f'Pitch desired:  {agent.pitch_desired}')
            # print(f'Pitch:          {agent.pitch}')
    return control_list

# Execute control loop using milling algorithm for all agents
# agent_list: list of all agents, as created by create_agent_list()
# alpha: half-angle of milling cone
# returns: list of controls for each agent
def control_all_agents_milling(agent_list: List[Agent], alpha: float, object_of_interest: np.ndarray) -> List[np.ndarray]:
    control_list = []
    for agent in agent_list:
        setpoint = agent.compute_milling_with_object_setpoint(agent_list, alpha, object_of_interest)
        line = (agent.position, setpoint)
        control_list.append(agent.compute_pure_pursuit_command(line[0], line[1]))
        # if agent.name == 'auv0':
        #     print(f'Position:       {agent.position}')
        #     print(f'Neighbor pos:   {agent.compute_nearest_neighbor_position(agent_list)}')
        #     print(f'Mill setpoint:  {setpoint}')
        #     print(f'Agent setpoint: {agent.setpoint}')
        #     print(f'Yaw desired:    {agent.yaw_desired}')
        #     print(f'Yaw:            {agent.yaw}')
    return control_list

# create line list depending on type of scenario
def create_line_list(agent_list: List[Agent], scenario: str='simple') -> List[Tuple[np.ndarray, np.ndarray]]:
    lines = []
    if scenario == 'simple':
        for a in agent_list:
            W_i1 = a.position + np.array([10, -10, -3])
            lines.append((a.position, W_i1))
    if scenario == 'middle':
        for a in agent_list:
            W_i1 = np.array([0, 0, -5])
            lines.append((a.position, W_i1))

    return lines

# Create a row of telemetry data for all agents at this time step
def create_telemetry_row(agent_list: List[Agent], time: float) -> List:
    row = []
    row.append(time)
    for a in agent_list:
        row.append(a.position[0])
        row.append(a.position[1])
        row.append(a.position[2])
        row.append(a.yaw)
        row.append(a.pitch)
    return row

# TELEMETRY_ON = False
TELEMETRY_ON = True

if __name__ == '__main__':
    # Load config
    # cfg = config.get_many_agent_test_cfg()
    cfg = config.get_medium_agent_test_cfg()
    ticks_per_second = cfg["ticks_per_sec"]
    # Create agent list
    agent_list = create_agent_list(cfg)

    if TELEMETRY_ON:
        telemetry_queue = queue.Queue()
        telemeter = Telemetry(telemetry_queue)
        telemeter.start()

    # Create environment
    with holoocean.make(scenario_cfg=cfg) as env:
        # Run simulation
        state = env.tick()
        update_all_agent_poses(agent_list, state)

        # Create line list
        lines = create_line_list(agent_list, scenario='middle')
        control_list = []
        for a in agent_list:
            control_list.append(a.nominal_command)

        iteration = 0
        target_distance = 10
        object_of_interest = None
        while iteration < 15000:
            if iteration == 7500:
                object_of_interest = [7, -7, -5]
                target_distance = 4
                env.draw_box(object_of_interest, [0.5, 0.5, 0.5], [0, 255, 0], lifetime = 0)
            # Check for exit key
            if '`' in pressed_keys:
                break
            # Update all agent poses
            update_all_agent_poses(agent_list, state)
            # Control all agents
            if iteration % control_ticks_per_update == 0:
                # control_list = control_allsd_agents(agent_list, lines)
                # control_list = control_all_agents_lj(agent_list, target_distance)
                control_list = control_all_agents_milling(agent_list, alpha=15, object_of_interest=object_of_interest)
                for a in agent_list:
                    env.draw_line([c for c in a.position], [c for c in a.setpoint], [255, 0, 0], lifetime=0.1)
                # Write telemetry data
                if TELEMETRY_ON:
                    telemetry_row = create_telemetry_row(agent_list, iteration/ticks_per_second)
                    telemetry_queue.put(telemetry_row)

            # Send commands and step environment
            for a, control in zip(agent_list, control_list):
                env.act(a.name, control)
            state = env.tick()
            iteration += 1

        if TELEMETRY_ON:
            telemeter.stop()