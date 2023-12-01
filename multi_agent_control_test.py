import holoocean
import numpy as np
from pynput import keyboard
from typing import Tuple, List, Dict
import time
import my_configs as config
from agent_wrapper import agent

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
def create_agent_list(cfg: Dict) -> List[agent]:
    agent_list = []
    ticks_per_sec = cfg["ticks_per_sec"]
    for agent_cfg in cfg["agents"]:
        agent_list.append(agent(name=agent_cfg["agent_name"], 
                                env_refresh_rate=1/ticks_per_sec, 
                                control_ticks_per_update=control_ticks_per_update))
    return agent_list

# Update pose of all agents from the global state
def update_all_agent_poses(agent_list: List[agent], state) -> None:
    for agent in agent_list:
        agent.update_pose(state[agent.name]['PoseSensor'])


# Execute line following control loop for all agents
# agent_list: list of all agents, as created by create_agent_list()
# lines: list of lines to be followed by each agent
#           lines[i] is a tuple of (start_point, end_point) for the line to be 
#           followed by agent i
# returns: list of controls for each agent
def control_all_agents(agent_list: List[agent], 
                       lines: List[Tuple[np.ndarray, np.ndarray]]
                       ) -> List[np.ndarray]:
    control_list = []
    for agent, line in zip(agent_list, lines):
        control_list.append(agent.compute_pure_pursuit_command(line[0], line[1]))
    return control_list

# Execute control loop under Lennard-Jones potentials for all agents
# agent_list: list of all agents, as created by create_agent_list()
# returns: list of controls for each agent
def control_all_agents_lj(agent_list: List[agent], target_distance: float) -> List[np.ndarray]:
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

# create line list depending on type of scenario
def create_line_list(agent_list: List[agent], scenario: str='simple') -> List[Tuple[np.ndarray, np.ndarray]]:
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

if __name__ == '__main__':
    # Load config
    cfg = config.get_many_agent_test_cfg()
    # Create agent list
    agent_list = create_agent_list(cfg)

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
        while iteration < 50000:
            if iteration == 6000:
                target_distance = 4
                env.draw_box([0, 0, 0], [5, 5, 5], [0, 255, 0], lifetime = 5)
            # Check for exit key
            if '`' in pressed_keys:
                break
            # Update all agent poses
            update_all_agent_poses(agent_list, state)
            # Control all agents
            if iteration % control_ticks_per_update == 0:
                # control_list = control_all_agents(agent_list, lines)
                control_list = control_all_agents_lj(agent_list, target_distance)
                for a in agent_list:
                    env.draw_line([c for c in a.position], [c for c in a.setpoint], [255, 0, 0], lifetime=0.1)

            # Send commands and step environment
            for a, control in zip(agent_list, control_list):
                env.act(a.name, control)
            state = env.tick()
            iteration += 1