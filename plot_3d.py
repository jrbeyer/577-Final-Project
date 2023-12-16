from mpl_toolkits import mplot3d

import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

csvfile = 'telemetry_2023-12-16_18-19-52.csv'

# Read the CSV file and store the data in a DataFrame
data = pd.read_csv(csvfile)

# Extract the necessary columns from the DataFrame
time = data['time']
num_agents = 1
agent_x_positions = data[[f'agent_{i}_x' for i in range(num_agents)]]  # Update with the appropriate column names
agent_y_positions = data[[f'agent_{i}_y' for i in range(num_agents)]]
agent_z_positions = data[[f'agent_{i}_z' for i in range(num_agents)]]

# also going to plot the line it is following: from [4, 4, -5] to [-25, -25, -8]
line_x_positions = pd.Series([4, -25])
line_y_positions = pd.Series([4, -25])
line_z_positions = pd.Series([-5, -8])


# create a static plot of trajectory in 3D
fig = plt.figure()
ax = plt.axes(projection='3d')
for agent in range(num_agents):
    # plt.plot(agent_x_positions.iloc[:, agent], agent_y_positions.iloc[:, agent], agent_z_positions.iloc[:, agent])
    ax.plot3D(agent_x_positions.iloc[:, agent], agent_y_positions.iloc[:, agent], agent_z_positions.iloc[:, agent], 'red')
ax.plot3D(line_x_positions, line_y_positions, line_z_positions, 'blue')
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
ax.set_zlabel('Z (m)')
plt.title('Pure Pursuit Trajectory')
plt.legend([f'Agent {i}' for i in range(num_agents)])
plt.grid(True)
plt.show()