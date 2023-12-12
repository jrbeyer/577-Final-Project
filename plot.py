import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
from matplotlib.animation import FuncAnimation

csvfile = 'telemetry_2023-12-12_13-16-56.csv'

# Read the CSV file and store the data in a DataFrame
data = pd.read_csv(csvfile)

# Extract the necessary columns from the DataFrame
time = data['time']
num_agents = int((len(data.columns) - 1) / 5)
agent_x_positions = data[[f'agent_{i}_x' for i in range(num_agents)]]  # Update with the appropriate column names
agent_y_positions = data[[f'agent_{i}_y' for i in range(num_agents)]]

# print(agent_x_positions)

# Create a static plot of trajectories in the x-y plane
plt.figure()
for agent in range(num_agents):
    # print(agent_x_positions.iloc[:, agent])
    plt.plot(agent_x_positions.iloc[:, agent], agent_y_positions.iloc[:, agent])
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Trajectories in the X-Y Plane')
plt.legend([f'Agent {i}' for i in range(num_agents)])
plt.grid(True)
plt.show()

# Create an animated plot of trajectories in the x-y plane
fig, ax = plt.subplots()
agent_x_npframe = agent_x_positions.to_numpy()
agent_y_npframe = agent_y_positions.to_numpy()
ax.set_xlim(agent_x_npframe.min(), agent_x_npframe.max())
ax.set_ylim(agent_y_npframe.min(), agent_y_npframe.max())
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_title('Animated Trajectories in the X-Y Plane')
lines = [ax.plot([], [], label=f'Agent {agent}')[0] for agent in range(num_agents)]
ax.legend()
ax.grid(True)

def animate(frame):
    for agent, line in enumerate(lines):
        line.set_data(agent_x_positions.iloc[:frame, agent], agent_y_positions.iloc[:frame, agent])
    return lines

animation = FuncAnimation(fig, animate, frames=len(time), interval=5, blit=True)
# animation.save('animated_trajectories.gif', writer='imagemagick', fps=30
# animation.save('animated_trajectories.mp4', writer='ffmpeg', fps=30)
# save at 4x speed
# animation.save('animated_trajectories_4x.mp4', writer='ffmpeg', fps=120)
plt.show()

# create plot showing distance to centroid
PLOT_CENTROID_DISTANCE = True

# Compute the centroid at each timestep from data
centroid_x_positions = agent_x_positions.mean(axis=1)
centroid_y_positions = agent_y_positions.mean(axis=1)
centroid_x_npframe = centroid_x_positions.to_numpy()
centroid_y_npframe = centroid_y_positions.to_numpy()

# Create a static plot of distance to centroid over time
plt.figure()
for agent in range(num_agents):
    plt.plot(time, np.sqrt((agent_x_positions.iloc[:, agent] - centroid_x_positions)**2 + (agent_y_positions.iloc[:, agent] - centroid_y_positions)**2))

plt.xlabel('Time (arbitrary units)')
plt.ylabel('Distance to Centroid (m)')
plt.title('Distance to Centroid over Time')
plt.legend([f'Agent {i}' for i in range(num_agents)])
plt.grid(True)
plt.show()

