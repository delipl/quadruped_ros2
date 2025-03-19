import numpy as np
import matplotlib.pyplot as plt

def generate_leg_trajectory(
    ticks, phase=0, periods=3, x0=0.3, y0=0.148, dx=0.0, dy=0.0, dz=0.15, dw=0.0
):
    z0 = -0.23
    x = np.linspace(x0, x0 + dx, ticks)
    xb = np.linspace(x0 + dx, x0, periods * ticks)
    y = np.linspace(y0, y0 + dy + dw, ticks)  # Include dw effect in Y
    yb = np.linspace(y0 + dy + dw, y0, periods * ticks)

    s = np.linspace(0, np.pi, ticks)
    sb = np.linspace(np.pi, 0, periods * ticks)

    z = z0 + dz * np.sin(s)
    if dz == 0:
        dminz = 0.0
    else:
        dminz = 0.01 

    x = np.concatenate((x, xb)) - dx / 2
    y = np.concatenate((y, yb)) - dy / 2
    z = np.concatenate((z, z0 - dminz * np.sin(sb)))

    cut = ticks * phase
    traj_x = np.hstack((x[cut:], x[:cut]))
    traj_y = np.hstack((y[cut:], y[:cut]))
    traj_z = np.hstack((z[cut:], z[:cut]))

    traj_contact = np.where(traj_z < (z0 + dminz), True, False)
    return traj_x, traj_y, traj_z, traj_contact

# Parameters
ticks = 30
dx, dy, dz, dw = 0.0, 0.0, 0.15, 0.1  # Added dw (angular velocity)
phases = [0, 1, 1, 0]  # Phase offset for each leg
x0es = 0.227 * np.array([1, 1, -1, -1])  # Initial X positions for each leg
y0es = 0.15 * np.array([1, -1, 1, -1])   # Initial Y positions for each leg
leg_names = ['FL (Front Left)', 'FR (Front Right)', 'RL (Rear Left)', 'RR (Rear Right)']

# Generate trajectories for all four legs
trajectories = []
for i in range(4):
    traj_x, traj_y, traj_z, _ = generate_leg_trajectory(
        ticks, phases[i], 1, x0es[i], y0es[i], dx, dy, dz, dw
    )
    trajectories.append((traj_x, traj_y, traj_z))

# Determine common Y-axis limits
all_y_values = []
for traj in trajectories:
    traj_x, traj_y, traj_z = traj
    all_y_values.extend(traj_x)
    all_y_values.extend(traj_y)
    all_y_values.extend(traj_z)

y_min = min(all_y_values) - 0.05
y_max = max(all_y_values) + 0.05

# Plot the trajectories using subplots
fig, axs = plt.subplots(4, 1, figsize=(10, 12), sharex=True)

for i, ax in enumerate(axs):
    traj_x, traj_y, traj_z = trajectories[i]
    ax.plot(traj_x, label='X trajectory')
    ax.plot(traj_y, label='Y trajectory')
    ax.plot(traj_z, label='Z trajectory')
    ax.set_title(leg_names[i])  # Add the leg name as the title
    ax.set_ylabel('Position')
    ax.set_ylim(y_min, y_max)  # Set consistent Y-axis limits
    ax.legend()
    ax.grid()

axs[-1].set_xlabel('Time Steps')  # Add a common X-axis label

# Save the figure
plt.tight_layout()
plt.savefig('/home/rabin/external_disk/robotics/master_thesis/figures/traj_dw.png', dpi=300)
plt.show()
