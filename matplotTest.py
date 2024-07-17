import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

# Define robot parameters
link1 = 0.045
link2 = 0.1115
link3 = 0.155

length = 0.25205
width = 0.105577

leg_origins = np.array([
    [length/2, width/2, 0],
    [-length/2, width/2, 0],
    [-length/2, -width/2, 0],
    [length/2, -width/2, 0],
])

# Initialize figure
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
plt.subplots_adjust(left=0.1, bottom=0.4)
ax.set_xlim([-0.3, 0.3])
ax.set_ylim([-0.3, 0.3])
ax.set_zlim([-0.3, 0.3])
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.grid(True)

# Plot initial robot structure
h_body, = ax.plot([], [], [], 'r', linewidth=2)
h_legs = [ax.plot([], [], [], 'b', linewidth=2)[0] for _ in range(4)]

# Initial positions
body_pos = [0, 0, 0]
body_rot = [0, 0, 0]

# Rotate points using Euler angles
def rotate_points(points, angles):
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(angles[0]), -np.sin(angles[0])],
                   [0, np.sin(angles[0]), np.cos(angles[0])]])
    Ry = np.array([[np.cos(angles[1]), 0, np.sin(angles[1])],
                   [0, 1, 0],
                   [-np.sin(angles[1]), 0, np.cos(angles[1])]])
    Rz = np.array([[np.cos(angles[2]), -np.sin(angles[2]), 0],
                   [np.sin(angles[2]), np.cos(angles[2]), 0],
                   [0, 0, 1]])
    R = Rz @ Ry @ Rx
    return np.dot(points, R.T)

# Update the robot plot
def update_robot(body_pos, body_rot):
    global h_body, h_legs
    leg_origins_rotated = rotate_points(leg_origins, body_rot)
    leg_origins_translated = leg_origins_rotated + body_pos

    # Update body plot
    body_coords = np.vstack([leg_origins_translated, leg_origins_translated[0]])
    h_body.set_data(body_coords[:, 0], body_coords[:, 1])
    h_body.set_3d_properties(body_coords[:, 2])

    # Update legs plot
    for i, leg_origin in enumerate(leg_origins_translated):
        leg_coords = np.array([
            leg_origin,
            leg_origin + [0, -link1, 0],
            leg_origin + [0, -link1 - link2, 0],
            leg_origin + [0, -link1 - link2 - link3, 0]
        ])
        h_legs[i].set_data(leg_coords[:, 0], leg_coords[:, 1])
        h_legs[i].set_3d_properties(leg_coords[:, 2])

    plt.draw()

# Initial plot
update_robot(body_pos, body_rot)

# Create sliders
axcolor = 'lightgoldenrodyellow'
ax_pos_x = plt.axes([0.1, 0.3, 0.65, 0.03], facecolor=axcolor)
ax_pos_y = plt.axes([0.1, 0.25, 0.65, 0.03], facecolor=axcolor)
ax_pos_z = plt.axes([0.1, 0.2, 0.65, 0.03], facecolor=axcolor)
ax_rot_pitch = plt.axes([0.1, 0.15, 0.65, 0.03], facecolor=axcolor)
ax_rot_roll = plt.axes([0.1, 0.1, 0.65, 0.03], facecolor=axcolor)
ax_rot_yaw = plt.axes([0.1, 0.05, 0.65, 0.03], facecolor=axcolor)

s_pos_x = Slider(ax_pos_x, 'Pos X', -0.1, 0.1, valinit=0)
s_pos_y = Slider(ax_pos_y, 'Pos Y', -0.1, 0.1, valinit=0)
s_pos_z = Slider(ax_pos_z, 'Pos Z', -0.1, 0.1, valinit=0)
s_rot_pitch = Slider(ax_rot_pitch, 'Pitch', -np.pi, np.pi, valinit=0)
s_rot_roll = Slider(ax_rot_roll, 'Roll', -np.pi, np.pi, valinit=0)
s_rot_yaw = Slider(ax_rot_yaw, 'Yaw', -np.pi, np.pi, valinit=0)

# Update function for sliders
def update(val):
    body_pos = [s_pos_x.val, s_pos_y.val, s_pos_z.val]
    body_rot = [s_rot_pitch.val, s_rot_roll.val, s_rot_yaw.val]
    update_robot(body_pos, body_rot)

s_pos_x.on_changed(update)
s_pos_y.on_changed(update)
s_pos_z.on_changed(update)
s_rot_pitch.on_changed(update)
s_rot_roll.on_changed(update)
s_rot_yaw.on_changed(update)

plt.show()