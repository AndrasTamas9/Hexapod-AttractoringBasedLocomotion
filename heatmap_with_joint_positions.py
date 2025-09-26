import numpy as np
import matplotlib.pyplot as plt
from matplotlib import rc
import math
import os
import glob
from matplotlib.ticker import FixedLocator

# -----------------------------------------------
# Joint position setup
# -----------------------------------------------
null_pose = [364, 512, 660, 660, 512, 364, 553, 553, 553, 471, 471, 471, 632, 632, 632, 392, 392, 392]
# Example for alternative starting pose (commented out)
# next_pose = [512, 512, 573, 573, 512, 450, 507, 522, 660, 360, 360, 360, 651, 652, 512, 754, 754, 754]
next_pose = [450, 826, 573, 510, 512, 450, 720, 884, 720, 272, 304, 304, 230, 340, 230, 753, 512, 794]

# -----------------------------------------------
# Robot geometry parameters
# -----------------------------------------------
phi_f = 0.261799388  # rad
phi_t = 0.785398163  # rad

# Shift vectors for each leg
shifts = np.array([
    np.array([[ 120, -60,  35],[0, -52, 0],[0, -66, 0],[0, -135, 0]]),
    np.array([[ 0,   -98,  35],[0, -52, 0],[0, -66, 0],[0, -135, 0]]), 
    np.array([[-120, -60,  35],[0, -52, 0],[0, -66, 0],[0, -135, 0]]), 
    np.array([[ 120,  60,  35],[0,  52, 0],[0,  66, 0],[0,  135, 0]]), 
    np.array([[ 0,    98,  35],[0,  52, 0],[0,  66, 0],[0,  135, 0]]), 
    np.array([[-120,  60,  35],[0,  52, 0],[0,  66, 0],[0,  135, 0]])
])

# Origin of the coordinate system
origo_coord = np.array([0, 0, 0])

# -----------------------------------------------
# Plot styling helper function
# -----------------------------------------------
def style_joint_axis(ax, ymin, ymax, major_ticks, minor_ticks):
    """Apply consistent styling for joint axis plots."""
    ax.set_ylim(ymin, ymax)
    ax.yaxis.set_major_locator(FixedLocator(major_ticks))
    ax.yaxis.set_minor_locator(FixedLocator(minor_ticks))
    ax.grid(True, which='major')
    ax.grid(True, which='minor', linewidth=0.8, color='0.8')
    ax.tick_params(axis='y', which='minor', length=3)

# -----------------------------------------------
# Encoder conversion and transformation matrices
# -----------------------------------------------
def encoder_to_rad(encoder_vect):
    """Convert encoder values to radians."""
    return np.array([val * (300/1023) * (math.pi / 180) for val in encoder_vect])

def ortnorm_x_rot_matrix(theta):
    """Orthogonal rotation matrix around x-axis."""
    return np.array([[1, 0, 0],[0, np.cos(theta), -np.sin(theta)],[0, np.sin(theta), np.cos(theta)]])

def ortnorm_z_rot_matrix(theta):
    """Orthogonal rotation matrix around z-axis."""
    return np.array([[np.cos(theta), -np.sin(theta), 0],[np.sin(theta), np.cos(theta), 0],[0, 0, 1]])

def hom_trans_matrix(ax, theta, vector):
    """Homogeneous transformation matrix for given axis."""
    if ax == 1:  # x-axis
        return np.array([
            [1, 0, 0, vector[0]],
            [0, np.cos(theta), -np.sin(theta), vector[1]],
            [0, np.sin(theta),  np.cos(theta), vector[2]],
            [0, 0, 0, 1]
        ])
    if ax == 2:  # y-axis
        return np.array([
            [np.cos(theta), 0, -np.sin(theta), vector[0]],
            [0, 1, 0, vector[1]],
            [np.sin(theta), 0, np.cos(theta), vector[2]],
            [0, 0, 0, 1]
        ])
    if ax == 3:  # z-axis
        return np.array([
            [np.cos(theta), -np.sin(theta), 0, vector[0]],
            [np.sin(theta),  np.cos(theta), 0, vector[1]],
            [0, 0, 1, vector[2]],
            [0, 0, 0, 1]
        ])

# -----------------------------------------------
# Kinematic calculations for one leg
# -----------------------------------------------
def calc_1leg(legnr, radians_coxa, radians_femur, radians_tibia):
    """Calculate end-point coordinate for a single leg."""
    theta_coxa = radians_null[legnr - 1] - radians_coxa[legnr - 1]
    theta_femur = radians_null[legnr + 5] - radians_femur[legnr - 1]
    theta_tibia = radians_null[legnr + 11] - radians_tibia[legnr - 1]

    shift = shifts[legnr - 1]
    if legnr > 3:  # for left side legs
        theta_coxa *= -1

    temp_f = np.matmul(hom_trans_matrix(3, theta_coxa, shift[0]), hom_trans_matrix(1, theta_femur, shift[1]))
    temp_t = np.matmul(temp_f, hom_trans_matrix(1, theta_tibia, shift[2]))
    temp_matrix = np.matmul(temp_t, hom_trans_matrix(1, 0, shift[3]))
    coordinate_vector = np.array(temp_matrix[0:3, 3])
    return coordinate_vector

# -----------------------------------------------
# Process all legs from data file
# -----------------------------------------------
def calc_legs(filename):
    """Calculate coordinates for all legs based on encoder data from file."""
    data_coxa_femur = np.loadtxt(filename, unpack=True)
    data = [[], [], [], [], [], []]
    for i in range(len(data_coxa_femur[0])):
        radians_coxa = encoder_to_rad(data_coxa_femur[3:9, i])
        radians_femur = encoder_to_rad(data_coxa_femur[9:15, i])
        radians_tibia = encoder_to_rad([230, 230, 230, 794, 794, 794])
        for j in range(6):
            data[j].append(calc_1leg(j + 1, radians_coxa, radians_femur, radians_tibia))
    return data

# -----------------------------------------------
# Initialize pose data
# -----------------------------------------------
radians_coxa = encoder_to_rad(next_pose[0:6])
radians_femur = encoder_to_rad(next_pose[6:12])
radians_tibia = encoder_to_rad(next_pose[12:18])
radians_null = encoder_to_rad(null_pose)

# -----------------------------------------------
# Analysis parameters
# -----------------------------------------------
TIME_WINDOW = 20   # seconds to visualize
TARGET_SHIFT = 20  # forward shift in indices

# -----------------------------------------------
# Process all .txt data files in folder
# -----------------------------------------------
txt_files = glob.glob("*.txt")

for file in txt_files:

    print(f"Processing: {file}")

    # Output filename
    save_name = file.replace('.txt', '_heatmap_current_terget_pos')

    # Load file, skip header row
    try:
        data_coxa_femur = np.loadtxt(file, skiprows=1, unpack=True)
    except Exception as e:
        print(f"Error reading {file}: {e}")
        continue

    # Reset time vector to start at 0
    data_coxa_femur[2] = data_coxa_femur[2] - np.min(data_coxa_femur[2])

    # Select last TIME_WINDOW seconds
    time_vector = data_coxa_femur[2]
    end_time = time_vector[-1]
    start_time = end_time - TIME_WINDOW

    # Crop data to time window
    data_start = np.searchsorted(time_vector, start_time)
    data_end = len(time_vector)

    radians_null = encoder_to_rad(null_pose)
    data = np.array(calc_legs(file))

    # Collect z-coordinates for all legs
    plot_data = [
        data[0][data_start:data_end, 2],
        data[1][data_start:data_end, 2],
        data[2][data_start:data_end, 2],
        data[3][data_start:data_end, 2],
        data[4][data_start:data_end, 2],
        data[5][data_start:data_end, 2]
    ]

    # -----------------------------------------------
    # Plot setup
    # -----------------------------------------------
    fig, (ax1, ax2, ax3) = plt.subplots(
        3, 1,
        figsize=(12, 10),
        sharex=True,
        gridspec_kw={'height_ratios': [2, 1, 1]}
    )

    # Heatmap of z-coordinates
    im = ax1.imshow(
        plot_data,
        aspect='auto',
        vmin=np.min(plot_data),
        vmax=np.max(plot_data),
        interpolation='none',
        extent=[data_coxa_femur[2, data_start], data_coxa_femur[2, data_end - 1], 6.5, 0.5],
        cmap='winter'
    )

    # Leg labels
    y_label_list = ['', 'RF 1', 'RM 2', 'RB 3', 'LF 4', 'LM 5', 'LB 6']
    ax1.set_yticklabels(y_label_list, fontsize=26)
    ax1.xaxis.set_tick_params(labelsize=26)
    ax1.tick_params(axis='both', labelsize=23)

    # Extract COXA data (current vs. target)
    coxa_raw = data_coxa_femur[3:9, data_start:data_end]
    coxa_rad = np.array([[val * (300/1023) * (math.pi / 180) for val in row] for row in coxa_raw])
    coxa_target_raw = data_coxa_femur[21:27, data_start:data_end]
    coxa_target_rad = np.array([[val * (300/1023) * (math.pi / 180) for val in row] for row in coxa_target_raw])

    # Extract FEMUR data (current vs. target)
    femur_raw = data_coxa_femur[9:15, data_start:data_end]
    femur_rad = np.array([[val * (300/1023) * (math.pi / 180) for val in row] for row in femur_raw])
    femur_target_raw = data_coxa_femur[27:, data_start:data_end]
    femur_target_rad = np.array([[val * (300/1023) * (math.pi / 180) for val in row] for row in femur_target_raw])

    # -----------------------------------------------
    # COXA plot (joints 1 and 2)
    # -----------------------------------------------
    ax2.plot(time_vector[data_start:data_end], coxa_rad[0], label=r"$q_1$", color='blue')
    ax2.plot(time_vector[data_start:data_end], coxa_target_rad[0], label=r"${}^{\\star}q_1$", color='blue', linestyle='--')
    ax2.plot(time_vector[data_start:data_end], coxa_rad[1], label=r"$q_2$", color='green')
    ax2.plot(time_vector[data_start:data_end], coxa_target_rad[1], label=r"${}^{\\star}q_2$", color='green', linestyle='--')
    ax2.set_ylabel(r"$q^{(c)}, \\;  {}^{\\star}q^{(c)}$", fontsize=26)
    ax2.legend(fontsize=26, loc='upper right', ncol=2)
    ax2.tick_params(axis='both', labelsize=26)
    style_joint_axis(ax2, ymin=2.0, ymax=3.0, major_ticks=[2.0, 2.5, 3.0], minor_ticks=[2.25, 2.75])

    # -----------------------------------------------
    # FEMUR plot (joints 1 and 2)
    # -----------------------------------------------
    ax3.plot(time_vector[data_start:data_end], femur_rad[0], label=r"$q_1$", color='blue')
    ax3.plot(time_vector[data_start:data_end], femur_target_rad[0], label=r"${}^{\\star}q_1$", color='blue', linestyle='--')
    ax3.plot(time_vector[data_start:data_end], femur_rad[1], label=r"$q_2$", color='green')
    ax3.plot(time_vector[data_start:data_end], femur_target_rad[1], label=r"${}^{\\star}q_2$", color='green', linestyle='--')
    ax3.set_ylabel(r"$q^{(f)}, \\; {}^{\\star}q^{(f)}$", fontsize=26)
    ax3.set_xlabel('t [s]', fontsize=26)
    ax3.legend(fontsize=26, loc='upper right', ncol=2)
    ax3.tick_params(axis='both', labelsize=26)
    style_joint_axis(ax3, ymin=3.0, ymax=4.0, major_ticks=[3.0, 3.5, 4.0], minor_ticks=[3.25, 3.75])

    # Save and close figure
    fig.tight_layout()
    plt.savefig(save_name, dpi=50)
    plt.close()
    print(f"Saved: {save_name}.png")
