import rosbag
import numpy as np
import matplotlib.pyplot as plt
import sys

def get_data(bag_filename):
    time_command = []
    time_pose = []
    time_euler = []

    x_command = []
    y_command = []
    z_command = []

    roll = []
    pitch = []
    yaw = []

    x_pose = []
    y_pose = []
    z_pose = []

    bag = rosbag.Bag(bag_filename)
    topics = set([
        "/command/pose",
        "/ground_truth_to_tf/euler",
        "/ground_truth_to_tf/pose"
    ])

    for topic, msg, t in bag.read_messages(topics=topics):
        if topic == '/command/pose':
            time_command.append(msg.header.stamp.to_sec())
            x_command.append(msg.pose.position.x)
            y_command.append(msg.pose.position.y)
            z_command.append(msg.pose.position.z)

        if topic == '/ground_truth_to_tf/euler':
            time_euler.append(msg.header.stamp.to_sec())
            roll.append(msg.vector.x)
            pitch.append(msg.vector.y)
            yaw.append(msg.vector.z)

        if topic == '/ground_truth_to_tf/pose':
            time_pose.append(msg.header.stamp.to_sec())
            x_pose.append(msg.pose.position.x)
            y_pose.append(msg.pose.position.y)
            z_pose.append(msg.pose.position.z)
    bag.close()
    return time_command, time_pose, time_euler, x_command, y_command, \
                  z_command, roll, pitch, yaw, x_pose, y_pose, z_pose

if __name__ == '__main__':
    ref_x = 1.0
    ref_y = 0.0
    ref_z = 1.0

    bag_filename = sys.argv[1]
    time_command, time_pose, time_euler, x_command, y_command, \
                  z_command, roll, pitch, yaw, x_pose, y_pose, z_pose = get_data(bag_filename)

    time_command = np.asarray(time_command)
    x_command = np.asarray(x_command)
    y_command = np.asarray(y_command)
    z_command = np.asarray(z_command)

    time_euler = np.asarray(time_euler)
    roll = np.asarray(roll)
    pitch = np.asarray(pitch)
    yaw = np.asarray(yaw)

    time_pose = np.asarray(time_pose)
    x_pose = np.asarray(x_pose)
    y_pose = np.asarray(y_pose)
    z_pose = np.asarray(z_pose)

    step_index = np.where(x_command > (ref_x / 2.0))[0][0]
    step_time = time_command[step_index]

    time_command = time_command - step_time
    time_pose = time_pose - step_time
    time_euler = time_euler - step_time

    start_index = np.where(time_pose > 0)[0][0]
    end_index = np.where(time_pose > 8)[0][0]
    SAE = np.sum(np.abs(x_pose[start_index:end_index] - ref_x)) + \
          np.sum(np.abs(y_pose[start_index:end_index] - ref_y)) + \
          np.sum(np.abs(z_pose[start_index:end_index] - ref_z))

    print('the sum of abosulte erros is', SAE)
    print('the overshoot is', np.max(x_pose) - ref_x)
    print('the ripple of pitch angle is', np.max(pitch[end_index:]) - np.min(pitch[end_index:]))

    fig, axs = plt.subplots(2, 1)

    axs[0].plot(time_command, x_command, time_pose, x_pose, linewidth=4)
    axs[0].set_xlim(-1, 8)
    axs[0].set_ylim(-0.1, 1.3)
    axs[0].set_ylabel('x (m)', fontsize=30)
    axs[0].grid(True)
    axs[0].legend(('reference', 'response'), loc='lower right', fontsize=30)
    axs[0].set_xticklabels([])
    axs[0].tick_params(axis="y", labelsize=30)
    fig.suptitle('Step Response', fontsize=30)

    axs[1].plot(time_euler, pitch, linewidth=4)
    axs[1].set_xlim(-1, 8)
    axs[1].set_ylim(-0.3, 0.4)
    axs[1].set_xlabel('time (second)', fontsize=30)
    axs[1].tick_params(axis="x", labelsize=30)
    axs[1].tick_params(axis="y", labelsize=30)
    axs[1].set_ylabel('pitch (rad)', fontsize=30)
    axs[1].grid(True)
    # fig.tight_layout()
    plt.show()
