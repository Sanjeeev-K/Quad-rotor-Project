import rosbag
import numpy as np
import matplotlib.pyplot as plt
import sys
import tf

def get_data(bag_filename):
    time_command = []
    time_pose = []

    yaw_command = []

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
            (yaw_ref, pitch_ref, roll_ref) = tf.transformations.euler_from_quaternion([msg.pose.orientation.x,
            msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w], axes= 'rzyx')
            yaw_command.append(yaw_ref)

        if topic == '/ground_truth_to_tf/pose':
            time_pose.append(msg.header.stamp.to_sec())
            x_pose.append(msg.pose.position.x)
            y_pose.append(msg.pose.position.y)
            z_pose.append(msg.pose.position.z)

            (yaw_tru, pitch_tru, roll_tru) = tf.transformations.euler_from_quaternion([msg.pose.orientation.x,
            msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w], axes= 'rzyx')

            roll.append(roll_tru)
            pitch.append(pitch_tru)
            yaw.append(yaw_tru)
    bag.close()
    return time_command, time_pose, yaw_command, \
                  roll, pitch, yaw, x_pose, y_pose, z_pose

if __name__ == '__main__':
    ref_yaw = 1.0

    bag_filename = sys.argv[1]
    time_command, time_pose, yaw_command, \
                  roll, pitch, yaw, x_pose, y_pose, z_pose = get_data(bag_filename)

    time_command = np.asarray(time_command)
    yaw_command = np.asarray(yaw_command)

    roll = np.asarray(roll)
    pitch = np.asarray(pitch)
    yaw = np.asarray(yaw)

    time_pose = np.asarray(time_pose)
    x_pose = np.asarray(x_pose)
    y_pose = np.asarray(y_pose)
    z_pose = np.asarray(z_pose)

    step_index = np.where(yaw_command > (ref_yaw / 2.0))[0][0]
    step_time = time_command[step_index]

    time_command = time_command - step_time
    time_pose = time_pose - step_time

    start_index = np.where(time_pose > 0)[0][0]
    end_index = np.where(time_pose > 4)[0][0]
    SAE = np.sum(np.abs(yaw[start_index:end_index] - ref_yaw))

    print('the sum of abosulte erros is', SAE)
    print('the overshoot is', np.max(yaw) - ref_yaw)

    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)

    ax.plot(time_command, yaw_command, time_pose, yaw, linewidth = 4)
    ax.set_xlim(-1,4)
    ax.set_ylim(0,1.2)
    ax.set_xlabel('time (second)', fontsize=30)
    ax.set_ylabel('yaw (rad)',fontsize=30)
    ax.grid(True)
    ax.legend(('reference','response'), loc='lower right',fontsize=30)
    ax.tick_params(axis="x", labelsize=30)
    ax.tick_params(axis="y", labelsize=30)
    fig.suptitle('Step Response', fontsize=30)
    plt.show()
