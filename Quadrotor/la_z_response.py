import rosbag
import numpy as np
import matplotlib.pyplot as plt

ref_x = 0.0;
ref_y = 0.0;
ref_z = 2.0;

bag = rosbag.Bag('la_s_z_2019-10-29-14-58-01.bag')
topics = set([
            "/command/pose",
            "/ground_truth_to_tf/euler",
            "/ground_truth_to_tf/pose"
        ])

time_command = []
time_pose = []
time_euler = []

x_command = []
y_command = []
z_command = []

roll = []
pitch = []
yaw = []

x_pose = [];
y_pose = [];
z_pose = [];

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


step_index = np.where(z_command > (1.0 + ref_z)/2.0)[0][0]
step_time = time_command[step_index]

time_command = time_command - step_time
time_pose = time_pose - step_time
time_euler = time_euler - step_time

start_index = np.where(time_pose  > 0)[0][0]
end_index   = np.where(time_pose  > 7)[0][0]
SAE = np.sum(np.abs(x_pose[start_index:end_index] - ref_x))  +  \
            np.sum(np.abs(y_pose[start_index:end_index] - ref_y))  + \
            np.sum(np.abs(z_pose[start_index:end_index] - ref_z))

print('the sum of abosulte erros is', SAE)

fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)

ax.plot(time_command, z_command, time_pose, z_pose, linewidth = 4)
ax.set_xlim(-1,7)
ax.set_ylim(0.9,2.1)
ax.set_xlabel('time (second)', fontsize=30)
ax.set_ylabel('z (m)',fontsize=30)
ax.grid(True)
ax.legend(('reference','response'), loc='lower right',fontsize=30)
ax.tick_params(axis="x", labelsize=30)
ax.tick_params(axis="y", labelsize=30)
fig.suptitle('Step Response', fontsize=30)
plt.show()
