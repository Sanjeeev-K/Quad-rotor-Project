bag = rosbag('bbb.bag');
bSel_command_pose = select(bag,'Topic','/drone1/command/pose');
bSel_euler = select(bag,'Topic','/drone1/ground_truth_to_tf/euler');
bSel_pose = select(bag,'Topic','/drone1/ground_truth_to_tf/pose');

msgStructs_command_pose = readMessages(bSel_command_pose,'DataFormat','struct');
msgStructs_euler = readMessages(bSel_euler,'DataFormat','struct');
msgStructs_pose = readMessages(bSel_pose,'DataFormat','struct');

Sec_command=  cellfun(@(m) double(m.Header.Stamp.Sec),msgStructs_command_pose);
Nsec_command=  cellfun(@(m) double(m.Header.Stamp.Nsec),msgStructs_command_pose);
time_command = Sec_command + Nsec_command/10^9;

Sec_pose=  cellfun(@(m) double(m.Header.Stamp.Sec),msgStructs_pose);
Nsec_pose=  cellfun(@(m) double(m.Header.Stamp.Nsec),msgStructs_pose);
time_pose = Sec_pose + Nsec_pose/10^9;

Sec_euler=  cellfun(@(m) double(m.Header.Stamp.Sec),msgStructs_euler);
Nsec_euler=  cellfun(@(m) double(m.Header.Stamp.Nsec),msgStructs_euler);
time_euler = Sec_euler + Nsec_euler/10^9;

x_command=  cellfun(@(m) double(m.Pose.Position.X),msgStructs_command_pose);
y_command=  cellfun(@(m) double(m.Pose.Position.Y),msgStructs_command_pose);
z_command=  cellfun(@(m) double(m.Pose.Position.Z),msgStructs_command_pose);

x_pose=  cellfun(@(m) double(m.Pose.Position.X),msgStructs_pose);
y_pose=  cellfun(@(m) double(m.Pose.Position.Y),msgStructs_pose);
z_pose=  cellfun(@(m) double(m.Pose.Position.Z),msgStructs_pose);

roll = cellfun(@(m) double(m.Vector.X),msgStructs_euler);
pitch = cellfun(@(m) double(m.Vector.Y),msgStructs_euler);
yaw = cellfun(@(m) double(m.Vector.Z),msgStructs_euler);

step_indexs = x_command > 0;
[~,step_index] = max(step_indexs);
step_time = time_command(step_index);

time_command = time_command - step_time;
time_pose = time_pose - step_time;
time_euler = time_euler - step_time;

plot(time_command,x_command,'LineWidth',4);
hold on;
plot(time_pose, x_pose,'LineWidth',4);
hold on;

title('Step Response','Interpreter','latex')
ylabel({'$x (m)$'},'Interpreter','latex','Rotation',0,'HorizontalAlignment','Right');
xlabel({'$time\, (second)$'},'Interpreter','latex');
ax = gca;
ax.XLim = [-1,7];
ax.YLim = [-0.5, 2];
ax.TickLabelInterpreter = 'latex';
legend('reference','response','Interpreter','latex');
set(gca,'FontSize',30,'linewidth',2);