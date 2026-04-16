clear all
clc
close all
%% Question 1[10 marks]
%% 1. (a) Create a robot model from the following DH Parameter Table.
% | i | θᵢ | dᵢ | aᵢ | αᵢ |
% |---|----|-----|----|-----|
% | 1 | θ₁ | 0.6 | 0.8| π/2 |
% | 2 | θ₂ | 0.3 | 0.3| π/2 |
% | 3 | θ₃ | 0 | 0.2| 0 |
% Complete the following code:
links(1) = Revolute('d', 0.6, 'a', 0.8, 'alpha', pi/2);
links(2) = Revolute('d', 0.3, 'a', 0.3, 'alpha', pi/2);
links(3) = Revolute('d', 0, 'a', 0.2, 'alpha', 0);
robot = SerialLink(links, 'name', 'AssignmentRobot');
% -------- DO NOT MODIFY THIS PART ------------ %
robot.teach;
disp("press enter to proceed to Question 2")
pause
clc
% ------------------------------------------------ %
%% The following code creates a plot of pick and place problem.
% -------- DO NOT MODIFY THIS PART ------------ %
close all
% The origin of the world frame is at the corner of the table.
pose_world = transl(0, 0, 0.0);
% The pose of the manipulator base frame relative to the world frame.
world_pose_base = transl(1.0, 1.0, 0.0);
% The robot's starting joint angles are:
q_start = [pi/4, pi/4, pi/4];
w = 3.0; h = 2.0; d = -0.6;
table_coords = [0, 0, 0;
w, 0, 0;
w, h, 0;
0, h, 0;
0, 0, d;
w, 0, d;
w, h, d;
0, h, d];
table_idx = [4, 8, 5, 1, 4; 
1, 5, 6, 2, 1; 
2, 6, 7, 3, 2; 
3, 7, 8, 4, 3; 
5, 8, 7, 6, 5; 
1, 4, 3, 2, 1]';
xc = table_coords(:,1);
yc = table_coords(:,2);
zc = table_coords(:,3);
figure(1);
hold on;
trplot(pose_world, 'frame', 'W', 'thick', 2, 'color', 'black');
trplot(world_pose_base, 'frame', 'B', 'thick', 1, 'color', 'red');
patch(xc(table_idx), yc(table_idx), zc(table_idx), 'g', 'facealpha', 1.0); 
robot.base = world_pose_base;
robot.plot(q_start, 'workspace', [-1, 4, -1, 3, -0.5, 4], 'view', [20, 30], 'nowrist');
title('Pick and Place Task');
% ------------------------------------------------ %
%% Question 2 [10 marks]
% The ball is measured to be at x=0.6m, y=0.6m, z=0.7m relative to the
% world frame. Orientation is not considered here. The ball is of size
% 0.1m.
% The following code adds the ball to the pick and place plot.
%% 2. (a) Compute the pose of the ball relative to the world frame [ 5 marks]
world_pose_ball = transl(0.6, 0.6, 0.7); 
x_goal = world_pose_ball(1:3, 4); 
% -------- DO NOT MODIFY THIS PART --------------- %
plot_sphere(x_goal, 0.1,'y');
% ------------------------------------------------ %
%% 2. (b) Compute the pose of the gripper frame for the current joint angles. [5 marks]
% NOTE: the fkine() function will return the pose of the gripper relative to
% the world frame.

% Complete the following code:
world_pose_gripper = robot.fkine(q_start);
% -------- DO NOT MODIFY THIS PART --------------- %
world_pose_gripper = world_pose_gripper.T;
% ------------------------------------------------ %
%% Question 3 [35 marks]
%% 3. (a) Compute the joint angles of the manipulator when the gripper pose is equal to the pose of the ball. [5 marks]
% Complete the following code:
addpath(genpath('C:\Users\yoyo9\AppData\Roaming\MathWorks\MATLAB Add-Ons\Toolboxes\Robotics Toolbox for MATLAB'));
savepath;
world_pose_ball_obj = SE3(world_pose_ball);
q_goal = robot.ikine(world_pose_ball_obj, q_start, 'mask', [1 1 1 0 0 0]); 
%% 3. (b) Design the joint trajectory in joint space [5 marks]
NT= 100;
q_traj = jtraj(q_start, q_goal, NT);
q_traj = q_traj';
%% 3. (c) Design the controller in joint space [10 marks]
clear joint_space_angles joint_space_ang_velocities
joint_space_angles(:,1)= q_start;
joint_space_ang_velocities(:,1)= zeros(3,1);
time_step = 0.01;
% Complete the following code:
Kp = 10*diag([1 1 1]);
"add any additional code here"
i=1;
for j = 1:NT-1
desired_q = q_traj(:,j);
current_q = joint_space_angles(:,i);
joint_error = desired_q - current_q; 
joint_space_ang_velocities(:,i) = Kp*joint_error; 
joint_space_angles(:,i+1) = current_q + time_step*joint_space_ang_velocities(:,i); 
i=i+1;
end
% -------- DO NOT MODIFY THIS PART --------------- %
joint_space_angles = joint_space_angles';
joint_space_ang_velocities=joint_space_ang_velocities';
robot.plot(joint_space_angles);
% ------------------------------------------------ %
%% 3. (d) Determine the Jacobian matrix relating the starting configuration angle velocity to that of the gripper [5 marks]
jacobian = robot.jacob0(q_start)
%% 3. (e) Create a 3D plot of the position gripper during the Joint-Space trajectory. [5 Marks]
% Complete the following code
joint_space_xs = zeros(NT, 1);
joint_space_ys = zeros(NT, 1);
joint_space_zs = zeros(NT, 1);
joint_space_target_xs = zeros(NT, 1);
joint_space_target_ys = zeros(NT, 1);
joint_space_target_zs = zeros(NT, 1);
for k=1:NT
% Actual position during joint-space control
world_pose_actual = robot.fkine(joint_space_angles(k,:)); 
joint_space_xs(k) = world_pose_actual.t(1);
joint_space_ys(k) = world_pose_actual.t(2);
joint_space_zs(k) = world_pose_actual.t(3);
% Desired position from jtraj
world_pose_desired = robot.fkine(q_traj(:,k)');
joint_space_target_xs(k) = world_pose_desired.t(1);
joint_space_target_ys(k) = world_pose_desired.t(2);
joint_space_target_zs(k) = world_pose_desired.t(3);
end
%
% -------- DO NOT MODIFY THIS PART --------------- %
figure(2);
clf;
plot3(joint_space_xs, joint_space_ys, joint_space_zs, 'bo-');
hold on
plot3(joint_space_target_xs, joint_space_target_ys, joint_space_target_zs, 'ro-', MarkerSize=20);
title('Joint-Space Trajectory Gripper Position');
xlabel('x');
ylabel('y');
zlabel('z');
grid on;
% ------------------------------------------------ %
%% 3. (f) Calculate and plot the and velocity of the gripper during the Joint-Space trajectory. [5 marks]
gripper_linear_velocities = [zeros(NT-1, 3)];

for i=1:NT-1
current_q = joint_space_angles(i,:);
current_q_dot = joint_space_ang_velocities(i,:)';
J = robot.jacob0(current_q); 
linear_velocity = J(1:3, :) * current_q_dot;
gripper_linear_velocities(i, :) = linear_velocity';
end
% -------- DO NOT MODIFY THIS PART --------------- %
figure(3);
plot(gripper_linear_velocities);
title('Gripper Linear Velocity during Joint-Space Trajectory');
xlabel('step');
ylabel('velocity (m/s)');
legend('x velocity', 'y velocity', 'z velocity');
grid on;
disp("press enter to proceed to Question 4")
pause
clc
% ------------------------------------------------ %
%% Question 4 [45 marks]
%% 4. (a) Design trajectory in cartesian space [5 marks]
NT= 100;
x_traj = ctraj(robot.fkine(q_start).T, world_pose_ball, NT);
%% 4 (b) Design the controller so that it follows the position prescribed by the Cartesian Space pose [20 marks]
% Complete the following code:
clear joint_space_angles joint_space_ang_velocities cart_space_poses cart_space_velocity
joint_space_angles(:,1)= q_start;
joint_space_ang_velocities(:,1)= zeros(3,1);
cart_space_poses(:,1)= robot.fkine(q_start).t;
cart_space_velocity(:,1)= zeros(3,1);
time_step = 0.01;
Kp = 10*diag([1 1 1]);
"add any additional code here"
% Complete the following code:
i=1;
for j = 1:NT-1
xd_matrix = x_traj(:,:,j);
xd_position = xd_matrix(1:3, 4); 
current_q = joint_space_angles(:,i)';
current_position = robot.fkine(current_q).t; 
error_x = xd_position - current_position;
desired_x_dot = Kp * error_x;
J = robot.jacob0(current_q);
J_linear = J(1:3, :); 
desired_q_dot = pinv(J_linear) * desired_x_dot;
cart_space_velocity(:,i) = desired_x_dot;
joint_space_ang_velocities(:,i)= desired_q_dot;
joint_space_angles(:,i+1) = joint_space_angles(:,i) + time_step * desired_q_dot;
cart_space_poses(:,i+1) = robot.fkine(joint_space_angles(:,i+1)').t;
cart_space_target(:,i)= xd_position;
i=i+1;
end
cart_space_target(:,NT) = x_traj(1:3, 4, NT);
cart_space_velocity(:,NT) = cart_space_velocity(:,NT-1);
joint_space_ang_velocities(:,NT) = joint_space_ang_velocities(:,NT-1);
% -------- DO NOT MODIFY THIS PART --------------- %
cart_space_poses = cart_space_poses';
cart_space_velocity = cart_space_velocity';
cart_space_target = cart_space_target';
joint_space_angles = joint_space_angles';
joint_space_ang_velocities = joint_space_ang_velocities';
figure(1)
robot.plot(joint_space_angles);
% ------------------------------------------------ %
%% 4. (c) Create a 2D plot of the position of the gripper along with the desired trajectory. [5 marks]
% Complete the following code:
"add any additional code here"

cartesian_space_xs = cart_space_poses(:,1); 
cartesian_space_ys = cart_space_poses(:,2);
cartesian_space_zs = cart_space_poses(:,3);
cart_space_target_xs = cart_space_target(:,1);
cart_space_target_ys = cart_space_target(:,2);
cart_space_target_zs = cart_space_target(:,3);
% -------- DO NOT MODIFY THIS PART --------------- %
figure(4);
hold on
plot3(cartesian_space_xs, cartesian_space_ys, cartesian_space_zs, 'bo-');
plot3(cart_space_target_xs, cart_space_target_ys, cart_space_target_zs, 'ro-');
title('Cartesian-Space Trajectory Gripper Position vs. Desired Trajectory');
xlabel('x');
ylabel('y');
zlabel('z');
grid on;
% ------------------------------------------------ %
%% 4. (d) See Assignment Sheet [15 marks]
