close all
clear
%% Robobot line sensor plot

basedir = 'data/build/';
%path = 'log_20231227_110201.074/'; % square with pose reset (1m)
%path = 'log_20231228_094248.370/';
path = 'log_20231228_100553.231/';

%basedir = 'saved/'
%path = 'log_20231227_101929.561/'; % -90 deg turn (OK for explain)
%path = 'log_20231227_110201.074/'; % square with pose reset (1m)

% Pose related logfile
% 1 	Time (sec)
% 2,3 	Velocity left, right (m/s)
% 4 	Robot velocity (m/s)
% 5 	Turnrate (rad/s)
% 6 	Turn radius (m)
% 7,8 	Position x,y (m)
% 9 	heading (rad)
% 10 	Driven distance (m) - signed
% 11 	Turned angle (rad) - signed
ddp = load(strcat(basedir,path,'log_pose.txt'));

% Pose related logfile
% 1 	Time (sec)
% 2,3 	Position x,y (m)
% 4 	heading (rad)
% 5 	Driven distance (m) - signed
dda = load(strcat(basedir,path,'log_pose_abs.txt'));

% Heading control logfile
% 1 	Time (sec)
% 2 	Reference for desired heading (rad)
% 3 	Measured heading (rad)
% 4 	Value after Kp (rad/s)
% 5 	Value after Lead (rad/s)
% 6 	Integrator value (rad/s)
% 7 	After controller (u) (rad/s)
% 8 	Is output limited (1=limited)
% PID parameters
% 	Kp = 10
% 	tau_d = 0, alphs = 1
% 	tau_i = 0 (used=0)
% 	sample time = 7.0 ms
% 	(derived values: le0=1, le1=0, lu1=0, ie=0)
ddh = load(strcat(basedir,path,'log_heading.txt'));

% Gyro logfile
% 1 	Time (sec)
% 2-4 	Gyro (x,y,z)
% Gyro offset 0 0 0
ddg = load(strcat(basedir,path,'log_gyro.txt'));


%%
series = 4;
fig = 1000 * series + 100000;

%% heading
figure(fig + 0)
hold off
plot(ddp(:,1) - ddp(1,1), ddp(:,5),':')
hold on
plot(ddp(:,1) - ddp(1,1), ddp(:,9))
plot(ddg(:,1) - ddp(1,1), -ddg(:,4)*pi/180)
plot(ddh(:,1) - ddp(1,1), ddh(:,7)/10)
legend('w (rad/s)', 'heading (rad)','gyro (rad/s)','u/10');
grid on
xlabel('(sec)')
title('Heading and turn')

%% pose
figure(fig + 7)
hold off
plot(ddp(:,7), ddp(:,8))
hold on
plot(dda(:,2), dda(:,3))
%legend('x (m)', 'y (m)');
grid on
axis equal
xlabel('(m)')
ylabel('(m)')
title('Heading and turn')

