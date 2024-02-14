close all
clear
%% load logfiles
% Pose related logfile
% 1 	Time (sec)
% 2,3 	Velocity left, right (m/s)
% 4 	Turnrate (rad/s)
% 5     turnradius
% 6,7 	Position x,y (m)
% 8 	heading (rad)
% 9 	Driven distance (m) - signed
% 10 	Turned angle (rad) - signed
pose = load('log/log_pose.txt');

% Mixer logfile
% Wheel base used in calculation: 0.24
% 1 	Time (sec)
% 2 	manual override mode
% 3 	Linear velocity (m/s)
% 4 	Turnrate reference (rad/sec) positive is CCV
% 5 	Turnrate after heading control (rad/sec) positive is CCV
% 6 	Desired left wheel velocity (m/s)
% 7 	Desired right wheel velocity (m/s)
% 8 	Desired turn radius (999 if straight) (m)
mix = load('log/log_mixer.txt');

% Motor control (left) logfile
% 1 	Time (sec)
% 2 	Reference for left motor (m/sec)
% 3 	Measured velocity for motor (m/sec)
% 4 	Value after Kp (V)
% 5 	Value after Lead (V)
% 6 	Integrator value (V)
% 7 	Motor voltage output (V)
% 8 	Is output limited (1=limited)
% PID parameters
% 	Kp = 5
% 	tau_d = 0.1, alphs = 1
% 	tau_i = 0.03 (used=1)
% 	sample time = 7.0 ms
% 	(derived values: le0=1, le1=0, lu1=0, ie=0.116667)
mot1 = load('log/log_motor_0.txt');
mot2 = load('log/log_motor_1.txt');

% Mission plan20 logfile
% 1 	Time (sec)
% 2 	Mission state
% 3 	% Mission status (mostly for debug)
mis20 = load('log/log_plan20.txt')
%% plot pose
% velocities
figure(102)
hold off
plot(pose(:,1) - pose(1,1), pose(:,2))
grid on
hold on
plot(pose(:,1) - pose(1,1), pose(:,3))
plot(pose(:,1) - pose(1,1), pose(:,4))
w = 20; % window size
f = filter( 1/w*ones(1,w),1, 1./pose(:,5));
plot(pose(:,1) - pose(1,1), f)
% mixer
plot(mix(:,1) - pose(1,1), 1./mix(:,8))
% state
plot(mis20(:,1) - pose(1,1), mis20(:,2)/10)
legend('vel left (m/s)','vel right (m/s)','turnrate rad/s', '1/(turn_radius)','desired turn radius','state' )

%% plot pose position
% position
figure(302)
hold off
plot(pose(:,6), pose(:,7))
axis equal
grid on
%% Plot wheel velocities 
figure(502)
hold off
w = 2;
a = 1;
b = 1/w*ones(1,w);
m1 = filter(b,a,pose(:,2))
m2 = filter(b,a,pose(:,3))
plot(pose(:,1) - pose(1,1), m1)
hold on
% plot(pose(:,1) - pose(1,1), pose(:,3))
plot(pose(:,1) - pose(1,1), m2)
grid on
plot(mix(:,1)-pose(1,1), mix(:,6))
plot(mix(:,1)-pose(1,1), mix(:,7))

legend('wheel 1 vel m/s','wheel 2 vel m/s','wheel 1 ref (m/s)','wheel 2 ref')
%% plot motor voltage
figure(805)
hold off
plot(mot1(:,1) - mot1(1,1), mot1(:,2))
hold on
plot(mot1(:,1) - mot1(1,1), mot1(:,3))
%plot(mot1(:,1) - mot1(1,1), mot1(:,7))
%plot(mot1(:,1) - mot1(1,1), mot1(:,5))
%plot(mot1(:,1) - mot1(1,1), mot1(:,6))
title('motor voltage')
legend('ref','measured','motor voltage', 'after kp', 'integr')
