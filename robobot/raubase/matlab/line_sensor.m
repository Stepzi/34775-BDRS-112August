close all
clear
%% Robobot line sensor plot

basedir = 'data/';
%path = 'log_20231228_085832.261/';
%path = 'log_20231228_094248.370/';
%path = 'log_20231228_100553.231/';
%path='log_20231228_101122.585/';
path = 'log_20240107_085137.286/';

%basedir = 'saved/';
%path = 'log_20231221_160451.621/'; % passing at angle
%path = 'log_20231221_161001.464/'; % follow with crossing right

% Edge sensor logfile normalized 'log_20231228_085457.802/log_edge_normalized.txt'
% 1 	Time (sec)
% 2..9 	Sensor value in 0..1000 scale for black to white
% 10 	Line width
ddn = load(strcat(basedir,path,'log_edge_normalized.txt'));
% Edge sensor logfile log_edge.txt
% 	Calib white 406 526 596 577 596 533 505 436
% 	Calib black 128 177 188 171 166 150 145 130
% 	White threshold (%of span) 750 %
% 	Crossing level (%of span) 800 %
% 1 	Time (sec)
% 2 	Edge valid
% 3 	Left edge position(m)
% 4 	Right edge position (m)
% 5 	Line width
ddl = load(strcat(basedir,path,'log_edge.txt'));
% Linesensor raw values logfile (reflectance values)
% Sensor power high=1
% 1 	Time (sec)
% 2..9 	Sensor 1..8 AD value difference (illuminated - not illuminated)
ddr = load(strcat(basedir,path,'log_edge_raw.txt'));
% Edge logfile: log_edge_ctrl.txt
% 1 	Time (sec)
% 2 	heading mode (edge control == 2)
% 3 	Edge 1=left, 0=right
% 4 	Edge offset (signed in m; should be less than about 0.01)
% 5 	Measured edge value (m; positive is left)
% 6 	control value (rad/sec; positive is CCV)
% 7 	limited
ddc = load(strcat(basedir,path,'log_edge_ctrl.txt'));
% Edge control logfile: log_20231227_174818.915/log_edge_pid.txt
% PID parameters
% 	Kp = 25
% 	tau_d = 0.3, alpha = 0.1 (use lead=1)
% 	tau_i = 0.3 (used=1)
% 	sample time = 8.0 ms
% 	(derived values: le0=8.94118, le1=-8.70588, lu1=-0.764706, ie=0.0133333)
% 1 	Time (sec)
% 2 	Reference for desired value
% 3 	Measured value
% 4 	Value after Kp
% 5 	Value after Lead
% 6 	Integrator value
% 7 	After controller (u)
% 8 	Is output limited (1=limited)
ddp = load(strcat(basedir,path,'log_edge_pid.txt'));
% Mission plan40 logfile
% 1 	Time (sec)
% 2 	Mission state
% 3 	% Mission status (mostly for debug)


%%
series = 1;
fig = 1000 * series + 100000;

%% edge and crossing
h = figure(fig + 0)
hold off
plot(ddl(:,1) - ddl(1,1), ddl(:,2),'linewidth',2)
hold on
plot(ddl(:,1) - ddl(1,1), ddl(:,3)*100,'linewidth',2)
plot(ddl(:,1) - ddl(1,1), ddl(:,4)*100,'linewidth',2)
plot(ddl(:,1) - ddl(1,1), ddl(:,5),'--','linewidth',2)
%plot(ddp(:,1) - ddl(1,1), ddp(:,4))
%plot(ddp(:,1) - ddl(1,1), ddp(:,5))
%plot(ddp(:,1) - ddl(1,1), ddp(:,6))
%plot(ddp(:,1) - ddl(1,1), ddp(:,7))
legend('valid', 'left (cm)','right (cm)', 'line width','location','north');
grid on
axis([0,4.5,-7,13])
xlabel('(sec)')
ylabel('(cm)')
title('Line sensor values')
saveas(h,"line-sensor-1_values.png")
%% edge control
figure(fig + 1)
hold off
plot(ddl(:,1) - ddl(1,1), ddl(:,2))
hold on
plot(ddl(:,1) - ddl(1,1), ddl(:,3)*100)
% plot(ddl(:,1) - ddl(1,1), ddl(:,4)*100)
% plot(ddl(:,1) - ddl(1,1), ddl(:,5))
% plot(ddl(:,1) - ddl(1,1), ddl(:,6)*100)
plot(ddp(:,1) - ddl(1,1), ddp(:,4))
plot(ddp(:,1) - ddl(1,1), ddp(:,5))
%plot(ddp(:,1) - ddl(1,1), ddp(:,6))
plot(ddp(:,1) - ddl(1,1), ddp(:,7))
legend('valid', 'left (cm)','after kp', 'after lead','u (rad/s)');
grid on
xlabel('(sec)')
title('Crossing to the right')

%% values normalized
figure(fig + 2)
hold off
plot(ddl(:,1) - ddl(1,1), ddl(:,2)/10)
hold on
plot(ddn(:,1) - ddl(1,1), ddn(:,2)/1000)
plot(ddn(:,1) - ddl(1,1), ddn(:,3)/1000)
plot(ddn(:,1) - ddl(1,1), ddn(:,4)/1000)
plot(ddn(:,1) - ddl(1,1), ddn(:,5)/1000)
plot(ddn(:,1) - ddl(1,1), ddn(:,6)/1000)
plot(ddn(:,1) - ddl(1,1), ddn(:,7)/1000)
plot(ddn(:,1) - ddl(1,1), ddn(:,8)/1000)
plot(ddn(:,1) - ddl(1,1), ddn(:,9)/1000)
legend('valid', 'ls1', 'ls2', 'ls3', 'ls4', 'ls5', 'ls6', 'ls7', 'ls8');
grid on
xlabel('(sec)')
title('normalized')
%% values normalized - heatmap / mesh
figure(fig + 3)
%heatmap(ddn(:,1) - ddl(1,1), (0:7)*12./7 - 6, ddn(:,2:9)')
mesh(ddn(:,1) - ddl(1,1), (0:7)*-12./7 + 6, ddn(:,2:9)')
xlabel('(sec)')
ylabel('left -- (cm) -- right')
grid on
title('normalized')
axis([0,4.5,-6,6,0,1000])


%% values raw
figure(fig + 5)
hold off
plot(ddl(:,1) - ddl(1,1), ddl(:,2)/10)
hold on
plot(ddr(:,1) - ddl(1,1), ddr(:,2))
plot(ddr(:,1) - ddl(1,1), ddr(:,3))
plot(ddr(:,1) - ddl(1,1), ddr(:,4))
plot(ddr(:,1) - ddl(1,1), ddr(:,5))
plot(ddr(:,1) - ddl(1,1), ddr(:,6))
plot(ddr(:,1) - ddl(1,1), ddr(:,7))
plot(ddr(:,1) - ddl(1,1), ddr(:,8))
plot(ddr(:,1) - ddl(1,1), ddr(:,9))
legend('valid', 'ls1', 'ls2', 'ls3', 'ls4', 'ls5', 'ls6', 'ls7', 'ls8');
grid on
xlabel('(sec)')
title('raw')
%% mesh
x = -3.5:1:3.5;
y = ddn(:,1) - ddl(1,1);
z = ddn(:,2:9);
figure(fig+7)
mesh(x,y,z)
%% calibrate - interval must be adjusted

white = mean(ddr(200:220,2:9))
black = mean(ddr(1:50,2:9))


