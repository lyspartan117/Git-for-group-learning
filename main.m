clear
clc

% sinx的追踪
kp = 2;
ki = 0.025;
kd = 20;
k = [kp,kd,ki]; % 系数
x = 0;
y = -1;
phi = 0.5;
l = 2;
v = 2;
init = [x,y,phi,l,v];% 初始值
dt = 0.1; % 最大时间间隔内距离v * dt
% 一条直线
tt_x = linspace(0,100,1000);
tt_y = 2 * sin(tt_x/3.0);
target = {dt,tt_x,tt_y};
plot(tt_x,tt_y,'.',tt_x,tt_y,'b-');
hold on;
[r_x,r_y,r_phi,r_delta_f,i] = PID_CET(k,init,target,0);
plot(r_x(1:i),r_y(1:i),'.',r_x(1:i),r_y(1:i),'r-');

% 直线的追踪