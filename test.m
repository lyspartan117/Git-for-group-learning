clear all;close all;clc;

kp = 2;
ki = 0.025;
kd = 20;

v = 2;
dt = 0.1;
L= 2.5;
curp=[0 -1 0.5];

x = linspace(0,100,1000);
y = 2 * sin(x/3.0);
path = [x' y'];
plot(path(:,1),path(:,2),'r.');
hold on;

intergral = 0;
pre_err =0;
for i=1:length(path)
    
    d = path(:,1:2) - curp(1:2);
    dis = d(:,1).^2 + d(:,2).^2;
    [~,ind] = min(dis);                                     %找路径最近点索引
    
    dx = curp(1) - path(ind,1);
    dy = curp(2) - path(ind,2);
    
    e = (sin(curp(3) - atan2(dy,dx)))*sqrt(dx*dx+dy*dy);   %横向偏差作为测量
    temp(i) = e;
     
    intergral = intergral + e;
    u = kp*e + ki*intergral + kd*(e - pre_err);             %pid生成控制量，前轮转角
    pre_err = e;

    curp(1) = curp(1) + dt*v*cos(curp(3));
    curp(2) = curp(2) + dt*v*sin(curp(3));  
    curp(3) = curp(3) + dt*v*tan(u)/L;
  
    plot(curp(1),curp(2),'g.');
end

%axis equal;