function [x,y,phi,delta_f,i] = PID_CET(k,init,target,noise)
%PID_CET 根据车辆横向追踪误差的PID，基于后轮为中心
%   控制量为a和delta_f,其中a为固定值0
%   k:kp,ki,kd比例，积分，微分系数所构成的向量,[kp,ki,kd]
%   init:初始位置信息，分别为初始时x坐标，y坐标，航向角phi，车辆长度l的向量，v初始速度信息,[x,y,phi,l,v];
%   target:目标信息，[dt,tt_x,tt_y],包含了采样时间dt，目标曲线的信息对应的(tt_x,tt_y)，目标曲线应为插值后的结果
%   返回车辆运行状态的位置信息,x,y,phi,delta_f,i,i对应跟踪结束的x,y中的坐标
%   干扰信息，暂时空缺

% 读入输入
n = size(target{2},2); % x,y最多会有n的数据点
kp = k(1); % k输入
ki = k(2);
kd = k(3);
x = linspace(0,0,n); % 初始位置信息
x(1) = init(1);
y = linspace(0,0,n);
y(1) = init(2);
phi = linspace(0,0,n);
phi(1) = init(3);
l = init(4);
v = init(5);
delta_f = linspace(0,0,n); % 前轮转向角，初始值默认为0
delta_f(1) = 0;
a = 0;
dt = target{1}; % 目标信息
tt_x = target{2};
tt_y = target{3};
% noise暂不管

err_now = 0; % 当前误差
err_sum = 0; % 累计误差
point = 0; % 点的下标
for i = 1:1:n
    % 根据当前位置寻找目标曲线最接近的点
    distance = (tt_x - x(i)).^2 + (tt_y - y(i)).^2;
    % 和轨迹最终点距离很小，结束程序
    if distance(n) < 1.0e-3
        return;
    end
    [~,point] = min(distance);
%     [x(i),y(i),point,tt_x(point),tt_y(point)]
%     plot(tt_x,tt_y,'.',tt_x,tt_y,'b-');
%     hold on;
%     plot(x(i),y(i),'ro',tt_x(point),tt_y(point),'bo');
%     hold off;

    % 计算目前位置的误差值
    err_pre = err_now;
    if( point + 1 > n) % 防止越界
        err_now = calcERR_CET([x(i),y(i)],[tt_x(point),tt_y(point)],phi(i),[-tt_x(point - 1),-tt_y(point - 1)]);
    else
        err_now = calcERR_CET([x(i),y(i)],[tt_x(point),tt_y(point)],phi(i),[tt_x(point + 1),tt_y(point + 1)]);
    end
    err_sum = err_sum + err_now;
%     disp(err_now)
    
    % 根据当前误差值更新控制量delta_f
    delta_f(i + 1) = kp * err_now + kd * err_sum +ki * (err_now - err_pre);
    % 限制delta_f的大小，防止出现过大的情况
%     if delta_f(i + 1) > pi / 6
%         delta_f(i + 1) = pi / 6;
%     elseif delta_f(i + 1) < -pi / 6
%         delta_f(i + 1) = - pi / 6;
%     end
    
    % 计算更新delta_f后的位置信息
    x(i + 1) = x(i) + v * cos(phi(i)) * dt; % x
    y(i + 1) = y(i) + v * sin(phi(i)) * dt; % y
    phi(i + 1) = phi(i) + v * tan(delta_f(i + 1)) / l * dt; % 航向角
    v = v + a * dt;
end
end

