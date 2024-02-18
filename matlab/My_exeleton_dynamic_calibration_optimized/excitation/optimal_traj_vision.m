%% optimal_traj_vis.m

close all;
clear; clc;
%% PARAMETERS
% sampling period
traj_Ts = 0.1;
% trajectory fundamental frequency = 1/T; T = run time of skeleton = 20s.
traj_f = 0.05;
% trajectory fundamental frequency in radian
traj_wf = traj_f * 2 * pi;
% number of sampling points
traj_n = 1 / traj_Ts / traj_f;
% order of trajectory generation 
traj_order = 5;
% number of revolute joints
dof = 6;
% load optimal parameter
load('.\data\mat\opt_x.mat');

%% 轨迹计算 (傅里叶)
opt_q = zeros(traj_n, dof);
opt_qd = zeros(traj_n, dof);
opt_qdd = zeros(traj_n, dof);
for k = 1:traj_n
    time = (k-1) * traj_Ts;
    [opt_q(k,:), opt_qd(k,:), opt_qdd(k,:)] = fourier_series_traj(opt_x, dof, time, traj_wf, traj_order);
    % [opt_q(k,:), opt_qd(k,:), opt_qdd(k,:)] = traj_func(0.3 * opt_x, dof, time, traj_wf, traj_order);
end

%% 输出轨迹（散点数值形式）
opt_traj_1 = opt_q(:,1); opt_traj_2 = opt_q(:,2); opt_traj_3 = opt_q(:,3);
opt_traj_4 = opt_q(:,4); opt_traj_5 = opt_q(:,5); opt_traj_6 = opt_q(:,6);
save('.\data\mat\opt_traj_1.mat', 'opt_traj_1');
mat2txt('.\data\txt\opt_traj_1.txt', opt_traj_1);
save('.\data\mat\opt_traj_2.mat', 'opt_traj_2');
mat2txt('.\data\txt\opt_traj_2.txt', opt_traj_2);
save('.\data\mat\opt_traj_3.mat', 'opt_traj_3');
mat2txt('.\data\txt\opt_traj_3.txt', opt_traj_3);
save('.\data\mat\opt_traj_4.mat', 'opt_traj_4');
mat2txt('.\data\txt\opt_traj_4.txt', opt_traj_4);
save('.\data\mat\opt_traj_5.mat', 'opt_traj_5');
mat2txt('.\data\txt\opt_traj_5.txt', opt_traj_5);
save('.\data\mat\opt_traj_6.mat', 'opt_traj_6');
mat2txt('.\data\txt\opt_traj_6.txt', opt_traj_6);


%% 轨迹计算 (正弦)
% opt_q = zeros(traj_n, dof);
% opt_qd = zeros(traj_n, dof);
% opt_qdd = zeros(traj_n, dof);
% opt_q(1, :) = [0, pi/6, 0, pi/3, 0, pi/2, 0]';
% A = [2.21, -2.21, 1.2, -2.1, -2.3, 2.1, -2.5];
% Ti = [3.68, 2.04, 2.98, 1.75, 4.43, 2.749, 1.06];
% for k = 0:1:traj_n-1
%     opt_qd(k+1, :) = A .* sin(2 * pi * 0.1 * k ./ Ti);
%     if k < traj_n && k > 0
%         opt_q(k+1, :) = opt_q(k, :) + opt_qd(k+1, :) * 0.1;
%     end
%     if k > 0
%         opt_qdd(k+1, :) = (opt_q(k+1, :) - opt_q(k, :)) / 0.1;
%     end
% end

%% 绘制曲线
t = linspace(0, traj_n, traj_n) * traj_Ts;
% joint angle: q
figure(1); 
set(gcf,'position',[0.1,0.1,0.9,0.9] );%调整位置大小
set(gcf,'unit','centimeters','position',[1,2,20,15]);
% plot(t, opt_q(:, 1), 'r', ...
%      t, opt_q(:, 2), 'c', ...
% 	 t, opt_q(:, 3), 'y', ...
% 	 t, opt_q(:, 4), 'g', ...
% 	 t, opt_q(:, 5), 'b', ...
% 	 t, opt_q(:, 6), 'm', ...
%    'LineWidth', 1.0);
yyaxis left;
plot(t, opt_q(:, 1), 'r', ...
   'LineWidth', 1.0);
ylabel('位移(mm)');

yyaxis right;
plot(t, opt_q(:, 2), 'c', ...
	 t, opt_q(:, 3), 'y', ...
	 t, opt_q(:, 4), 'g', ...
	 t, opt_q(:, 5), 'b', ...
	 t, opt_q(:, 6), 'm', ...
   'LineWidth', 1.0);
title('激励轨迹关节位置曲线'); xlabel('时间(s)'); ylabel('角度(rad)');
% legend('关节1', '关节2', '关节3', '关节4', '关节5', '关节6');
legend('关节1', '关节2', '关节3', '关节4', '关节5', '关节6');
print -f1 -dpng -r600 figs\excitedJointRad_n.png
% joint velocity: qd
figure(2);
set(gcf,'position',[0.1,0.1,0.9,0.9] );
set(gcf,'unit','centimeters','position',[1,2,20,15]);
yyaxis left;
plot(t, opt_qd(:, 1), 'r', ...
   'LineWidth', 1.0);
ylabel('速度(mm/s)');

yyaxis right;
plot(t, opt_qd(:, 2), 'c', ...
	 t, opt_qd(:, 3), 'y', ...
	 t, opt_qd(:, 4), 'g', ...
	 t, opt_qd(:, 5), 'b', ...
	 t, opt_qd(:, 6), 'm', ...
   'LineWidth', 1.0);
title('激励轨迹关节角速度(速度)曲线'); xlabel('时间(s)'); ylabel('角速度(rad/s)');

legend('关节1', '关节2', '关节3', '关节4', '关节5', '关节6');
print -f2 -dpng -r600 figs\excitedJointVel_n.png
% joint acceleration: qdd
figure(3); 
set(gcf,'position',[0.1,0.1,0.9,0.9] );
set(gcf,'unit','centimeters','position',[1,2,20,15]);
yyaxis left;
plot(t, opt_qdd(:, 1), 'r', ...
   'LineWidth', 1.0);
ylabel('加速度(mm/s^2)');

yyaxis right;
plot(t, opt_qdd(:, 2), 'c', ...
	 t, opt_qdd(:, 3), 'y', ...
	 t, opt_qdd(:, 4), 'g', ...
	 t, opt_qdd(:, 5), 'b', ...
	 t, opt_qdd(:, 6), 'm', ...
   'LineWidth', 1.0);
title('激励轨迹关节角加速度(加速度)曲线'); xlabel('时间(s)'); ylabel('角加速度(rad/s^2)');

legend('关节1', '关节2', '关节3', '关节4', '关节5', '关节6');
print -f3 -dpng -r600 figs\excitedJointAcc_n.png

%% LOAD ROBOT MODEL
%工具坐标系偏移
transl = [1 0 0 0
          0 1 0 122.68
          0 0 1 85.5
          0 0 0 1];
% LB = Link('d', 0,  'a', 0,   'alpha', 0, 'modified');   % fixed
% L1 = Link('a', 0, 'alpha', 0, 'theta', 0, 'prismatic', 'modified');L1.qlim = [0, 0.15];
% L2 = Link('d', -0.18876, 'a', 0, 'alpha', -pi/2, 'modified');L2.qlim = [-30, 90]* pi / 180;
% L3 = Link('d', -0.08124,  'a', 0, 'alpha', pi/2, 'modified');L3.qlim = [-120, 30]* pi / 180;
% L4 = Link('d', 0,       'a', 0,   'alpha', 4/9*pi, 'offset', -pi/2, 'modified');L4.qlim = [-30, 120]* pi / 180;
% L5 = Link('d', 0.11303,  'a', 0.2675, 'alpha', 0, 'offset', pi/2, 'modified');L5.qlim = [0, 120]* pi / 180;
% L6 = Link('d', 0.11150,  'a', 0,   'alpha', pi/2, 'modified');L6.qlim = [-90, 90]* pi / 180;
LB = Link('d', 0,  'a', 0,   'alpha', 0, 'modified');   % fixed
L1 = Link('a', 0, 'alpha', 0, 'theta', 0, 'prismatic', 'modified');L1.qlim = [0, 150];
L2 = Link('d', -188.76, 'a', 0, 'alpha', -pi/2, 'modified');L2.qlim = [-30, 90]* pi / 180;
L3 = Link('d', -81.24,  'a', 0, 'alpha', pi/2, 'modified');L3.qlim = [-120, 30]* pi / 180;
L4 = Link('d', 0,       'a', 0,   'alpha', 4/9*pi, 'modified');L4.qlim = [-210, -30]* pi / 180;
L5 = Link('d', -12.15,  'a', 252.5, 'alpha', 0, 'modified');L5.qlim = [60, 210]* pi / 180;
L6 = Link('d', 112,  'a', 0,   'alpha', pi/2, 'modified');L6.qlim = [-90, 90]* pi / 180;
% L7 = Link('d', 388.96,  'a', 0,   'alpha', 0);
% L1.qlim([-170, 170] * pi / 180);
% L2.qlim([-120, 120] * pi / 180);
% L3.qlim([-170, 170] * pi / 180);
% L4.qlim([-120, 120] * pi / 180);
% L5.qlim([-170, 170] * pi / 180);
% L6.qlim([-120, 120] * pi / 180);
ske_modDH = SerialLink([LB, L1, L2, L3, L4, L5, L6]);
ske_modDH.tool = transl;
ske_modDH.name = 'skeleton-MDH';
% ske_modDH.base = [1, 0, 0, 0; 0, 1, 0, 0; 0, 0, 1, 0; 0, 0, 0, 1];
% ske_modDH.display();
% figure(4); hold on;
% ske_modDH.plot([0, 0, pi/6, 0, pi/3, 0, pi/2, 0]); hold on;

%% 激励轨迹的三维姿态
for i = 1:traj_n
	fkine = ske_modDH.fkine([0, opt_q(i, 1), opt_q(i, 2), opt_q(i, 3), ...
							 opt_q(i, 4), opt_q(i, 5), opt_q(i, 6)]);
    T = double(fkine);
    trplot(T, 'rgb', 'length', 0.1, 'notext'); hold on;
    title('最优激励轨迹示意图'); 
    xlabel('x/mm'); ylabel('y/mm'); zlabel('z/mm');
    grid on;
end
axis equal;
axis([-1000 1000 -1000 1000 -1600 1600]);  % 根据需要自行调整

%% 激励轨迹的动画GIF
last_fpos = zeros(3, 1);
gap_pnts = zeros(3, 100);
for i = 1:traj_n
    q = [0, opt_q(i, 1), opt_q(i, 2), opt_q(i, 3), opt_q(i, 4), opt_q(i, 5), opt_q(i, 6)];
    ske_modDH.plot(q); hold on;     %动画
    fpos = ske_modDH.fkine(q);

    if i > 1
        gap_pnts(1, :) = linspace(last_fpos(1), fpos.t(1), 100);
        gap_pnts(2, :) = linspace(last_fpos(2), fpos.t(2), 100);
        gap_pnts(3, :) = linspace(last_fpos(3), fpos.t(3), 100);
        plot3(gap_pnts(1, :), gap_pnts(2, :), gap_pnts(3, :), 'r', 'LineWidth', 1.0);
    else
        plot3(fpos.t(1), fpos.t(2), fpos.t(3), 'r', 'LineWidth', 1.0);
    end

    zlim([-1400, 700]);
    xlim([-900, 900]);
    ylim([-900, 900]);
    title('激励轨迹仿真示意图');
    last_fpos = [fpos.t(1), fpos.t(2), fpos.t(3)];
    
    % Capture the plot as an image 
    F = getframe(gcf);
    I = frame2im(F);
    [I, map] = rgb2ind(I, 256);

    % Write to the GIF File 
    if i == 1
        imwrite(I, map, '.\figs\excit_traj_simulation_1.gif', 'gif', 'Loopcount', inf, 'DelayTime', 0.1);
    else
        imwrite(I, map, '.\figs\excit_traj_simulation_1.gif', 'gif', 'WriteMode', 'append', 'DelayTime', 0.1);
    end
end

%% 可达空间
lim_max = [170 120 170 120 170 120];
lim_min = [-170 -120 -170 -120 -170 -120];

sample_pnts = 3e6; 
q_rand = zeros(sample_pnts, 6);
for i = 1:6
    q_rand(:, i) = (lim_min(i) + (lim_max(i)) * rand(sample_pnts, 1)) * pi / 180;
end

fpos = ske_modDH.fkine(q_rand);
x = zeros(sample_pnts, 1);
y = zeros(sample_pnts, 1);
z = zeros(sample_pnts, 1);
for n = 1:1:sample_pnts
    x(n) = fpos(n).t(1);
    y(n) = fpos(n).t(2);
    z(n) = fpos(n).t(3);
end
plot3(x, y, z, 'r.', 'MarkerSize', 1.0);

