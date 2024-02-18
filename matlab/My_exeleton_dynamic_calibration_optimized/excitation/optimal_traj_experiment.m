%% optimal_traj_experiment.m
%% 发送信号频率50Hz，即每0.02s发送一次

close all;
clear; clc;
%% PARAMETERS
% sampling period
traj_Ts = 0.02;
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
save('.\data\mat\opt_traj(50Hz)_1.mat', 'opt_traj_1');
mat2txt('.\data\txt\opt_traj(50Hz)_1.txt', opt_traj_1);
save('.\data\mat\opt_traj(50Hz)_2.mat', 'opt_traj_2');
mat2txt('.\data\txt\opt_traj(50Hz)_2.txt', opt_traj_2);
save('.\data\mat\opt_traj(50Hz)_3.mat', 'opt_traj_3');
mat2txt('.\data\txt\opt_traj(50Hz)_3.txt', opt_traj_3);
save('.\data\mat\opt_traj(50Hz)_4.mat', 'opt_traj_4');
mat2txt('.\data\txt\opt_traj(50Hz)_4.txt', opt_traj_4);
save('.\data\mat\opt_traj(50Hz)_5.mat', 'opt_traj_5');
mat2txt('.\data\txt\opt_traj(50Hz)_5.txt', opt_traj_5);
save('.\data\mat\opt_traj(50Hz)_6.mat', 'opt_traj_6');
mat2txt('.\data\txt\opt_traj(50Hz)_6.txt', opt_traj_6);

