function [c, ceq] = optimize_traj_nonl_constraints(x)
% @brief: 求不等式和等式非线性约束
% @param[in]: 超参数矩阵 (q_{0}, alpha and beta)
%             e.g. for Joint 1 with traj_order being 5,
%                  x([1, 3, 5, 7, 9]) denotes coeff for sine: alpha
%                  x([2, 4, 6, 8, 10]) denotes coeff for cosine: beta
%                  x(11) denotes initial joint angle: q0
% @param[out] c: such that c <= 0
% @param[out] ceq: such that ceq = 0
% @dependency: forward_kine.m 

%% PARAMETERS
% 采样周期
traj_Ts = 0.1;
% 轨迹基频
traj_f = 0.05;
% 轨迹基频（弧度）
traj_wf = traj_f * 2 * pi;
% 采样点数
traj_n = 1 / traj_Ts / traj_f;
% 轨迹阶数
traj_order = 5;
% 关节自由度
dof = 6;

%% 环境约束参数(单位：mm)
evirm_cnstrn_x = 200;
evirm_cnstrn_y = 130;
evirm_cnstrn_z = -150;

%% FORWARD KINEMATICS
ceq = [];
c = zeros(traj_n * 2, 1);%不等式非线性约束的个数
for k = 1:traj_n
	time = k * traj_Ts;
    ind = 2 * (k - 1) + 1;

    % 计算 tcp
	[opt_q, ~, ~] = fourier_series_traj(x, dof, time, traj_wf, traj_order);	% size:(dof, 1)
    [~, tcp_tool, ~] = forward_kinematics(opt_q);

    [~, tcp_5, ~] = forward_kinematics_njoint(opt_q, 5);%第5个关节运动约束

%     con1 = evirm_cnstrn_x - abs(tcp(1));
%     con2 = evirm_cnstrn_y + tcp(2);
%     c(ind) = con1 * con2;%c <= 0

    c(ind) = evirm_cnstrn_y + tcp_tool(2);
    c(ind+1) = evirm_cnstrn_y + tcp_5(2);
    
    % 平面约束：|x|>=250; y<=-150; z>=-100;
%     c(ind) = evirm_cnstrn_x - abs(tcp(1));
%     c(ind+1) = evirm_cnstrn_y + tcp(2);
%    c(ind+2) = evirm_cnstrn_z - tcp(3);

end

end

