%% optimize_traj_main
% @brief: 求解约束非线性优化问题激发轨迹生成
% @dependency: optimize_traj_vis.m (轨迹可视化)
%              optimize_traj_object_func.m (优化目标函数)
%              optimize_traj_constraints_mat.m (优化约束条件)

%% PARAMETERS
% 采样周期traj_Ts
traj_Ts = 0.1;
% 基频traj_f = 1/（运行周期T），运行周期为20s
traj_f = 0.05;
% 基频（rad/s）traj_wf
traj_wf = traj_f * 2 * pi;
% 采样点数traj_n
traj_n = 1 / traj_Ts / traj_f;
% 傅里叶级数谐波数traj_order
traj_order = 5;
% 关节数量dof
dof = 6;
% 最小参数集pnum_min
pnum_min = 48;

%% CONSTRAINTS
[A, b, Aeq, beq] = optimize_traj_constraints(traj_wf, traj_Ts, traj_n);

%% SOLVE OPTIMIZATION PROBLEM
% load("opt_x0.mat");
opt_x0 = zeros(66, 1) + 0.1;		% initial value
opt_x0(11,:) = 99;
opt_x0(44,:) = -pi/2;
opt_x0(55,:) = pi/2;
object = @optimize_traj_object_fun;
nonlcon = @optimize_traj_nonl_constraints;% optional 防止执行激励轨迹时发生自碰撞
options = optimoptions(@fmincon, 'MaxIterations', 66666, 'MaxFunctionEvaluations', 66666);
[opt_x, opt_fval] = fmincon(object, opt_x0, A, b, Aeq, beq, [], [], nonlcon, options);

%% VISUALIZATION
% optimal_traj_vision();

