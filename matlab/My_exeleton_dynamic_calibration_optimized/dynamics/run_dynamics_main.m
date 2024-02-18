clear, clc, close all;

%% 求解外骨骼机械臂动力学
NewtonEuler_dynamic_tau;
% Save to txt
t1 = char(tau(1)); t2 = char(tau(2)); t3 = char(tau(3)); t4 = char(tau(4)); t5 = char(tau(5)); t6 = char(tau(6)); 
fid = fopen('.\data\txt\dyn_inertia_link_jointAxis.txt', 'w');
fprintf(fid, ...
    'T1=%s\rT2=%s\rT3=%s\rT4=%s\rT5=%s\rT6=%s\r', ...
    t1, t2, t3, t4, t5, t6);
fclose(fid);
% Save to mat
clear fid t1 t2 t3 t4 t5 t6 ans;
save('.\data\mat\dyn_inertia_link_jointAxis.mat');

%% 动力学方程线性化：将dyn_param_linearization.txt的数据复制到 w_regressor.m中
dyn_param_linearization; 
% Save to txt
fid = fopen('.\data\txt\dyn_param_linearization.txt','w');
for i = 1:6
    for j = 1:pnum_sum
	    fprintf(fid, 'w%d%d=%s;\r', i, j, char(W(i, j)));
    end
end

fclose(fid);
% Save to mat
clear fid i j k1 k2 Q_ ans;
save('.\data\mat\dyn_param_linearization.mat')
save('.\data\mat\regressor.mat', 'W');

%% 推导最小回归矩阵W (QR 分解)
% dyn_minimal_param_syms;  % (OR dyn_minimal_param_math)
dyn_minimal_param_math;%生成dyn_minimal_param_math.txt文件，将其数据复制到excitation\min_regressor
% Save to mat
clear ans;
save('.\data\mat\dyn_minimal_param_syms.mat');  % (OR save('.\data\mat\dyn_minimal_param_math.mat') )
save('.\data\mat\min_regressor.mat', 'W_min');
save('..\identify\data\min_regressor.mat', 'W_min');



%% 最小参数集向标准参数集转换
clear, clc, close all;
load('.\data\mat\dyn_minimal_param_syms.mat'); % (OR load('.\data\mat\dyn_minimal_param_math.mat'))
load('.\data\mat\least_square.mat');

dyn_Pmin2P;

save('.\data\mat\mapping_Pmin2P.mat');
save('.\data\mat\P_link.mat', 'P_link');
save('.\data\mat\P_center.mat', 'P_center');
save('.\utils\gen_params\P_center.mat', 'P_center');
save('.\utils\gen_params\P_link.mat', 'P_link');
% Save to txt
addpath('.\utils\')
mat2txt('.\data\txt\P_center.txt', P_center);
mat2txt('.\data\txt\P_link.txt', P_link);
rmpath('.\utils\')


%% 验证标准参数集
load('.\data\mat\mapping_Pmin2P.mat');
load('.\data\mat\excit_filtering.mat');

compute_model_error;



