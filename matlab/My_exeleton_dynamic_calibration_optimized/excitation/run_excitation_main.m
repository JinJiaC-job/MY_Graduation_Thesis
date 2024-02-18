%%run_excitation_main.此为运行主程序

clear, clc, close all;

optimize_traj_main; 

% save to mat
clear ans;
save('.\data\mat\opt_x.mat', 'opt_x');

% save to txt
mat2txt('.\data\txt\opt_x.txt', opt_x);



