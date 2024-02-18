%% PART I: IMPLEMENT LEAST SQUARE ESTIMATION
% if 1
    clear, clc, close all;
    load('data\min_regressor.mat');
    load('data\excit_filtering.mat');
    % trajectory sampling period（需改）
    traj_Ts = 0.05;
    % number of minimal parameter set
    pnum_min = 48;
    
    %% REMEMBER TO MANUALLY:
    % 1. check inf/nan substitution in `least_square_estimation.m`
    % 2. copy `..\excitation\utils\min_regressor.m` to `.\utils\min_regressor.m`
    % 3. check SystemOfUnits of {DH, g} in 
    % - `miniparam_inverse_dynamics.m`
    % - `least_square_estimation_syms.m` or `least_square_estimation_math.m` 
    % - `.\utils\min_regressor.m`
    
    least_square_estimation_math;  % (OR least_square_estimation_syms)
%     clear ans;
    save('.\data\least_square.mat');
    save('..\dynamics\data\mat\least_square.mat');
% end

%% PART II: VERIFY WITH MINIMAL PARAMETER SET
% if 1
%     clear;
%     load('data\least_square.mat');
    verify_identified_parameter;
% end

