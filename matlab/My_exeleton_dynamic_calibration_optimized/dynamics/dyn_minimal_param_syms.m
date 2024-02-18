%% dyn_minimal_param_syms.m
% @brief: QR decomposition to figure out minimal parameter set (eval and subs from syms)
% @param[out] W_min: regression matrix for minimal parameter set 
% @param[out] param_min: minimal parameter set
% @param[out] pnum_min: volume of minimal parameter set
% @param[out] R1: matrix consisting of independent columns
% @param[out] R2: matrix consisting of dependent columns
% @note: Unit:mm(Nmm) is used throughout the project.
function [W_min, min_param_ind, pnum_min, R1, R2] = dyn_minimal_param_syms()

addpath('.\utils');
%% PARAMETER
% load parameters
% fe1 = 0; fe2 = 0; fe3 = 0; ne1 = 0; ne2 = 0; ne3 = 0;%load
% DH parameters
% th = [0, thetaANDd(2), thetaANDd(3), -pi/2+thetaANDd(4), pi/2+thetaANDd(5), thetaANDd(6)]';
% d = [thetaANDd(1), -188.76, -81.24, 0, 113.03, 111.50]';
% a = [0, 0, 0, 0, 267.5, 0]';
% alp = [0, -pi/2, pi/2, 4/9*pi, 0, pi/2]';
d2 = -188.76; d3 = -81.24; d5 = -12.15; d6 = 112;
a5 = 252.5; alp4 = 4/9*pi; g = 9802;%单位：mm
% number of dynamic parameters = 13
% m, mc1, mc2, mc3, Ioxx, Ioyy, Iozz, Ioxy, Ioxz, Ioyz, Ia, fv, fc
pnum_sum = evalin('base', 'pnum_sum');
% regression matrix for standard parameter set (syms 6 x 78)
W = evalin('base', 'W');

%% INSTANTIATION
min_param_ind = zeros(1, pnum_sum);     % 最小参数的索引
WW = zeros(pnum_sum * 6, pnum_sum);
for i = 1:pnum_sum
    q = unifrnd(-pi, pi, 1, 6);	
    qd = unifrnd(-5*pi, 5*pi, 1, 6);
    qdd = unifrnd(-10*pi, 10*pi, 1, 6);

    s1 = sin(q(1)); c1 = cos(q(1)); 
    s2 = sin(q(2)); c2 = cos(q(2));
    s3 = sin(q(3)); c3 = cos(q(3));
    s4 = sin(q(4)); c4 = cos(q(4));
    s5 = sin(q(5)); c5 = cos(q(5));
    s6 = sin(q(6)); c6 = cos(q(6));

    qd1 = qd(1); qdd1 = qdd(1);  
	qd2 = qd(2); qdd2 = qdd(2);
	qd3 = qd(3); qdd3 = qdd(3);
	qd4 = qd(4); qdd4 = qdd(4);
	qd5 = qd(5); qdd5 = qdd(5); 
	qd6 = qd(6); qdd6 = qdd(6);
    
    W_ = eval(subs(W, ...
	         {'sin(q1)', 'cos(q1)', 'sin(q2)', 'cos(q2)', 'sin(q3)', 'cos(q3)', 'sin(q4)', 'cos(q4)', 'sin(q5)', 'cos(q5)', 'sin(q6)', 'cos(q6)', ...
              'qd1', 'qd2', 'qd3', 'qd4', 'qd5', 'qd6', ...
              'qdd1', 'qdd2', 'qdd3', 'qdd4', 'qdd5', 'qdd6' ...
              'g', 'd2', 'd3', 'd5', 'd6', 'a5', 'alp4'}, ...
             {s1, c1, s2, c2, s3, c3, s4, c4, s5, c5, s6, c6, ...
              qd1, qd2, qd3, qd4, qd5, qd6, ...
              qdd1, qdd2, qdd3, qdd4, qdd5, qdd6, ...
              g, d2, d3, d5, d6, a5, alp4}));
			  
    row1 = 1+6*(i-1); row2 = 6+6*(i-1);
	WW(row1:row2, :) = W_;

    disp(['<INFO> Param No.', num2str(i), ' SUBSTITUTED!!']);
end

%% LOOK FOR NAN AND INF
if (sum(sum(isnan(WW))) > 0)
    ind_nan = find(isnan(WW));    % linear index of nan
    sub_nan = zeros(length(ind_nan), 2);    % array index of nan
    joint_nan = zeros(1, length(ind_nan));  % among 6 joints
    iter_nan = zeros(1, length(ind_nan));   % among 200 trajectory setpoints
    for n = 1:length(ind_nan)
        [sub_nan(n, 1), sub_nan(n, 2)] = ind2sub(size(WW), ind_nan(n));
        joint_nan(n) = mod(sub_nan(n, 1), 6);
        iter_nan(n) = fix(sub_nan(n, 1)/6) + 1;
    end
	disp(sub_nan)
    disp(joint_nan)
	pause
else
    disp('No NaN in matrix.');
end

if (sum(sum(isinf(WW))) > 0)
    ind_inf = find(isinf(WW));    % linear index if inf
    sub_inf = zeros(length(ind_inf), 2);    % array index of inf
    joint_inf = zeros(length(ind_inf), 1);  % among 6 joints
    iter_inf = zeros(length(ind_inf), 1);   % among 200 trajectory setpoints
    for n = 1:length(ind_inf)
        [sub_inf(n, 1), sub_inf(n, 2)] = ind2sub(size(WW), ind_inf(n));
        joint_inf(n) = mod(sub_inf(n, 1), 6);
        iter_inf(n) = fix(sub_inf(n, 1)/6) + 1;
    end
    disp(sub_inf)
	disp(joint_inf)
	pause
else
    disp('No Inf in matrix.');
end

%% QR DECOMPOSITION
% WW=Q*R, WW:(6*pnum_sum, pnum_sum), Q:(6*pnum_sum, 6*pnum_sum), R:(6*pnum_sum, pnum_sum)
[~, R] = qr(WW);
pnum_min = 0;	% number of independent parameter
for i = 1:pnum_sum
   if (abs(R(i, i)) < 10^(-5))
       min_param_ind(i) = 0;
   else
       min_param_ind(i) = 1;
       pnum_min = pnum_min + 1;
   end
end
disp('<INFO> QR DECOMPOSITION complete!!');

W_min = sym(zeros(6, pnum_min));	% regression matrix (minimal set)
R1 = zeros(pnum_min, pnum_min);     
R2 = zeros(pnum_min, pnum_sum - pnum_min);  
cind = 1; cdep = 1;	% count the number of independent and dependent columns
for i = 1:pnum_sum
   if (min_param_ind(i) == 1)
      W_min(:, cind) = W(:, i);		% compose independent columns in W to form matrix WB
      R1(1:pnum_min, cind) = R(1:pnum_min, i);	% compose independent columns in R to form matrix RB
      cind = cind + 1;
   else
      R2(1:pnum_min, cdep) = R(1:pnum_min, i);
      cdep = cdep + 1;
   end
end
disp('<INFO> WB (W_min) matrix OBTAINED!!');

%% SAVE DATA
fid = fopen('.\data\txt\dyn_minimal_param_syms.txt', 'w');
for i = 1:6
    for j = 1:pnum_min
        fprintf(fid, 'w_min%d%d=%s;\r', i, j, char(W_min(i, j)));
    end
end
fclose(fid);

%% RETURN VARIABLE TO BASE WORKSPACE
assignin('base', 'W_min', W_min);
assignin('base', 'min_param_ind', min_param_ind);
assignin('base', 'pnum_min', pnum_min);
assignin('base', 'R1', R1);
assignin('base', 'R2', R2);

rmpath('.\utils\');




