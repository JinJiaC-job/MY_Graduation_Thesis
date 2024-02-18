function [A, b, Aeq, beq] = optimize_traj_constraints(wf, Ts, N)
% @brief: figure out fourier-based series coeffs for q, qd, qdd
% @param[in] wf: trajectory fundamental frequency in radian
% @param[in] Ts: sampling period
% @param[in] N: number of sampling points
% @param[out] A, b: inequality constraints, where matrix A(3*6*2*N, 66) and vector b(3*6*2*N, 1)
%                   s.t. |q|<q_max, |qd|<qd_max, |qdd|<qdd_max 
% @param[out] Aeq, beq: equality constraints, where matrix A(3*6*2, 66) and vector b(3*6*2, 1)
%                       s.t. q0=qn=q_init, qd0=qdn=0, qdd0=qddn=0

% zeros row vector of specified length
% o11 = zeros(1, 11);
o6 = zeros(6, 1) * 1e-4;
% 机械臂初始角度
q_init = [95.35; 0; 0; -pi/2; pi/2; 0];

A_ = zeros(6*3, 66);
A = zeros(6*2*3*(N-1), 66); b = zeros(6*2*3*(N-1), 1);
Aeq = zeros(6*3*2, 66); beq = zeros(6*3*2, 1);

for k = 0:N
	time = k * Ts;

	% fourier-series coeffs for q (1x11)
	qs = [sin(wf * time * 1)/(wf * 1), -cos(wf * time * 1)/(wf * 1), ...
		  sin(wf * time * 2)/(wf * 2), -cos(wf * time * 2)/(wf * 2), ...
		  sin(wf * time * 3)/(wf * 3), -cos(wf * time * 3)/(wf * 3), ...
		  sin(wf * time * 4)/(wf * 4), -cos(wf * time * 4)/(wf * 4), ...
	      sin(wf * time * 5)/(wf * 5), -cos(wf * time * 5)/(wf * 5), 1];
	
	% fourier-series coeffs for qd (1x11)
	qds = [cos(wf * time * 1), sin(wf * time * 1), ...
           cos(wf * time * 2), sin(wf * time * 2), ...
	       cos(wf * time * 3), sin(wf * time * 3), ...
	       cos(wf * time * 4), sin(wf * time * 4), ...
	       cos(wf * time * 5), sin(wf * time * 5), 0];
    
    % fourier-series coeffs for qdd (1x11)
	qdds = [-wf * 1 * sin(wf * time * 1), wf * 1 * cos(wf * time * 1), ...
            -wf * 2 * sin(wf * time * 2), wf * 2 * cos(wf * time * 2), ...
	        -wf * 3 * sin(wf * time * 3), wf * 3 * cos(wf * time * 3), ...
	        -wf * 4 * sin(wf * time * 4), wf * 4 * cos(wf * time * 4), ...
	        -wf * 5 * sin(wf * time * 5), wf * 5 * cos(wf * time * 5), 0];
    
    for j = 1:6
        s_col = 11 * (j - 1) + 1; e_col = 11 * j;
        A_(j, s_col:e_col) = qs;
        A_(6 + j, s_col:e_col) = qds;
        A_(12 + j, s_col:e_col) = qdds;
    end

    if (k >= 1 && k <= N-1)     % 不等式约束 s.t. Ax<b
% 	    b_ = [150; 90; 30; 60; 120; 90; %joint1为直线电机
% 	          100; 45; 45; 90; 120; 60;
% 		      200; 90; 90; 180; 240; 120;
% 		      0; 30; 120; 120; 30; 90;
% 	          100; 45; 45; 90; 120; 60;
% 		      200; 90; 90; 180; 240; 120;];
	    b_ = [245.35; pi/2; pi/6; -pi/6; 7*pi/6; 4*pi/9;%直线关节的线速度线加速度需要纠正，单位为mm
	          50; pi/6; pi/4; pi/6; pi/2; pi/3;
		      100; pi/3; pi/2; pi/3; pi; 2*pi/3;
		      -95.35; pi/6; 2*pi/3; 7*pi/6; -pi/3; 7*pi/18;
	          50; pi/6; pi/4; pi/6; pi/2; pi/3;
		      100; pi/3; pi/2; pi/3; pi; 2*pi/3];
	    
        % b = [q1, q2, q3, q4, q5, q6,
        %      qd1, qd2, qd3, qd4, qd5, qd6,
        %      qdd1, qdd2, qdd3, qdd4, qdd5, qdd6]
        
        s_ind = 36 * (k-1) + 1; e_ind = 36 * k;
        b(s_ind:e_ind, :) = b_;
        A(s_ind:e_ind, :) = [A_; -A_];
    elseif (k == 0)     % EQUALITY CONSTRAINTS s.t. A(q0, qd0, qdd0)x=b_init
		beq_ = [q_init; o6; o6];

        beq(1:18, :) = beq_;
        Aeq(1:18, :) = A_;
    elseif (k == N)     % EQUALITY CONSTRAINTS s.t. A(qn, qdn, qddn)x=b_init
        beq_ = [q_init; o6; o6];

        Aeq(19:36, :) = A_;
        beq(19:36, :) = beq_;
    end
end

end


