function out = getPolCoeffs(traj_param, dof, time, traj_wf, traj_order)
% -----------------------------------------------------------------------
% The function computes coefficeints of the 5-th order polynomail from a and b coefficients of the finite furier series and  initial position of the robot.
% Inputes:
%   T - period of motion
%   a - sine coeffs in finite fourier series
%   b - cosine coeffs in finite fourier series
%   wf - fundamental frequency
%   N - number of harmonics
%   q0 - initial position of the robot
% Output:
%   out - coefficients of the fifth order polynomail that guarantees
%         that initial and final constraints on position, velocities
%         and accelerations are satisfied
% -----------------------------------------------------------------------
q0 = zeros(dof,1);
for ii = 1:dof
   m = (order_prod_2 + 1) * (ii - 1); 
   q0(ii) = traj_param(m + order_prod_2 + 1);%q0
   for jj = 1:traj_order
       % alpha(a)=traj_param(m+2*(jj-1)+1), beta(b)=traj_param(m+2*(jj-1)+2)
        qh0 = -sum(traj_param(m+2*(jj-1)+2)./((1:traj_order)*traj_wf),2);
        qdh0 = sum(traj_param(m+2*(jj-1)+1),2);
        q2dh0 = sum(traj_param(m+2*(jj-1)+2).*(1:1:traj_order)*traj_wf,2);
   end    
end

[qhT,qdhT,q2dhT] = fourier_series_traj(traj_param, dof, time, traj_wf, traj_order);

I_6x6 = eye(6);
O_6x6 = zeros(6);

Ac = [I_6x6, O_6x6, O_6x6, O_6x6, O_6x6, O_6x6;
      O_6x6, I_6x6, O_6x6, O_6x6, O_6x6, O_6x6;
      O_6x6, O_6x6, 2*I_6x6, O_6x6, O_6x6, O_6x6;
      I_6x6, T*I_6x6, T^2*I_6x6, T^3*I_6x6, T^4*I_6x6, T^5*I_6x6;
      O_6x6, I_6x6, 2*T*I_6x6, 3*T^2*I_6x6, 4*T^3*I_6x6, 5*T^4*I_6x6;
      O_6x6, O_6x6, 2*I_6x6, 6*T*I_6x6, 12*T^2*I_6x6, 20*T^3*I_6x6];

c = Ac\[q0-qh0; -qdh0; -q2dh0; q0-qhT; -qdhT; -q2dhT];
out = reshape(c,[6,6]);

