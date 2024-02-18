%% mixed_traj
function [q,qd,qdd] = mixed_traj(traj_param, dof, time, traj_wf, traj_order)

[qh,qhd,qh2d] = fourier_series_traj(traj_param, dof, time, traj_wf, traj_order);
C = getPolCoeffs(traj_param, dof, time, traj_wf, traj_order);
% fifth order polynomail trajectory
qp = C(:,1) + C(:,2).*t + C(:,3).*t.^2 + C(:,4).*t.^3 + ...
                                    C(:,5).*t.^4 + C(:,6).*t.^5;
qpd = C(:,2) + 2*C(:,3).*t + 3*C(:,4).*t.^2 + ...
                                4*C(:,5).*t.^3 + 5*C(:,6).*t.^4;
qp2d = 2*C(:,3) + 6*C(:,4).*t + 12*C(:,5).*t.^2 + 20*C(:,6).*t.^3;

q = qh + qp;
qd = qhd + qpd;
qdd = qh2d + qp2d;


end
