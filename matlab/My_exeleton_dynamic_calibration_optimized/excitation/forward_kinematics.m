function [T, tip, rot] = forward_kinematics(q)
% @brief: 求齐次变换矩阵
% @param[in] q: 关节角度矢量 (radian, joint 1~6)
% @param[out] T: 齐次变换矩阵 (4×4 matrix)
% @param[out] tip: 工具中心位置, in x-y-z (3×1 vector, in m)
% @param[out] rot: 旋转矩阵 (3×3 matrix)

% DH parameters (in mm/ rad), 加上工具坐标系
a = [0 0 0 0 252.5 0];
d = [q(1) -188.76 -81.24 0 12.15 112];
alpha = [0 -pi/2 pi/2 4/9*pi 0 pi/2];
theta = [0 q(2) q(3) q(4) q(5) q(6)];
T67 = [1 0 0 0; 
       0 1 0 122.68; 
       0 0 1 85.5; 
       0 0 0 1];%DH坐标法末端坐标系与工具坐标系之间的转化

% Transformation matrix
T = eye(4);
for i = 1:6
    t = eye(4);

    t(1, 1) = cos(theta(i));
    t(1, 2) = -sin(theta(i));
    t(1, 3) = 0;
    t(1, 4) = a(i);

    t(2, 1) = sin(theta(i)) * cos(alpha(i));
    t(2, 2) = cos(theta(i)) * cos(alpha(i));
    t(2, 3) = -sin(alpha(i));
    t(2, 4) = -d(i) * sin(alpha(i));

    t(3, 1) = sin(theta(i)) * sin(alpha(i));
    t(3, 2) = cos(theta(i)) * sin(alpha(i));
    t(3, 3) = cos(alpha(i));
    t(3, 4) = d(i) * cos(alpha(i));
    
    T = T * t;
end
T = T * T67;
tip = [T(1,4); T(2,4); T(3,4)];
rot = T(1:3, 1:3);

