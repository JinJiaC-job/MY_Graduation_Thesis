function T = MDHTrans(alpha, a, d, theta)
%UNTITLED5 此处提供此函数的摘要
%   此处提供详细说明 
    T = [cos(theta)                -sin(theta)                0                     a;
        sin(theta)*cos(alpha)     cos(theta)*cos(alpha)    -sin(alpha)    -sin(alpha)*d;
        sin(theta)*sin(alpha)     cos(theta)*sin(alpha)    cos(alpha)     cos(alpha)*d;
        0                             0                       0                  1];
 
end