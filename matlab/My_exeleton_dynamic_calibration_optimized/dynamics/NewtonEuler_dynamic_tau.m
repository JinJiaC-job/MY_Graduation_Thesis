%该函数使用牛顿欧拉法求解机械臂动力学
% clear; close all; clc;

%重力加速度符号
syms g real;
%摩擦参数符号化
fc = sym('fc%d',[6,1],'real');
fv = sym('fv%d',[6,1],'real'); 
%输入参数符号化
q = sym('q%d',[6,1],'real');
qd = sym('qd%d',[6,1],'real');
qdd = sym('qdd%d',[6,1],'real');
% MDH参数
a_sym = sym('a%d',[6,1],'real');
d_sym = sym('d%d',[6,1],'real');
alp_sym = sym('alp%d',[6,1],'real');

th = sym([0,q(2),q(3),q(4),q(5),q(6)]);
d = sym([q(1),d_sym(2),d_sym(3),0,d_sym(5),d_sym(6)]);
a = sym([0,0,0,0,a_sym(5),0]);
alp = [0,-pi/2,pi/2,alp_sym(4),0,pi/2];%alp_sym(4):pi_4_9: 4/9*pi
% th = [0, thetaANDd(2), thetaANDd(3), -pi/2+thetaANDd(4), pi/2+thetaANDd(5), thetaANDd(6)]';
% d = [thetaANDd(1), -188.76, -81.24, 0, 113.03, 111.50]';
% a = [0, 0, 0, 0, 267.5, 0]';
% alp = [0, -pi/2, pi/2, 4/9*pi, 0, pi/2]';
%各连杆质量符号化
m = sym('m%d',[6,1],'real');%m1,m2...
% 电机惯量
Ia = sym('Ia%d',[6,1],'real');%Ia1,Ia2...

% 各齐次矩阵function T = MDHTrans(alpha, a, d, theta)
T01 = MDHTrans(alp(1), a(1), d(1), th(1));
T12 = MDHTrans(alp(2), a(2), d(2), th(2));
T23 = MDHTrans(alp(3), a(3), d(3), th(3));
T34 = MDHTrans(alp(4), a(4), d(4), th(4));
T45 = MDHTrans(alp(5), a(5), d(5), th(5));
T56 = MDHTrans(alp(6), a(6), d(6), th(6));
T67 = [1 0 0 0; 
       0 1 0 122.68; 
       0 0 1 85.5; 
       0 0 0 1];%DH坐标法末端坐标系与工具坐标系之间的转化（withoutHand）
% 旋转矩阵
R01 = T01(1:3, 1:3); R12 = T12(1:3, 1:3); R23 = T23(1:3, 1:3);
R34 = T34(1:3, 1:3); R45 = T45(1:3, 1:3); R56 = T56(1:3, 1:3);
R10 = R01'; R21 = R12'; R32 = R23';
R43 = R34'; R54 = R45'; R65 = R56';
R67 = [1 0 0; 0 1 0; 0 0 1]; R76 = R67';
% 各关节p及各连杆质心pc的距离
p10 = T01(1: 3, 4); p21 = T12(1: 3, 4); p32 = T23(1: 3, 4);
p43 = T34(1: 3, 4); p54 = T45(1: 3, 4); p65 = T56(1: 3, 4); p76 = T67(1: 3, 4);
Cx = sym('C%dx',[6,1],'real');%C11x~C66x
Cy = sym('C%dy',[6,1],'real');%C11y~C66y
Cz = sym('C%dz',[6,1],'real');%C11x~C66z
pc11 = [Cx(1); Cy(1); Cz(1)];
pc22 = [Cx(2); Cy(2); Cz(2)];
pc33 = [Cx(3); Cy(3); Cz(3)];
pc44 = [Cx(4); Cy(4); Cz(4)];
pc55 = [Cx(5); Cy(5); Cz(5)];
pc66 = [Cx(6); Cy(6); Cz(6)];
% inertia matrix w.r.t link
Ioxx = sym('Ioxx%d',[6,1],'real');
Ioxy = sym('Ioxy%d',[6,1],'real');
Ioxz = sym('Ioxz%d',[6,1],'real');
Ioyy = sym('Ioyy%d',[6,1],'real');
Ioyz = sym('Ioyz%d',[6,1],'real');
Iozz = sym('Iozz%d',[6,1],'real');

Io1 = [Ioxx(1), Ioxy(1), Ioxz(1); 
       Ioxy(1), Ioyy(1), Ioyz(1); 
       Ioxz(1), Ioyz(1), Iozz(1)];
Io2 = [Ioxx(2), Ioxy(2), Ioxz(2); 
       Ioxy(2), Ioyy(2), Ioyz(2); 
       Ioxz(2), Ioyz(2), Iozz(2)];
Io3 = [Ioxx(3), Ioxy(3), Ioxz(3); 
       Ioxy(3), Ioyy(3), Ioyz(3); 
       Ioxz(3), Ioyz(3), Iozz(3)];
Io4 = [Ioxx(4), Ioxy(4), Ioxz(4); 
       Ioxy(4), Ioyy(4), Ioyz(4); 
       Ioxz(4), Ioyz(4), Iozz(4)];
Io5 = [Ioxx(5), Ioxy(5), Ioxz(5); 
       Ioxy(5), Ioyy(5), Ioyz(5); 
       Ioxz(5), Ioyz(5), Iozz(5)];
Io6 = [Ioxx(6), Ioxy(6), Ioxz(6); 
       Ioxy(6), Ioyy(6), Ioyz(6); 
       Ioxz(6), Ioyz(6), Iozz(6)];
% 惯性张量
% Ixx = sym('Ixx%d',[6,1],'real');
% Ixy = sym('Ixy%d',[6,1],'real');
% Ixz = sym('Ixz%d',[6,1],'real');
% Iyy = sym('Iyy%d',[6,1],'real');
% Iyz = sym('Iyz%d',[6,1],'real');
% Izz = sym('Izz%d',[6,1],'real');
I = eye(3);
% Ic1 = [Ixx(1),  Ixy(1),  Ixz(1); 
%       Ixy(1),  Iyy(1),  Iyz(1); 
%       Ixz(1),  Iyz(1),  Izz(1)]; 
% Ic2 = [Ixx(2),  Ixy(2),  Ixz(2); 
%       Ixy(2),  Iyy(2),  Iyz(2); 
%       Ixz(2),  Iyz(2),  Izz(2)]; 
% Ic3 = [Ixx(3),  Ixy(3),  Ixz(3); 
%       Ixy(3),  Iyy(3),  Iyz(3); 
%       Ixz(3),  Iyz(3),  Izz(3)]; 
% Ic4 = [Ixx(4),  Ixy(4),  Ixz(4); 
%       Ixy(4),  Iyy(4),  Iyz(4); 
%       Ixz(4),  Iyz(4),  Izz(4)]; 
% Ic5 = [Ixx(5),  Ixy(5),  Ixz(5); 
%       Ixy(5),  Iyy(5),  Iyz(5); 
%       Ixz(5),  Iyz(5),  Izz(5)]; 
% Ic6 = [Ixx(6),  Ixy(6),  Ixz(6); 
%       Ixy(6),  Iyy(6),  Iyz(6); 
%       Ixz(6),  Iyz(6),  Izz(6)];
% I11 = Ic1 + m(1)*(pc11'*pc11*I-pc11*pc11');
% I22 = Ic2 + m(2)*(pc22'*pc22*I-pc22*pc22');
% I33 = Ic3 + m(3)*(pc33'*pc33*I-pc33*pc33');
% I44 = Ic4 + m(4)*(pc44'*pc44*I-pc44*pc44');
% I55 = Ic5 + m(5)*(pc55'*pc55*I-pc55*pc55');
% I66 = Ic6 + m(6)*(pc66'*pc66*I-pc66*pc66');
Ic1 = Io1 - m(1) * (pc11'*pc11*I-pc11*pc11');
Ic2 = Io2 - m(2) * (pc22'*pc22*I-pc22*pc22');
Ic3 = Io3 - m(3) * (pc33'*pc33*I-pc33*pc33');
Ic4 = Io4 - m(4) * (pc44'*pc44*I-pc44*pc44');
Ic5 = Io5 - m(5) * (pc55'*pc55*I-pc55*pc55');
Ic6 = Io6 - m(6) * (pc66'*pc66*I-pc66*pc66');

w00 = [0; 0; 0]; v00 = [0; 0; 0]; w00d = [0; 0; 0]; v00d = [0; 0; g];%基座各项初始值
z = [0; 0; 1];

%% 外推: i: 0->5
% 连杆1到连杆6向外迭代
% i = 0:移动关节
w11 = w00;
w11d = R10*w00d;
v11d = R10*(cross(w00d, p10) + cross(w00, cross(w00, p10)) + v00d) + cross(2*w11,qd(1)*z)+qdd(1)*z;
vc11d = cross(w11d, pc11) + cross(w11, cross(w11, pc11)) + v11d;
F11 = m(1)*vc11d;
N11 = Ic1*w11d + cross(w11, Ic1*w11);
% N11_1 = cross(m(1)*pc11, v11d) + I11*w11d + cross(w11, I11*w11);
% i = 1
w22 = R21*w11 + qd(2)*z;
w22d = R21*w11d + cross(R21*w11, z*qd(2)) + qdd(2)*z;
v22d = R21*(cross(w11d, p21) + cross(w11, cross(w11, p21)) + v11d);
vc22d = cross(w22d, pc22) + cross(w22, cross(w22, pc22)) + v22d;
F22 = m(2)*vc22d;
N22 = Ic2*w22d + cross(w22, Ic2*w22);
% N22_1 = cross(m(2)*pc22, v22d) + I22*w22d + cross(w22, I22*w22);
% i = 2
w33 = R32*w22 + qd(3)*z;
w33d = R32*w22d + cross(R32*w22, z*qd(3)) + qdd(3)*z;
v33d = R32*(cross(w22d, p32) + cross(w22, cross(w22, p32)) + v22d);
vc33d = cross(w33d, pc33) + cross(w33, cross(w33, pc33)) + v33d;
F33 = m(3)*vc33d;
N33 = Ic3*w33d + cross(w33, Ic3*w33);
% N33_1 = cross(m(3)*pc33, v33d) + I33*w33d + cross(w33, I33*w33);
% i= 3
w44 = R43*w33 + qd(4)*z;
w44d = R43*w33d + cross(R43*w33, z*qd(4)) + qdd(4)*z;
v44d = R43*(cross(w33d, p43) + cross(w33, cross(w33, p43)) + v33d);
vc44d = cross(w44d, pc44) + cross(w44, cross(w44, pc44)) + v44d;
F44 = m(4)*vc44d;
N44 = Ic4*w44d + cross(w44, Ic4*w44);
% N44_1 = cross(m(4)*pc44, v44d) + I44*w44d + cross(w44, I44*w44);
% i = 4
w55 = R54*w44 + qd(5)*z;
w55d = R54*w44d + cross(R54*w44, z*qd(5)) + qdd(5)*z;
v55d = R54*(cross(w44d, p54) + cross(w44, cross(w44, p54)) + v44d);
vc55d = cross(w55d, pc55) + cross(w55, cross(w55, pc55)) + v55d;
F55 = m(5)*vc55d;
N55 = Ic5*w55d + cross(w55, Ic5*w55);
% N55_1 = cross(m(5)*pc55, v55d) + I55*w55d + cross(w55, I55*w55);
% i = 5
w66 = R65*w55 + qd(6)*z;
w66d = R65*w55d + cross(R65*w55, z*qd(6)) + qdd(6)*z;
v66d = R65*(cross(w55d, p65) + cross(w55, cross(w55, p65)) + v55d);
vc66d = cross(w66d, pc66) + cross(w66, cross(w66, pc66)) + v66d;
F66 = m(6)*vc66d;
N66 = Ic6*w66d + cross(w66, Ic6*w66);
% N66_1 = cross(m(6)*pc66, v66d) + I66*w66d + cross(w66, I66*w66);

%% 内推: i: 6->1
% 连杆6到连杆1向内迭代
f77 = [0; 0; 0]; n77 = [0; 0; 0];%外力
% i = 6
f66 = R67*f77 + F66;
n66 = N66 + R67*n77 + cross(pc66, F66) + cross(p76, R67*f77);
% n66 = N66_1 + R67*n77 + cross(p76, R67*f77);
tau(6) = n66'*z + fc(6)*sign(qd(6)) + fv(6)*qd(6);
% i = 5
f55 = R56*f66 + F55;
n55 = N55 + R56*n66 + cross(pc55, F55) + cross(p65, R56*f66);
% n55 = N55_1 + R56*n66 + cross(p65, R56*f66);
tau(5) = n55'*z + fc(5)*sign(qd(5)) + fv(5)*qd(5);
% i = 4
f44 = R45*f55 + F44;
n44 = N44 + R45*n55 + cross(pc44, F44) + cross(p54, R45*f55);
% n44 = N44_1 + R45*n55 + cross(p54, R45*f55);
tau(4) = n44'*z + fc(4)*sign(qd(4)) + fv(4)*qd(4);
% i = 3
f33 = R34*f44 + F33;
n33 = N33 + R34*n44 + cross(pc33, F33) + cross(p43, R34*f44);
% n33 = N33_1 + R34*n44 + cross(p43, R34*f44);
tau(3) = n33'*z + fc(3)*sign(qd(3)) + fv(3)*qd(3);
% i = 2
f22 = R23*f33 + F22;
n22 = N22 + R23*n33 + cross(pc22, F22) + cross(p32, R23*f33);
% n22 = N22_1 + R23*n33 + cross(p32, R23*f33);
tau(2) = n22'*z + fc(2)*sign(qd(2)) + fv(2)*qd(2);
% i =1：移动关节
f11 = R12*f22 + F11;
n11 = N11 + R12*n22 + cross(pc11, F11) + cross(p21, R12*f22);
% n11 = N11_1 + R12*n22 + cross(p21, R12*f22);
tau(1) = f11'*z + fc(1)*sign(qd(1)) + fv(1)*qd(1);
%%
%加上电机惯量项
tau(1) = tau(1) + Ia(1)*qdd(1);
tau(2) = tau(2) + Ia(2)*qdd(2);
tau(3) = tau(3) + Ia(3)*qdd(3);
tau(4) = tau(4) + Ia(4)*qdd(4);
tau(5) = tau(5) + Ia(5)*qdd(5);
tau(6) = tau(6) + Ia(6)*qdd(6);

tau = [tau(1);tau(2);tau(3);tau(4);tau(5);tau(6)];
% tau_stdard_para = simplify(tau);

% assignin('base', 'tau_stdard_para', tau_stdard_para);
disp("<INFO> DYNAMIC EQUATION obtained");


