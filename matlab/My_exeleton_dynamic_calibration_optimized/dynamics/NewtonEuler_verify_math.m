%% NewtonEuler_verify_math.m
function T = NewtonEuler_verify_math(q, qd, qdd, mode)

%% GENERAL PARAMETERS
% s1 = sin(q(1)); c1 = cos(q(1)); 
% s2 = sin(q(2)); c2 = cos(q(2));
% s3 = sin(q(3)); c3 = cos(q(3));
% s4 = sin(q(4)); c4 = cos(q(4));
% s5 = sin(q(5)); c5 = cos(q(5));
% s6 = sin(q(6)); c6 = cos(q(6));
d2 = -188.76; d3 = -81.24; d5 = -12.15; d6 = 112;
a5 = 252.5; alp4 = 4/9*pi; g = 9802;%单位：mm
fe1 = 0; fe2 = 0; fe3 = 0; ne1 = 0; ne2 = 0; ne3 = 0;

th = sym([0,q(2),q(3),q(4),q(5),q(6)]);
d = sym([q(1),d2,d3,0,d5,d6]);
a = sym([0,0,0,0,a5,0]);
alp = [0,-pi/2,pi/2,alp4,0,pi/2];%alp_sym(4):pi_4_9: 4/9*pi

dQ1 = qd(1); ddQ1 = qdd(1); 
dQ2 = qd(2); ddQ2 = qdd(2);
dQ3 = qd(3); ddQ3 = qdd(3); 
dQ4 = qd(4); ddQ4 = qdd(4);
dQ5 = qd(5); ddQ5 = qdd(5);
dQ6 = qd(6); ddQ6 = qdd(6);

%% STANDARD SET BY P_link
if (mode == "P_link")
P_link = evalin('base', 'P_link');

m1 = P_link(1); 
m2 = P_link(14); 
m3 = P_link(27); 
m4 = P_link(40); 
m5 = P_link(53); 
m6 = P_link(66); 

mc11 = P_link(2) /m1; mc12 = P_link(3) /m1; mc13 = P_link(4) /m1;
mc21 = P_link(15)/m2; mc22 = P_link(16)/m2; mc23 = P_link(17)/m2;
mc31 = P_link(28)/m3; mc32 = P_link(29)/m3; mc33 = P_link(30)/m3;
mc41 = P_link(41)/m4; mc42 = P_link(42)/m4; mc43 = P_link(43)/m4;
mc51 = P_link(54)/m5; mc52 = P_link(55)/m5; mc53 = P_link(56)/m5;
mc61 = P_link(67)/m6; mc62 = P_link(68)/m6; mc63 = P_link(69)/m6;

Io111 = P_link(5) ; Io122 = P_link(6) ; Io133 = P_link(7) ; Io112 = P_link(8) ; Io113 = P_link(9) ; Io123 = P_link(10);
Io211 = P_link(18); Io222 = P_link(19); Io233 = P_link(20); Io212 = P_link(21); Io213 = P_link(22); Io223 = P_link(23);
Io311 = P_link(31); Io322 = P_link(32); Io333 = P_link(33); Io312 = P_link(34); Io313 = P_link(35); Io323 = P_link(36);
Io411 = P_link(44); Io422 = P_link(45); Io433 = P_link(46); Io412 = P_link(47); Io413 = P_link(48); Io423 = P_link(49);
Io511 = P_link(57); Io522 = P_link(58); Io533 = P_link(59); Io512 = P_link(60); Io513 = P_link(61); Io523 = P_link(62);
Io611 = P_link(70); Io622 = P_link(71); Io633 = P_link(72); Io612 = P_link(73); Io613 = P_link(74); Io623 = P_link(75);

Ia1 = P_link(11);
Ia2 = P_link(24);
Ia3 = P_link(37);
Ia4 = P_link(50);
Ia5 = P_link(63);
Ia6 = P_link(76);

fv1 = P_link(12); fc1 = P_link(13); 
fv2 = P_link(25); fc2 = P_link(26); 
fv3 = P_link(38); fc3 = P_link(39); 
fv4 = P_link(51); fc4 = P_link(52); 
fv5 = P_link(64); fc5 = P_link(65); 
fv6 = P_link(77); fc6 = P_link(78); 


elseif (mode == "P_center")
%% STANDARD SET BY P_center
P_center = evalin('base', 'P_center');

m1 = P_center(1); 
m2 = P_center(14); 
m3 = P_center(27); 
m4 = P_center(40); 
m5 = P_center(53); 
m6 = P_center(66); 

mc11 = P_center(2) ; mc12 = P_center(3) ; mc13 = P_center(4) ;
mc21 = P_center(15); mc22 = P_center(16); mc23 = P_center(17);
mc31 = P_center(28); mc32 = P_center(29); mc33 = P_center(30);
mc41 = P_center(41); mc42 = P_center(42); mc43 = P_center(43);
mc51 = P_center(54); mc52 = P_center(55); mc53 = P_center(56);
mc61 = P_center(67); mc62 = P_center(68); mc63 = P_center(69);

Ic111 = P_center(5) ; Ic122 = P_center(6) ; Ic133 = P_center(7) ; Ic112 = P_center(8) ; Ic113 = P_center(9) ; Ic123 = P_center(10);
Ic211 = P_center(18); Ic222 = P_center(19); Ic233 = P_center(20); Ic212 = P_center(21); Ic213 = P_center(22); Ic223 = P_center(23);
Ic311 = P_center(31); Ic322 = P_center(32); Ic333 = P_center(33); Ic312 = P_center(34); Ic313 = P_center(35); Ic323 = P_center(36);
Ic411 = P_center(44); Ic422 = P_center(45); Ic433 = P_center(46); Ic412 = P_center(47); Ic413 = P_center(48); Ic423 = P_center(49);
Ic511 = P_center(57); Ic522 = P_center(58); Ic533 = P_center(59); Ic512 = P_center(60); Ic513 = P_center(61); Ic523 = P_center(62);
Ic611 = P_center(70); Ic622 = P_center(71); Ic633 = P_center(72); Ic612 = P_center(73); Ic613 = P_center(74); Ic623 = P_center(75);

Ia1 = P_center(11);
Ia2 = P_center(24);
Ia3 = P_center(37);
Ia4 = P_center(50);
Ia5 = P_center(63);
Ia6 = P_center(76);

fv1 = P_center(12); fc1 = P_center(13); 
fv2 = P_center(25); fc2 = P_center(26); 
fv3 = P_center(38); fc3 = P_center(39); 
fv4 = P_center(51); fc4 = P_center(52); 
fv5 = P_center(64); fc5 = P_center(65); 
fv6 = P_center(77); fc6 = P_center(78); 
end
disp("<INFO> PARAMETERS Loaded!!");

%% VECTORIZATION
% 各齐次矩阵function T = MDHTrans(alpha, a, d, theta)
%   R01: frame{1} relative to frame{0}
%   R10: frame{0} relative to frame{1}
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

% joint velocity
dQZ1 = [0; 0; dQ1];
dQZ2 = [0; 0; dQ2];
dQZ3 = [0; 0; dQ3];
dQZ4 = [0; 0; dQ4];
dQZ5 = [0; 0; dQ5];
dQZ6 = [0; 0; dQ6];
% joint acceleration
ddQZ1 = [0; 0; ddQ1];
ddQZ2 = [0; 0; ddQ2];
ddQZ3 = [0; 0; ddQ3];
ddQZ4 = [0; 0; ddQ4];
ddQZ5 = [0; 0; ddQ5];
ddQZ6 = [0; 0; ddQ6];
% coordinate of next origin(frame) w.r.t. current origin(frame)
% DEBUG: POINT 2
p10 = T01(1: 3, 4); p21 = T12(1: 3, 4); p32 = T23(1: 3, 4);
p43 = T34(1: 3, 4); p54 = T45(1: 3, 4); p65 = T56(1: 3, 4); p76 = T67(1: 3, 4);
% P01 = [0; 0; d1];
% P12 = [0; 0; 0];
% P23 = [0; -d3; 0];
% P34 = [0; 0; 0];
% P45 = [0; -d5; 0];
% P56 = [0; 0; 0];
% P67 = [0; -d7; 0];
% P7e = [0; 0; 0];
% position of CoM of link
Pc1 = [mc11; mc12; mc13];
Pc2 = [mc21; mc22; mc23];
Pc3 = [mc31; mc32; mc33];
Pc4 = [mc41; mc42; mc43];
Pc5 = [mc51; mc52; mc53];
Pc6 = [mc61; mc62; mc63]; 

% inertia matrix w.r.t. CoM
if (mode == "P_center")
    Ic1 = [Ic111 Ic112 Ic113; Ic112 Ic122 Ic123; Ic113 Ic123 Ic133]; 
    Ic2 = [Ic211 Ic212 Ic213; Ic212 Ic222 Ic223; Ic213 Ic223 Ic233]; 
    Ic3 = [Ic311 Ic312 Ic313; Ic312 Ic322 Ic323; Ic313 Ic323 Ic333]; 
    Ic4 = [Ic411 Ic412 Ic413; Ic412 Ic422 Ic423; Ic413 Ic423 Ic433]; 
    Ic5 = [Ic511 Ic512 Ic513; Ic512 Ic522 Ic523; Ic513 Ic523 Ic533];
    Ic6 = [Ic611 Ic612 Ic613; Ic612 Ic622 Ic623; Ic613 Ic623 Ic633]; 
elseif (mode == "P_link")
% inertia matrix w.r.t. CoM
    I = eye(3);
    Io1 = [Io111 Io112 Io113; Io112 Io122 Io123; Io113 Io123 Io133]; 
    Io2 = [Io211 Io212 Io213; Io212 Io222 Io223; Io213 Io223 Io233]; 
    Io3 = [Io311 Io312 Io313; Io312 Io322 Io323; Io313 Io323 Io333]; 
    Io4 = [Io411 Io412 Io413; Io412 Io422 Io423; Io413 Io423 Io433]; 
    Io5 = [Io511 Io512 Io513; Io512 Io522 Io523; Io513 Io523 Io533];
    Io6 = [Io611 Io612 Io613; Io612 Io622 Io623; Io613 Io623 Io633]; 
    Ic1 = Io1 - m1 * (Pc1' * Pc1 * I - Pc1 * Pc1');
    Ic2 = Io2 - m2 * (Pc2' * Pc2 * I - Pc2 * Pc2');
    Ic3 = Io3 - m3 * (Pc3' * Pc3 * I - Pc3 * Pc3');
    Ic4 = Io4 - m4 * (Pc4' * Pc4 * I - Pc4 * Pc4');
    Ic5 = Io5 - m5 * (Pc5' * Pc5 * I - Pc5 * Pc5');
    Ic6 = Io6 - m6 * (Pc6' * Pc6 * I - Pc6 * Pc6');
end
% external force/torque acting on ee
fe = [fe1; fe2; fe3]; ne = [ne1; ne2; ne3];
disp("<INFO> VECTORIZATION complete!!");

%% ITERATIVE NETWON-EULER DYNAMIC FORMULATION
% using Craig's notation: 
%   wn = link 'n' velocity (\omega)
% 	dwn = link 'n' acceleration(\alpha)
% 	dvn = link 'n' acceleration (\a)
%   dvcn = link 'n' acceleration of the center of mass (\a_c)
%   Fn = link 'n' force acting on the center of the mass
%   Nn = link 'n' torque acting on the center of the mass 
w00 = [0; 0; 0]; v00 = [0; 0; 0]; w00d = [0; 0; 0]; v00d = [0; 0; g];%基座各项初始值
z = [0; 0; 1];

%% 外推: i: 0->5
% 连杆1到连杆6向外迭代
% i = 0:移动关节
w11 = w00;
w11d = R10*w00d;
v11d = R10*(cross(w00d, p10) + cross(w00, cross(w00, p10)) + v00d) + cross(2*w11,qd(1)*z)+qdd(1)*z;
vc11d = cross(w11d, Pc1) + cross(w11, cross(w11, Pc1)) + v11d;
F11 = m1*vc11d;
N11 = Ic1*w11d + cross(w11, Ic1*w11);
disp("<INFO> J1 outward ITERATION complete.");

% i = 1
w22 = R21*w11 + qd(2)*z;
w22d = R21*w11d + cross(R21*w11, z*qd(2)) + qdd(2)*z;
v22d = R21*(cross(w11d, p21) + cross(w11, cross(w11, p21)) + v11d);
vc22d = cross(w22d, Pc2) + cross(w22, cross(w22, Pc2)) + v22d;
F22 = m2*vc22d;
N22 = Ic2*w22d + cross(w22, Ic2*w22);
disp("<INFO> J2 outward ITERATION complete.");

% i = 2
w33 = R32*w22 + qd(3)*z;
w33d = R32*w22d + cross(R32*w22, z*qd(3)) + qdd(3)*z;
v33d = R32*(cross(w22d, p32) + cross(w22, cross(w22, p32)) + v22d);
vc33d = cross(w33d, Pc3) + cross(w33, cross(w33, Pc3)) + v33d;
F33 = m3*vc33d;
N33 = Ic3*w33d + cross(w33, Ic3*w33);
disp("<INFO> J3 outward ITERATION complete.");

% i= 3
w44 = R43*w33 + qd(4)*z;
w44d = R43*w33d + cross(R43*w33, z*qd(4)) + qdd(4)*z;
v44d = R43*(cross(w33d, p43) + cross(w33, cross(w33, p43)) + v33d);
vc44d = cross(w44d, Pc4) + cross(w44, cross(w44, Pc4)) + v44d;
F44 = m4*vc44d;
N44 = Ic4*w44d + cross(w44, Ic4*w44);

% i = 4
w55 = R54*w44 + qd(5)*z;
w55d = R54*w44d + cross(R54*w44, z*qd(5)) + qdd(5)*z;
v55d = R54*(cross(w44d, p54) + cross(w44, cross(w44, p54)) + v44d);
vc55d = cross(w55d, Pc5) + cross(w55, cross(w55, Pc5)) + v55d;
F55 = m5*vc55d;
N55 = Ic5*w55d + cross(w55, Ic5*w55);
disp("<INFO> J5 outward ITERATION complete.");

% i = 5
w66 = R65*w55 + qd(6)*z;
w66d = R65*w55d + cross(R65*w55, z*qd(6)) + qdd(6)*z;
v66d = R65*(cross(w55d, p65) + cross(w55, cross(w55, p65)) + v55d);
vc66d = cross(w66d, Pc6) + cross(w66, cross(w66, Pc6)) + v66d;
F66 = m6*vc66d;
N66 = Ic6*w66d + cross(w66, Ic6*w66);
disp("<INFO> J6 outward ITERATION complete.");

%% 内推: i: 6->1
% 连杆6到连杆1向内迭代
f77 = [0; 0; 0]; n77 = [0; 0; 0];%外力
% i = 6
f66 = R67*f77 + F66;
n66 = N66 + R67*n77 + cross(Pc6, F66) + cross(p76, R67*f77);
% n66 = N66_1 + R67*n77 + cross(p76, R67*f77);
T6 = n66'*z + fc6*sign(qd(6)) + fv6*qd(6);
disp("<INFO> J6 inward ITERATION complete.");

% i = 5
f55 = R56*f66 + F55;
n55 = N55 + R56*n66 + cross(Pc5, F55) + cross(p65, R56*f66);
% n55 = N55_1 + R56*n66 + cross(p65, R56*f66);
T5 = n55'*z + fc5*sign(qd(5)) + fv5*qd(5);
disp("<INFO> J5 inward ITERATION complete.");

% i = 4
f44 = R45*f55 + F44;
n44 = N44 + R45*n55 + cross(Pc4, F44) + cross(p54, R45*f55);
% n44 = N44_1 + R45*n55 + cross(p54, R45*f55);
T4 = n44'*z + fc4*sign(qd(4)) + fv4*qd(4);
disp("<INFO> J4 inward ITERATION complete.");

% i = 3
f33 = R34*f44 + F33;
n33 = N33 + R34*n44 + cross(Pc3, F33) + cross(p43, R34*f44);
% n33 = N33_1 + R34*n44 + cross(p43, R34*f44);
T3 = n33'*z + fc3*sign(qd(3)) + fv3*qd(3);
disp("<INFO> J3 inward ITERATION complete.");

% i = 2
f22 = R23*f33 + F22;
n22 = N22 + R23*n33 + cross(Pc2, F22) + cross(p32, R23*f33);
% n22 = N22_1 + R23*n33 + cross(p32, R23*f33);
T2 = n22'*z + fc2*sign(qd(2)) + fv2*qd(2);
disp("<INFO> J2 inward ITERATION complete.");

% i =1：移动关节
f11 = R12*f22 + F11;
n11 = N11 + R12*n22 + cross(Pc1, F11) + cross(p21, R12*f22);
% n11 = N11_1 + R12*n22 + cross(p21, R12*f22);
T1 = f11'*z + fc1*sign(qd(1)) + fv1*qd(1);
disp("<INFO> J1 inward ITERATION complete.");

%% INTEGRATE DYNAMIC EQUATION WITH FRICTION
T1 = T1 + Ia1*ddQ1;
T2 = T2 + Ia2*ddQ2;
T3 = T3 + Ia3*ddQ3;
T4 = T4 + Ia4*ddQ4;
T5 = T5 + Ia5*ddQ5;
T6 = T6 + Ia6*ddQ6;
T = [T1; T2; T3; T4; T5; T6];
disp("<INFO> DYNAMIC EQUATION obtained");



