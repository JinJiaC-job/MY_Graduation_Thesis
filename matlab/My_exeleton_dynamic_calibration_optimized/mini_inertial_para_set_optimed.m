%确定杆6~杆1的最小惯性参数集
% clear; close all; clc;

%输入量参数符号化
q = sym('q%d',[6,1],'real');
qd = sym('qd%d',[6,1],'real');
qdd = sym('qdd%d',[6,1],'real');
%摩擦参数符号化
fc = sym('fc%d',[6,1],'real');
fv = sym('fv%d',[6,1],'real'); 
%惯性参数符号化:xxi~zzi表示在关节坐标系i下连杆i惯性张量Iii的元素；mi为连杆i质量与质心坐标的乘积
xx = sym('xx%d',[6,1],'real');
xy = sym('xy%d',[6,1],'real');
xz = sym('xz%d',[6,1],'real');
yy = sym('yy%d',[6,1],'real');
yz = sym('yz%d',[6,1],'real');
zz = sym('zz%d',[6,1],'real');
mx = sym('mx%d',[6,1],'real');
my = sym('my%d',[6,1],'real');
mz = sym('mz%d',[6,1],'real');
%质量参数符号化
m  = sym('m%d',[6,1],'real');
%运动学参数
a_sym = sym('a%d',[6,1],'real');
d_sym = sym('d%d',[6,1],'real');
alp_sym = sym('alp%d',[6,1],'real');

th = sym([0,q(2),q(3),-pi/2+q(4),pi/2+q(5),q(6)]);
d = sym([q(1),d_sym(2),d_sym(3),0,d_sym(5),d_sym(6)]);
a = sym([0,0,0,0,a_sym(5),0]);
alp = [0,-pi/2,pi/2,alp_sym(4),0,pi/2];%alp_sym(4):pi_4_9: 4/9*pi

%设置初始值
XX = sym([0,0,0,0,0,xx(6)]);
XY = sym([0,0,0,0,0,xy(6)]);
XZ = sym([0,0,0,0,0,xz(6)]);
YY = sym([0,0,0,0,0,yy(6)]);
YZ = sym([0,0,0,0,0,yz(6)]);
ZZ = sym([0,0,0,0,0,zz(6)]);
MX = sym([0,0,0,0,0,mz(6)]);
MY = sym([0,0,0,0,0,my(6)]);
MZ = sym([0,0,0,0,0,mz(6)]);
M  = sym([0,0,0,0,0,m(6)]);
XX_1 = sym([0,0,0,0,0,0]);

% %当关节i(1)属于情况T4:杆i的最小惯性参数为M(n)
% XX(n-1) = xx(n-1);
% XY(n-1) = xy(n-1);
% XZ(n-1) = xz(n-1);
% YY(n-1) = yy(n-1);
% YZ(n-1) = yz(n-1);
% ZZ(n-1) = zz(n-1);
% MX(n-1) = mx(n-1);
% MY(n-1) = my(n-1);
% MZ(n-1) = mz(n-1);
% M(n-1)  = m(n-1);
% %当关节i(6,5,4,3)属于情况R1:杆i的最小惯性参数为XX(n)*=XX(n)-YY(n),XY(n),XZ(n),YY(n),YZ(n),ZZ(n),MX(n),MY(n)
% XX_1(n)=XX(n)-YY(n);
% XX(n-1) = xx(n-1)+YY(n)+2*d(n)*MZ(n)+d(n)^2*M(n);
% XY(n-1) = xy(n-1)+a(n)*sin(alp(n))*(MZ(n)+d(n)*M(n));
% XZ(n-1) = xz(n-1)-a(n)*cos(alp(n))*(MZ(n)+d(n)*M(n));
% YY(n-1) = yy(n-1)+(cos(alp(n)))^2*YY(n)+2*d(n)*(cos(alp(n)))^2*MZ(n)+(d(n)^2*(cos(alp(n)))^2+a(n)^2)*M(n);
% YZ(n-1) = yz(n-1)+cos(alp(n))*sin(alp(n))*(YY(n)+2*d(n)*MZ(n)+d(n)^2*M(n));
% ZZ(n-1) = zz(n-1)+(sin(alp(n)))^2*YY(n)+2*d(n)*(sin(alp(n)))^2*MZ(n)+(d(n)^2*(sin(alp(n)))^2+a(n)^2)*M(n);
% MX(n-1) = mx(n-1)+a(n)*M(n);
% MY(n-1) = my(n-1)-sin(alp(n))*(MZ(n)+d(n)*M(n));
% MZ(n-1) = mz(n-1)+cos(alp(n))*(MZ(n)+d(n)*M(n));
% M(n-1)  = m(n-1)+M(n);
% %当关节i(2)属于情况R2:杆i的最小惯性参数为ZZ(n),MX(n),MY(n)
% XX(n-1) = xx(n-1)+d(n)^2*M(n);
% XY(n-1) = xy(n-1)+a(n)*d(n)*sin(alp(n))*M(n);
% XZ(n-1) = xz(n-1)-a(n)*d(n)*cos(alp(n))*M(n);
% YY(n-1) = yy(n-1)+(d(n)^2*(cos(alp(n)))^2+a(n)^2)*M(n);
% YZ(n-1) = yz(n-1)+d(n)^2*cos(alp(n))*sin(alp(n))*M(n);
% ZZ(n-1) = zz(n-1)+(d(n)^2*(sin(alp(n)))^2+a(n)^2)*M(n);
% MX(n-1) = mx(n-1)+a(n)*M(n);
% MY(n-1) = my(n-1)-d(n)*sin(alp(n))*M(n);
% MZ(n-1) = mz(n-1)+d(n)*cos(alp(n))*M(n);
% M(n-1)  = m(n-1)+M(n);

for n = 6:-1:3
    XX_1(n) = XX(n)-YY(n);
    XX(n-1) = xx(n-1)+YY(n)+2*d(n)*MZ(n)+d(n)^2*M(n);
    XY(n-1) = xy(n-1)+a(n)*sin(alp(n))*(MZ(n)+d(n)*M(n));
    XZ(n-1) = xz(n-1)-a(n)*cos(alp(n))*(MZ(n)+d(n)*M(n));
    YY(n-1) = yy(n-1)+(cos(alp(n)))^2*YY(n)+2*d(n)*(cos(alp(n)))^2*MZ(n)+(d(n)^2*(cos(alp(n)))^2+a(n)^2)*M(n);
    YZ(n-1) = yz(n-1)+cos(alp(n))*sin(alp(n))*(YY(n)+2*d(n)*MZ(n)+d(n)^2*M(n));
    ZZ(n-1) = zz(n-1)+(sin(alp(n)))^2*YY(n)+2*d(n)*(sin(alp(n)))^2*MZ(n)+(d(n)^2*(sin(alp(n)))^2+a(n)^2)*M(n);
    MX(n-1) = mx(n-1)+a(n)*M(n);
    MY(n-1) = my(n-1)-sin(alp(n))*(MZ(n)+d(n)*M(n));
    MZ(n-1) = mz(n-1)+cos(alp(n))*(MZ(n)+d(n)*M(n));
    M(n-1)  = m(n-1)+M(n);
end

n = 2;
XX(n-1) = xx(n-1)+d(n)^2*M(n);
XY(n-1) = xy(n-1)+a(n)*d(n)*sin(alp(n))*M(n);
XZ(n-1) = xz(n-1)-a(n)*d(n)*cos(alp(n))*M(n);
YY(n-1) = yy(n-1)+(d(n)^2*(cos(alp(n)))^2+a(n)^2)*M(n);
YZ(n-1) = yz(n-1)+d(n)^2*cos(alp(n))*sin(alp(n))*M(n);
ZZ(n-1) = zz(n-1)+(d(n)^2*(sin(alp(n)))^2+a(n)^2)*M(n);
MX(n-1) = mx(n-1)+a(n)*M(n);
MY(n-1) = my(n-1)-d(n)*sin(alp(n))*M(n);
MZ(n-1) = mz(n-1)+d(n)*cos(alp(n))*M(n);
M(n-1)  = m(n-1)+M(n);

% n = 1;
% XX(n-1) = xx(n-1);
% XY(n-1) = xy(n-1);
% XZ(n-1) = xz(n-1);
% YY(n-1) = yy(n-1);
% YZ(n-1) = yz(n-1);
% ZZ(n-1) = zz(n-1);
% MX(n-1) = mx(n-1);
% MY(n-1) = my(n-1);
% MZ(n-1) = mz(n-1);
% M(n-1)  = m(n-1);

p = [XX',XY',XZ',YY',YZ',ZZ',MX',MY',MZ',M'];
p(1:2,1) = XX(1:2)';
p(3:6,1) = XX_1(3:6)';

pj6 = p([6],[1 2 3 5 6 7 8]); pj6(8) = fc(6); pj6(9) = fv(6);
pj5 = p([5],[1 2 3 5 6 7 8]); pj5(8) = fc(5); pj5(9) = fv(5);
pj4 = p([4],[1 2 3 5 6 7 8]); pj4(8) = fc(4); pj4(9) = fv(4);
pj3 = p([3],[1 2 3 5 6 7 8]); pj3(8) = fc(3); pj3(9) = fv(3);
pj2 = p([2],[6 7 8]); pj2(4) = fc(2); pj2(5) = fv(2);
pj1 = p([1],[10]); pj1(2) = fc(1); pj1(3) = fv(1);
pmin = [pj1 pj2 pj3 pj4 pj5 pj6]';


assignin('base', 'pmin', pmin);





