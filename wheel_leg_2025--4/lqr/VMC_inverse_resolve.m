function [M,I,L] = VMC_inverse_resolve (M,L,I,Q)
%连杆质心参数
L1 = 0.10406;                   %大腿质心到髋关节距离
L2 = 0.10377;                   %小腿质心到膝关节距离
%VMC逆解算
q0 = Q * pi / 180;
Xc = L.L * cos(q0);
Yc = L.L * sin(q0);
%q12为五边形L1延长线与L2外部夹角 
q12 = acos((Xc^2+Yc^2-L.L1^2-L.L2^2)/(2*L.L1*L.L2));
%延长L1，作C点垂直L1延长线为temp1,temp2为改垂直点到原点的距离
temp1 = L.L2 * sin(q12);
temp2 = L.L1 + L.L2 * cos(q12);
%计算q1，q2
q1 = acos(temp2 / sqrt(temp1^2 + temp2^2)) + acos(Xc / sqrt(temp1^2 + temp2^2));
q2 = q1 - q12;%dddddddddddddd

r1 = [L1*cos(q1),L1*sin(q1)];
r2 = [L.L1*cos(q1)+L2*cos(q2),L.L1*sin(q1)+L2*sin(q2)];
rw = [L.L1*cos(q1)+L.L2*cos(q2),L.L1*sin(q1)+L.L2*sin(q2)];

M.mL = M.m1 + M.m2 + M.m_mw;


rL = (M.m1*r1+M.m2*r2+M.m_mw*rw)/M.mL;

%求解虚拟摆杆的转动惯量
d1 = norm(r1-rL);
d2 = norm(r2-rL);
d5 = norm(rw-rL);
I1 = I.I1 + M.m1 * d1^2;
I2 = I.I2 + M.m2 * d2^2;
IW = I.Iwm + M.m_mw * d5^2;
%预改
%IW = I.Iwm + M.m_mw * d2^2;%----------------定子部分按绕膝关节(按平行轴定理)

I.Ill = I1 + I2 + IW ;
I.Ilr = I1 + I2 + IW ;


if L.L_l == L.L
    L.Lb_l = norm(rL);
    L.Lw_l = L.L - L.Lb_l;
end
if L.L_r == L.L
   L.Lb_r = norm(rL);
   L.Lw_r = L.L - L.Lb_r;
end
