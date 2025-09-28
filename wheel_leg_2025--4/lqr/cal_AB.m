% 定义符号变量
clear;clc;
syms Rw Rl l_l l_r Lw_l Lw_r Lb_l Lb_r Lc Iw Ill Ilr Ib Iz mw ml mb g
syms s(t) s_b(t) s_l_l(t) s_l_r(t) h_b(t) h_l_l(t) h_l_r(t)
syms theta_w_l(t) theta_w_r(t) theta_l_l(t) theta_l_r(t) theta_b(t) phi(t)
syms F_wh_l F_wh_r F_bh_l F_bh_r F_bs_l F_bs_r F_ws_l F_ws_r f_l f_r
syms T_lw_l T_lw_r T_bl_l T_bl_r  

%定义替换结构体
vM = struct('mw',mw,'mL',ml,'mb',mb,'g',g);
vL = struct('L_l',l_l,'L_r',l_r,'Lw_l',Lw_l,'Lw_r',Lw_r,'Lb_l',Lb_l,'Lb_r',Lb_r,'Lb',Lc,'Rw',Rw,'Rl',Rl);
vI = struct('Iw',Iw,'Ill',Ill,'Ilr',Ilr,'Ib',Ib,'Iz',Iz);

assume(Rw~=0);

% k->kinematics 某公式有l,r之分时k用l或者r代替
% 标号对应上交开源文件公式编号

% 运动学分析公式 
Ek201 = s(t) == Rw/2 * (theta_w_l(t) + theta_w_r(t));
Ek202 = h_b(t) == 1/2 * (l_l * cos(theta_l_l(t)) + l_r * cos(theta_l_r(t)));
El203 = s_l_l(t) == Rw * theta_w_l(t) + Lw_l * sin(theta_l_l(t));
Er203 = s_l_r(t) == Rw * theta_w_r(t) + Lw_r * sin(theta_l_r(t));
El204 = h_l_l(t) == h_b(t) - Lb_l * cos(theta_l_l(t));
Er204 = h_l_r(t) == h_b(t) - Lb_r * cos(theta_l_r(t));
Ek207 = phi(t) == Rw/(2*Rl) * (-theta_w_l(t) + theta_w_r(t)) + ...
    1/(2*Rl) * (-l_l * sin(theta_l_l(t)) + l_r * sin(theta_l_r(t)));
Ek208 = s_b(t) == Rw/2 * (theta_w_l(t) + theta_w_r(t)) + ...
    1/2 * (l_l * sin(theta_l_l(t)) + l_r * sin(theta_l_r(t)));

% 二阶导数
Ek210 = diff(Ek201, t, 2);
Ek212 = diff(Ek207, t, 2);
Ek214 = diff(Ek208, t, 2);
Ek216 = diff(Ek202, t, 2);
El218 = diff(El203, t, 2);
Er218 = diff(Er203, t, 2);
El220 = diff(El204, t, 2);
Er220 = diff(Er204, t, 2);
%为方便化简提前代入
El220 = subs(El220, diff(h_b(t), t, 2), rhs(Ek216));
Er220 = subs(Er220, diff(h_b(t), t, 2), rhs(Ek216));

% d->dynamics 某公式有l,r之分时k用l或者r代替
% 标号对应上交开源文件公式编号

%对驱动轮进行力学分析
El301 = mw * Rw * diff(theta_w_l(t), t, 2) == f_l - F_ws_l;
Er301 = mw * Rw * diff(theta_w_r(t), t, 2) == f_r - F_ws_r;
El302 = Iw * diff(theta_w_l(t), t, 2) == T_lw_l - f_l * Rw;
Er302 = Iw * diff(theta_w_r(t), t, 2) == T_lw_r - f_r * Rw;

%对等效腿部摆杆进行力学分析
El303 = ml * diff(s_l_l(t), t, 2) == F_ws_l - F_bs_l;
Er303 = ml * diff(s_l_r(t), t, 2) == F_ws_r - F_bs_r;
El304 = ml * diff(h_l_l(t), t, 2) == F_wh_l - F_bh_l - ml * g;
Er304 = ml * diff(h_l_r(t), t, 2) == F_wh_r - F_bh_r - ml * g;
El305 = Ill * diff(theta_l_l(t), t, 2) == ...
    (F_wh_l * Lw_l + F_bh_l * Lb_l) * sin(theta_l_l(t)) - ...
    (F_ws_l * Lw_l + F_bs_l * Lb_l) * cos(theta_l_l(t)) - T_lw_l + T_bl_l;
Er305 = Ilr * diff(theta_l_r(t), t, 2) == ...
    (F_wh_r * Lw_r + F_bh_r * Lb_r) * sin(theta_l_r(t)) - ...
    (F_ws_r * Lw_r + F_bs_r * Lb_r) * cos(theta_l_r(t)) - T_lw_r + T_bl_r;

% 对机体进行力学分析
Ed306 = mb * diff(s_b(t), t, 2) == F_bs_l + F_bs_r;
Ed307 = mb * diff(h_b(t), t, 2) == F_bh_l + F_bh_r - mb * g;
Ed308 = Ib * diff(theta_b(t), t, 2) == -(T_bl_l + T_bl_r) - ...
    (F_bs_l + F_bs_r) * Lc * cos(theta_b(t)) + ...
    (F_bh_l + F_bh_r) * Lc * sin(theta_b(t));
Ed309 = Iz * diff(phi(t), t, 2) == (-f_l + f_r) * Rl;
Ed310 = F_wh_l == F_wh_r;

%解出10个约束力
sol= solve([El302, Er302, El301, Er301, El303, Er303, El304, Er304, Ed307, Ed310],...
[f_l, f_r, F_ws_l, F_ws_r, F_bs_l, F_bs_r, F_wh_l, F_wh_r, F_bh_l, F_bh_r]);

% 定义中间变量用于替换
syms dl dr

% 解方程组 temp1 和 temp2
[temp1, temp2] = solve([...
    subs(Ek210, [diff(theta_w_l(t), t, 2), diff(theta_w_r(t), t, 2)], [dl, dr]), ...
    subs(Ek212, [diff(theta_w_l(t), t, 2), diff(theta_w_r(t), t, 2)], [dl, dr])], [dl, dr]);

% 定义符号变量的替换向量 v1 和 v2
v1 = [f_l, f_r, F_ws_l, F_ws_r, F_bs_l, F_bs_r, F_wh_l, F_wh_r, F_bh_l, F_bh_r];
v2 = [sol.f_l, sol.f_r, sol.F_ws_l, sol.F_ws_r, sol.F_bs_l, sol.F_bs_r, sol.F_wh_l, sol.F_wh_r, sol.F_bh_l, sol.F_bh_r];

% 定义二阶导数的替换向量 v3 和 v4
v3 = [diff(s_b(t), t, 2), diff(h_b(t), t, 2), diff(s_l_l(t), t, 2), diff(s_l_r(t), t, 2), ...
      diff(h_l_l(t), t, 2), diff(h_l_r(t), t, 2)];
v4 = [rhs(Ek214), rhs(Ek216), rhs(El218), rhs(Er218), rhs(El220), rhs(Er220)];

% 小角度近似替换
v51 = [cos(theta_l_l(t)), cos(theta_l_r(t)), sin(theta_l_l(t)), sin(theta_l_r(t)), ...
       diff(theta_l_l(t), t)^2, diff(theta_l_r(t), t)^2, theta_l_l(t) * theta_l_r(t), ...
       theta_l_l(t)^2, theta_l_r(t)^2];
v61 = [1, 1, theta_l_l(t), theta_l_r(t), 0, 0, 0, 0, 0];

v52 = [diff(theta_l_l(t), t)^2, diff(theta_l_r(t), t)^2, cos(theta_l_l(t)), cos(theta_l_r(t)), ...
       sin(theta_l_l(t)), sin(theta_l_r(t)), sin(theta_b(t)), cos(theta_b(t))];
v62 = [0, 0, 1, 1, 0, 0, theta_b(t), 1];

% 状态变量替换
v7 = [diff(theta_w_l(t), t, 2), diff(theta_w_r(t), t, 2)];
v8 = [temp1, temp2];

% 线性化变量替换
syms x2_dot x4_dot x6_dot x8_dot x10_dot x1 x2 x3 x4 x5 x6 x7 x8 x9 x10
v9 = [diff(s(t), t, 2), diff(phi(t), t, 2), diff(theta_l_l(t), t, 2), diff(theta_l_r(t), t, 2), ...
      diff(theta_b(t), t, 2), s(t), diff(s(t), t), phi(t), diff(phi(t), t), ...
      theta_l_l(t), diff(theta_l_l(t), t), theta_l_r(t), diff(theta_l_r(t), t), ...
      theta_b(t), diff(theta_b(t), t)];
v10 = [x2_dot, x4_dot, x6_dot, x8_dot, x10_dot, x1, x2, x3, x4, x5, x6, x7, x8, x9, x10];

%级联替换
E1 = simplify(subs(subs(subs(subs(simplify(subs(subs(El305, v1, v2), v3, v4)), v51, v61), v51, v61), v7, v8), v9, v10));
E2 = simplify(subs(subs(subs(subs(simplify(subs(subs(Er305, v1, v2), v3, v4)), v51, v61), v51, v61), v7, v8), v9, v10));
E3 = simplify(subs(subs(subs(simplify(subs(subs(Ed306, v1, v2), v3, v4)), v51, v61), v7, v8), v9, v10));
E4 = simplify(subs(subs(subs(simplify(subs(subs(Ed308, v1, v2), v3, v4)), v52, v62), v7, v8), v9, v10));
E5 = simplify(subs(subs(subs(simplify(subs(subs(Ed309, v1, v2), v3, v4)), v52, v62), v7, v8), v9, v10));

% 解方程组得到 x2_dot, x4_dot, x6_dot, x8_dot, x10_dot
[x2_dot, x4_dot, x6_dot, x8_dot, x10_dot] = solve([E1, E2, E3, E4, E5], [x2_dot, x4_dot, x6_dot, x8_dot, x10_dot]);


% 在平衡点处线性化
JA = jacobian([x2, x2_dot, x4, x4_dot, x6, x6_dot, x8, x8_dot, x10, x10_dot],...
              [x1, x2, x3, x4, x5, x6, x7, x8, x9, x10]);

JB = jacobian([x2, x2_dot, x4, x4_dot, x6, x6_dot, x8, x8_dot, x10, x10_dot],...
              [T_lw_l, T_lw_r, T_bl_l, T_bl_r]);
        
A = simplify(subs(JA, [x1, x2, x3, x4, x5, x6, x7, x8, x9, x10], [0,0,0,0,0,0,0,0,0,0]));
B = simplify(subs(JB, [x1, x2, x3, x4, x5, x6, x7, x8, x9, x10], [0,0,0,0,0,0,0,0,0,0]));

save('cal_AB.mat', 'A', 'B', 'vM', 'vL', 'vI');
