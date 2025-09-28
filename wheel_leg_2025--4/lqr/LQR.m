clear; clc;
load('cal_AB.mat', 'A', 'B', 'vM', 'vL', 'vI');
M = struct('mw', 0, 'mL', 0, 'mb', 0, 'g', 0, 'm1', 0, 'm2', 0, 'm3', 0, 'm4', 0, 'm_mw', 0);
I = struct('Iw', 0, 'Ill', 0, 'Ilr', 0, 'Ib', 0, 'Iz', 0, 'I1', 0, 'I2', 0, 'I3', 0, 'I4', 0, 'Iwm', 0);
L = struct( 'L_l', 0, 'L_r', 0, 'Lw_l', 0, 'Lw_r', 0, 'Lb_l', 0, 'Lb_r', 0, 'Lb', 0, 'Rw', 0, 'Rl', 0, ...
             'L1', 0, 'L2', 0, 'L3', 0, 'L4', 0, 'L5', 0, 'L', 0);         
% Q = diag([2500, 2500, 5000, 1200, 60000, 200, 60000, 200, 60000, 400]);
% R = diag([350, 350, 15, 15]);%0.24 I.Ib = 0.197272;%----------------远古

% Q = diag([2000, 2000, 20000, 1000, 40000, 100, 40000, 100, 80000, 100]);
% R = diag([180, 180, 25.5, 25.5]);%0.22 22调这个QR
    








% 腿长：0.18m
Q = diag([1300, 4000 , 8000, 200,    10000 , 700, 10000 , 700,    20000, 1000]);
R = diag([230, 230, 30, 30]);


[L.L, L.L_l, L.L_r] = deal(0.18);            % 0.20m 腿长
[M, I, L] = data_init(M, I, L);              % 参数初始化
[M, I, L] = VMC_inverse_resolve(M, L, I, 90);% VMC 逆解算

% 将 M, I, L 转换为 cell 数组，用于替换符号变量
sM = struct2cell(M);sL = struct2cell(L);sI = struct2cell(I);

% 将符号矩阵 A 和 B 中的符号变量替换为对应的数值
A_numeric = (double(vpa(subs(A, [struct2cell(vM); struct2cell(vL); struct2cell(vI)], [sM(1:4); sL(1:9); sI(1:5)]))));
B_numeric = (double(vpa(subs(B, [struct2cell(vM); struct2cell(vL); struct2cell(vI)], [sM(1:4); sL(1:9); sI(1:5)]))));

% 使用 LQR 计算增益矩阵 K
K = lqrd(A_numeric, B_numeric, Q, R, 0.0001);%腿长拟合见data_init

% 简洁输出目标格式并换行
fprintf('{%s\n};', strjoin(arrayfun(@(i) ...
sprintf('{%s}', strjoin(string(K(i, :)), ', ')), 1:size(K, 1), 'UniformOutput', false), ',\n'));
%state_predict
% P = state_predict(A_numeric, B_numeric, L.Rw, L.Rl, 0.002);
