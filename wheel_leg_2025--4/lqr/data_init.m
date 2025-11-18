function [M,I,L] = data_init(M,I,L)
M.m1 = 0.404;%单位：Kg
M.m2 = 0.406;
M.mw = 0.515;                            %轮毂电机转子
M.m_mw = 0.130;                          %轮毂电机定子
M.mb  = 18.548;
M.g   = 9.81;
I.I1 =  0.009132;                        %大腿转动惯量 %单位：Kg * m^2
I.I2 =  0.010229;                        %小腿转动惯量
I.Iwm = 0.000053;                        %驱动轮定子转动惯量(绕膝关节)
% I.Iw = 0.004301;                       %驱动轮转子转动惯量
I.Iw = 0.001094;                         %驱动轮转子转动惯量
%I.Ib = 0.2997272;                       %pitch转动惯量
I.Ib = 0.197272;
I.Iz = 0.3227;                           %整车绕Z轴转动惯量
L.Rw = 0.065;                            %单位：m
L.Rl = 0.240;                            %两轮间距/2
L.L1 = 0.250;
L.L2 = 0.300;
L.Lb = 0.01;                             %机体质心到髋关节距离

% % 腿长拟合 (直接cv即可)------->待改进腿长拟合精度
% K_all = zeros(400,40);L_all = zeros(400,2);K_give = zeros(40,9);
% for i=1:20
%     for j=1:20
%         [M,I,L] = VMC_inverse_resolve(M,L,I,90);%VMC逆解算
%         sM = struct2cell(M);sL = struct2cell(L);sI = struct2cell(I);
%         A_numeric = sparse(double(vpa(subs(A, [struct2cell(vM); struct2cell(vL); struct2cell(vI)], [sM(1:4); sL(1:9); sI(1:5)]))));
%         B_numeric = sparse(double(vpa(subs(B, [struct2cell(vM); struct2cell(vL); struct2cell(vI)], [sM(1:4); sL(1:9); sI(1:5)]))));
%         K=lqrd(A_numeric,B_numeric,Q,R,0.0001);
%         Kt = K';
%         K_all((i-1)*20+j,:) = Kt(:);
%         L_all((i-1)*20+j,:) = [L.L_l, L.L_r];
%         [L.L, L.L_l] = deal(L.L_l +0.01);
%     end
%     [L.L, L.L_r] = deal(L.L_r +0.01);
%     [M,I,L] = VMC_inverse_resolve(M,L,I,90);        
%     [L.L, L.L_l] = deal(0.10);
% end
% 
% for i=1:40
%     f=fit(L_all,K_all(:,i),'poly22');
%     K_give(i,:)=[f.p00,f.p01,f.p02,f.p10,f.p11,0,f.p20,0,0]; 
% end
% disp(round(vpa(K_give),4))



