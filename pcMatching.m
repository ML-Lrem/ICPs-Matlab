close all;
clc;
clear;

% 对ICP算法的一些疑惑
% 为什么相同的点云，最邻近搜索的对应点的点距不全为零：因为如果是随机下采样就会产生无法匹配的点
% 为什么自己写的icp程序，对100%点云的配准效果并不好
 
% 添加包含目录
addpath(genpath('.\Prepro'));
addpath(genpath('.\PPF'));
addpath(genpath('.\PP_ICP'));
addpath(genpath('.\GICP'));
addpath(genpath('.\NICP'));
addpath(genpath('.\PL_ICP'));
addpath(genpath('.\Ours'));
addpath(genpath('.\0Data'));

% 设置全局绘图样式
set(groot,'defaultAxesFontName','Times New Roman')
set(groot,'defaultAxesFontSize',24)
set(groot,'defaultFigurePosition',[200 200 1000 800])

% 导入点云txt格式
% 数据集
P = pcread2(".\0Data\100\Armadillo_P_Little.txt");  % 目标点云
Q = pcread2(".\0Data\100\Armadillo_Q.txt");  % 源点云
% 测试集
% P = pcread2(".\0Data\test\dragon_P_Little.txt"); % 目标点云
% Q = pcread2(".\0Data\test\dragon_Q.txt"); % 源点云

% 设置阈值
transformation_epsilon = 1e-8;      % 迭代停止条件：变换矩阵变化率
sample_count = 5000;                % 采样点数

% 点云预处理
time1 = tic;
[P_prepro,Q_prepro] = pcPrepro(P,Q,sample_count);
time_prepro = toc(time1);

% 点云初配准【得到变换矩阵】

% Our's Method
time0 = tic;
disp("Our's Method");
[R,t,rmses,Q_reg,e_overlap,sigma] = ours_dicp(P_prepro,Q_prepro,transformation_epsilon);
time_icp = toc(time0);
% 点云变换
% time1 = tic;
% Q_reg = (R*Q'+t)';
% time_Rt = toc(time1);

% 显示配置后点云
figure;
pcshowpair(pointCloud(Q),pointCloud(P));%创建一个可视化描述两个输入点云之间的差异。
title('点云初始位置')
figure;
pcshowpair(pointCloud(Q_reg),pointCloud(P));%创建一个可视化描述两个输入点云之间的差异。
title('点云配准后的位置')

% 各步骤耗时
fprintf("ICP总耗时：%d\n",time_icp);
fprintf("迭代次数：%d\n",length(rmses));
fprintf("估计重叠率：%2.2f%%\n", e_overlap*100);
fprintf("置信度：%d\n",sigma);
fprintf("配准误差：%d\n",rmses(end));

% 存储点云
Q_reg = double(Q_reg);
P = double(P);
save 'Q_reg.txt' -ascii Q_reg;
save 'P.txt' -ascii P;
disp('点云存储成功')
