close all;
clc;
clear;

% ��ICP�㷨��һЩ�ɻ�
% Ϊʲô��ͬ�ĵ��ƣ����ڽ������Ķ�Ӧ��ĵ�಻ȫΪ�㣺��Ϊ���������²����ͻ�����޷�ƥ��ĵ�
% Ϊʲô�Լ�д��icp���򣬶�100%���Ƶ���׼Ч��������
 
% ��Ӱ���Ŀ¼
addpath(genpath('.\Prepro'));
addpath(genpath('.\PPF'));
addpath(genpath('.\PP_ICP'));
addpath(genpath('.\GICP'));
addpath(genpath('.\NICP'));
addpath(genpath('.\PL_ICP'));
addpath(genpath('.\Ours'));
addpath(genpath('.\0Data'));

% ����ȫ�ֻ�ͼ��ʽ
set(groot,'defaultAxesFontName','Times New Roman')
set(groot,'defaultAxesFontSize',24)
set(groot,'defaultFigurePosition',[200 200 1000 800])

% �������txt��ʽ
% ���ݼ�
P = pcread2(".\0Data\100\Armadillo_P_Little.txt");  % Ŀ�����
Q = pcread2(".\0Data\100\Armadillo_Q.txt");  % Դ����
% ���Լ�
% P = pcread2(".\0Data\test\dragon_P_Little.txt"); % Ŀ�����
% Q = pcread2(".\0Data\test\dragon_Q.txt"); % Դ����

% ������ֵ
transformation_epsilon = 1e-8;      % ����ֹͣ�������任����仯��
sample_count = 5000;                % ��������

% ����Ԥ����
time1 = tic;
[P_prepro,Q_prepro] = pcPrepro(P,Q,sample_count);
time_prepro = toc(time1);

% ���Ƴ���׼���õ��任����

% Our's Method
time0 = tic;
disp("Our's Method");
[R,t,rmses,Q_reg,e_overlap,sigma] = ours_dicp(P_prepro,Q_prepro,transformation_epsilon);
time_icp = toc(time0);
% ���Ʊ任
% time1 = tic;
% Q_reg = (R*Q'+t)';
% time_Rt = toc(time1);

% ��ʾ���ú����
figure;
pcshowpair(pointCloud(Q),pointCloud(P));%����һ�����ӻ����������������֮��Ĳ��졣
title('���Ƴ�ʼλ��')
figure;
pcshowpair(pointCloud(Q_reg),pointCloud(P));%����һ�����ӻ����������������֮��Ĳ��졣
title('������׼���λ��')

% �������ʱ
fprintf("ICP�ܺ�ʱ��%d\n",time_icp);
fprintf("����������%d\n",length(rmses));
fprintf("�����ص��ʣ�%2.2f%%\n", e_overlap*100);
fprintf("���Ŷȣ�%d\n",sigma);
fprintf("��׼��%d\n",rmses(end));

% �洢����
Q_reg = double(Q_reg);
P = double(P);
save 'Q_reg.txt' -ascii Q_reg;
save 'P.txt' -ascii P;
disp('���ƴ洢�ɹ�')
