% 传统的ICP算法
% input: P-目标点云N*3, Q-源点云N*3
% output: R,t-最优旋转平移矩阵,errors-迭代过程的所有误差
function [global_R,global_t,rmses,Q_reg,e_overlap,sigma]=ours_dicp(P,Q,transformation_epsilon)
% 初始化变量
rmses = [];                                                                % 迭代过程中的全部误差
e_overlap = 1;                                                             % 重叠率
sigma = 0;                                                                 % 重叠率估计置信度
R = eye(3);                                                                % 当前旋转平移
t = ones(3,1);

% 备份变量
last_sigma = 0;                                                            % 上次迭代的重叠率估计置信度
last_R = ones(3);   last_t = ones(3,1);                                    % 上次迭代的旋转平移
global_R = eye(3);  global_t = ones(3,1);                                  % 全局旋转平移

% 创建目标点云P的kd-tree
P_kdtree = createns(P,'NSMethod','kdtree');

% 最近邻(ICP)迭代
n = 0;
while 1
    n = n+1;
    % Step1:基于kd_tree的对应点查找, 输入参数1-仅搜索一个邻近点
    % 计算对应点，并得到所有对应点的欧式距离
    [P_idx,D] = knnsearch(P_kdtree,Q,'k',1);
    % 距离排序
    [D_sorted,Q_idx] = sort(D);
    
    % Step4:更新error并判定退出条件
    D_corresponding_points = D_sorted(1:round((e_overlap)*length(D_sorted))); 
    rmses(end+1)=sqrt(mean(D_corresponding_points.^2));                    % 误差被定义为所有对应点的欧式距离均值【也不合理，因为模型存在缩放，应该做适当修正，改成100%的形式，改进的误差取值范围为[0,1]】  
%     d_transformation = complete_dtran(last_R,last_t,R,t);                %【尚未解决的问题：定义变换矩阵变化率】
    d_transformation = 1;
    % 收敛或异常退出
    if d_transformation < transformation_epsilon || n >= 100               % 唯一停止迭代的方法
        if sigma < 0.9                                                     % 使用改进的配准误差定义方式
            disp("配准失败，点云初始位置太差或点云重叠率太低");
        end
        break;
    end
    
    % 重叠率估计及点对筛选
    if abs(last_sigma-sigma)<1e-3 && sigma > 0.90                          % 为加快运算速度，当置信度较高且变化率不大时，不再对重叠率进行估计，而是直接进行随机下降法
        Q_idx = Q_idx(1:round(e_overlap*length(Q_idx)));
        Q_idx_sample = datasample(Q_idx,round(0.01*length(Q_idx)));        % 对重叠范围内的点对进行固定采样率采样
        P_idx = P_idx(Q_idx_sample,:);
        Q_overlap = Q(Q_idx_sample,:); 
        P_closest = P(P_idx,:);
%         Q = Q(Q_idx,:);                                                    % 当以很高的置信度找到正确的重叠率后，不需要搜索所有的点【有效果】
%         e_overlap = 1;
        disp("快速迭代");
    else
        % 重叠率估计，并根据重叠率筛选对应点，得到索引
        last_sigma = sigma;
        [e_overlap,sigma] = estimate_overlap1(D_sorted);                   % 点云重叠率估计,并重新筛选对应点
        e_overlap = 1;
        Q_idx = Q_idx(1:round(e_overlap*length(Q_idx)));
        P_idx = P_idx(Q_idx,:);
        Q_overlap = Q(Q_idx,:);
        P_closest = P(P_idx,:);
    end
    
    % Step2:计算旋转平移矩阵
    % 去中心化
    P_center=mean(P_closest);                                              % P点集的质心点
    Q_center=mean(Q_overlap);                                              % 对应点集的质心点
    P_temp = P_closest-P_center;                                           % 进行去中心化
    Q_temp = Q_overlap-Q_center;
    % SVD分解 【可不可以不使用SVD分解，因为可能太耗时】
    H=Q_temp'*P_temp;                                                      % 计算H矩阵
    [U,~,V]=svd(H); 
    R=V*U';
    t=P_center'-R*Q_center';                                               % 利用质心点求解T参数
    
    % Step3:使用转换参数得到新的点集Q,3*N
    Q = (R*Q'+t)';
    
    % 更新数据
    last_R = R;
    last_t = t;
    
    if n == 1 || n == 2 || n == 5 || n == 10 || n == 20
        figure;
        plot(D_sorted,'^-','color','k');
        title_str =  ['Correspondence points distance curve; Iteration times:',num2str(n)];
        title({title_str;''});
        xlabel('i');
        ylabel('D_i');
        xlim([0,length(D_sorted)]);
        hold on
    end
end
Q_reg = Q;

% 绘制误差曲线、距离曲线
figure;
plot(rmses,'o-');
title("迭代误差曲线");
end
