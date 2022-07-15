function [R,t,rmse] = gicp(P, Q, max_iterations)

% 定义阈值、中间变量
rmse = 0;             % 均方根误差
last_rmse = 0;        % 上一次迭代误差
drmse = 0;            % 定义误差变化率(考虑模型缩放)，这样的停止条件更加鲁棒
min_drmse = 1e-4;        % 误差变化率阈值

% 创建目标点云P的kd_tree
P_kdtree = createns(P,'NSMethod','kdtree'); 

% 计算目标点云和源点云各自的协方差矩阵【不理解为什么称为协方差矩阵】
time0 = tic;
P_normal = pcnormals(pointCloud(P));    % 计算法线
Q_normal = pcnormals(pointCloud(Q));
e = 0.02;
C = [e,0,0;
     0,1,0;
     0,0,1];
Cp = zeros(3,3,length(P));
Cq = zeros(3,3,length(Q));
for n = 1:length(P)
    Rmu = [(P_normal(n,:))',[0,1,0]',[0,0,1]'];
    Cp(:,:,n) = Rmu*C*Rmu';
end
for n = 1:length(Q)
    Rv  = [(Q_normal(n,:))',[0,1,0]',[0,0,1]'];
    Cq(:,:,n) = Rv*C*Rv';
end
time_compute_cov = toc(time0);
fprintf("计算点云法线协方差：%d\n",time_compute_cov);

% 定义损失函数，GICP的关键步骤
    function c = cost(Tp)
        R_cost = quat2dcm(Tp(1:4));    % 将四元数转化为方向余弦矩阵，即旋转矩阵R【这个R是否是正确的？为啥需要一次转置】
        t_cost = Tp(5:7)';
        Qt = (R_cost*Q'+t_cost)';      % 刚体变换【可以之间让Q旋转】
        
        dis = P(P_idx,:) - Qt;   % 计算距离,非欧式距离
        c = 0;
        for j=1:length(P_idx)
            covar_error = Cp(:,:,P_idx(j)) + R_cost*Cq(:,:,j)*R_cost';  % 为什么是R，不是T，对应点的协方差矩阵
            er = dis(j,:) * (covar_error\(dis(j,:))');                       % \:A^-1*B
            c = c + er;
        end
    end

% 查找对应点、基于GICP的损失函数计算最优旋转平移矩阵
time0 = tic;
qt = [1 0 0 0 0 0 0];
for n = 1:max_iterations
    % Step1:基于kd_tree的对应点查找, 输入参数1-仅搜索一个邻近点
    % 计算对应点，并范围所有对应点的欧式距离
    % 【是否一定需要满足：点云数量P>Q，Idx的个数等于Q：可以，当P<Q时，会存在多对一的情况，这种情况如何解决？论文创新点3】
    [P_idx,D] = knnsearch(P_kdtree,Q,'k',1);        % P_idx,当次迭代Q对应的最近点的P_kdtree索引，这里的D是欧式距离
    
    % Step4:更新error【不应该计算所有的点，因为有非重叠点,而且P，Q的点云数量可能不一样】
    rmse = sqrt(mean(D.^2));
    drmse = (last_rmse-rmse)/last_rmse;      % 误差变化率
    
    % 筛选点：去除最远点、根据距离比例去除点(内插比)
    
    
    % Step2:基于GICP的损失函数计算旋转平移矩阵【这里面的参数包括哪些？】
    [qt, ~] = fminsearch(@cost, qt , struct('Display', 'final', 'TolFun', min_drmse, 'TolX',0.1));

    % Step3:使用转换参数得到新的点集Q,3*N
    R = quat2dcm(qt(1:4))';
    t = qt(5:7)';
    Q = (R*Q'+t)';
    
    fprintf("迭代次数：%d,  均方根误差：%d\n",n,rmse);
   
    % 收敛或异常退出
    if drmse < 0
        disp("此次配准误差比上次误差更大或相等，ICP陷入局部最优");            % 如果发生可能的原因有：ICP陷入局部最优
        break;
    elseif drmse<min_drmse
        disp("相对变化率达到收敛条件");
        break;
    elseif rmse==0
        disp("最优配准：error=0");
        break;
    end
end

time_iteration_gicp = toc(time0);
fprintf("gicp迭代耗时：%d\n",time_iteration_gicp);
end