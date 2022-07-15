function [R1,t,mse_profile] = gicp_github(P, Q, varargin)
% Generalized ICP based on:
% [1] Generalized-ICP
%     Segal, A. and Haehnel, D. and Thrun, 
%     S.?Proc. of Robotics: Science and Systems (RSS) ? ? (2009)
% model B P, frame A Q
P_n = length(P);
Q_n = length(Q);
P_normal = pcnormals(pointCloud(P));
Q_normal = pcnormals(pointCloud(Q));

P = P';
Q = Q';
P_normal = P_normal';
Q_normal = Q_normal';

p = inputParser;
p.addParameter('MaxIterations', 40, @(x)isnumeric(x));% 【阈值1】
p.addParameter('MSEThreshold',  1e-6, @(x)isnumeric(x));% 【阈值2】
p.addParameter('MaxCorrespondenceDist',20, @(x)isnumeric(x));% 【阈值3】
p.addParameter('OverlapFraction', 1);     % 【重叠率阈值4】【可优化的地方】
p.addParameter('CovarEpsilon',  0.01, @(x)isnumeric(x));        % 【这是什么参数？归一化参数？法线的协方差分布】
p.addParameter('Method', 'icp', @(x)strcmpi(x,'gicp') || strcmpi(x,'wsm') || strcmpi(x,'icp'));
p.addParameter('Verbose', true);
p.parse(varargin{:});

maxiter = p.Results.MaxIterations;
msethresh = p.Results.MSEThreshold;
d_max = p.Results.MaxCorrespondenceDist;
e = p.Results.CovarEpsilon;
verbose = p.Results.Verbose;
method = p.Results.Method;
overlap = p.Results.OverlapFraction;

if P_n > 10000 || Q_n > 10000
    fprintf('It is probably not wise to attempt ICP with %dx%d points',P_n, Q_n)
end

use_covars = strcmpi(method, 'gicp') || strcmpi(method, 'wsm');

if use_covars
    % Precompute covariances ( see [1] Section III.B )
    C = [e 0 0;
         0 1 0;
         0 0 1];
    C_p = zeros(3,3,Q_n);               % P点云表面法线的协方差矩
    C_q = zeros(3,3,Q_n);
    for i=1:Q_n                         % 【是否一定要求P_n>Q_n】
        Rmu = [P_normal(:,i),[0 1 0]',[0 0 1]'];   % 【可能存在顺序问题】
        Rnu = [Q_normal(:,i) [0 1 0]' [0 0 1]'];
        C_p(:,:,i) = Rmu * C * Rmu';
        C_q(:,:,i) = Rnu * C * Rnu';
    end
end

% Octree for model data P
P_kdtree = createns(P','NSMethod','kdtree');

% The cost function we'll try to minimize: (2) in [1]   【这个才是GICP的核心】
    function c = cost(Tp)
        R = quat2dcm(Tp(1:4))';    % 将四元数转化为方向余弦矩阵，即旋转矩阵R
        T = [R Tp(5:7)'; 0 0 0 1]; % 旋转平移矩阵 
        At = T * [Q(:,Q_idx); ones(1,size(Q_idx,2))];    % 刚体变换
        At = At(1:3, :);
        
        D = P(:,P_idx) - At;   % 计算距离,非欧式距离
        c = 0;
        for j=1:size(P_idx,2)
            covar_error = C_p(:,:,P_idx(j)) + R*C_q(:,:,Q_idx(j))*R';  % 为什么是R，不是T
            er = D(:,j)' * (covar_error\D(:,j));                       % \:A^-1*B
            c = c + er;
        end
    end

% The transformation, T, as it's called in [1].
% This is [q(1) q(2) q(3) q(4) t(1) t(2) t(3)]

qt = [1 0 0 0 0 0 0];

% Run the ICP loop until difference in MSE is < msethresh

last_rmse = inf;
mse_profile = [];


for iter = 1:maxiter
    P_idx = 1:size(P,2); % B used indexes (so we can filter)
    Q_idx = 1:size(Q,2); % A used indexes 
    
    % 刚体变换 AT = rigid_transform(qt, A);
    R1 = quat2dcm(qt(1:4))'; % 【是否需要转置？】 
    t = qt(5:7)';
    QT = R1*Q+t;
    
    
    % Find closest points
    [P_idx,D] = knnsearch(P_kdtree,QT','k',1);
    P_idx = P_idx';
    D = D';
    
    d = D.^2;   % 误差平方
    rmse = sqrt(mean(d));
    mse_profile(iter) = rmse; %#ok<AGROW>
    
    % Break if done
    if maxiter == 1 || strcmpi(method,'icp') && (abs(rmse - last_rmse) < msethresh)
        break
    end
        
    if verbose
        fprintf('Iter %d MSE %g\n',iter, rmse);
    end
    
    % Filter the percentage of closest points【可优化的点】
    if overlap < 1   % 重叠率小于1
        [~,idx] = sort(d);
        idx = idx(1:round(numel(idx)*overlap));
        Q_idx = Q_idx(idx);
        P_idx = P_idx(idx);
        fprintf('Filtering for %.0f%% overlap. %d points left. (Cutoff at %.6f)\n',overlap*100,numel(idx),d(idx(end)));
    %Filter out non-correspondences according to max distance.
    elseif d_max < inf  % 去除远点,d_max是最大允许的距离
%         mask = d < d_max;
        mask = D < d_max;
        Q_idx = Q_idx(mask);
        P_idx = P_idx(mask);
        fprintf('Filtering for d_max %.5f. %d points left.\n',d_max,sum(mask));
    end

    % Compute the new transformation
    % As [1] mentions, there is no closed-form solution anymore
    % So we use fminsearch
    if (strcmpi(method, 'gicp'))
        [qt, ~] = fminsearch(@cost, qt , struct('Display', 'final', 'TolFun', msethresh*100, 'TolX',0.1));
    elseif (strcmpi(method, 'icp'))
        disp("不支持icp");
    end
    last_rmse = rmse;
end
R1 = R1';
disp(qt);
end