% 传统的ICP算法
% input: P-目标点云N*3, Q-源点云3*N【或许改成N*3会好些】
% output: R,t-最优旋转平移矩阵,errors-迭代过程的所有误差
function [R,t,rmse]=pp_icp(P,Q,max_iterations)
    [tform,~,rmse] = pcregistericp(pointCloud(Q),pointCloud(P),...
    'Extrapolate',true,...                % 是否使用外推法进行优化
    'MaxIterations',max_iterations,...    % 最大迭代次数
    'Metric','pointToPoint');             % 点到点的ICP【传统icp算法】
    R = (tform.T(1:3,1:3))';
    t = (tform.T(4,1:3))';
end
