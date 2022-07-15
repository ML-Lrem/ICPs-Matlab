function [P_prepro,Q_prepro]=pcPrepro(P,Q,sample_count)
% 随机下采样到固定点数
P = pcSample(P,sample_count);
Q = pcSample(Q,sample_count);

% 基于点云关键点提取的局部采样方法【论文创新点2】

P_prepro = P;
Q_prepro = Q;
end