function [e_overlap,sigma] = estimate_overlap1(D)

N = length(D);

% 获取拐点，并得到置信度
% 计算各点到曲线对角线的距离
Q1 = [1,D(1)];
Q2 = [length(D),D(end)];

% 三分法查找极大值点
eps = 100;                                                                 % 粗定位精度设置。eps越大，在精定位阶段需要查找的数据就越多，速度也就越慢，eps越小，对距离曲线的局部突变比较敏感，定位总体精度会下降
idx_left = 1;
idx_right = length(D);
while 1
    idx_mid = round((idx_left + idx_right)/2);
    P_left = [idx_mid-eps,D(idx_mid-eps)];
    P_right = [idx_mid+eps,D(idx_mid+eps)];
    h_left = abs(det([Q2-Q1;P_left-Q1]))/norm(Q2-Q1);
    h_right = abs(det([Q2-Q1;P_right-Q1]))/norm(Q2-Q1);
    if h_left < h_right
        idx_left = idx_mid;
    else
        idx_right = idx_mid;
    end
    if (idx_right - idx_left) < eps
        for idx = idx_left:idx_right                                       % 取峰值的领域进行遍历查找
            P = [idx,D(idx)];
            h(idx) = abs(det([Q2-Q1;P-Q1]))/norm(Q2-Q1);
        end
        break;
    end
end
[~,R] = max(h);

% 定义置信度
kn = (D(end)-D(1))/(N-1);       % 端点斜率
kr = (D(R)-D(1))/(R-1);         % 拐点斜率

% 【尚未解决的问题(1)：对100%重叠点云的辨识，和问题(2)其实是同一个问题，只要sigma<0.5都当作重叠率100%】
sigma = (kn*2 - kr)/(2*kn);       % sigma：置信度，sigma-[0,1]
e_overlap = R/N * sigma;          % 重叠率估计

fprintf("端点斜率：%d\t",kn);
fprintf("拐点斜率：%d\t",kr);
fprintf("估计重叠率：%d\t",e_overlap)
fprintf("置信度：%d\n",sigma);
end