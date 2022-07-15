function [inflection_idx,sigma] = find_inflection(y,d1_k)
    
% 计算各点到曲线对角线的距离
Q1 = [1,y(1)];
Q2 = [length(y),y(end)];
for i = 1:length(y)
    P = [i,y(i)];
    d(i) = abs(det([Q2-Q1;P-Q1]))/norm(Q2-Q1);
end

[~,inflection_idx] = max(d);

% 拐点置信度
% 拐点斜率
k_inflection = (y(inflection_idx)-y(1))/(inflection_idx-1);

sigma = (d1_k - k_inflection)/d1_k;       % 当k_inflection越接近0时，置信度越高

end