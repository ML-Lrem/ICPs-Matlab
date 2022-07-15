function [Q_idx,e_overlap,D_sorted,sigma] = estimate_overlap(D,d_means)

% 低重叠率、初始位置不佳的情况，收敛性不好：40%

% 初始化
[D_sorted,Q_idx] = sort(D);
all_point_count = length(D_sorted);                    

if isempty(d_means)     % 去除初始点             
    e_overlap = 1;
    sigma = 0;
else
    [inflection_idx,sigma] = find_inflection(D_sorted,d_means(1));
    if sigma <= 0.5     % 【重叠率较高的点云无法准确估计出重叠率】
        % 迭代第一阶段：摸索【标识出迭代次数】【随机重叠率，无法保证收敛】
%         e_overlap = sigma + (1-sigma)*rand(1);
        e_overlap = 1;
        sigma = 0;
        fprintf("随机重叠率：%d\t",e_overlap)
        fprintf("置信度：%d\n",sigma);
    else
        % 迭代第二阶段：精确定位【可优化】
        % % 重叠率估计
%     % 当对应点中，“估计内点”的数量占总对应点的p%以上后，再开始查找拐点
%     % 这样做的原因是：迭代之初，对应点大部分都可能是错误匹配的，点距曲线趋于平缓，拐点不明显【这个应该要识别出来，加入估计重叠率置信度后，可以不管】
%     % 定义估计内点EIP：点距小于初次平均点距的所有点【使用哪个点距更好，平均点距or均方根误差？为什么？使用初次数据还是上次数据？】
%     eip_count = sum(D_sorted < d_means(1));                % “估计内点”数量，正式进入快速迭代阶段时的第一个平均点距
%     eip_p = eip_count/all_point_count;                     % “估计内点”比例【这个可能不叫估计内点，只能说点距中有大于该值的，那么重叠率必然不可能为1】
%     
%     if eip_p == 1
%     %   disp("估计重叠率：100%");
%         e_overlap = 1;
%         sigma = 1;
%     else
%     %   disp("寻找拐点");    % 【这个函数还是可以再优化】
%         [inflection_idx,sigma] = find_inflection(D_sorted);
%         e_overlap = sigma * inflection_idx/all_point_count;               % K：拐点的置信度，K-[0,1]【创新思想1：估计置信度】
%         Q_idx = Q_idx(1:round(e_overlap*all_point_count));                % 取出内点
%     end
        e_overlap = sigma * inflection_idx/all_point_count;               % K：拐点的置信度，K-[0,1]【创新思想1：估计置信度】
        Q_idx = Q_idx(1:round(e_overlap*all_point_count));                % 取出内点
    end
    fprintf("估计重叠率：%d\t",e_overlap)
    fprintf("置信度：%d\n",sigma);
end

end