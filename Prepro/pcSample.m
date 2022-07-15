function pc_sample = pcSample(pc,sample_count)
pc_count = length(pc);

if sample_count > pc_count
    sample_count = pc_count;
    disp("原始点云数量太少");
end
% -----------------------均匀采样固定个数的点------------------------------
sample_idx = ceil(0 + (pc_count-0) .* rand(sample_count,1));
% ----------------------根据索引提取点云并保存-----------------------------
pc_sample = pc(sample_idx,:);

end