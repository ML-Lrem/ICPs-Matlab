function pc_sample = pcSample(pc,sample_count)
pc_count = length(pc);

if sample_count > pc_count
    sample_count = pc_count;
    disp("ԭʼ��������̫��");
end
% -----------------------���Ȳ����̶������ĵ�------------------------------
sample_idx = ceil(0 + (pc_count-0) .* rand(sample_count,1));
% ----------------------����������ȡ���Ʋ�����-----------------------------
pc_sample = pc(sample_idx,:);

end