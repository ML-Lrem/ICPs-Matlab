function [inflection_idx,sigma] = find_inflection(y,d1_k)
    
% ������㵽���߶Խ��ߵľ���
Q1 = [1,y(1)];
Q2 = [length(y),y(end)];
for i = 1:length(y)
    P = [i,y(i)];
    d(i) = abs(det([Q2-Q1;P-Q1]))/norm(Q2-Q1);
end

[~,inflection_idx] = max(d);

% �յ����Ŷ�
% �յ�б��
k_inflection = (y(inflection_idx)-y(1))/(inflection_idx-1);

sigma = (d1_k - k_inflection)/d1_k;       % ��k_inflectionԽ�ӽ�0ʱ�����Ŷ�Խ��

end