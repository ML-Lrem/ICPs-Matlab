function [e_overlap,sigma] = estimate_overlap1(D)

N = length(D);

% ��ȡ�յ㣬���õ����Ŷ�
% ������㵽���߶Խ��ߵľ���
Q1 = [1,D(1)];
Q2 = [length(D),D(end)];

% ���ַ����Ҽ���ֵ��
eps = 100;                                                                 % �ֶ�λ�������á�epsԽ���ھ���λ�׶���Ҫ���ҵ����ݾ�Խ�࣬�ٶ�Ҳ��Խ����epsԽС���Ծ������ߵľֲ�ͻ��Ƚ����У���λ���徫�Ȼ��½�
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
        for idx = idx_left:idx_right                                       % ȡ��ֵ��������б�������
            P = [idx,D(idx)];
            h(idx) = abs(det([Q2-Q1;P-Q1]))/norm(Q2-Q1);
        end
        break;
    end
end
[~,R] = max(h);

% �������Ŷ�
kn = (D(end)-D(1))/(N-1);       % �˵�б��
kr = (D(R)-D(1))/(R-1);         % �յ�б��

% ����δ���������(1)����100%�ص����Ƶı�ʶ��������(2)��ʵ��ͬһ�����⣬ֻҪsigma<0.5�������ص���100%��
sigma = (kn*2 - kr)/(2*kn);       % sigma�����Ŷȣ�sigma-[0,1]
e_overlap = R/N * sigma;          % �ص��ʹ���

fprintf("�˵�б�ʣ�%d\t",kn);
fprintf("�յ�б�ʣ�%d\t",kr);
fprintf("�����ص��ʣ�%d\t",e_overlap)
fprintf("���Ŷȣ�%d\n",sigma);
end