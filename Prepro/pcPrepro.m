function [P_prepro,Q_prepro]=pcPrepro(P,Q,sample_count)
% ����²������̶�����
P = pcSample(P,sample_count);
Q = pcSample(Q,sample_count);

% ���ڵ��ƹؼ�����ȡ�ľֲ��������������Ĵ��µ�2��

P_prepro = P;
Q_prepro = Q;
end