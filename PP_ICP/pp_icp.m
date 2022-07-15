% ��ͳ��ICP�㷨
% input: P-Ŀ�����N*3, Q-Դ����3*N������ĳ�N*3���Щ��
% output: R,t-������תƽ�ƾ���,errors-�������̵��������
function [R,t,rmse]=pp_icp(P,Q,max_iterations)
    [tform,~,rmse] = pcregistericp(pointCloud(Q),pointCloud(P),...
    'Extrapolate',true,...                % �Ƿ�ʹ�����Ʒ������Ż�
    'MaxIterations',max_iterations,...    % ����������
    'Metric','pointToPoint');             % �㵽���ICP����ͳicp�㷨��
    R = (tform.T(1:3,1:3))';
    t = (tform.T(4,1:3))';
end
