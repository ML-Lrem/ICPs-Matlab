function [R,t,rmse] = gicp(P, Q, max_iterations)

% ������ֵ���м����
rmse = 0;             % ���������
last_rmse = 0;        % ��һ�ε������
drmse = 0;            % �������仯��(����ģ������)��������ֹͣ��������³��
min_drmse = 1e-4;        % ���仯����ֵ

% ����Ŀ�����P��kd_tree
P_kdtree = createns(P,'NSMethod','kdtree'); 

% ����Ŀ����ƺ�Դ���Ƹ��Ե�Э������󡾲����Ϊʲô��ΪЭ�������
time0 = tic;
P_normal = pcnormals(pointCloud(P));    % ���㷨��
Q_normal = pcnormals(pointCloud(Q));
e = 0.02;
C = [e,0,0;
     0,1,0;
     0,0,1];
Cp = zeros(3,3,length(P));
Cq = zeros(3,3,length(Q));
for n = 1:length(P)
    Rmu = [(P_normal(n,:))',[0,1,0]',[0,0,1]'];
    Cp(:,:,n) = Rmu*C*Rmu';
end
for n = 1:length(Q)
    Rv  = [(Q_normal(n,:))',[0,1,0]',[0,0,1]'];
    Cq(:,:,n) = Rv*C*Rv';
end
time_compute_cov = toc(time0);
fprintf("������Ʒ���Э���%d\n",time_compute_cov);

% ������ʧ������GICP�Ĺؼ�����
    function c = cost(Tp)
        R_cost = quat2dcm(Tp(1:4));    % ����Ԫ��ת��Ϊ�������Ҿ��󣬼���ת����R�����R�Ƿ�����ȷ�ģ�Ϊɶ��Ҫһ��ת�á�
        t_cost = Tp(5:7)';
        Qt = (R_cost*Q'+t_cost)';      % ����任������֮����Q��ת��
        
        dis = P(P_idx,:) - Qt;   % �������,��ŷʽ����
        c = 0;
        for j=1:length(P_idx)
            covar_error = Cp(:,:,P_idx(j)) + R_cost*Cq(:,:,j)*R_cost';  % Ϊʲô��R������T����Ӧ���Э�������
            er = dis(j,:) * (covar_error\(dis(j,:))');                       % \:A^-1*B
            c = c + er;
        end
    end

% ���Ҷ�Ӧ�㡢����GICP����ʧ��������������תƽ�ƾ���
time0 = tic;
qt = [1 0 0 0 0 0 0];
for n = 1:max_iterations
    % Step1:����kd_tree�Ķ�Ӧ�����, �������1-������һ���ڽ���
    % �����Ӧ�㣬����Χ���ж�Ӧ���ŷʽ����
    % ���Ƿ�һ����Ҫ���㣺��������P>Q��Idx�ĸ�������Q�����ԣ���P<Qʱ������ڶ��һ����������������ν�������Ĵ��µ�3��
    [P_idx,D] = knnsearch(P_kdtree,Q,'k',1);        % P_idx,���ε���Q��Ӧ��������P_kdtree�����������D��ŷʽ����
    
    % Step4:����error����Ӧ�ü������еĵ㣬��Ϊ�з��ص���,����P��Q�ĵ����������ܲ�һ����
    rmse = sqrt(mean(D.^2));
    drmse = (last_rmse-rmse)/last_rmse;      % ���仯��
    
    % ɸѡ�㣺ȥ����Զ�㡢���ݾ������ȥ����(�ڲ��)
    
    
    % Step2:����GICP����ʧ����������תƽ�ƾ���������Ĳ���������Щ����
    [qt, ~] = fminsearch(@cost, qt , struct('Display', 'final', 'TolFun', min_drmse, 'TolX',0.1));

    % Step3:ʹ��ת�������õ��µĵ㼯Q,3*N
    R = quat2dcm(qt(1:4))';
    t = qt(5:7)';
    Q = (R*Q'+t)';
    
    fprintf("����������%d,  ��������%d\n",n,rmse);
   
    % �������쳣�˳�
    if drmse < 0
        disp("�˴���׼�����ϴ����������ȣ�ICP����ֲ�����");            % ����������ܵ�ԭ���У�ICP����ֲ�����
        break;
    elseif drmse<min_drmse
        disp("��Ա仯�ʴﵽ��������");
        break;
    elseif rmse==0
        disp("������׼��error=0");
        break;
    end
end

time_iteration_gicp = toc(time0);
fprintf("gicp������ʱ��%d\n",time_iteration_gicp);
end