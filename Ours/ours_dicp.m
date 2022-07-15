% ��ͳ��ICP�㷨
% input: P-Ŀ�����N*3, Q-Դ����N*3
% output: R,t-������תƽ�ƾ���,errors-�������̵��������
function [global_R,global_t,rmses,Q_reg,e_overlap,sigma]=ours_dicp(P,Q,transformation_epsilon)
% ��ʼ������
rmses = [];                                                                % ���������е�ȫ�����
e_overlap = 1;                                                             % �ص���
sigma = 0;                                                                 % �ص��ʹ������Ŷ�
R = eye(3);                                                                % ��ǰ��תƽ��
t = ones(3,1);

% ���ݱ���
last_sigma = 0;                                                            % �ϴε������ص��ʹ������Ŷ�
last_R = ones(3);   last_t = ones(3,1);                                    % �ϴε�������תƽ��
global_R = eye(3);  global_t = ones(3,1);                                  % ȫ����תƽ��

% ����Ŀ�����P��kd-tree
P_kdtree = createns(P,'NSMethod','kdtree');

% �����(ICP)����
n = 0;
while 1
    n = n+1;
    % Step1:����kd_tree�Ķ�Ӧ�����, �������1-������һ���ڽ���
    % �����Ӧ�㣬���õ����ж�Ӧ���ŷʽ����
    [P_idx,D] = knnsearch(P_kdtree,Q,'k',1);
    % ��������
    [D_sorted,Q_idx] = sort(D);
    
    % Step4:����error���ж��˳�����
    D_corresponding_points = D_sorted(1:round((e_overlap)*length(D_sorted))); 
    rmses(end+1)=sqrt(mean(D_corresponding_points.^2));                    % ������Ϊ���ж�Ӧ���ŷʽ�����ֵ��Ҳ��������Ϊģ�ʹ������ţ�Ӧ�����ʵ��������ĳ�100%����ʽ���Ľ������ȡֵ��ΧΪ[0,1]��  
%     d_transformation = complete_dtran(last_R,last_t,R,t);                %����δ��������⣺����任����仯�ʡ�
    d_transformation = 1;
    % �������쳣�˳�
    if d_transformation < transformation_epsilon || n >= 100               % Ψһֹͣ�����ķ���
        if sigma < 0.9                                                     % ʹ�øĽ�����׼���巽ʽ
            disp("��׼ʧ�ܣ����Ƴ�ʼλ��̫�������ص���̫��");
        end
        break;
    end
    
    % �ص��ʹ��Ƽ����ɸѡ
    if abs(last_sigma-sigma)<1e-3 && sigma > 0.90                          % Ϊ�ӿ������ٶȣ������ŶȽϸ��ұ仯�ʲ���ʱ�����ٶ��ص��ʽ��й��ƣ�����ֱ�ӽ�������½���
        Q_idx = Q_idx(1:round(e_overlap*length(Q_idx)));
        Q_idx_sample = datasample(Q_idx,round(0.01*length(Q_idx)));        % ���ص���Χ�ڵĵ�Խ��й̶������ʲ���
        P_idx = P_idx(Q_idx_sample,:);
        Q_overlap = Q(Q_idx_sample,:); 
        P_closest = P(P_idx,:);
%         Q = Q(Q_idx,:);                                                    % ���Ժܸߵ����Ŷ��ҵ���ȷ���ص��ʺ󣬲���Ҫ�������еĵ㡾��Ч����
%         e_overlap = 1;
        disp("���ٵ���");
    else
        % �ص��ʹ��ƣ��������ص���ɸѡ��Ӧ�㣬�õ�����
        last_sigma = sigma;
        [e_overlap,sigma] = estimate_overlap1(D_sorted);                   % �����ص��ʹ���,������ɸѡ��Ӧ��
        e_overlap = 1;
        Q_idx = Q_idx(1:round(e_overlap*length(Q_idx)));
        P_idx = P_idx(Q_idx,:);
        Q_overlap = Q(Q_idx,:);
        P_closest = P(P_idx,:);
    end
    
    % Step2:������תƽ�ƾ���
    % ȥ���Ļ�
    P_center=mean(P_closest);                                              % P�㼯�����ĵ�
    Q_center=mean(Q_overlap);                                              % ��Ӧ�㼯�����ĵ�
    P_temp = P_closest-P_center;                                           % ����ȥ���Ļ�
    Q_temp = Q_overlap-Q_center;
    % SVD�ֽ� ���ɲ����Բ�ʹ��SVD�ֽ⣬��Ϊ����̫��ʱ��
    H=Q_temp'*P_temp;                                                      % ����H����
    [U,~,V]=svd(H); 
    R=V*U';
    t=P_center'-R*Q_center';                                               % �������ĵ����T����
    
    % Step3:ʹ��ת�������õ��µĵ㼯Q,3*N
    Q = (R*Q'+t)';
    
    % ��������
    last_R = R;
    last_t = t;
    
    if n == 1 || n == 2 || n == 5 || n == 10 || n == 20
        figure;
        plot(D_sorted,'^-','color','k');
        title_str =  ['Correspondence points distance curve; Iteration times:',num2str(n)];
        title({title_str;''});
        xlabel('i');
        ylabel('D_i');
        xlim([0,length(D_sorted)]);
        hold on
    end
end
Q_reg = Q;

% ����������ߡ���������
figure;
plot(rmses,'o-');
title("�����������");
end
