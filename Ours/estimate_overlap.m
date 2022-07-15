function [Q_idx,e_overlap,D_sorted,sigma] = estimate_overlap(D,d_means)

% ���ص��ʡ���ʼλ�ò��ѵ�����������Բ��ã�40%

% ��ʼ��
[D_sorted,Q_idx] = sort(D);
all_point_count = length(D_sorted);                    

if isempty(d_means)     % ȥ����ʼ��             
    e_overlap = 1;
    sigma = 0;
else
    [inflection_idx,sigma] = find_inflection(D_sorted,d_means(1));
    if sigma <= 0.5     % ���ص��ʽϸߵĵ����޷�׼ȷ���Ƴ��ص��ʡ�
        % ������һ�׶Σ���������ʶ������������������ص��ʣ��޷���֤������
%         e_overlap = sigma + (1-sigma)*rand(1);
        e_overlap = 1;
        sigma = 0;
        fprintf("����ص��ʣ�%d\t",e_overlap)
        fprintf("���Ŷȣ�%d\n",sigma);
    else
        % �����ڶ��׶Σ���ȷ��λ�����Ż���
        % % �ص��ʹ���
%     % ����Ӧ���У��������ڵ㡱������ռ�ܶ�Ӧ���p%���Ϻ��ٿ�ʼ���ҹյ�
%     % ��������ԭ���ǣ�����֮������Ӧ��󲿷ֶ������Ǵ���ƥ��ģ������������ƽ�����յ㲻���ԡ����Ӧ��Ҫʶ���������������ص������ŶȺ󣬿��Բ��ܡ�
%     % ��������ڵ�EIP�����С�ڳ���ƽ���������е㡾ʹ���ĸ������ã�ƽ�����or��������Ϊʲô��ʹ�ó������ݻ����ϴ����ݣ���
%     eip_count = sum(D_sorted < d_means(1));                % �������ڵ㡱��������ʽ������ٵ����׶�ʱ�ĵ�һ��ƽ�����
%     eip_p = eip_count/all_point_count;                     % �������ڵ㡱������������ܲ��й����ڵ㣬ֻ��˵������д��ڸ�ֵ�ģ���ô�ص��ʱ�Ȼ������Ϊ1��
%     
%     if eip_p == 1
%     %   disp("�����ص��ʣ�100%");
%         e_overlap = 1;
%         sigma = 1;
%     else
%     %   disp("Ѱ�ҹյ�");    % ������������ǿ������Ż���
%         [inflection_idx,sigma] = find_inflection(D_sorted);
%         e_overlap = sigma * inflection_idx/all_point_count;               % K���յ�����Ŷȣ�K-[0,1]������˼��1���������Ŷȡ�
%         Q_idx = Q_idx(1:round(e_overlap*all_point_count));                % ȡ���ڵ�
%     end
        e_overlap = sigma * inflection_idx/all_point_count;               % K���յ�����Ŷȣ�K-[0,1]������˼��1���������Ŷȡ�
        Q_idx = Q_idx(1:round(e_overlap*all_point_count));                % ȡ���ڵ�
    end
    fprintf("�����ص��ʣ�%d\t",e_overlap)
    fprintf("���Ŷȣ�%d\n",sigma);
end

end