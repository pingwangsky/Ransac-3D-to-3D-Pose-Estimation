close all;
clc;
clear;

%����ransac+3Dλ�˹��Ƶķ���������λ��;

%���ɲ�������;
npt=20;  %�������ݵĸ���
Xc  = [xrand(1,npt,[-2 2]); xrand(1,npt,[-2 2]); xrand(1,npt,[4 8])];
t   = mean(Xc,2)        %��ʵ��λ��         
R   = rodrigues(randn(3,1))     %��ʵ����̬
Xw = inv(R)*(Xc-repmat(t,1,npt));

%���������������쳣ֵ;
pout=0.2;   %�쳣���ݰٷֱ�
nout =round(npt * pout+1);  %��֤������һ���쳣ֵ��
idx=randIndex(npt,nout);
Xw(:,idx)=[xrand(1,nout,[-2 2]); xrand(1,nout,[-2 2]); xrand(1,nout,[4 8])];    %��������쳣�Ķ�Ӧ��,��ʱXw�а����쳣ֵ

%����Ransac�ķ���ɸѡ��ȷ�Ķ�Ӧ�㣨inliners�����޳��쳣ֵ�㣨outliers��
iterNum=30; %��������,���԰�����Ҫ�Լ��޸�
n=size(Xc,2);
sampleNum=3;    %ÿ�ε���ɸѡʱ��ѡ���ĸ���;
th=0.1; %inliersɸѡ����ֵ�����԰�����Ҫ�Լ��޸�;
inliers_max=[]; %�����洢inliers�����;
for i=1:iterNum
    %���ѡ��������
    sampleIdx=randIndex(n,sampleNum);
    XcSample=Xc(:,sampleIdx);
    XwSample=Xw(:,sampleIdx);
    %�������ѡ���������������λ����Ϣ;
    [R_temp,t_temp]=SVDdecomposition(XcSample,XwSample);
    %���ù����λ����ȷ��inliers;
    XXc=R_temp*Xw+t_temp*ones(1,n);
    %ÿһ����ԭʼ���ݱȽϼ������;
    error=abs(sum(XXc-Xc));
    inliers=find(error<th);
    if length(inliers)>length(inliers_max)
        inliers_max=inliers;
    end
end

%��ʾ���Խ��
disp('*************************');
%inliers�����
inliers_max
%outliers�����
idx

%û���޳�outliersʱ���������ݹ���λ��;
disp('*************************');
[R1,T1]=SVDdecomposition(Xc,Xw)

%����inliers���������յ�λ��
disp('*************************');
[optR,optT]=SVDdecomposition(Xc(:,inliers_max),Xw(:,inliers_max))


