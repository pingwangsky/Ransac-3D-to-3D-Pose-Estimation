close all;
clc;
clear;

%利用ransac+3D位姿估计的方法求解最佳位姿;

%生成测试数据;
npt=20;  %测试数据的个数
Xc  = [xrand(1,npt,[-2 2]); xrand(1,npt,[-2 2]); xrand(1,npt,[4 8])];
t   = mean(Xc,2)        %真实的位置         
R   = rodrigues(randn(3,1))     %真实的姿态
Xw = inv(R)*(Xc-repmat(t,1,npt));

%测试数据中增加异常值;
pout=0.2;   %异常数据百分比
nout =round(npt * pout+1);  %保证至少有一个异常值点
idx=randIndex(npt,nout);
Xw(:,idx)=[xrand(1,nout,[-2 2]); xrand(1,nout,[-2 2]); xrand(1,nout,[4 8])];    %随机赋予异常的对应点,此时Xw中包含异常值

%利用Ransac的方法筛选正确的对应点（inliners）和剔除异常值点（outliers）
iterNum=30; %迭代次数,可以按照需要自己修改
n=size(Xc,2);
sampleNum=3;    %每次迭代筛选时候选择点的个数;
th=0.1; %inliers筛选的阈值，可以按照需要自己修改;
inliers_max=[]; %用来存储inliers的序号;
for i=1:iterNum
    %随机选择三个点
    sampleIdx=randIndex(n,sampleNum);
    XcSample=Xc(:,sampleIdx);
    XwSample=Xw(:,sampleIdx);
    %利用随机选择的三个点来估算位姿信息;
    [R_temp,t_temp]=SVDdecomposition(XcSample,XwSample);
    %利用估算的位姿来确定inliers;
    XXc=R_temp*Xw+t_temp*ones(1,n);
    %每一个和原始数据比较计算误差;
    error=abs(sum(XXc-Xc));
    inliers=find(error<th);
    if length(inliers)>length(inliers_max)
        inliers_max=inliers;
    end
end

%显示测试结果
disp('*************************');
%inliers的序号
inliers_max
%outliers的序号
idx

%没有剔除outliers时候所有数据估计位姿;
disp('*************************');
[R1,T1]=SVDdecomposition(Xc,Xw)

%利用inliers来估算最终的位姿
disp('*************************');
[optR,optT]=SVDdecomposition(Xc(:,inliers_max),Xw(:,inliers_max))


