%******************************************************************************
% This programe is implemented via MATLAB 2013.                              *
% Author :  Ping Wang                                                        *
% Contact:  pingwangsky@gmail.com                                            *
% License:  Copyright (c) 2019 Ping Wang, All rights reserved.               *
% Address:  College of Electrical and Information Engineering,               *
%           Lanzhou University of Technology                                 *
% My site:  https://sites.google.com/view/ping-wang-homepage                 *
%*****************************************************************************/

function [R,T]=SVDdecomposition(Xc,Xw)
% This routine solves the exterior orientation problem for a point cloud
%  given in both camera and world coordinates. It is described in:
%Umeyama S. Least-Squares Estimation of Transformation Parameters Between 2 Point Patterns[J]. 
%IEEE Transactions on Pattern Analysis & Machine Intelligence, 1991, 13(4):376-380.

%% using SVD's method;
n=size(Xc,2);
% derive the centroid of the two point-clouds
p1=sum(Xc,2)/n;
p2=sum(Xw,2)/n;
%compute the matrix H = sum(F'*G^{T})
H1=Xc-p1*ones(1,n);
H2=Xw-p1*ones(1,n);
H=H1*H2';
%decompose this matrix (SVD) to obtain rotation
[U,S,V]=svd(H);
%calculate R and T;
R=U*V';  %or R=(V*U')';
if det(R)<0
    V_prime(:,1)=V(:,1);
    V_prime(:,2)=V(:,2);
    V_prime(:,3)=-V(:,3);
    R=U*V_prime';
end
T=p1-R*p2;
end



