clear;clc;close all;

mean = [0;0;0];		% [phi1;phi2;y0]
cov  = [1,0,0;0,1,0;0,0,1];

[X,Y,Z] = meshgrid(-1:0.1:1,-1:0.1:1,-1:0.1:1);

FlatX = X(:);
FlatY = Y(:);
FlatZ = Z(:);

XYZ = [FlatX';FlatY';FlatZ'];

Diff = XYZ-repmat(mean,1,size(XYZ,2));

DiffT = Diff';

for i = 1 : size(Diff,2)
	P(i) = (1/(2*pi)^(3/2)*sqrt(det(cov)))*exp(-(DiffT(i,:)*inv(cov)*Diff(:,i))/2);
end

scatter3(FlatX,FlatY,FlatZ,4,P,'filled');