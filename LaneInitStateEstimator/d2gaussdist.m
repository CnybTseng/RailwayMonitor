% function d2gaussdist

clear;clc;close all;

mean = [0;0];
cov = [1,0;0,1];
[X,Y] = meshgrid([-10:0.5:10],[-10:0.5:10]);

FX = X(:);
FY = Y(:);
XY = [FX';FY'];

diff = XY-repmat(mean,1,size(XY,2));
difft = diff';

for i = 1 : size(diff,2)
	Z(i) = (1/(2*pi*sqrt(det(cov)))).*exp(-0.5.*difft(i,:)*inv(cov)*diff(:,i));
end

Z = reshape(Z,size(X));

mesh(X,Y,Z);

xlabel('X');
ylabel('Y');
zlabel('P([X,Y])');