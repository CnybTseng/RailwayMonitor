function lh = likelihood(SGMagnitude,SGDirection,EX,EY,XL,XR,YL,YR,LDirection,RDirection,radius,sigma)

% LH = LIKELIHOOD(...,RADIUS,SIGMA)
% Likelihood function of railway state estimator.
%
% Input: SGMagnitude - strongest edge magnitude.
%        SGDirection - strongest edge direction.
%                 EX - x position of strongest edge.
%                 EY - y position of strongest edge.
%                 XL - x position of left track.
%                 XR - x position of right track.
%                 YL - y position of left track.
%                 YR - y position of right track.
%         LDirection - direction of left track.
%         RDirection - direction of right track.
%             radius - fuzzy membership function radius.
%              sigma - Gaussian weight window radius.
%
% Output: lh - likelihood
%
% Author:Zhiwei Zeng
% Date:2018.07.12
%
% Copyright (C) 2018 Zhiwei Zeng.
% Copyright (C) 2018 Chengdu ZLT Technology Co., Ltd.
% All rights reserved.
%
% This file is part of the railway monitor toolkit and is made available under
% the terms of the BSD license (see the COPYING file).

global debug;

lh = 0;

if debug == 1
	global I;
	figure,imshow(I,[]);
	arrow = 3;
end

for i = 1 : size(SGMagnitude,1)
	% Find the nearest left track point.
	ldists = sqrt((EX(i)-XL).^2+(EY(i)-YL).^2);	
	[asdists,index] = sort(ldists,'ascend');
	minid = index(1);
	lmindist = ldists(minid);
	
	% Edge direction
	if SGDirection(i) > 0
		edirection = SGDirection(i)-pi/2;
	else
		edirection = SGDirection(i)+pi/2;
    end

	theta = abs(edirection-LDirection(minid));
	if theta>pi/2
		theta = theta-pi/2;
	end
	
	dlh = SGMagnitude(i)*weight(radius,sigma,lmindist)*cos(theta);
	lh = lh+dlh;
	
	if debug == 1 && dlh > 0
		% line([EX(i),XL(minid)],[EY(i),YL(minid)]);
		
		ex = XL(minid)+arrow*cos(LDirection(minid));
		ey = YL(minid)+arrow*sin(LDirection(minid));
		line([XL(minid),ex],[YL(minid),ey],'Color',[1,1,0]);
		% hold on;plot(XL(minid), YL(minid), 'yo');
		
		ex = EX(i)+arrow*cos(edirection);
		ey = EY(i)+arrow*sin(edirection);
		line([EX(i),ex],[EY(i),ey],'Color',[1,0,0]);
		% hold on;plot(EX(i), EY(i), 'ro');
	end
	
	% Find the nearest right track point.
	rdists = sqrt((EX(i)-XR).^2+(EY(i)-YR).^2);	
	[asdists,index] = sort(rdists,'ascend');
	minid = index(1);
	rmindist = rdists(minid);
	
	theta = abs(edirection-RDirection(minid));
	if theta>pi/2
		theta = theta-pi/2;
	end
	
	dlh = SGMagnitude(i)*weight(radius,sigma,rmindist)*cos(theta);
	lh = lh+dlh;
	
	if debug == 1 && dlh > 0
		% line([EX(i),XR(minid)],[EY(i),YR(minid)]);
		
		ex = XR(minid)+arrow*cos(RDirection(minid));
		ey = YR(minid)+arrow*sin(RDirection(minid));
		line([XR(minid),ex],[YR(minid),ey],'Color',[1,1,0]);
		% hold on;plot(XR(minid), YR(minid), 'yo');
		
		ex = EX(i)+arrow*cos(edirection);
		ey = EY(i)+arrow*sin(edirection);
		line([EX(i),ex],[EY(i),ey],'Color',[1,0,0]);
		% hold on;plot(EX(i), EY(i), 'ro');
	end
end