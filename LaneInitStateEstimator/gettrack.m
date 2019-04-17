function [XL,XR,YL,YR,LDirection,RDirection,nw] = gettrack(camera,model)

% [XL,XR,YL,TR,LDIRECTION,RDIRECTION] = GETTRACK(CAMERA,MODEL)
% Calculate track points.
%
% Input: camera - camera parameters.
%         model - railway state model.
%
% Output:         XL - x position of left track.
%                 XR - x position of right track.
%                 YL - y position of left track.
%                 YR - y position of right track.
%         LDirection - direction of left track.
%         RDirection - direction of right track.
%
% Author:Zhiwei Zeng
% Date:2018.07.11
%
% Copyright (C) 2018 Zhiwei Zeng.
% Copyright (C) 2018 Chengdu ZLT Technology Co., Ltd.
% All rights reserved.
%
% This file is part of the railway monitor toolkit and is made available under
% the terms of the BSD license (see the COPYING file).

global debug;

[X,Y,Direction,nL,nw] = caltrackpoint(camera,model);	% Calculate track point

XL = double(X(1:nL));
XR = double(X(nL+1:end));
YL = double(Y(1:nL));
YR = double(Y(nL+1:end));
LDirection = Direction(1:nL);
RDirection = Direction(nL+1:end);

if debug == 1
	mask = zeros(camera.yreso, camera.xreso, 'uint8');
	for i = 1 : size(X,1)
		mask(Y(i,1),X(i,1)) = 255;
	end
	imshow(mask);
end

% Direction calculation method based on curve fitting.

% fobj = fit(YL,XL,'poly7','robust','Bisquare');

% if debug == 1
% 	[FX,FY,mask] = calfittedpoint(fobj,size(mask));
% 	figure,imshow(mask);
% end
% 
% dxy = 7*fobj.p1.*YL.^6+6*fobj.p2.*YL.^5+5*fobj.p3.*YL.^4+4*fobj.p4.*YL.^3+3*fobj.p5.*YL.^2+2*fobj.p6.*YL+fobj.p7;
% dyx = 1./dxy;
% LDirection = atan(dyx);		% [-pi/2,pi/2]
% 
% if debug == 1
% 	figure,plot(1:size(LDirection,1),LDirection);
% end
% 
% fobj = fit(YR,XR,'poly7','robust','Bisquare');
% 
% if debug == 1
% 	[FX,FY,mask] = calfittedpoint(fobj,size(mask));
% 	figure,imshow(mask);
% end
% 
% dxy = 7*fobj.p1.*YR.^6+6*fobj.p2.*YR.^5+5*fobj.p3.*YR.^4+4*fobj.p4.*YR.^3+3*fobj.p5.*YR.^2+2*fobj.p6.*YR+fobj.p7;
% dyx = 1./dxy;
% RDirection = atan(dyx);		% [-pi/2,pi/2]
% 
% if debug == 1
% 	figure,plot(1:size(RDirection,1),RDirection);
% end