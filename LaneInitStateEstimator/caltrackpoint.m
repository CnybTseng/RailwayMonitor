function [X,Y,Direction,nL,nw] = caltrackpoint(camera,model)

% [X,Y,DIRECTION,NL] = CALTRACKPOINT(CAMERA,MODEL)
% Calculate track points.
%
% Input: camera - camera parameters.
%         model - railway state model.
%
% Output:         X - x position of track points.
%                 Y - y position of track points.
%         Direction - track direction.
%                nL - number of left track points.
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

slopthresh = 0.1;	% Projected curve slop threshold

% Y position of lane boundary.
yb = camera.yreso:-1:1;	% from near to far
yb = yb';

% Calculate look ahead distance.
dist = calookaheadist(camera,model,yb);

% Remove 'yb's and 'dist's after vanish point. (Invalid look ahead distance)
yb = yb(dist>0);
dist = dist(dist>0);

% Calculate x position of lane boundary.
yll = -model.b/2+model.ch0.*dist.^2/2+model.ch1.*dist.^3/6;
xbl = camera.fx.*(yll-model.y0+model.phi1*dist)./dist+camera.cx;

ylr =  model.b/2+model.ch0.*dist.^2/2+model.ch1.*dist.^3/6;
xbr = camera.fx.*(ylr-model.y0+model.phi1*dist)./dist+camera.cx;

% Remove outliers 'xl's, 'yl's, and 'distl's.
idx = xbl>0.5 & xbl<camera.xreso+0.5;
xl = int32(xbl(idx)+0.5);
yl = int32(yb(idx));
distl = dist(idx);

% Remove outliers 'xr's, 'yr's, and 'distr's.
idx = xbr>0.5 & xbr<camera.xreso+0.5;
xr = int32(xbr(idx)+0.5);
yr = int32(yb(idx));
distr = dist(idx);

% Fill lane boundary gap.
[nxl,nyl,ndistl] = fillgap(xl,yl,distl);
[nxr,nyr,ndistr] = fillgap(xr,yr,distr);

% Calculate dy/dx=(dy/dl)/(dx/dl)
dyll = -camera.fy*camera.h./ndistl.^2-camera.fy*model.cv0/2;					% dy/dl of left
dxll = -camera.fx*(-model.b/2-model.y0)./ndistl.^2+camera.fx*model.ch0/2+...
camera.fx*model.ch1.*ndistl/3;													% dx/dl of left
dyxl = dyll./dxll;																% dy/dx of left

dylr = -camera.fy*camera.h./ndistr.^2-camera.fy*model.cv0/2;					% dy/dl of right
dxlr = -camera.fx*( model.b/2-model.y0)./ndistr.^2+camera.fx*model.ch0/2+...
camera.fx*model.ch1.*ndistr/3;													% dx/dl of right
dyxr = dylr./dxlr;																% dy/dx of right

% Calculate lane directions
LDirection = atan(dyxl);														% left lane
RDirection = atan(dyxr);														% right lane

% Remove too faraway points.
idx = abs(LDirection)>slopthresh;
nxl = nxl(idx);
nyl = nyl(idx);
LDirection = LDirection(idx);

idx = abs(RDirection)>slopthresh;
nxr = nxr(idx);
nyr = nyr(idx);
RDirection = RDirection(idx);

% Remove too close to each other points
oil = 0;
oir = 0;
for i = 1 : size(nxl,2)
	oil = i;
	for j = 1 : size(nxr,2)
		if nxl(i)==nxr(j) && nyl(i)==nyr(j)
			oir = j;
			break;
		end
	end
	if oir ~= 0
		break;
	end
end

if oil ~= 0 && oir ~= 0
	nxl = nxl(1:oil);
	nyl = nyl(1:oil);
	nxr = nxr(1:oir);
	nyr = nyr(1:oir);
end

if isempty(nxl) || isempty(nxr)
	X = [];
	Y = [];
	Direction = [];
	nL = 0;
	nw = 0;
	return;
end

X = [nxl';nxr'];
Y = [nyl';nyr'];
Direction = [LDirection';RDirection'];
nL = size(nxl,2);
nw = nxr(1)-nxl(1);