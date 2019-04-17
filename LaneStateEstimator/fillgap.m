function [nx,ny,ndist] = fillgap(x,y,dist)

% [NX,NY,NDIST] = FILLGAP(X,Y,DIST)
% Fill lane boundary gap.
%
% Input:    x - x position of lane boundary.
%           y - y position of lane boundary.
%        dist - look ahead distance.
%
% Output:    nx - new x position of lane boundary.
%            ny - new y position of lane boundary.
%         ndist - new look ahead distance.
%
% Author:Zhiwei Zeng
% Date:2018.07.21
%
% Copyright (C) 2018 Zhiwei Zeng.
% Copyright (C) 2018 Chengdu ZLT Technology Co., Ltd.
% All rights reserved.
%
% This file is part of the railway monitor toolkit and is made available under
% the terms of the BSD license (see the COPYING file).

% Filling left lane boundary gap.
nx = [];
ny = [];
ndist = [];
cnt = 1;

if isempty(x) || isempty(y) || isempty(dist)
	return;
end

for i = 1 : size(dist,1)-1
	nx(cnt) = x(i);
	ny(cnt) = y(i);
	ndist(cnt) = dist(i);
	cnt = cnt+1;
	if abs(x(i+1)-x(i)) > 1
		step = (-1)^(x(i+1)<x(i));
		for j = x(i)+step:step:x(i+1)-step
			nx(cnt) = j;
			ny(cnt) = y(i);
			ndist(cnt) = dist(i);
			cnt = cnt+1;
		end
	end
end

nx(cnt) = x(end);
ny(cnt) = y(end);
ndist(cnt) = dist(end);