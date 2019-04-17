function dist = calookaheadist(camera,model,yb)

% DIST = CALOOKAHEADIST(CAMERA,MODEL,YB)
% Calculate look ahead distance.
%
% Input: camera - camera parameters.
%         model - railway state model.
%            yb - y position of lane boundary.
%
% Output: dist - look ahead distance.
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

dist = zeros(size(yb));

% Solve equation like p1*x^2+p2*x+p3=0
p1 = -camera.fy*model.cv0/2;
p3 = camera.fy*camera.h;

for i = 1 : size(yb,1)
	p2 = camera.fy*model.phi2+camera.cy-yb(i);
	if abs(p1) > 1e-10				% p1 != 0
		delta = p2*p2-4*p1*p3;
		if (delta > 0)
			sol = (-p2-sqrt(delta))/(2*p1);
			dist(i) = max(sol,0);
		else
			dist(i) = 0;
		end
	else							% p2 == 0
		if abs(p2) > 1e-30
			sol = -p3/p2;
			dist(i) = max(sol,0);
		else
			dist(i) = 0;
		end
	end
end