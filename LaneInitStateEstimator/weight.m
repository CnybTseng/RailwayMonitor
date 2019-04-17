function w = weight(radius,sigma,dist)

% W = WEIGHT(RADIUS,SIGMA)
% Gaussian weight function
%
% Input: radius - fuzzy membership function radius.    
%         sigma - Gaussian weight window radius.
%          dist - distance to mean.
%        
% Output: weight of dist-distance point.
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

if dist>= 0 && dist<radius
	w = exp(-dist*dist/sigma/sigma);
else
	w = 0;
end