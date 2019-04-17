function pitch = calpitch(camera,lpw)

% PITCH = CALPITCH(CAMERA,LPW)
% Estimate pitch angle.
%
% Input: camera - camera model.
%        lpw    - nearest lane pixel width.
%
% Output: pitch - pitch angle.
%
% Author:Zhiwei Zeng
% Date:2018.07.28
%
% Copyright (C) 2018 Zhiwei Zeng.
% Copyright (C) 2018 Chengdu ZLT Technology Co., Ltd.
% All rights reserved.
%
% This file is part of the railway monitor toolkit and is made available under
% the terms of the BSD license (see the COPYING file).

alpha = camera.alpha*pi/180;
beta  = camera.beta*pi/180;
b = 1.435;

temp = atan(camera.xreso*b/(2*tan(alpha/2)*camera.h*lpw))+beta/2;
pitch = temp-pi/2;