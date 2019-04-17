function [SGMagnitude,SGDirection,EX,EY] = getstrongedge(GMagnitude,GDirection,LowTh,mask)

% [SGMAGNITUDE,SGDIRECTION,EX,EY] = GETSTRONGEDGE(GMAGNITUDE,GDIRECTION,LOWTH,MASK)
% Get gradient magnitude and direction of strongest edge.
%
% Input: GMagnitude - gradient magnitude.
%        GDirection - gradient direction.
%             LowTh - lower threshold of gradient magnitude.
%              mask - region of interest mask.
%
% Output: SGMagnitude - strongest edge gradient magnitude.
%         SGDirection - strongest edge gradient direction.
%                  EX - x position of strongest edge.
%                  EY - y position of strongest edge.
%
% Author:Zhiwei Zeng
% Date:2018.07.18
%
% Copyright (C) 2018 Zhiwei Zeng.
% Copyright (C) 2018 Chengdu ZLT Technology Co., Ltd.
% All rights reserved.
%
% This file is part of the railway monitor toolkit and is made available under
% the terms of the BSD license (see the COPYING file).

roi = GMagnitude>LowTh & mask;
	
SGMagnitude = GMagnitude(roi);	
SGDirection = GDirection(roi);	

[EY,EX] = find(roi);			