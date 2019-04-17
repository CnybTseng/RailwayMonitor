function [SGMagnitude,SGDirection,EX,EY] = getstrongedge(I,mask)

% [SGMAGNITUDE,SGDIRECTIION,EX,EY] = GETSTRONGEDGE(I)
% Get strongest edge elements.
%
% Input: I - single channel image.
%
% Output: SGMagnitude - strongest edge gradient magnitude.
%         SGDirection - strongest edge gradient direction.
%                  EX - x position of strongest edge.
%                  EY - y position of strongest edge.
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

I = double(I);

sx = [-1,0,1;-2,0,2;-1,0,1];			% X-Sobel kernel
sy = [-1,-2,-1;0,0,0;1,2,1];			% Y-Sobel kernel
										
Dx = imfilter(I,sx,'symmetric');		% X-Differential
Dy = imfilter(I,sy,'symmetric');		% Y-Differential
										
GMagnitude = sqrt(Dx.^2+Dy.^2);			% Gradient magnitude
GDirection = atan(Dy./Dx);				% Gradient direction

LowTh = (min(GMagnitude(:))+max(GMagnitude(:)))/2;	% Magnitude threshold for strong edge

sub = GMagnitude>LowTh & mask;			% Choose strong edge elements
SGMagnitude = GMagnitude(sub);			% Strong edge gradient magnitude
SGDirection = GDirection(sub);			% Strong edge gradient direction

[EY,EX] = find(sub);					% Strong edge elements subscript