function [GMagnitude,GDirection,mask] = edgerespond(I)

% [GMAGNITUDE,GDIRECTION,MASK] = EDGERESPOND(I)
% Calculate edge response
%
% Input: I - single channel image.
%
% Output: GMagnitude - gradient magnitude.
%         GDirection - gradient direction.
%               mask - strong edge mask.
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

I = double(I);

sx = [-1,0,1;-2,0,2;-1,0,1];			% X-Sobel kernel
sy = [-1,-2,-1;0,0,0;1,2,1];			% Y-Sobel kernel
										
Dx = imfilter(I,sx,'symmetric');		% X-Differential
Dy = imfilter(I,sy,'symmetric');		% Y-Differential
										
GMagnitude = sqrt(Dx.^2+Dy.^2);			% Gradient magnitude
GDirection = atan(Dy./(Dx+1e-30));				% Gradient direction

LowTh = (min(GMagnitude(:))+max(GMagnitude(:)))/15;	% Magnitude threshold for strong edge

mask = GMagnitude>LowTh;