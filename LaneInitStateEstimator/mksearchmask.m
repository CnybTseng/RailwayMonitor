function mask = mksearchmask(sz,XL,XR,YL,YR,radius)

% MASK = MKSEARCHMASK(sz, XL,XR,YL,YR)
% Make matching point searching mask.
%
% Input: sz - size of the mask.
%        XL - x position of left track.
%        XR - x position of right track.
%        YL - y position of left track.
%        YR - y position of right track.
%
% Output: mask - searching mask.
%
% Author:Zhiwei Zeng
% Date:2018.07.16
%
% Copyright (C) 2018 Zhiwei Zeng.
% Copyright (C) 2018 Chengdu ZLT Technology Co., Ltd.
% All rights reserved.
%
% This file is part of the railway monitor toolkit and is made available under
% the terms of the BSD license (see the COPYING file).

mask = zeros(sz,'uint8');

for i = 1 : size(XL,1)
	mask(YL(i,1),XL(i,1)) = 255;
end

for i = 1 : size(XR,1)
	mask(YR(i,1),XR(i,1)) = 255;
end

mask = imdilate(mask,strel('disk',radius));