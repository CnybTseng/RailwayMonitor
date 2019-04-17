function [FX,FY,mask] = calfittedpoint(fobj,sz)

% CALFITTEDPOINT(FOBJ,SZ)
% Likelihood function of railway state estimator.
%
% Input: fobj - fitting object
%          sz - size of image
%
% Output:   FX - fitted xs
%           FY - fitted ys
%         mask - mask image
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

mask = zeros(sz,'uint8');

FY = size(mask,1) : -1 : 1;
FX = fobj.p1.*FY.^7+fobj.p2.*FY.^6+fobj.p3.*FY.^5+fobj.p4.*FY.^4+fobj.p5.*FY.^3+fobj.p6.*FY.^2+fobj.p7.*FY+fobj.p8;

findex = FX >= 1 & FX <= size(mask,2);
FX = int32(FX(findex));
FY = int32(FY(findex));

for i = 1:size(FX,2)
	mask(FY(1,i),FX(1,i)) = 255;
end