function lh = likelihoodv2(GM,GD,mask,TX,TY,TD,nL,radius,sigma)

% LH = LIKELIHOOD(GM,GD,MASK,TX,TY,TD,RADIUS,SIGMA)
% Likelihood function of railway state estimator.
%
% Input: GM - gradient magnitude.
%        GD - gradient direction.
%               mask - strong edge mask.
%                 XL - x position of left track.
%                 XR - x position of right track.
%                 YL - y position of left track.
%                 YR - y position of right track.
%         TD - direction of left track.
%         RDirection - direction of right track.
%             radius - fuzzy membership function radius.
%              sigma - Gaussian weight window radius.
%
% Output: lh - likelihood
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

lh = 0;

ltindmap = zeros(size(GM),'int32');	% left track index map
rtindmap = zeros(size(GM),'int32');	% right track index map

for i = 1 : nL
	ltindmap(TY(i),TX(i)) = i;
end

for i = nL+1 : size(TX,1)
	rtindmap(TY(i),TX(i)) = i;
end

SGM = GM(mask);
SGD = GD(mask);
[EY,EX] = find(mask);

for i = 1 : size(SGM,1)
	lh = lh+likelihooditerm(size(GM),EX(i),EY(i),SGM(i),SGD(i),radius,sigma,TD,ltindmap);
	lh = lh+likelihooditerm(size(GM),EX(i),EY(i),SGM(i),SGD(i),radius,sigma,TD,rtindmap);
end