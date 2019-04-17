function [MX,XL,XR,YL,YR,LDirection,RDirection,nw] = mkrandomove(camera,X,nmoves,LB,UB,md)

% MX = MKRANDOMOVE(X,NMOVES,LB,UB,MD)
% Make random movement around neighbor of X.
%
% Input:      X - original variable.
%        nmoves - number of movement.
%            LB - lower bounds of X.
%            UB - upper bounds of X.
%            md - movement dimension.
%
% Output: MX - moved X.
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

p = (UB(md)-LB(md))/2;				% maximum movement
MX = repmat(X,nmoves,1);			% #nmoves copies of X in the column dimension
MX = mat2cell(MX,ones(1,size(MX,1)),size(MX,2));

mf = @match;

for i = 1 : nmoves
	h = 1/4^(i-1);
	ntries = 100;
	while ntries > 0
		L = 2*(rand(1,1)-0.5);		% random movement direction
		MX{i}(md) = X(md)+L*p*h;		% produce random neighbor		
		% check if the neighbor is valid
		model = vec2str(MX{i});
		[XL{i},XR{i},YL{i},YR{i},LDirection{i},RDirection{i},nw{i}] = gettrack(camera,model);
		if ~isempty(XL{i}) && ~isempty(XR{i}) && ~isempty(YL{i}) && ~isempty(YR{i})
			break;
		end
		ntries = ntries-1;
	end
	if ntries == 0
		disp('make bad movement!');
		MX{i} = [];
		nw{i} = [];
	end
end

MX(cellfun(@isempty,MX)) = [];
XL(cellfun(@isempty,XL)) = [];
XR(cellfun(@isempty,XR)) = [];
YL(cellfun(@isempty,YL)) = [];
YR(cellfun(@isempty,YR)) = [];
LDirection(cellfun(@isempty,LDirection)) = [];
RDirection(cellfun(@isempty,RDirection)) = [];
nw(cellfun(@isempty,nw)) = [];





