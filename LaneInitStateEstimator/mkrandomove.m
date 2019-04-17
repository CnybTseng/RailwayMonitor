function [NS,TX,TY,TD,nL,nw] = mkrandomove(camera,X,nmoves,LB,UB,md)

% NS = MKRANDOMOVE(X,NMOVES,LB,UB,MD)
% Make random movement around neighbor of X.
%
% Input:      X - original variable.
%        nmoves - number of movement.
%            LB - lower bounds of X.
%            UB - upper bounds of X.
%            md - movement dimension.
%
% Output: NS - new state.
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
NS = repmat(X,nmoves,1);			% #nmoves copies of X in the column dimension
NS = mat2cell(NS,ones(1,size(NS,1)),size(NS,2));

for i = 1 : nmoves
	h = 1/2^(i-1);
	ntries = 100;
	while ntries > 0
		L = 2*(rand(1,1)-0.5);		% random movement direction
		NS{i}(md) = X(md)+L*p*h;	% generate random state
		
		% check if the random move violate bound conditions
		if NS{i}(md) < LB(md) || NS{i}(md) > UB(md)
			ntries = ntries-1;
			continue;
		end
		
		% check if the random move is feasible
		model = vec2str(NS{i});		
		[TX{i},TY{i},TD{i},nL{i},nw{i}] = caltrackpoint(camera,model);
		if ~isempty(TX{i}) && ~isempty(TY{i}) && ~isempty(TD{i}) && nL{i}>0 &&...
			nw{i}>0 && TY{i}(1)==camera.yreso && TY{i}(nL{i}+1)==camera.yreso
			break;
		end
		
		ntries = ntries-1;
	end
	if ntries == 0
		disp('make bad movement!');
		NS{i} = [];
		TX{i} = [];
		TY{i} = [];
		TD{i} = [];
		nL{i} = [];
		nw{i} = [];
	end
end

NS(cellfun(@isempty,NS)) = [];
TX(cellfun(@isempty,TX)) = [];
TY(cellfun(@isempty,TY)) = [];
TD(cellfun(@isempty,TD)) = [];
nL(cellfun(@isempty,nL)) = [];
nw(cellfun(@isempty,nw)) = [];