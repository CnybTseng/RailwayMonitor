% function bm = tabusearch(I,camera,radius,sigma,mean,cov)

% BM = TABUSEARCH(I,CAMERA,RADIUS,SIGMA,MEAN,COV)
% Tabu search the best model parameters
%
% Input:      I - single channel image.
%        camera - camera parameters.
%        radius - fuzzy membership function radius.
%         sigma - Gaussian weight window radius.
%          mean - mean vector of a priori distribution.
%           cov - covariation matrix of a priori distribution.
%
% Output: bm - best model
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

global bm;

% Set state vector initial value
X = [0,0,0,0,0,0];

% Set lower and upper bounds for variables.
LB = [-0.05,-0.1,-0.1,-0.000008,-0.00000000001,-0.000008];
UB = [ 0.05, 0.1, 0.1, 0.000008, 0.00000000001, 0.000008];

maxiter = 20;		% maximum iterations

% Initial solution
solution.x = X;

% Get corresponding track
model = vec2str(solution.x);
[XL,XR,YL,YR,DL,DR,nw] = gettrack(camera,model);

% Select neighborhood edge elements
mask = mksearchmask(size(I),XL,XR,YL,YR,radius);
[GMagnitude,GDirection,LowTh] = edgerespond(I);
[SGMagnitude,SGDirection,EX,EY] = getstrongedge(GMagnitude,GDirection,LowTh,mask);

strongedge = zeros(size(I),'uint8');
strongedge(GMagnitude>LowTh) = 128;
result = repmat(strongedge,1,1,3);
figure,imshow(result);

lh = likelihood(SGMagnitude,SGDirection,EX,EY,XL,XR,YL,YR,DL,DR,radius,sigma);
solution.cost = -apriori(nw,mean,stdev)*lh;
solution.XL = XL;
solution.XR = XR;
solution.YL = YL;
solution.YR = YR;

% Initialize best solution
bestsolution = solution;

dims = 6;			% state vector dimensions
nsteps = 16;		% number of movement steps

TL = CQueue;		% Tabu list
TLength = 8;		% maximum length of Tabu list

mf = @match;		% matching function handle

costs = zeros(1,maxiter+1);
costs(1) = solution.cost;

for iter = 1 : maxiter
	fprintf('iteration=%d.\n',iter);
	for md = 1 : dims	% for every movement dimension
		disp('solution=');
		disp(solution.x);
		% Make random movement
		[MX,XL,XR,YL,YR,DL,DR,nw] = mkrandomove(camera,solution.x,nsteps,LB,UB,md);

		% Calculate cost of candidate solutions
		candidates.x    = zeros(size(MX,1),dims);	% state
		candidates.cost = zeros(size(MX,1),1);		% cost
		for i = 1 : size(MX,1)			
			mask = mksearchmask(size(I),XL{i},XR{i},YL{i},YR{i},radius);
			[SGMagnitude,SGDirection,EX,EY] = getstrongedge(GMagnitude,GDirection,LowTh,mask);
			
			% Calculate likelihood of the solution
			lh = likelihood(SGMagnitude,SGDirection,EX,EY,XL{i},XR{i},YL{i},YR{i},DL{i},DR{i},radius,sigma);
			
			candidates.cost(i) = -apriori(nw{i}(1),mean,stdev)*lh;
			candidates.x(i,:) = MX{i};
		end

		% Ascend sort candidates' cost.
		[ascandidates.cost,Idx] = sort(candidates.cost,'ascend');
		ascandidates.x(1:size(MX,1),:) = candidates.x(Idx,:);

		localbestsolut.cost = Inf;
		for i = 1 : size(MX,1)		
			ibtsf = ascandidates.cost(i) < bestsolution.cost;		% is better than so far
			if TL.find(ascandidates.x(i,:), mf, ibtsf) && ~ibtsf
				continue;
			end
			
			localbestsolut.x = ascandidates.x(i,:);
			localbestsolut.cost = ascandidates.cost(i);
			localbestsolut.XL = XL{i};
			localbestsolut.XR = XR{i};
			localbestsolut.YL = YL{i};
			localbestsolut.YR = YR{i};
			
			solution = localbestsolut;			
			TL.push(localbestsolut.x);
			% disp('push');
			% disp(localbestsolut.x);
			
			if TL.size() > TLength
				% disp('pop');
				% disp(TL.front());
				TL.pop();
			end

			break;
		end

		if localbestsolut.cost < bestsolution.cost
			bestsolution = localbestsolut;
		end
	end

	disp('best solution so far');
	disp(bestsolution.x);
	fprintf('corresponding cost = %f.\n',bestsolution.cost);
	
	costs(iter+1) = bestsolution.cost;
	
	localresult = result;
	for i = 1 : size(bestsolution.XL,1)
		localresult(bestsolution.YL(i,1),bestsolution.XL(i,1),:) = [255,255,0];
	end
	
	for i = 1 : size(bestsolution.XR,1)
		localresult(bestsolution.YR(i,1),bestsolution.XR(i,1),:) = [255,255,0];
	end
	
	close all;
	figure,imshow(localresult);
end

figure,plot(1:size(costs,2),costs,'-s');
xlabel('iteration');
ylabel('cost');
grid on;

bm.phi1 = bestsolution.x(1);
bm.phi2 = bestsolution.x(2);
bm.y0   = bestsolution.x(3);
bm.b    = 1.435;
bm.ch0  = bestsolution.x(4);
bm.ch1  = bestsolution.x(5);
bm.cv0  = bestsolution.x(6);