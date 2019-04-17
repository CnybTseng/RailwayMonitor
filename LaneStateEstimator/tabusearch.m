function bm = tabusearch(I,camera,radius,sigma,mean,cov)

% BM = TABUSEARCH(I,CAMERA,RADIUS,SIGMA,MEAN,COV)
% Tabu search the best model parameters
%
% Input:      I - single channel image.
%        camera - camera parameters.
%        radius - fuzzy membership function radius.
%         sigma - Gaussian weight window radius.
%          mean - mean vector of a priori distribution.
%         cov - standard deviation of a priori distribution.
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

% Set state vector initial value
pitch = mean(2);
X = [0,pitch,0,0,0,0];

% Side conditions for state model parameters.
LB = [-0.05, pitch-0.01,-0.1,-5e-40,-1e-100,-1e-50];
UB = [ 0.05, pitch+0.01, 0.1, 5e-40, 1e-100, 1e-50];

maxiter = 50;		% maximum iterations

% Initial solution
solution.x = X;

% Get corresponding track
model = vec2str(solution.x);
[TX,TY,TD,nL,nw] = caltrackpoint(camera,model);

[GM,GD,mask] = edgerespond(I);

% Calculate initial solution and cost
lh = calikelihood(GM,GD,mask,TX,TY,TD,radius,sigma);
solution.cost = -apriori(model,mean,cov)*lh;
solution.TX = TX;
solution.TY = TY;

% Initialize best solution
bestsolution = solution;

dims = 6;			% state vector dimensions
nsteps = 16;		% number of movement steps

TL = CQueue;		% Tabu list
TLength = 16;		% maximum length of Tabu list

mf = @match;		% matching function handle

cyclebestcosts = zeros(1,maxiter);

costs = zeros(1,maxiter);

usedstates = {};
counter = ones(1,6);
titles = {'yaw angle','pitch angle','lateral offset','ch0','ch1','cv0'};

strongedge = zeros(size(mask),'uint8');
strongedge = uint8(255*(GM-min(GM(:)))./(max(GM(:)-min(GM(:)))));
strongedge(~mask) = 0;
result = repmat(strongedge,[1,1,3]);
figure;

for iter = 1 : maxiter
	fprintf('iteration = %d.\n',iter);
	cyclebestcosts(iter) = Inf;
	for md = 1 : dims	% for every movement dimension
		% Generate new state randomly in neighborhood
		[NS,TX,TY,TD,nL,nw] = mkrandomove(camera,solution.x,nsteps,LB,UB,md);
		
		% Calculate cost of candidate solutions
		candidates.x    = zeros(size(NS,1),dims);
		candidates.cost = zeros(size(NS,1),1);
		for i = 1 : size(NS,1)					
			lh = calikelihood(GM,GD,mask,TX{i},TY{i},TD{i},radius,sigma);
			candidates.cost(i) = -apriori(model,mean,cov)*lh;
			candidates.x(i,:) = NS{i};
		end

		% Ascend sort candidates' cost
		[ascandidates.cost,Idx] = sort(candidates.cost,'ascend');
		ascandidates.x(1:size(NS,1),:) = candidates.x(Idx,:);

		% Find local best solution
		localbestsolut.cost = Inf;
		for i = 1 : size(NS,1)
			mac = ascandidates.cost(i) < bestsolution.cost;		% match aspiration criterion
			forbided = TL.find(ascandidates.x(i,:),mf,0);		% belong to Tabu list
			if forbided && ~mac
				continue;
			end
			
			if forbided && mac
				disp('aspiration condition');
				disp(bestsolution.cost);
				disp(ascandidates.cost(i));
			end
			
			localbestsolut.x = ascandidates.x(i,:);
			localbestsolut.cost = ascandidates.cost(i);
			localbestsolut.TX = TX{i};
			localbestsolut.TY = TY{i};
			
			solution = localbestsolut;				% update current state			
			TL.push(localbestsolut.x);				% update Tabu list
			% disp('push');
			% disp(localbestsolut.x);
			
			if TL.size() > TLength					% remove timeout tabu state
				% disp('pop');
				% disp(TL.front());
				TL.pop();
			end

			break;
		end

		if localbestsolut.cost < cyclebestcosts(iter)
			cyclebestcosts(iter) = localbestsolut.cost;
		end
		
		if localbestsolut.cost < bestsolution.cost
			bestsolution = localbestsolut;			% update the best solution so far
		end
		
		for i = 1 : size(NS,1)
			for j = 1 : 6
				usedstates{j}(counter(j)) = NS{i}(j);
				counter(j) = counter(j)+1;
			end
		end
	end

	disp('best solution so far');
	disp(bestsolution.x);
	fprintf('corresponding cost = %f.\n',bestsolution.cost);
	
	costs(iter) = bestsolution.cost;
	
	localresult = result;
	for i = 1 : size(bestsolution.TX,1)
		localresult(bestsolution.TY(i),bestsolution.TX(i),:) = [255,255,0];
	end
	
	imshow(localresult);
	imwrite(localresult,sprintf('local\\%d.png',iter));
end

figure;
subplot(1,2,1),plot(1:size(cyclebestcosts,2),cyclebestcosts,'-s');
xlabel('iteration');
ylabel('cycle minimum cost');
grid on;

subplot(1,2,2),plot(1:size(costs,2),costs,'-s');
xlabel('iteration');
ylabel('global minimum cost');
grid on;

% figure;
% for i = 1 : 6
% 	delta = (UB(i)-LB(i))/100;
% 	subplot(2,3,i),hist(usedstates{i},LB(i):delta:UB(i));
% 	title(titles{i});
% end

bm.phi1 = bestsolution.x(1);
bm.phi2 = bestsolution.x(2);
bm.y0   = bestsolution.x(3);
bm.b    = 1.435;
bm.ch0  = bestsolution.x(4);
bm.ch1  = bestsolution.x(5);
bm.cv0  = bestsolution.x(6);