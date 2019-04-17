function lh = likelihood(GM,GD,mask,TX,TY,TD,radius,sigma)

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

global debug;

lh = 0;

if debug == 1
	image = zeros([size(GM,1),size(GM,2),3],'uint8');
end

TX = double(TX);
TY = double(TY);
searchradius = radius;

for i = 1 : 5 : size(TX,1)
	% Distance from origin to searching line
	pho = TX(i)*cos(TD(i))+TY(i)*sin(TD(i));
	
	% Get edge points in the searching line
	if abs(TD(i))>pi/4
		X = 1:size(GM,2);
		Y = (pho-X*cos(TD(i)))/sin(TD(i));
		idx = Y>0.5&Y<=size(GM,1);
	else
		Y = 1 : size(GM,1);
		X = (pho-Y*sin(TD(i)))/cos(TD(i));
		idx = X>0.5&X<=size(GM,2);
    end
	
	X = round(X(idx));
	Y = round(Y(idx));
	
	% Remove too faraway edge points
	Dist = sqrt((X-TX(i)).^2+(Y-TY(i)).^2);
	idx = Dist<searchradius;
	X = X(idx);
	Y = Y(idx);
	Dist = Dist(idx);
	
	X = int32(X);
	Y = int32(Y);
	
	dlh = 0;
	for j = 1 : size(X,2)
		if mask(Y(j),X(j)) == 0
			continue;
		end
	
		% Calculate edge direction
		if GD(Y(j),X(j)) > 0
			ed = GD(Y(j),X(j))-pi/2;
		else
			ed = GD(Y(j),X(j))+pi/2;
		end
		
		dlh = dlh+GM(Y(j),X(j))*weight(radius,sigma,Dist(j))*abs(cos(ed-TD(i)));
	end
	
	lh = lh+dlh;
	
    if debug == 1
		image(TY(i),TX(i),:) = [255,255,255];
		if mod(i,10) == 0
			for j = 1 : size(X,2)
				image(Y(j),X(j),:) = [255,255,0];
			end
		end
    end
end

if debug == 1
	figure,imshow(image);title('edge searching lines');
end