% function estimator(varargin)

% ESTIMATOR(VARARGIN)
% Railway state model MAP estimation base on Tabu search
%
% Input:
%
% Output:
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

clear;clc;close all;

global fig;
global debug;
global I;
global result;
global model;
global camera;
global radius;
global sigma;
global mean;
global cov;
global stdev;
global bm;

debug = 1;									% debug switch
inference = ~debug;

I = imread('test2.png');
result = repmat(I,[1,1,3]);

% Camera parameters
camera.xreso = 640;							% X-resolution (pix)
camera.yreso = 480;							% Y-resolution (pix)
camera.pp    = 17;							% pixel pitch (um)
camera.lens  = 150;							% focal length (mm)
camera.sx    = 1000/camera.pp;				% X-scale factor (pix/mm)
camera.sy    = camera.sx;					% Y-scale factor (pix/mm)
camera.fx    = camera.sx * camera.lens;		% pixel unit focal length in x direction (pix)
camera.fy    = camera.fx;					% pixel unit focal length in y direction (pix)
camera.cx    = camera.xreso/2;				% x of principal point (pix)
camera.cy    = camera.yreso/2;				% y of principal point (pix)
camera.C     = 17.45;
alpha = camera.pp*camera.xreso/camera.lens/camera.C;
beta  = camera.pp*camera.yreso/camera.lens/camera.C;
camera.alpha = alpha;						% horizontal field of view (degree)
camera.beta  = beta;						% vertical field of view (degree)
camera.h     = 1.75;						% camera installation height

% Railway state model
model.phi1 = 0;							% yaw angle
model.phi2 = 0;						% pitch angle
model.y0   = 0;								% lateral offset
model.b    = 1.435;							% railway width
model.ch0  = 0;								% horizontal curvature
model.ch1  = 0;								% alteration of ch0
model.cv0  = 0;						% vertical curvature

% A priori knowledge about railway.
mean = 381;									% mean of nearest lane width (pix)
stdev = 15;									% standard deviation of nearest lane width

radius = 63;								% cutoff radius of weight function
sigma  = 15;								% standard deviation of of weight function

% Lane width distribution
lane = [1:size(I,2)];
prob = (1/(sqrt(2*pi)*stdev)).*exp(-(double(lane)-double(mean)).^2/(2*stdev*stdev));
figure,plot(lane,prob);title('nearest lane width distribution');
xlabel('nearest lane width');
ylabel('probability');

if ~inference
	[handle,model] = ctlmodel(camera,model,I);
	while ishandle(handle)
		pause(1);
	end
	
	[GM,GD,mask] = edgerespond(I);
	figure,subplot(1,2,1),imshow(GM,[]);title('Gradient maginitude');
	
	[TX,TY,TD,nL,nw] = caltrackpoint(camera,model);
	
	track = zeros(size(I),'uint8');
	for i = 1 : size(TX,1)
		track(TY(i),TX(i)) = 255;
	end
	
	subplot(1,2,2),imshow(track);title('projected track points');
	imwrite(track,'track.png');
	
	lh = likelihood(GM,GD,mask,TX,TY,TD,radius,sigma);
else
	warning('off');
	format long;
	format compact;
	tabusearch;
	format;
end