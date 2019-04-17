% function test_railway_state_model(varargin)

% TEST_RAILWAY_STATE_MODEL(VARARGIN)
% Test railway state model
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
model.phi1 = 0;								% yaw angle
model.phi2 = -0.01;								% pitch angle
model.y0   = 0;								% lateral offset
model.b    = 1.435;							% railway width
model.ch0  = 0.00003;						% horizontal curvature
model.ch1  = 0;								% alteration of ch0
model.cv0  = 0;								% vertical curvature

[X,Y,nL] = caltrackpoint(camera,model);

mask = zeros(camera.yreso, camera.xreso);

for i = 1:size(X,1)
	mask(Y(i,1),X(i,1)) = 255;
end

imshow(mask);