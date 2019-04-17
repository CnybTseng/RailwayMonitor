% function railway_state_model(varargin)

% MAIN(VARARGIN)
% Railway state model demo.
%
% Input:
%
% Output:
%
% Author:Zhiwei Zeng
% Date:2018.07.11
%
% Copyright (C) 2018 Zhiwei Zeng.
% Copyright (C) 2018 Chengdu ZLT Technology Co., Ltd.
% All rights reserved.
%
% This file is part of the railway monitor toolkit and is made available under
% the terms of the BSD license (see the COPYING file).

clear;clc;close all;

% Camera parameters
xreso = 640;				% X-resolution (pix)
yreso = 480;				% Y-resolution (pix)
pp    = 17;					% pixel pitch (um)
lens  = 150;				% focal length (mm)
sx    = 1000/pp;			% X-scale factor (pix/mm)
sy    = sx;					% Y-scale factor (pix/mm)
fx    = sx * lens;			% pixel unit focal length in x direction (pix)
fy    = fx;					% pixel unit focal length in y direction (pix)
cx    = xreso/2;			% x of principal point (pix)
cy    = yreso/2;			% y of principal point (pix)
C     = 17.45;

I = imread('test.png');

gx = [-1,0,1;-2,0,2;-1,0,1];
gy = [-1,-2,-1;0,0,0;1,2,1];

Dx = imfilter(double(I),gx,'symmetric');
Dy = imfilter(double(I),gy,'symmetric');

figure,imshow(Dx,[]);
title('Dx');

figure,imshow(Dy,[]);
title('Dy');

Mag = sqrt(Dx.^2+Dy.^2);
figure,imshow(Mag,[]);
title('Magnitude');

Dir = atan(Dy./Dx);
figure,imshow(Dir,[]);
title('Direction');

LowTh = (min(Mag(:))+max(Mag(:)))/5;

sub = Mag>LowTh;
FMag = Mag(sub);
FDir = Dir(sub);

A = [-0.1,-0.1,-0.7175,-0.001,-0.0001,-0.001];
B = [ 0.1, 0.1, 0.7175, 0.001, 0.0001, 0.001];

X0 = [0,0,0,0,0,0];
