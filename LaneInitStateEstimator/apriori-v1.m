function ap = apriori(phi1,phi2,y0,mean,cov)

% LIKELIHOOD(I,MODEL,CAMERA,RADIUS,SIGMA)
% Likelihood function of railway state estimator.
%
% Input:      I - single channel image.
%         model - railway state model.
%        camera - camera parameters.
%        radius - fuzzy membership function radius.
%         sigma - Gaussian weight window radius.
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

x = [phi1;phi2;y0];
d = x-mean;

ap = (1/(2*pi)^(3/2)*sqrt(det(cov)))*exp(-(d'*inv(cov)*d)/2);