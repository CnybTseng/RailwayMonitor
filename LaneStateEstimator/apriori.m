function ap = apriori(model,mean,cov)

% LIKELIHOOD(I,MODEL,CAMERA,RADIUS,SIGMA)
% Likelihood function of railway state estimator.
%
% Input:    nw - nearest lane width.
%         mean - mean of nearest lane width.
%          cov - .
%
% Output: ap - priori probability
%
% Author:Zhiwei Zeng
% Date:2018.07.17
%
% Copyright (C) 2018 Zhiwei Zeng.
% Copyright (C) 2018 Chengdu ZLT Technology Co., Ltd.
% All rights reserved.
%
% This file is part of the railway monitor toolkit and is made available under
% the terms of the BSD license (see the COPYING file).

x = [model.phi1;model.phi2;model.y0;model.ch0;model.ch1;model.cv0];
z = x-mean;
ap = (1/(2*pi)^(3/2)*sqrt(det(cov)))*exp(-(z'*inv(cov)*z)/2);