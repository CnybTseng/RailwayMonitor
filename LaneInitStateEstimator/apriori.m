function ap = apriori(nw,mean,stdev)

% LIKELIHOOD(I,MODEL,CAMERA,RADIUS,SIGMA)
% Likelihood function of railway state estimator.
%
% Input:    nw - nearest lane width.
%         mean - mean of nearest lane width.
%        stdev - standard deviation of nearest lane width.
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

ap = (1/(sqrt(2*pi)*stdev))*exp(-(double(nw)-double(mean))^2/(2*stdev*stdev));