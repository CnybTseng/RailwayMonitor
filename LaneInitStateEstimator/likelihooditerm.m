function lhi = likelihooditerm(sz,ex,ey,sgm,sgd,radius,sigma,TD,tindmap)

lhi = 0;
top = max(1,ey-radius);
bottom = min(sz(1),ey+radius);
left = max(1,ex-radius);
right = min(sz(2),ex+radius);
mindist = Inf;
minx = 0;
miny = 0;

for y = top : bottom
	for x = left : right
		if tindmap(y,x) == 0
			continue;
		end
		dist = sqrt((x-ex)^2+(y-ey)^2);
		if dist < mindist
			mindist = dist;
			minx = x;
			miny = y;
		end
	end
end

if mindist == Inf
	return;
end

% Edge direction
if sgd > 0
	ed = sgd-pi/2;
else
	ed = sgd+pi/2;
end

theta = abs(ed-TD(tindmap(miny,minx)));			% [0,pi]
if theta>pi/2
	theta = pi-theta;
end

lhi = sgm*weight(radius,sigma,mindist)*cos(theta);
% fprintf('sgm=%f,theta=%f,lhi=%f\n',sgm,theta,lhi);