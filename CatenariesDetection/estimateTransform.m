function tform = estimateTransform(I,npoints)
total_points = 2*npoints;
points = zeros(total_points,2);
figure,imshow(I,'InitialMagnification','fit');
[points(:,1),points(:,2)] = ginput(total_points);
dlmwrite('points.txt',points,' ');

from = points(1:2:size(points,1),:);
minx = min(from(:,1));
maxx = max(from(:,1));
miny = min(from(:,2));
maxy = max(from(:,2));
from(:,1) = from(:,1)-(maxx-minx)/2;
from(:,2) = from(:,2)-(maxy-miny)/2;

to   = points(2:2:size(points,1),:);
minx = min(to(:,1));
maxx = max(to(:,1));
miny = min(to(:,2));
maxy = max(to(:,2));
to(:,1) = to(:,1)-(maxx-minx)/2;
to(:,2) = to(:,2)-(maxy-miny)/2;

tform = estimateGeometricTransform(from,to,'projective');