function J = setLine(I,keypoints)
minx = min(keypoints(1,1),keypoints(2,1));
miny = min(keypoints(1,2),keypoints(2,2));
maxx = max(keypoints(1,1),keypoints(2,1));
maxy = max(keypoints(1,2),keypoints(2,2));

J = I;

if maxy-miny > maxx-minx
    for y = miny : maxy
        if minx ~= maxx
            x = ((y-keypoints(1,2))*keypoints(2,1)-(y-keypoints(2,2))*keypoints(1,1))/(keypoints(2,2)-keypoints(1,2));
            J(round(y),round(x))=1;
        else
            x = minx;
            J(round(y),round(x))=1;
        end
    end
else
    for x = minx : maxx
        y = ((x-keypoints(2,1))*keypoints(1,2)-(x-keypoints(1,1))*keypoints(2,2))/(keypoints(1,1)-keypoints(2,1));
        J(round(y),round(x))=1;
    end
end
