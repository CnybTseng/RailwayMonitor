function BI = drawBoundaries(I,boundaries,color)
BI = I;
for i = 1 : size(boundaries,1)
    Boundary = boundaries{i};
    for j = 1 : size(Boundary,1)
        BI(Boundary(j,1),Boundary(j,2),1) = color(1);
        BI(Boundary(j,1),Boundary(j,2),2) = color(2);
        BI(Boundary(j,1),Boundary(j,2),3) = color(3);
    end
end