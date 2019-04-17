function Carenaries = drawCarenary(I,CP,px,py)

Carenaries = I;
mask = zeros(size(I));

for i = 1 : size(px,1)
    % Top side
    % Carenaries(py(i),px(i):px(i)+size(CP,2),1) = 255;
    % Carenaries(py(i),px(i):px(i)+size(CP,2),2) = 255;
    % Carenaries(py(i),px(i):px(i)+size(CP,2),3) = 0;
    % Bottom side
    % Carenaries(py(i)+size(CP,1),px(i):px(i)+size(CP,2),1) = 255;
    % Carenaries(py(i)+size(CP,1),px(i):px(i)+size(CP,2),2) = 255;
    % Carenaries(py(i)+size(CP,1),px(i):px(i)+size(CP,2),3) = 0;
    % Left side
    % Carenaries(py(i):py(i)+size(CP,1),px(i),1) = 255;
    % Carenaries(py(i):py(i)+size(CP,1),px(i),2) = 255;
    % Carenaries(py(i):py(i)+size(CP,1),px(i),3) = 0;
    % Right side
    % Carenaries(py(i):py(i)+size(CP,1),px(i)+size(CP,2),1) = 255;
    % Carenaries(py(i):py(i)+size(CP,1),px(i)+size(CP,2),2) = 255;
    % Carenaries(py(i):py(i)+size(CP,1),px(i)+size(CP,2),3) = 0;
    % Pattern
    mask(py(i):py(i)+size(CP,1)-1,px(i):px(i)+size(CP,2)-1,1) = CP;
    mask(py(i):py(i)+size(CP,1)-1,px(i):px(i)+size(CP,2)-1,2) = CP;
    mask(py(i):py(i)+size(CP,1)-1,px(i):px(i)+size(CP,2)-1,3) = 0;
end

mask = logical(mask);
Carenaries(mask) = 255;