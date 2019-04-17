function flag = match(a,b)

% dist = sqrt(sum((a-b).^2));
dist = sum(abs((a-b)./a))/size(a,2);

flag = dist<0.005;