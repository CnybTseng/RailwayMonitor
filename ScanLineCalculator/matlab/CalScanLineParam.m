function [maxinter,scale] = CalScanLineParam(numinter,mininter,suminter)

P = zeros(1,numinter+1);
P(1) = suminter-mininter;
P(2) = -suminter;
P(end) = mininter;
rs = roots(P);

% Get real solution
scale = 1;
for i = 1 : size(rs,1)
	if real(rs(i)) < 0.99999 && abs(imag(rs(i))) < 1e-10
		scale = real(rs(i));
		break;
	end
end

scale = single(scale);
maxinter = suminter*(1-scale)/(1-scale^numinter);