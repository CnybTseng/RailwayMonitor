BW = zeros(480,640,'uint8');

clear;clc;close all;

%====================V1=================

% a1 = double(32);
% q = 0.92;
% sum = double(450);
% 
% q = max(q,1-a1/sum+0.01);
% 
% n = int32(log(1-sum*(1-q)/a1)/log(q)); 
% 
% BW(480,1:640)=uint8(255);
% for i = 1 : n
% 	si = a1*(1-q^double(i))/(1-q);
% 	y = int32(480-si);
% 	if y < 1
% 		break;
% 	end
% 	BW(y,1:640)=uint8(255);
% end

%====================V2=================

n = 30;
an = 2;
sum = double(281);

% q = 0.95;
% a1 = sum*(1-q)/(1-q^n);
% a1 = max(a1,2*q^(1-n));

[a1,q] = CalScanLineParam(n,an,sum);

BW(480,1:640)=uint8(255);
for i = 1 : n
	si = a1*(1-q^double(i))/(1-q);
	y = int32(480-si+0.5);
	if y < 1
		break;
	end
	BW(y,1:640)=uint8(255);
end

fprintf('real sum %f.',a1*(1-q^n)/(1-q));
fprintf('\r\n');

imshow(BW);
title('search lines');
imwrite(BW,'searchline.png');