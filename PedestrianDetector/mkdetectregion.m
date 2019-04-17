clear;clc;
close all;

% Camera parameters
camera.xreso = 640;							% X-resolution (pix)
camera.yreso = 480;							% Y-resolution (pix)
camera.pp    = 17;							% pixel pitch (um)
camera.lens  = 150;							% focal length (mm)
camera.sx    = 1000/camera.pp;				% X-scale factor (pix/mm)
camera.sy    = camera.sx;					% Y-scale factor (pix/mm)
camera.fx    = camera.sx * camera.lens;		% pixel unit focal length in x direction (pix)
camera.fy    = camera.fx;					% pixel unit focal length in y direction (pix)
camera.cx    = camera.xreso/2;				% x of principal point (pix)
camera.cy    = camera.yreso/2;				% y of principal point (pix)
camera.C     = 17.45;
alpha = camera.pp*camera.xreso/camera.lens/camera.C;
beta  = camera.pp*camera.yreso/camera.lens/camera.C;
camera.alpha = alpha;						% horizontal field of view (degree)
camera.beta  = beta;						% vertical field of view (degree)
camera.h     = 1.75;						% camera installation height

% Railway state model
model.phi1 = 0;								% yaw angle
model.phi2 = 0;								% pitch angle
model.y0   = 0;								% lateral offset
model.b    = 1.435;							% railway width
model.ch0  = 0;								% horizontal curvature
model.ch1  = 0;								% alteration of ch0
model.cv0  = -1e-5;								% vertical curvature

[TX,TY,TD,nL,nw] = caltrackpoint(camera,model);

model2 = model;
model2.b = 4.88;
[TX2,TY2,TD2,nL2,nw2] = caltrackpoint(camera,model2);
	
image = zeros(camera.yreso,camera.xreso,'uint8');
for i = 1 : size(TX,1)
	image(TY(i),TX(i)) = 255;
end

for i = 1 : size(TX2,1)
	image(TY2(i),TX2(i)) = 255;
end

vanishY = min(TY(:));
cellsz = 4;
winw = 16;
winh = 32;
shoubredth = 0.4;							% shoulder breadth
firstline=1;
firsty = 0;						

lastli = 1;
lastly = TY(1);
lastri = nL+1;
lastry = TY(nL+1);
for i = 1 : nL
	% Align Y position of left and right track.
	while lastli<=nL && TY(lastli)~=lastly
		lastli = lastli+1;
	end
	
	while lastri<=size(TY,1) && TY(lastri)~=lastry
		lastri = lastri+1;
	end
	
	if lastli>nL || lastri>size(TY,1)
		break;
	end
	
	wid = int32((shoubredth/model.b)*(TX(lastri)-TX(lastli)));
	hei = int32(wid*4);
	
	if wid < winw && firstline==1
		image(TY(lastli)-hei+1,:)=128;
		firsty=TY(lastli);
		firstline=0;
	end
	
	fprintf('footprint:%d,%d;%d,%d.win:%d,%d.\n',TX(lastli),TY(lastli),TX(lastri),TY(lastri),wid,hei);
	image = drawrect(image,TX(lastli),TY(lastli),wid,hei);
	image = drawrect(image,TX(lastri)-wid+1,TY(lastri),wid,hei);
		
	lastly = lastly-16;
	lastry = lastry-16;
end

% imshow(image);
imwrite(image,'pdr.png');

minscale = 1;
maxscale = min(camera.xreso/winw,camera.yreso/winh);

scale = minscale:.05:maxscale;

N = camera.yreso-firsty+1;
weight = zeros(N,size(scale,2));
sigma = maxscale/3;

figure;
for i = 1:20:N
	mean = (i-1)*(maxscale-minscale)/(N-1)+minscale;
	weight(i,:) = exp(-(scale-mean).^2/sigma^2);
	plot(scale,weight(i,:));
	xlim([minscale,maxscale]);
	hold on;
end

xlabel('scale');
ylabel('confidence');
















% confidence1 = ((scale-maxscale)/(maxscale-minscale)).^12;
% confidence2 = ((scale-minscale)/(maxscale-minscale)).^12;

% figure,plot(scale,confidence1);
% hold on;plot(scale,confidence2);
% xlim([minscale,maxscale]);
% ylim([0,1]);

% for y = firsty:20:camera.yreso
% 	confidence = ((y-firsty)/(camera.yreso-firsty))*confidence2+((camera.yreso-y)/(camera.yreso-firsty))*confidence1;
% 	hold on;plot(scale,confidence);
% end

% confidence = 0:.001:1;
% constant = 1e8;
% scale2 = (maxscale-minscale).*(1-1./(1+constant.^(.5-confidence)))+minscale;
% figure,plot(confidence,scale2);
% 
% scale = minscale:.0005:maxscale;
% confidence = 0.5-(log(scale-minscale)-log(maxscale-scale))/log(constant);
% figure,plot(scale,confidence);
% xlim([minscale,maxscale]);