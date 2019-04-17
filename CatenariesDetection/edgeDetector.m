clear;clc;close;

fid = fopen('E:\Workspace\MVS\RailwayMonitor\x64\Debug\samples\00000011.dat', 'rb');
[data,length] = fread(fid, inf, '*uint16');
fclose(fid);

width = 640;
height = 480;
npixels = width * height;
nframes = length / npixels;

for f = 1 : nframes    
    % Get a frame from buffer
	from = (f - 1) * npixels + 1;
	to = from + npixels - 1;
	frame = data(from : to);
	image = reshape(frame, [width height]);
    image = flipud(image');
    
    blurImage = medfilt2(image);
    [BW,thresh] = edge(blurImage,'Canny');
    
    % singleImage = single(image);
    % sobelX = imfilter(singleImage,[-3,0,3;-10,0,10;-3,0,3],'replicate');
    % sobelY = imfilter(singleImage,[-3,-10,-3;0,0,0;3,10,3],'replicate');
    % BW = sqrt(sobelX.^2+sobelY.^2);
    % BW = abs(sobelX)+abs(sobelY);
    
    imshow(BW,[]);
end