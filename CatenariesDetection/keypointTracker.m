clear;clc;close;

run('E:\Workspace\SDK\vlfeat-0.9.21-bin\vlfeat-0.9.21\toolbox\vl_setup');

fid = fopen('E:\Workspace\MVS\RailwayMonitor\x64\Debug\samples\00000011.dat', 'rb');
[data,length] = fread(fid, inf, '*uint16');
fclose(fid);

width = 640;
height = 480;
npixels = width * height;
nframes = length / npixels;

handle = figure;
for f = 1 : nframes    
    % Get a frame from buffer
	from = (f - 1) * npixels + 1;
	to = from + npixels - 1;
	frame = data(from : to);
	image = reshape(frame, [width height]);
    image = flipud(image');

    % Unsigned short to unsigned char
    minimum = min(image(:));
    maximum = max(image(:));
    imageSingle = 255*(image-minimum)/(maximum-minimum);
    imageU8 = uint8(imageSingle);

    % edgeImage = edge(image,'canny');
    % figure,imshow(edgeImage);
    
    % dilateImage = imdilate(edgeImage,strel('rectangle',[9,9]));
    % figure,imshow(dilateImage);
    
    % points = detectSURFFeatures(imageU8,'ROI',[1,1,2*width/3,2*height/3]);
    % [features,validPoints]  = extractFeatures(imageU8,points);
        
    % figure,imshow(imageU8);hold on;
    % plot(points.selectStrongest(1000));
    
    [sift_frame,sift_desc] = vl_sift(single(image(1:2*height/3,:)));
    
    % figure, imshow(imageU8);hold on;
    % vl_plotframe(sift_frame);
    
    if f == 1
        imagePrev = imageU8;
        % dilateImagePrev = dilateImage;
        % pointsPrev = validPoints;
        % featuresPrev = features;
        framePrev = sift_frame;
        descPrev = sift_desc;
        continue;
    end
    
    % indexPairs = matchFeatures(featuresPrev, features);
    % matchedPrev = pointsPrev(indexPairs(:,1));
    % matchedCurr = points(indexPairs(:,2));
    [matches,scores] = vl_ubcmatch(descPrev,sift_desc);
    
%     m = 1;
%     SZ = size(matches,2);
%     while (m < SZ)
%         if dilateImagePrev(floor(framePrev(2,matches(1,m))),floor(framePrev(1,matches(1,m)))) == 0 || ...
%             dilateImage(floor(sift_frame(2,matches(2,m))),floor(sift_frame(1,matches(2,m)))) == 0  
%             matches(:,m) = [];
%             scores(m) = [];
%             SZ = SZ - 1;
%         else
%             m = m + 1;
%         end
%     end
    
    % figure,showMatchedFeatures(imagePrev,imageU8,matchedPrev,matchedCurr);
    matchedFramePrev = framePrev(:,matches(1,:));
    matchedFrameCurr = sift_frame(:,matches(2,:));
    
    % matched_image = [imagePrev,imageU8];
    % figure;imshow(matched_image);hold on;
    
    set(gca,'position',[0 0 1 1]);
    imshow(imageU8);hold on;
    for m = 1 : size(matchedFramePrev,2)
        % line([matchedFramePrev(1,m),matchedFrameCurr(1,m)+width],...
        %     [matchedFramePrev(2,m),matchedFrameCurr(2,m)],'Color',[1,1,0],'Marker','s');
        % text(matchedFramePrev(1,m),matchedFramePrev(2,m),sprintf('%d',m),'Color',[0,0,1]);
        % text(matchedFrameCurr(1,m)+width,matchedFrameCurr(2,m),sprintf('%d',m),'Color',[0,0,1]);
        
        dist = sqrt((matchedFramePrev(1,m)-matchedFrameCurr(1,m))^2+(matchedFramePrev(2,m)-matchedFrameCurr(2,m))^2);
        if dist > 50
            continue;
        end
        
        plot(matchedFrameCurr(1,m),matchedFrameCurr(2,m),'Marker','x','Color',[0,0,1]);
        line([matchedFramePrev(1,m),matchedFrameCurr(1,m)],...
            [matchedFramePrev(2,m),matchedFrameCurr(2,m)],'Color',[1,1,0]);
    end
    
    print(sprintf('sift\\%05d.png',f),'-dpng','-r300','-opengl');
    clf;
    
    imagePrev = imageU8;
    % dilateImagePrev = dilateImage;
    framePrev = sift_frame;
    descPrev = sift_desc;
end