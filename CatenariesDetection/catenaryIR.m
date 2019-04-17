xclear;clc;close;

fid = fopen('E:\Workspace\MVS\RailwayMonitor\x64\Debug\samples\00000011.dat', 'rb');
[data,length] = fread(fid, inf, '*uint16');
fclose(fid);

width = 640;
height = 480;
npixels = width * height;
nframes = length / npixels;

% Select keypoints of catenary
if exist('patterns.mat','file') ~= 2
    createPattern;
else
    patterns_struct = load('patterns.mat','-mat');
    patterns = patterns_struct.patterns;
end

% handle_edge = figure;
handle_catenaries = figure;

for f = 1 : nframes    
    % Get a frame from buffer
	from = (f - 1) * npixels + 1;
	to = from + npixels - 1;
	frame = data(from : to);
	image = reshape(frame, [width height]);
    image = flipud(image');
    
    minimum = min(image(:));
    maximum = max(image(:));
    imageU8 = uint8(255*(image-minimum)/(maximum-minimum));
    
    % Set region of interest including catenaries
    if f == 1 && exist('ROISet.mat', 'file') ~= 2
        figure,imshow(imageU8,'InitialMagnification','fit');
        h = imrect;
        position = wait(h);
        if ~isempty(position)
            position = round(position);
            save('ROISet.mat','position');
            imwrite(imageU8,'first.png');
        end
    else
        ROISet = load('ROISet.mat','-mat');
        position = ROISet.position;
    end
    
    % Clip region of interest
    ROI = image(position(2):position(2)+position(4),position(1):position(1)+position(3));
    
    % Dilate
    % dilateROI = imdilate(ROI,strel('rectangle',[5,5]));
    
    % ROIU8 = uint8(255*(ROI-min(ROI(:)))/(max(ROI(:))-min(ROI(:))));
    % imwrite(ROIU8,'ROI.png');
    
    % Binarization
    % level = graythresh(ROIU8);
    % BW = im2bw(ROIU8,0.5);
    
    [BW,thresh] = edge(ROI,'canny',[0.1,0.3]);
    
    % stats = regionprops(BW,'all');
    % se = strel('rectangle',[7,7]);
    % BW = imclose(BW,se);
    % BW = bwmorph(BW, 'skel', inf);
    
    % blurROI = imgaussfilt(ROI);
    % sobelX = single(imfilter(blurROI,[-1 -2 -1;0 0 0;1 2 1],'symmetric'));
    % sobelY = single(imfilter(blurROI,[-1 0 1;-2 0 2; -1 0 1],'symmetric'));
    % sobelImage = sqrt(sobelX.^2+sobelY.^2);
    
    % level = (min(sobelImage(:))+max(sobelImage(:)))/7;
    % BW = sobelImage>level;
    
    imwrite(BW,sprintf('edge\\%05d.png',f));
    % imshow(BW,[]);
    % continue;
    
    % Hough transform
%     [H,theta,rho] = hough(BW,'RhoResolution',0.5,'ThetaResolution',0.1);
%     peaks = houghpeaks(H,64,'threshold',ceil(0.1*max(H(:))));
%     lines = houghlines(BW,theta,rho,peaks,'FillGap',5,'MinLength',16);
%     
%     BWU8 = uint8(255*BW);
%     RGBBW(:,:,1) = BWU8;
%     RGBBW(:,:,2) = BWU8;
%     RGBBW(:,:,3) = BWU8;
%     h = figure;imshow(RGBBW),hold on;
%     for k = 1:size(lines,2)
%        xy = [lines(k).point1; lines(k).point2];
%        plot(xy(:,1),xy(:,2),'LineWidth',1,'Color','green');
% 
%        % Plot beginnings and ends of lines
%        plot(xy(1,1),xy(1,2),'x','LineWidth',1,'Color','yellow');
%        plot(xy(2,1),xy(2,2),'x','LineWidth',1,'Color','red');
%     end

    % Thinning
    % IBW = ~BW;
    % TBW = bwmorph(IBW, 'skel', inf);
    
    % Edge = imageU8;
    % Edge(position(2):position(2)+position(4),position(1):position(1)+position(3)) = 255*TBW;
    % imwrite(Edge,'edge.png');
    % figure(handle_edge),imshow(Edge);

    factor = 0.95;
    Carenaries(:,:,1) = imageU8;
    Carenaries(:,:,2) = imageU8;
    Carenaries(:,:,3) = imageU8;
    counter = 1;
    CPS = {};
    matches = [];
    
    % For each scale
    for s = 1 : 20
        % For each pattern
        for p = 1 : size(patterns,2)
            % Scale pattern keypoints
            keypoints = patterns{p};
            if s ~= 1
                scaled_keypoints = keypoints*factor^(s-1);
                minx = min(scaled_keypoints(:,1));
                miny = min(scaled_keypoints(:,2));
                scaled_keypoints(:,1) = scaled_keypoints(:,1)-minx+1;
                scaled_keypoints(:,2) = scaled_keypoints(:,2)-miny+1;
            else
               scaled_keypoints = keypoints+1; 
            end

            % Create scaled catenary pattern
            CP = cantileverPattern(scaled_keypoints);
            % imwrite(CP,sprintf('CP_%d_%d.png',s,p));

            CPS{s,p} = CP;
            
            % Chamfer match
            DIST = bwdist(BW);
            ChamferDist = sqrt(filter2(CP,DIST,'valid')/numel(CP))/3;
            [py,px] = find(ChamferDist==min(ChamferDist(:)));
            px = px + position(1);
            py = py + position(2);
            % Carenaries = drawCarenary(Carenaries,CP,px,py);
            for i = 1 : size(px,1)
                dist = ChamferDist(py(i)-position(2),px(i)-position(1));
                if dist > 1.0
                    continue;
                end
                matches(counter,1) = px(i);
                matches(counter,2) = py(i);
                matches(counter,3) = size(CP,2);
                matches(counter,4) = size(CP,1);
                matches(counter,5) = dist;
                matches(counter,6) = s;
                matches(counter,7) = p;
                counter = counter + 1;
            end
        end
    end

    nms = nonMaximumSurpression(matches);
    for i = 1 : size(nms,1)
        Carenaries = drawCarenary(Carenaries,CPS{nms(i,6),nms(i,7)},nms(i,1),nms(i,2));
        % se = strel('rectangle',[7,7]);
        % dilatePattern = imdilate(CPS{nms(i,6),nms(i,7)},se);
        % dilatePattern = CPS{nms(i,6),nms(i,7)};
        % subRoi = BW(nms(i,2)-position(2):nms(i,2)-position(2)+nms(i,4)-1,...
        %     nms(i,1)-position(1):nms(i,1)-position(1)+nms(i,3)-1);
        % care = dilatePattern&subRoi;
        % Carenaries = drawCarenary(Carenaries,care,nms(i,1),nms(i,2));
    end
    
    set(gca,'position',[0 0 1 1]);
    figure(handle_catenaries),imshow(Carenaries);
    rectangle('Position',position,'EdgeColor',[1,1,0]);
    
    print(sprintf('chamfer\\%05d.png',f),'-dpng','-r100','-opengl');
    clf;
    
    % break;
end

