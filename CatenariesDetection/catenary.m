clear;clc;close all;
RGB = imread('vlcsnap-2018-03-12-18h48m43s724.png');

% Estimate transform
% if exist('transform.mat','file') ~= 2
%     tform = estimateTransform(RGB,10);
% else
%     tform = load('transform.mat','-mat');
% end

% Create sky mask
[Sky,maskedImage] = createMask(RGB);
se = strel('square',15);
CloseSky = imclose(Sky,se);
Ground = imfill(~CloseSky,'holes');
CleanSky = ~Ground;

% Create color object mask
[ColorObjectMask,ColorObjectImage] = createColorObjectMask(RGB);
% ColorObjectMask = imfill(ColorObjectMask, 'holes');

% Set region of interest including catenaries
if exist('ROI.mat', 'file') ~= 2
    figure,imshow(RGB,'InitialMagnification','fit');
    h = imrect;
    position = wait(h);
    position = round(position);
    ROI.position = position;
    ROI.data = RGB(position(2):position(2)+position(4),position(1):position(1)+position(3),:);
    save('ROI.mat','ROI');
    GrayROI = rgb2gray(ROI.data);
else
    ROI = load('ROI.mat','-mat');
    position = ROI.ROI.position;
    GrayROI = rgb2gray(ROI.ROI.data);
end

% Binarization
level = graythresh(GrayROI);
BW = im2bw(GrayROI,level);
imwrite(BW,'BW.png');

% Thinning
IBW = ~BW;
TBW = bwmorph(IBW, 'skel', inf);

Edge = RGB;
Edge(position(2):position(2)+position(4),position(1):position(1)+position(3),1) = 255*TBW;
Edge(position(2):position(2)+position(4),position(1):position(1)+position(3),2) = 255*TBW;
Edge(position(2):position(2)+position(4),position(1):position(1)+position(3),3) = 255*TBW;
imwrite(Edge,'edge.png');

% Select keypoints of catenary
if exist('keypoints.mat','file') ~= 2
    figure,imshow(GrayROI);
    keypoints = zeros(10,2);
    [keypoints(:,1),keypoints(:,2)] = ginput(10);
    minx = min(keypoints(:,1));
    miny = min(keypoints(:,2));
    keypoints(:,1) = keypoints(:,1)-minx;
    keypoints(:,2) = keypoints(:,2)-miny;
    save('keypoints.mat','keypoints');
else
    keypoints = load('keypoints.mat','-mat');
    keypoints = keypoints.keypoints;
end

Carenaries = RGB;
factor = 0.95;
NeighborNode = zeros(2,2);
counter = 1;

for i = 1 : 10
    fprintf('Pattern scale %f,', factor^(i-1));
    % Scale pattern keypoints
    if i ~= 1
        scale_points = keypoints*factor^(i-1);
        minx = min(scale_points(:,1));
        miny = min(scale_points(:,2));
        scale_points(:,1) = scale_points(:,1)-minx+1;
        scale_points(:,2) = scale_points(:,2)-miny+1;
        % For test code
        % [tox,toy] = transformPointsForward(tform.tform.T,keypoints(:,1),keypoints(:,2));
        % fprintf('%f:%f<-->%f:%f\n',scale_points(1,1),scale_points(1,2),tox(1),toy(1));
    else
       scale_points = keypoints+1; 
    end
    
    % Create scaled catenary pattern
    CP = cantileverPattern(scale_points);
    imwrite(CP,sprintf('CP_%d.png',i));
    
    % Chamfer match
    DIST = bwdist(TBW);
    % max_dist = max(DIST(:));
    % DIST(Ground(position(2):position(2)+position(4),position(1):position(1)+position(3),:)) = max_dist;
    ChamferDist = sqrt(filter2(CP,DIST,'valid')/numel(CP))/3;
    [py,px] = find(ChamferDist==min(ChamferDist(:)));

    PatternDist = bwdist(CP);
    
    % Print minimum chamfer distance
    for j = 1 : size(px,1)
        region = TBW(py(j):py(j)+size(CP,1)-1,px(j):px(j)+size(CP,2)-1);
        ReverseChamferDist = sqrt(filter2(region,PatternDist,'valid')/numel(CP))/3;
        fprintf('chamfer distance %f at [%d,%d],reverse %f,%f\n',ChamferDist(py(j),px(j)),px(j) + position(1),py(j) + position(2),...
            ReverseChamferDist,ChamferDist(py(j),px(j))*ReverseChamferDist);
    end
    
    % Draw chamfer match result
    px = px + position(1);
    py = py + position(2);
    if i == 1 || i == 10
        NeighborNode(counter,1) = scale_points(1,1)+px(1);
        NeighborNode(counter,2) = scale_points(1,2)+py(1);
        counter = counter+1;
        Carenaries = drawCarenary(Carenaries,CP,px,py);
    end
end

% Obstacle detection region
DetectionRegionMask = roipoly(RGB,[NeighborNode(1,1) NeighborNode(1,1)+64 NeighborNode(2,1)+64 NeighborNode(2,1)],...
    [NeighborNode(1,2) NeighborNode(1,2) NeighborNode(2,2) NeighborNode(2,2)]);
DetectionRegionMaskU8 = uint8(DetectionRegionMask);
DetectionRegion(:,:,1) = RGB(:,:,1).*DetectionRegionMaskU8;
DetectionRegion(:,:,2) = RGB(:,:,2).*DetectionRegionMaskU8;
DetectionRegion(:,:,3) = RGB(:,:,3).*DetectionRegionMaskU8;
DetectionBoundary = bwboundaries(DetectionRegionMask);

ObstacleMask = ColorObjectMask&DetectionRegionMask;
ObstacleBoundaries = bwboundaries(ObstacleMask);

% Draw obstacle image
ObstacleImage = RGB;
ObstacleImage = drawBoundaries(ObstacleImage,DetectionBoundary,[0 0 255]);
ObstacleImage = drawBoundaries(ObstacleImage,ObstacleBoundaries,[255 255 0]);

% Save result as pictures
imwrite(Sky,'sky.png');
imwrite(CleanSky,'cleansky.png');
imwrite(Carenaries,'catenaries.png');
imwrite(ColorObjectMask,'ColorObjectMask.png');
imwrite(DetectionRegion,'DetectionRegion.png');
imwrite(ObstacleImage,'CatenariesObstacle.png');






