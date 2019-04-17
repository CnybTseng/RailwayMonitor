clear;clc;close;

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

for p = 1 : size(patterns,2)
    keypoints = patterns{p}+1;
    CP = cantileverPattern(keypoints);
    break;
end

threshold = 0.12;
lineMatchingPara = struct(...
    'NUMBER_DIRECTION',60,...
    'DIRECTIONAL_COST',0.5,...
    'MAXIMUM_EDGE_COST',30,...
    'MATCHING_SCALE',1.0,...
    'TEMPLATE_SCALE',0.6761,...
    'BASE_SEARCH_SCALE',1.20,...
    'MIN_SEARCH_SCALE',-7,...
    'MAX_SEARCH_SCALE',0,...
    'BASE_SEARCH_ASPECT',1.1,...
    'MIN_SEARCH_ASPECT',-1,...
    'MAX_SEARCH_ASPECT',1,...    
    'SEARCH_STEP_SIZE',2,...
    'SEARCH_BOUNDARY_SIZE',2,...
    'MIN_COST_RATIO',1.0...    
    );

% Set the parameter for line fitting function
lineFittingPara = struct(...
    'SIGMA_FIT_A_LINE',0.5,...
    'SIGMA_FIND_SUPPORT',0.5,...
    'MAX_GAP',5.0,...
    'N_LINES_TO_FIT_IN_STAGE_1',300,...
    'N_TRIALS_PER_LINE_IN_STAGE_1',100,...
    'N_LINES_TO_FIT_IN_STAGE_2',100000,...
    'N_TRIALS_PER_LINE_IN_STAGE_2',1);

[lineRep,lineMap] = mex_fitline(double(CP),lineFittingPara);
imshow(lineMap);

% Set the parameter for line fitting function
lineFittingPara2 = struct(...
    'SIGMA_FIT_A_LINE',0.5,...
    'SIGMA_FIND_SUPPORT',0.5,...
    'MAX_GAP',2.0,...
    'N_LINES_TO_FIT_IN_STAGE_1',0,...
    'N_TRIALS_PER_LINE_IN_STAGE_1',0,...
    'N_LINES_TO_FIT_IN_STAGE_2',100000,...
    'N_TRIALS_PER_LINE_IN_STAGE_2',1);


template = cell(1);
tempate{1} = lineRep;

% handle_edge = figure;
% handle_catenaries = figure;

for f = 1 : nframes    
    % Get a frame from buffer
	from = (f - 1) * npixels + 1;
	to = from + npixels - 1;
	frame = data(from : to);
	image = reshape(frame, [width height]);
    image = flipud(image');
    
    % Gray range compression
    minimum = min(image(:));
    maximum = max(image(:));
    imageU8 = uint8(255*(image-minimum)/(maximum-minimum));
    
    [edgeImage,thresh] = edge(image,'canny');
    
    imwrite(edgeImage,sprintf('edge\\%05d.png',f));
    
%     [detWinds] = mex_fdcm_detect(double(query),tempate,threshold,...
%     lineFittingPara2,lineMatchingPara);
%     
%     imshow(imageU8);hold on;
%     set(gca,'position',[0 0 1 1]);
%     color = [1 1 0];
%     lineWidth = 3;
%     for i=1:size(detWinds)
%         sx = detWinds(i,1);
%         ex = sx + detWinds(i,3);
%         sy = detWinds(i,2);
%         ey = sy + detWinds(i,4);
%         line([sx ex],[sy sy],'Color',color,'LineWidth',lineWidth);
%         line([sx ex],[ey ey],'Color',color,'LineWidth',lineWidth);
%         line([sx sx],[sy ey],'Color',color,'LineWidth',lineWidth);
%         line([ex ex],[sy ey],'Color',color,'LineWidth',lineWidth);
%     end
% 
%     print(sprintf('chamfer\\%05d.png',f),'-dpng','-r100','-opengl');
%     clf;
end