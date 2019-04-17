function createPattern

pattern_image_list = {'positive.png','negative.png'};
nkeypoints = [12,12];
patterns = cell(1,2);

for i = 1 : size(pattern_image_list,2)
    image = imread(pattern_image_list{i});
    figure,imshow(image);
    keypoints = zeros(nkeypoints(i),2);
    [keypoints(:,1),keypoints(:,2)] = ginput(nkeypoints(i));
    close;
    minx = min(keypoints(:,1));
    miny = min(keypoints(:,2));
    keypoints(:,1) = keypoints(:,1)-minx;
    keypoints(:,2) = keypoints(:,2)-miny;
    patterns{i} = keypoints;
end

save('patterns.mat','patterns');