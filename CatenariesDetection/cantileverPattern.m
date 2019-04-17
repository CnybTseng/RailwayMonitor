function CP = cantileverPattern(keypoints)

width = max(keypoints(:,1));
height = max(keypoints(:,2));
CP = zeros(round(height),round(width));

nkeypoints = size(keypoints,1)/2;
for i = 1 : nkeypoints
    start_id = 2*(i-1)+1;
    CP = setLine(CP,keypoints(start_id:start_id+1,:)); 
end