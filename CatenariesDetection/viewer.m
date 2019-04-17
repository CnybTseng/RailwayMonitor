nframes = 200;

videoName = 'tracks-3.avi';
fps = 25;
videoObject = VideoWriter(videoName);
videoObject.FrameRate = fps;
open(videoObject);

for i = 1 : nframes

filename = sprintf('chamfer\\%05d.png', i);
I = imread(filename);
writeVideo(videoObject, I);
imshow(I);
title('Railway');

end

close(videoObject);