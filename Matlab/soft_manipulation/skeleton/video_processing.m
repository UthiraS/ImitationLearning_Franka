function[] = video_processing(str_loc)
close all
% Location and folder # to store plots

vid1 = str_loc + '/robotVideo.avi';
vid2 = str_loc + '/curveVideo.avi';
vid1 = VideoReader(vid1);
vid2 = VideoReader(vid2);

videoPlayer = vision.VideoPlayer;

% new video
newvid = str_loc + '/split.avi';
outputVideo = VideoWriter(newvid);
outputVideo.FrameRate = vid1.FrameRate/4;
open(outputVideo);

while hasFrame(vid1) && hasFrame(vid2)
    img1 = readFrame(vid1);
    img2 = readFrame(vid2);

    imgt = horzcat(img1, img2);

    % play video
    step(videoPlayer, imgt);

    % record new video
    writeVideo(outputVideo, imgt);
end

release(videoPlayer);
close(outputVideo);
