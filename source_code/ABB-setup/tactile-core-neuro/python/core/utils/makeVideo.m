clear all; clc

% testing and training sets
homeDir = fullfile(getenv('DATAPATH'), 'TacTip_servocontrol', 'explore3d');
% replayDirName = 'waveSlide\explore5dCNNBatch080921'; time = cellstr(num2str([25]','%02d'));
% replayDirName = 'headSlide\explore3dCNNBatch082220'; time = cellstr(num2str([01 15 50]','%02d'));
% replayDirName = 'ballSlide\rTheta\explore3dCNNBatch081609'; time = cellstr(num2str([45 48 51 57]','%02d'));
replayDirName = 'waveSlide\explore3dCNNBatch081613'; time = cellstr(num2str([41 47 54 58]','%02d'));
% replayDirName = 'objectSlide\explore5dCNNBatch080921'; time = cellstr(num2str([45 50 56]','%02d'));
videos = {'figContour' 'figFrames' 'figControl' 'figCamera'};

% homeDir = fullfile(getenv('DATAPATH'), 'TacTip_servocontrol', 'servo5d');
% % replayDirName = 'servo2d_dnn_032319'; time = cellstr(num2str([54]','%02d'));
% % replayDirName = 'saddle\servo5d_dnn_032221'; time = cellstr(num2str([01]','%02d'));
% % replayDirName = 'lid\servo5d_dnn_batch_032321'; time = cellstr(num2str([30 37 41]','%02d'));
% % replayDirName = 'wave\servo5d_dnn_032415'; time = cellstr(num2str([02]','%02d'));
% replayDirName = 'servo5d_dnn_032217'; time = cellstr(num2str([11]','%02d'));
% videos = {'contour' 'frames' 'control' 'camera'};

replayDir = fullfile(homeDir, replayDirName);

% open saved videos
for i = 1:length(videos) 
    for t = 1:length(time), [i t]      
        vidFile{i,t} = fullfile([replayDir time{t}], [videos{i} '.avi']);
        video{i,t} = VideoReader(vidFile{i,t});
        nFrames(i,t) = round(video{i,t}.Duration*video{i,t}.FrameRate);
        image_t{i,t} = read(video{i,t}, [1 nFrames(i,t)]);     
    end
end
for t = 1:length(time)
    rFrames = 1 : min(nFrames(:,t));
    for i = 1:length(videos)
        image_t{i,t} = image_t{i,t}(:,:,:,rFrames);
    end
end
for i = 1:length(videos); image{i} = cat(4, image_t{i,:}); end
r = 75 + (1:600); for i = 4; image{i} = image{i}(:,r,:,:); end % truncate video

% % combined video
% imageNew = cat(2, cat(1,image{2},image{3}), cat(1,image{4},image{1}));
% % imageNew = cat(1, image{1}, cat(2,image{2},image{3}));
% 
% % startup video
% videoNew = VideoWriter(fullfile(fileparts(replayDir), 'video.avi'));
% videoNew.FrameRate = 10;
% open(videoNew)
% 
% for f = 1:size(imageNew,4)
%     frame.cdata = imageNew(:,:,:,f); frame.colormap = [];
%     writeVideo(videoNew, frame)
% end
% close(videoNew); 

% individual videos
for i = 1:length(videos)
    videoNew = VideoWriter(fullfile(fileparts(replayDir), ['video' videos{i}(4:end) '.avi']));
    videoNew.FrameRate = 10;
    open(videoNew)
    for f = 1:size(image{i},4)
        frame.cdata = image{i}(:,:,:,f); frame.colormap = [];
        writeVideo(videoNew, frame)
    end
    close(videoNew)
end
