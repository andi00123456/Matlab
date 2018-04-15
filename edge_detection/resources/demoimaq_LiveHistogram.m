%% Video Display with Live Histogram
%
% This example shows how to set up and display a live histogram.
%
% The Image Acquisition Toolbox(TM) together with the Image Processing Toolbox(TM) 
% can be used to display a video feed with a live histogram. This can be
% useful when calibrating camera settings such as aperture using manual 
% controls. This example shows how to use the PREVIEW function, its 
% associated custom update function and the IMHIST function to place a 
% video preview window adjacent to a live histogram. The techniques here 
% can be used to display other live information too. For example, a live
% video feed can be placed next to a filtered version of the video.
%
% This example uses a callback function, found in the
%
% * MATLABROOT\toolbox\imaq\imaqdemos\helper
% 
% directory, that displays the live histogram in a figure window.
%
% <matlab:web('http://www.mathworks.com/videos/video-display-with-live-histogram-101623.html','-browser') Watch 
% a clip of the video feed and histogram>. (8 seconds)
%
% Copyright 2007-2015 The MathWorks, Inc.

%%

% Add the callback function directory to the MATLAB(R) path.
utilpath = fullfile(matlabroot, 'toolbox', 'imaq', 'imaqdemos', 'helper');
addpath(utilpath);

%% Set Up Video Object and Figure

% Access an image acquisition device.
vidobj = videoinput('winvideo');

% Convert the input images to grayscale.
vidobj.ReturnedColorSpace = 'grayscale';

%%
% An image object of the same size as the video is used to store and
% display incoming frames.

% Retrieve the video resolution. 
vidRes = vidobj.VideoResolution;

% Create a figure and an image object.
f = figure('Visible', 'off');

% The Video Resolution property returns values as width by height, but
% MATLAB images are height by width, so flip the values.
imageRes = fliplr(vidRes);

subplot(1,2,1);

hImage = imshow(zeros(imageRes));

% Set the axis of the displayed image to maintain the aspect ratio of the 
% incoming frame.
axis image;

%%
% Specify the UpdatePreviewWindowFcn callback function that is called each 
% time a new frame is available. The callback function is responsible for
% displaying new frames and updating the histgram. It can also be used to 
% apply custom processing to the frames. More details on how to use this 
% callback can be found in the documentation for the PREVIEW function. 
% This callback function itself is defined in the file 
% <matlab:edit(fullfile(matlabroot,'toolbox','imaq','imaqdemos','helper','update_livehistogram_display.m')) update_livehistogram_display.m>
setappdata(hImage,'UpdatePreviewWindowFcn',@update_livehistogram_display);

%% Define the Callback Function 

% Here are the contents of update_livehistogram_display.m which contains
% the callback function.
dbtype('update_livehistogram_display.m')

%% Start Previewing

% The PREVIEW function starts the camera and display. The image on which to
% display the video feed is also specified.
preview(vidobj, hImage);

% View the histogram for 30 seconds.
pause(30);

%%
% Above is a sample image of the histogram and video feed.

% Stop the preview image and delete the figure.
stoppreview(vidobj);
delete(f);

%%
% Once the video input object is no longer needed, delete and 
% clear the associated variable.
delete(vidobj)
clear vidobj
