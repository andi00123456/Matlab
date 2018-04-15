% Access an image acquisition device.
vidobj = videoinput('winvideo');

% Convert the input images to grayscale.
vidobj.ReturnedColorSpace = 'grayscale';

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

setappdata(hImage,'UpdatePreviewWindowFcn',@update_livehistogram_display);

% Here are the contents of update_livehistogram_display.m which contains
% the callback function.
dbtype('update_livehistogram_display.m')

% The PREVIEW function starts the camera and display. The image on which to
% display the video feed is also specified.
preview(vidobj, hImage);

% View the histogram for 30 seconds.
pause(30);


% Stop the preview image and delete the figure.
stoppreview(vidobj);
delete(f);


delete(vidobj)
clear vidobj