%% Acquiring a Single Image in a Loop
% This example shows how to use the GETSNAPSHOT function and provides some tips for efficient use.
% The GETSNAPSHOT function allows for quick acquisition of a single video frame.  

% Copyright 2006-2015 The MathWorks, Inc.

%% Set up the Acquisition Object
% Most interaction with Image Acquisition Toolbox is done through a
% video input object.  These objects are created with the VIDEOINPUT
% command.  This example uses a webcam that is accessed through the toolbox's
% "winvideo" adaptor.
vidobj = videoinput('winvideo');

%% Acquire a Frame
% To acquire a single frame, use the GETSNAPSHOT function.
snapshot = getsnapshot(vidobj);

% Display the frame in a figure window.
imagesc(snapshot)

%% Acquire Multiple Frames
% A common task is to repeatedly acquire a single image, process it, and
% then store the result.  To do this, GETSNAPSHOT can be called in a loop.

for i = 1:5
    snapshot = getsnapshot(vidobj);
    imagesc(snapshot);
end

%% Timing Implications
% The GETSNAPSHOT function performs a lot of work when it is called.  It
% must connect to the device, configure it, start the acquisition, acquire
% one frame, stop the acquisition, and then close the device.  This means
% that the acquisition of one frame can take significantly longer than
% would be expected based on the frame rate of the camera.  To illustrate
% this, call GETSNAPSHOT in a loop.

% Measure the time to acquire 20 frames.
tic
for i = 1:20
    snapshot = getsnapshot(vidobj);
end

elapsedTime = toc

% Compute the time per frame and effective frame rate.
timePerFrame = elapsedTime/20
effectiveFrameRate = 1/timePerFrame

%%
% The next example illustrates a more efficient way to perform the loop.

%% Using Manual Trigger Mode
% You can avoid the overhead of GETSNAPSHOT described in the previous
% setting by using the manual triggering mode of the videoinput object.
% Manual triggering mode allows the toolbox to connect to and configure the
% device a single time without logging data to memory. This means that
% frames can be returned to MATLAB(R) with less of a delay.

% Configure the object for manual trigger mode.
triggerconfig(vidobj, 'manual');

% Now that the device is configured for manual triggering, call START.
% This will cause the device to send data back to MATLAB, but will not log
% frames to memory at this point.
start(vidobj)

% Measure the time to acquire 20 frames.
tic
for i = 1:20
    snapshot = getsnapshot(vidobj);
end

elapsedTime = toc

% Compute the time per frame and effective frame rate.
timePerFrame = elapsedTime/20
effectiveFrameRate = 1/timePerFrame

% Call the STOP function to stop the device.
stop(vidobj)

%%
% You can see that the elapsed time using manual triggering is much smaller
% than the previous example.

%% Cleanup
% Once the video input object is no longer needed, delete the associated
% variable.
delete(vidobj)
