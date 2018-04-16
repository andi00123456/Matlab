camList = webcamlist;
cam = webcam(1);

preview(cam);



f = figure;
w = waitforbuttonpress;
while w ~= 1
    disp('Button click')    
    w = waitforbuttonpress;
end
disp('Key press')
clear cam
close(f);