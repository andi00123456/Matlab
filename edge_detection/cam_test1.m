camList = webcamlist;
cam = webcam(1);

preview(cam);

% wait for key press to capture a frame
f_activate = figure('Name', 'activate');
w = waitforbuttonpress;
close(f_activate);

img = snapshot(cam);
f = figure('Name', 'first origin');
imshow(img);

%figure('Name', 'ori');
%imshow(img);

%img_gray = rgb2gray(img);
%img_sobel = edge(img_gray,'sobel');

%figure('Name', 'sobel');
%imshow(img_sobel);

%I = img_gray;
%BW1 = edge(I,'sobel');
%BW2 = edge(I,'canny');
f_c = figure('Name', 'compare');
%imshowpair(BW1,BW2,'montage')
%title('Sobel Filter                                   Canny Filter');


% press any key to close and stop

while w ~= 1  % press any key to break
    
    figure(f);
    w = waitforbuttonpress;
    
    img = snapshot(cam);
    figure(f);
    imshow(img);
    
    
    img_gray = rgb2gray(img);
    I = img_gray;
    BW1 = edge(I,'sobel');
    BW2 = edge(I,'canny');
    
    figure(f_c);
    imshowpair(BW1,BW2,'montage')
    title('Sobel Filter                                   Canny Filter');
    
end
disp('Key press')
clear cam

