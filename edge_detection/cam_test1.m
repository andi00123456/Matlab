

camList = webcamlist
cam = webcam(1)

preview(cam);
img = snapshot(cam);
image(img);


img_gray = rgb2gray(img)
img_sobel = edge(img_gray,'sobel');

image(img_sobel);