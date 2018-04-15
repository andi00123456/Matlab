% edge detection
% Sobel, Canny, Prewitt, Roberts, and fuzzy logic methods

I = imread('coins.png');
imshow(I)

BW1 = edge(I,'sobel');
BW2 = edge(I,'canny');
figure;
imshowpair(BW1,BW2,'montage')
title('Sobel Filter                                   Canny Filter');