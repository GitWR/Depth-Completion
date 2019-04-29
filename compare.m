clc
close
clear all
mlb_b = double(imread('canny_boundary.png') / 255);
cv_b = double(imread('bdry_38_40_default.png') / 255) ;
diff = mlb_b - cv_b;
figure
subplot(2,2,1);
imshow(mlb_b)
title('matlab')
subplot(2,2,2);
imshow(cv_b)
title('opencv')
subplot(2,2,3);
imshow(diff)
title('diff map')