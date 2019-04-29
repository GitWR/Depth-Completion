function [W_v, W_h] = pairwise_weigths(input_color_inf)

% input_color_YUV_inf = zeros(size(input_color_inf));
% 
% matrix_transform = [0.299 0.587 0.114 0; -0.169 -0.331 0.5 128; 0.5 -0.419 -0.081 128];

% input_color_YUV_inf(:,:,1) = matrix_transform(1,1).*input_color_inf(:,:,1) + matrix_transform(1,2).*input_color_inf(:,:,2) ...
%                                 + matrix_transform(1,3).*input_color_inf(:,:,3) + matrix_transform(1,4);
% input_color_YUV_inf(:,:,2) = matrix_transform(2,1).*input_color_inf(:,:,1) + matrix_transform(2,2).*input_color_inf(:,:,2) ...
%                                 + matrix_transform(2,3).*input_color_inf(:,:,3) + matrix_transform(2,4);
% input_color_YUV_inf(:,:,3) = matrix_transform(3,1).*input_color_inf(:,:,1) + matrix_transform(3,2).*input_color_inf(:,:,2) ...
%                                 + matrix_transform(3,3).*input_color_inf(:,:,3) + matrix_transform(3,4);                            

input_color_inf = double (input_color_inf) ./ 255;
W_h_l = exp(-1.*sum(abs(input_color_inf - circshift(input_color_inf, 1, 2)).^2, 3));
W_h_r = exp(-1.*sum(abs(input_color_inf - circshift(input_color_inf, -1, 2)).^2, 3));

% W_h_l = exp(-1.*sum(abs(input_color_YUV_inf - circshift(input_color_YUV_inf, 1, 2)).^2, 3));
% W_h_r = exp(-1.*sum(abs(input_color_YUV_inf - circshift(input_color_YUV_inf, -1, 2)).^2, 3));


W_h_l(:, 1) = [];
W_h_l(:, end) = [];                

W_h_r(:, 1) = [];
W_h_r(:, end) = [];

W_h = W_h_l.*W_h_r;

W_v_u = exp(-1.*sum(abs(input_color_inf - circshift(input_color_inf, 1, 1)).^2, 3));
W_v_d = exp(-1.*sum(abs(input_color_inf - circshift(input_color_inf, -1, 1)).^2, 3));

% W_v_u = exp(-1.*sum(abs(input_color_YUV_inf - circshift(input_color_YUV_inf, 1, 1)).^2, 3));
% W_v_d = exp(-1.*sum(abs(input_color_YUV_inf - circshift(input_color_YUV_inf, -1, 1)).^2, 3));

W_v_u(1, :) = [];
W_v_u(end, :) = [];

W_v_d(1, :) = [];
W_v_d(end, :) = [];

W_v = W_v_u.*W_v_d;


