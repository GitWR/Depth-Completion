clc
close all
clear
%% configuration initialization
xres = 320; yres = 240;
x_overlap_band = 0; y_overlap_band = 0;
fx = 308.331; fy = 308.331; % focal length 
% cx = fx/2; cy = fy/2; % camera center location 
cx = 165.7475; cy = 119.8889; % camera center location 
inertia_weight = 1000; smoothness_weight = 1e-3; tangent_weight = 1; % corresponding to lambda_D、lambda_S and lambda_N in the paper
camera_intrinsics = [fx fy ; cx cy];
weiths_vector = [inertia_weight smoothness_weight tangent_weight]; 
%% load images
index_image = '004'; %046
% load the input depth image 
input_depth_filename = ['../data/realsense/',index_image ,'_depth_open.png'];
input_depth_inf = imread(input_depth_filename);
input_depth_inf = double (input_depth_inf) ./ 4000;
input_depth_inf_re = imresize(input_depth_inf,1,'bicubic');
% input_depth_inf_mod = input_depth_inf;

% load the input depth image 
input_RGB_filename = ['../data/realsense/',index_image ,'_color.png'];
input_RGB_inf = imread(input_RGB_filename);

figure
imshow(uint8(input_RGB_inf));
% SE = strel('disk',10);
% imd_1 = imdilate(input_RGB_inf,SE);
% input_RGB_inf = imd_1 - input_RGB_inf;



% load the input normals 
input_normal_filename = ['../torch/result/normal_scannet_realsense_test/realsense_',index_image,'_normal_est.h5'];
input_normal_inf = h5read(input_normal_filename,'/result');
input_normal_inf_new(:,:,1) = input_normal_inf(:,:,1).';
input_normal_inf_new(:,:,2) = input_normal_inf(:,:,3).';
input_normal_inf_new(:,:,3) = -1*input_normal_inf(:,:,2).';
input_normal_inf = input_normal_inf_new;
% load the input tangent weight
load('004')
edg = (edg - min(edg(:))) / (max(edg(:)) - min(edg(:)));
input_tangent_weigth_inf = edg;
% input_tangent_weight_filename = ['../torch/result/bound_realsense_weight/realsense_',index_image,'_weight.png'];
% input_tangent_weigth_inf = imread(input_tangent_weight_filename);
% input_tangent_weigth_inf = double (input_tangent_weigth_inf) / 1000;

% input_tangent_weigth_inf_2 = Gabor_image(input_RGB_inf);
% input_tangent_weigth_inf_1 = (input_tangent_weigth_inf_1-min(input_tangent_weigth_inf_1(:))) / (max(input_tangent_weigth_inf_1(:))-min(input_tangent_weigth_inf_1(:)));
% input_tangent_weigth_inf = 1 - input_tangent_weigth_inf_1;
% SE = strel('disk',20);
% imd_1 = imdilate(input_tangent_weigth_inf,SE);
% input_tangent_weigth_inf = imd_1 - input_tangent_weigth_inf;


% input_tangent_weigth_inf_1 =  gen_Gabor_maps(input_depth_inf_re,5,8,15,15);
% input_tangent_weigth_inf_1 = (input_tangent_weigth_inf_1-min(input_tangent_weigth_inf_1(:))) / (max(input_tangent_weigth_inf_1(:))-min(input_tangent_weigth_inf_1(:)));
% input_tangent_weigth_inf_1 = 1 - input_tangent_weigth_inf_1;
% input_tangent_weigth_inf_1  =  1 - edge(input_tangent_weigth_inf_1,'canny', [0.02,0.05],0.07);%input_tangent_weigth_inf_2;, [0.2,0.3],1)
% SE = strel('disk',12); % square 6
% imd_1 = imdilate(input_tangent_weigth_inf_1,SE);  
% input_tangent_weigth_inf_1 = imd_1 - input_tangent_weigth_inf_1;

In = double(rgb2gray(uint8(input_RGB_inf)));
h1 = 0.08;
h2 = 0.03;
%figure('Position',get(0,'ScreenSize'))
for i = 1 : 200 
% w = fspecial('gaussian',[10,10],1);
%replicate:图像大小通过赋值外边界的值来扩展
%symmetric 图像大小通过沿自身的边界进行镜像映射扩展
% t = i;
% %In = conv2(In,w,'same');%imfilter(In,w,'replicate');
% %h2 = 0.0003 * t;
% std_img = std(In(:));
% if (std_img < 50 && std_img > 35)
%     t1 = 0.2;
%     t2 = 0.6;
%     theta = 1.38;
% elseif (std_img < 65 && std_img > 50)
%     t1 = 0.09;
%     t2 = 0.11;
%     theta = 1.38;
% else
%     t1 = 0.3;
%     t2 = 0.5;
%     theta = 1.2;
% end
% input_tangent_weigth_inf_1  = 1 - edge(In,'canny', [t1,t2], theta);%input_tangent_weigth_inf_2;, [0.3,0.5],1.2) for 046
% input_tangent_weigth_inf_2 = gen_Gabor_maps(input_RGB_inf,5,8,24,24);%;Gabor_image(input_RGB_inf
% input_tangent_weigth_inf_2 = (input_tangent_weigth_inf_2-min(input_tangent_weigth_inf_2(:))) / (max(input_tangent_weigth_inf_2(:))-min(input_tangent_weigth_inf_2(:)));
% input_tangent_weigth_inf_2 = 1 - input_tangent_weigth_inf_2;
% input_tangent_weigth_inf = input_tangent_weigth_inf_1 ;% .* input_tangent_weigth_inf_2;
% % input_RGB_inf = input_tangent_weigth_inf_2;
% % input_tangent_weigth_inf_2 = input_RGB_inf;
% % input_tangent_weigth_inf_2 = (input_tangent_weigth_inf_2-min(input_tangent_weigth_inf_2(:))) / (max(input_tangent_weigth_inf_2(:))-min(input_tangent_weigth_inf_2(:)));
% % input_tangent_weigth_inf_2 = 1 - input_tangent_weigth_inf_2;
% SE_2 = strel('disk',2); % 12 4 original is the disk
% input_tangent_weigth_inf = imopen(input_tangent_weigth_inf,SE_2); % disk + 10 + imdilate
% SE_3 = strel('rectangle',[2,3]); % 2,3
% input_tangent_weigth_inf = imerode(input_tangent_weigth_inf,SE_3);
% SE_4 = strel('rectangle',[1,1]); % 2,3
% input_tangent_weigth_inf = imdilate(input_tangent_weigth_inf,SE_4);

% input_tangent_weigth_inf = input_tangent_weigth_inf .* input_tangent_weigth_inf_1;
% input_tangent_weigth_inf =  imd_2 - input_tangent_weigth_inf_2;
% % input_tangent_weigth_inf = imresize(input_tangent_weigth_inf,[240,320],'bilinear');
% input_tangent_weigth_inf = input_tangent_weigth_inf_1 .* input_tangent_weigth_inf;
% % input_tangent_weigth_inf = imdilate(input_tangent_weigth_inf, SE_2);

% figure
% imshow(input_tangent_weigth_inf_1);
% figure
% imshow(input_tangent_weigth_inf_2);

%% canny
% In = double(rgb2gray(uint8(input_RGB_inf)));
% input_tangent_weigth_inf  =  1 - edge(In,'canny', [0.05,0.07], 1);%input_tangent_weigth_inf_2;, [0.2,0.3],1)


% load output depth image(Yinda Zhang)
output_depth_filename = ['../results/realsense/realsense_',index_image,'_1.png'];
output_depth_inf = imread(output_depth_filename);
output_depth_inf = double (output_depth_inf) ./ 4000;
% load true depth image
true_depth_filename = ['../data/realsense/',index_image,'_depth.png'];
true_depth_inf = imread(true_depth_filename);
true_depth_inf = double (true_depth_inf) ./ 4000;
% imshow(true_depth_inf);

%% 
subplot(2,2,1)
imshow(input_depth_inf,[]);
title('input\_depth\_image');
colormap jet  
colorbar
caxis([0, 4])

subplot(2,2,2)
imshow(input_tangent_weigth_inf,[])
% title('input\_RGB\_image');
title('boundary\_image');
colormap jet  
colorbar
caxis([0, 1])
%% generate matrix A and Vector B, M_A*V_x = V_B
[M_A, V_B] = opt_constraint_generate_sparse(input_depth_inf, input_normal_inf, input_tangent_weigth_inf, weiths_vector, camera_intrinsics);
%% conpute V_x
ATA = M_A.' * M_A;
ATB = M_A.' * V_B;
R = chol(ATA);
Y = R.'\ATB;
V_x = R\Y;               
% V_x = pinv(ATA)*ATB;
% V_x = ATA\ATB;
%% plot output depth image
V_M = (reshape(V_x, xres, yres)).';
V_M = full(V_M);
saveas(gcf,strcat('image/','output_depth',num2str(i),'.png'));
subplot(2,2,3)
imshow(V_M, []);
title('output\_depth\_image\_sxf');
colormap jet  
colorbar
caxis([0, 4])

subplot(2,2,4)
imshow(output_depth_inf,[ ]);
colormap jet  
colorbar
caxis([0, 4])
title('output\_depth\_image\_yindaLi');
%%
index_nonzeros = find(true_depth_inf>0);
err_obs_sxf = sum((true_depth_inf(index_nonzeros)-V_M(index_nonzeros)).^2);
err_obs_lyd = sum((true_depth_inf(index_nonzeros)-output_depth_inf(index_nonzeros)).^2);
err_full_sxf = sum(sum((true_depth_inf-V_M).^2));
err_full_lyd = sum(sum((true_depth_inf-output_depth_inf).^2));
disp(['err_obs_sxf: ', num2str(err_obs_sxf)]);
disp(['err_obs_lyd: ', num2str(err_obs_lyd)]);
disp(['err_full_sxf: ', num2str(err_full_sxf)]);
disp(['err_full_lyd: ', num2str(err_full_lyd)]);
 end