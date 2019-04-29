clc
close all
clear
%% configuration initialization
xres = 320; yres = 240;
x_overlap_band = 0; y_overlap_band = 0;
fx = 308.331; fy = 308.331; % focal length 
% cx = fx/2; cy = fy/2; % camera center location 
cx = 165.7475; cy = 119.8889; % camera center location 
inertia_weight = 1000; smoothness_weight = 1e-3; tangent_weight = fx; % corresponding to lambda_D、lambda_S and lambda_N in the paper
camera_intrinsics = [fx fy ; cx cy];
weiths_vector = [inertia_weight smoothness_weight tangent_weight];
%% load images
index_image = '004';
% load the input depth image 
input_depth_filename = ['../data/realsense/',index_image ,'_depth_open.png'];
input_depth_inf = imread(input_depth_filename);
input_depth_inf = double (input_depth_inf) ./ 4000;
% input_depth_inf_mod = input_depth_inf;

% % load the input depth image 
% input_RGB_filename = ['../data/realsense/',index_image ,'_color.png'];
% input_RGB_inf = imread(input_RGB_filename);
input_color_filename = ['../data/realsense/',index_image,'_color.png'];
input_RGB_inf = imread(input_color_filename);
% input_RGB_inf = double (input_RGB_inf) ./ 255;
[W_v, W_h] = pairwise_weigths(input_RGB_inf);
figure
imshow(input_RGB_inf);

% load the input normals 
input_normal_filename = ['../torch/result/normal_scannet_realsense_test/realsense_',index_image,'_normal_est.h5'];
input_normal_inf = h5read(input_normal_filename,'/result');
input_normal_inf_new(:,:,1) = input_normal_inf(:,:,1).';
input_normal_inf_new(:,:,2) = input_normal_inf(:,:,3).';
input_normal_inf_new(:,:,3) = -1*input_normal_inf(:,:,2).';
input_normal_inf = input_normal_inf_new;
% load the input tangent weight
% input_tangent_w`eight_filename = ['../torch/result/bound_realsense_weight/realsense_',index_image,'_weight.png'];
% input_tangent_weigth_inf = imread(input_tangent_weight_filename);
% input_tangent_weigth_inf = double (input_tangent_weigth_inf) / 1000;


%%
In = double(rgb2gray(uint8(input_RGB_inf)));
std_img = std(In(:));
% t1 = 0.06;
for i = 1 : 20
if (std_img < 50 && std_img > 35)
    t1 = 0.2;
    t2 = 0.6;
    theta = 1.38;
elseif (std_img < 65 && std_img > 50)
    t1 = 0.08; % 这么看还是0.08的结果比较好
    t2 = 0.09;
    theta = 1.38; % 1.5的时候MAD的值不好，原来为1.38，其实从1.4往上开始MAD就不太行，1.38往下也不太好
else
    t1 = 0.08; % 原始0.3，旁边的椅子搞不出来
    t2 = 0.20; % 0.2的时候比较好
    theta = 1.20;
end
input_tangent_weigth_inf_1  = 1 - edge(In,'canny', [t1,t2], theta);%input_tangent_weigth_inf_2;, [0.3,0.5],1.2) for 046
% SE_2 = strel('disk',12); % 12
% imd_2 = imdilate(input_tangent_weigth_inf_2,SE_2); % disk + 10 + imdilate
% input_tangent_weigth_inf =  imd_2 - input_tangent_weigth_inf_2;
input_tangent_weigth_inf_2 = gen_Gabor_maps(input_RGB_inf,5,8,24,24);%;Gabor_image(input_RGB_inf
input_tangent_weigth_inf_2 = (input_tangent_weigth_inf_2-min(input_tangent_weigth_inf_2(:))) / (max(input_tangent_weigth_inf_2(:))-min(input_tangent_weigth_inf_2(:)));
input_tangent_weigth_inf_2 = 1 - input_tangent_weigth_inf_2;
input_tangent_weigth_inf = input_tangent_weigth_inf_1 ;
% input_RGB_inf = input_tangent_weigth_inf_2;
% input_tangent_weigth_inf_2 = input_RGB_inf;
% input_tangent_weigth_inf_2 = (input_tangent_weigth_inf_2-min(input_tangent_weigth_inf_2(:))) / (max(input_tangent_weigth_inf_2(:))-min(input_tangent_weigth_inf_2(:)));
% input_tangent_weigth_inf_2 = 1 - input_tangent_weigth_inf_2;
SE_2 = strel('square',2); % 12 4 original is the disk
input_tangent_weigth_inf = imopen(input_tangent_weigth_inf,SE_2); % disk + 10 + imdilate
SE_3 = strel('rectangle',[1,3]); % 2,3
input_tangent_weigth_inf = imerode(input_tangent_weigth_inf,SE_3);

% load output depth image(Yinda Zhang)
output_depth_filename = ['../results/realsense/realsense_',index_image,'_1.png'];
output_depth_inf = imread(output_depth_filename);
output_depth_inf = double (output_depth_inf) ./ 4000;
% load true depth image
true_depth_filename = ['../data/realsense/',index_image,'_depth.png'];
true_depth_inf = imread(true_depth_filename);
true_depth_inf = double (true_depth_inf) ./ 4000;


% load the input depth image 
input_RGB_filename = ['../data/realsense/',index_image ,'_color.png'];
input_RGB_inf = imread(input_RGB_filename);
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
% caxis([0, 4])
%% generate matrix A and Vector B, M_A*V_x = V_B
[M_A, V_B] = opt_constraint_generate_sparse_3dpoint_bk(input_depth_inf, W_v, W_h, input_tangent_weigth_inf, weiths_vector, camera_intrinsics);
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
subplot(2,2,3)
imshow(V_M./4, []);
title('output\_depth\_image\_sxf');
colormap jet  
colorbar
caxis([0, 1])

subplot(2,2,4)
imshow(output_depth_inf,[ ]);
colormap jet  
colorbar
caxis([0, 4])
title('output\_depth\_image\_yindaLi');
saveas(gcf,strcat('image_004_t1/','output_depth',num2str(i),'.png'));
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