function [M_A, V_B] = opt_constraint_generate_sparse_3dpoint_bk(input_depth_inf, W_v, W_h, input_tangent_weigth_inf, weights_vector, camera_intrinsics)

yres = size(input_depth_inf, 1);
xres = size(input_depth_inf, 2);
%% inertia cost
input_depth_vector = reshape(input_depth_inf.',  numel(input_depth_inf), 1);
index_obs = find(input_depth_vector>0);
V_B_iner = weights_vector(1)*input_depth_vector(index_obs);
V_B_iner = sparse(V_B_iner);
index_obs = (0:length(index_obs)-1)*xres*yres + index_obs.';
M_A_iner = sparse(xres*yres, length(index_obs));
M_A_iner(index_obs) = weights_vector(1);
M_A_iner = M_A_iner.';
%% smoothness cost
% left pixel
I_M = eye(xres);
M_A_block = weights_vector(2)*I_M + -1*weights_vector(2)*circshift(I_M, -1, 2);
M_A_block = sparse(M_A_block(2:end, :));
M_A_left = sparse(kron(sparse(eye(yres)), M_A_block));
% right pixel
M_A_block = weights_vector(2)*I_M + -1*weights_vector(2)*circshift(I_M, 1, 2);
M_A_block = sparse(M_A_block(1:end-1, :));
M_A_right = sparse(kron(sparse(eye(yres)), M_A_block));
% upper pixel
I_M = eye(yres);
M_A_block = weights_vector(2)*I_M + -1*weights_vector(2)*circshift(I_M, -1, 1);
M_A_block = sparse(M_A_block(:, 2:end));
M_A_up = sparse((kron(M_A_block, sparse(eye(xres))))).';
% underneath pixel
M_A_block = weights_vector(2)*I_M + -1*weights_vector(2)*circshift(I_M, 1, 1);
M_A_block = M_A_block(:, 1:end-1);
M_A_under = sparse((kron(M_A_block, sparse(eye(xres))))).';
% 
M_A_smooth = sparse([M_A_left;M_A_right;M_A_up;M_A_under]);
%% tangent cost
ix = 1:xres;
iy = 1:yres;
[Ix, Iy] = meshgrid(ix, iy);
Ix_v = reshape(Ix.', numel(Ix), 1) - 1;
Iy_v = reshape(Iy.', numel(Iy), 1) - 1;

input_tangent_weirht_vec = reshape(input_tangent_weigth_inf.', xres*yres, 1);
% input_tangent_weirht_vec = 1;

% horizontal
M_X_vec = (Ix_v-camera_intrinsics(2,1))/camera_intrinsics(1,1).*weights_vector(3);
M_X_center = diag(sparse(double (M_X_vec)));
M_X_left = circshift(M_X_center, 1, 1);
M_X_right = circshift(M_X_center, -1, 1);
M_X_horizontal = 2.*M_X_center - M_X_left - M_X_right;
M_X_horizontal = input_tangent_weirht_vec.*circshift(input_tangent_weirht_vec, 1, 1).* circshift(input_tangent_weirht_vec, -1, 1).*  M_X_horizontal;
M_X_horizontal([(0:yres-1)*xres+1 (1:yres)*xres], :) = [];
M_X_horizontal = sparse(reshape(W_h.',numel(W_h), 1)).*M_X_horizontal;

M_Y_vec = (Iy_v-camera_intrinsics(2,2))/camera_intrinsics(1,2).*weights_vector(3);
M_Y_center = diag(sparse(double (M_Y_vec)));
M_Y_left = circshift(M_Y_center, 1, 1);
M_Y_right = circshift(M_Y_center, -1, 1);
M_Y_horizontal = 2.*M_Y_center - M_Y_left - M_Y_right;
M_Y_horizontal = input_tangent_weirht_vec.*circshift(input_tangent_weirht_vec, 1, 1).* circshift(input_tangent_weirht_vec, -1, 1).* M_Y_horizontal;
M_Y_horizontal([(0:yres-1)*xres+1 (1:yres)*xres], :) = [];
M_Y_horizontal = sparse(reshape(W_h.',numel(W_h), 1)).*M_Y_horizontal;

% vertical
M_X_vec = (Ix_v-camera_intrinsics(2,1))/camera_intrinsics(1,1).*weights_vector(3);
M_X_center = diag(sparse(double (M_X_vec)));
M_X_upper = circshift(M_X_center, xres, 1);
M_X_under = circshift(M_X_center, -xres, 1);
M_X_vertical = 2.*M_X_center - M_X_upper - M_X_under;
M_X_vertical = input_tangent_weirht_vec.* circshift(input_tangent_weirht_vec, xres, 1).* circshift(input_tangent_weirht_vec, -xres, 1).*M_X_vertical;
M_X_vertical([1:xres end-xres+1:end], :) = [];
M_X_vertical = sparse(reshape(W_v.',numel(W_v), 1)).*M_X_vertical;

M_Y_vec = (Iy_v-camera_intrinsics(2,2))/camera_intrinsics(1,2).*weights_vector(3);
M_Y_center = diag(sparse(double (M_Y_vec)));
M_Y_upper = circshift(M_Y_center, xres, 1);
M_Y_under = circshift(M_Y_center, -xres, 1);
M_Y_vertical = 2.*M_Y_center - M_Y_upper - M_Y_under;
M_Y_vertical = input_tangent_weirht_vec.*circshift(input_tangent_weirht_vec, xres, 1).* circshift(input_tangent_weirht_vec, -xres, 1).*M_Y_vertical;
M_Y_vertical([1:xres end-xres+1:end], :) = [];
M_Y_vertical = sparse(reshape(W_v.',numel(W_v), 1)).*M_Y_vertical;

% M_A_tangent = [M_Y_horizontal;M_X_vertical];
M_A_tangent = [M_X_horizontal;M_Y_horizontal;M_X_vertical;M_Y_vertical];
%%
M_A = [M_A_iner; M_A_smooth; M_A_tangent];
% M_A = [M_A_iner; M_A_smooth];
V_B = sparse(size(M_A, 1), 1);
V_B(1:length(V_B_iner)) = V_B_iner;
%%
sum_M = sum(abs(M_A), 2);
ind_non = sum_M ==0;
M_A(ind_non,:) = [];
V_B(ind_non) = [];
end
