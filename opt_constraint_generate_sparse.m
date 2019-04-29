function [M_A, V_B] = opt_constraint_generate_sparse(input_depth_inf, input_normal_inf, input_tangent_weigth_inf, weiths_vector, camera_intrinsics)

yres = size(input_tangent_weigth_inf, 1);
xres = size(input_tangent_weigth_inf, 2);
%% inertia cost
input_depth_vector = reshape(input_depth_inf.',  numel(input_depth_inf), 1);
index_obs = find(input_depth_vector>0);
V_B_iner = weiths_vector(1)*input_depth_vector(index_obs);
V_B_iner = sparse(V_B_iner);
index_obs = (0:length(index_obs)-1)*xres*yres + index_obs.';
M_A_iner = sparse(xres*yres, length(index_obs));
M_A_iner(index_obs) = weiths_vector(1);
M_A_iner = M_A_iner.';
%% smoothness cost
% left pixel
I_M = eye(xres);
M_A_block = weiths_vector(2)*I_M + -1*weiths_vector(2)*circshift(I_M, -1, 2);
M_A_block = sparse(M_A_block(2:end, :));
M_A_left = sparse(kron(sparse(eye(yres)), M_A_block));
% right pixel
M_A_block = weiths_vector(2)*I_M + -1*weiths_vector(2)*circshift(I_M, 1, 2);
M_A_block = sparse(M_A_block(1:end-1, :));
M_A_right = sparse(kron(sparse(eye(yres)), M_A_block));
% upper pixel
I_M = eye(yres);
M_A_block = weiths_vector(2)*I_M + -1*weiths_vector(2)*circshift(I_M, -1, 1);
M_A_block = sparse(M_A_block(:, 2:end));
M_A_up = sparse((kron(M_A_block, sparse(eye(xres))))).';
% underneath pixel
M_A_block = weiths_vector(2)*I_M + -1*weiths_vector(2)*circshift(I_M, 1, 1);
M_A_block = M_A_block(:, 1:end-1);
M_A_under = sparse((kron(M_A_block, sparse(eye(xres))))).';
% 
M_A_smooth = sparse([M_A_left;M_A_right;M_A_up;M_A_under]);
%% tangent cost
ix = 1:xres;
iy = 1:yres;
[Ix, Iy] = meshgrid(ix, iy);
Ix_v = reshape(Ix.', numel(Ix), 1) - 1;
Iy_v = yres - reshape(Iy.', numel(Ix), 1);

input_normal_nx = reshape(input_normal_inf(:,:,1).', xres*yres, 1);
input_normal_ny = reshape(input_normal_inf(:,:,2).', xres*yres, 1);
input_normal_nz = reshape(input_normal_inf(:,:,3).', xres*yres, 1);

input_tangent_weirht_vec = reshape(input_tangent_weigth_inf.', xres*yres, 1);

% input_tangent_weirht_vec = 1;

% left pixel
M_A_vec = -1*((Ix_v-camera_intrinsics(2,1))/camera_intrinsics(1,1).*input_normal_nx + ...
            (Iy_v-camera_intrinsics(2,2))/camera_intrinsics(1,2).*input_normal_ny - ...
            input_normal_nz).*input_tangent_weirht_vec*weiths_vector(3)*camera_intrinsics(1,1);
M_A_left_tmp1 = diag(sparse(double (M_A_vec)));
M_A_left_tmp1((0:yres-1)*xres+1, :) = [];

M_A_vec = ((Ix_v-1-camera_intrinsics(2,1))/camera_intrinsics(1,1).*input_normal_nx + ...
            (Iy_v-camera_intrinsics(2,2))/camera_intrinsics(1,2).*input_normal_ny - ...
            input_normal_nz).*input_tangent_weirht_vec*weiths_vector(3)*camera_intrinsics(1,1);
M_A_left_tmp2 = diag(sparse(double (M_A_vec)));
M_A_left_tmp2((0:yres-1)*xres+1, :) = [];
M_A_left_tmp2 = circshift(M_A_left_tmp2, -1, 2);

M_A_left = M_A_left_tmp1 + M_A_left_tmp2;

% right pixel
M_A_vec = -1*((Ix_v-camera_intrinsics(2,1))/camera_intrinsics(1,1).*input_normal_nx + ...
            (Iy_v-camera_intrinsics(2,2))/camera_intrinsics(1,2).*input_normal_ny - ...
            input_normal_nz).*input_tangent_weirht_vec*weiths_vector(3)*camera_intrinsics(1,1);
M_A_right_tmp1 = diag(sparse(double (M_A_vec)));
M_A_right_tmp1((1:yres)*xres, :) = [];

M_A_vec = ((Ix_v+1-camera_intrinsics(2,1))/camera_intrinsics(1,1).*input_normal_nx + ...
            (Iy_v-camera_intrinsics(2,2))/camera_intrinsics(1,2).*input_normal_ny - ...
            input_normal_nz).*input_tangent_weirht_vec*weiths_vector(3)*camera_intrinsics(1,1);
M_A_right_tmp2 = diag(sparse(double (M_A_vec)));
M_A_right_tmp2((1:yres)*xres, :) = [];
M_A_right_tmp2 = circshift(M_A_right_tmp2, 1, 2);

M_A_right = M_A_right_tmp1 + M_A_right_tmp2;

% upper pixel
M_A_vec = -1*((Ix_v-camera_intrinsics(2,1))/camera_intrinsics(1,1).*input_normal_nx + ...
            (Iy_v-camera_intrinsics(2,2))/camera_intrinsics(1,2).*input_normal_ny - ...
            input_normal_nz).*input_tangent_weirht_vec*weiths_vector(3)*camera_intrinsics(1,1);
M_A_up_tmp1 = diag(sparse(double (M_A_vec)));
M_A_up_tmp1(1:xres, :) = [];

M_A_vec = ((Ix_v-camera_intrinsics(2,1))/camera_intrinsics(1,1).*input_normal_nx + ...
            (Iy_v+1-camera_intrinsics(2,2))/camera_intrinsics(1,2).*input_normal_ny - ...
            input_normal_nz).*input_tangent_weirht_vec*weiths_vector(3)*camera_intrinsics(1,1);
M_A_up_tmp2 = diag(sparse(double (M_A_vec)));
M_A_up_tmp2(1:xres, :) = [];
M_A_up_tmp2 = circshift(M_A_up_tmp2, -xres, 2);

M_A_up = M_A_up_tmp1 + M_A_up_tmp2;

% underneath pixel
M_A_vec = -1*((Ix_v-camera_intrinsics(2,1))/camera_intrinsics(1,1).*input_normal_nx + ...
            (Iy_v-camera_intrinsics(2,2))/camera_intrinsics(1,2).*input_normal_ny - ...
            input_normal_nz).*input_tangent_weirht_vec*weiths_vector(3)*camera_intrinsics(1,1);
M_A_under_tmp1 = diag(sparse(double (M_A_vec)));
M_A_under_tmp1(end-xres+1:end, :) = [];

M_A_vec = ((Ix_v-camera_intrinsics(2,1))/camera_intrinsics(1,1).*input_normal_nx + ...
            (Iy_v-1-camera_intrinsics(2,2))/camera_intrinsics(1,2).*input_normal_ny - ...
            input_normal_nz).*input_tangent_weirht_vec*weiths_vector(3)*camera_intrinsics(1,1);
M_A_under_tmp2 = diag(sparse(double (M_A_vec)));
M_A_under_tmp2(end-xres+1:end, :) = [];
M_A_under_tmp2 = circshift(M_A_under_tmp2, xres, 2);

M_A_under = M_A_under_tmp1 + M_A_under_tmp2;

M_A_tangent = [M_A_left;M_A_right;M_A_up;M_A_under];
%% merge constraint
M_A = [M_A_iner; M_A_smooth; M_A_tangent];
% M_A = [M_A_iner; M_A_tangent];
V_B = sparse(size(M_A, 1), 1);
V_B(1:length(V_B_iner)) = V_B_iner;
%% delete null constraints
sum_M = sum(abs(M_A), 2);
ind_non = sum_M ==0;
M_A(ind_non,:) = [];
V_B(ind_non) = [];
end
