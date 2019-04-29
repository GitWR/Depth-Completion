clc
close all
clear all
% load('edge_k7'); % 导入以7*7卷积核学习到的two-stage high-frequency maps
% sum = zeros(240,320);
% temp = ftrain{4}; % 取出004图的hf maps
% save('004.mat','temp');
load('004');
sum = zeros(240,320);
filter_1 = 20;
filter_2 = 8;
% 将所有高频图像叠加起来
for i = 1 : filter_1 * filter_2
    sum = sum + temp{i}; 
end
% normalization
for i = 1 : 240
    for j = 1 : 320
        if ( sum(i,j) < 0 )
            sum(i,j) = 0;
        end
    end
end
% edge detection
t1 = 0.07;
t2 = 0.10;
sigma = sqrt(2);
edge_map = edge(sum,'canny',[t1,t2],sigma);
imwrite(edge_map,'pcanet_004_k7_0.07_0.10_1.414.png')