function [edge_map] = get_edge_map_fun(t1)
% load('edge_k7'); % 导入以7*7卷积核学习到的two-stage high-frequency maps
% sum = zeros(240,320);
% temp = ftrain{4}; % 取出004图的hf maps
% save('004.mat','temp');
load('004_k9');
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
        if ( sum(i,j) < 1.4 )
            sum(i,j) = 0;
        elseif (sum(i,j) >= 1.4)
            sum(i,j) = 1;
        end
    end
end
% imshow(sum)
% figure
% imshow(1-sum)
edge_map = sum;
% edge detection
% t1 = 0.07;
% t2 = 0.086; % according to the experiments, we choose a suitable one
% sigma = 1.40;
% edge_map = edge(sum,'canny',[t1,t2],sigma);
% imwrite(edge_map,['pcanet_004_k7_',sprintf('%5f',t1),'_',sprintf('%5f',t2),'0.10_1.40_1.414.png'],'png')
end

