function [Hf, Ia, V] = Gau_lower_pass(IA)
  % clc
  % clear all
  % close all
  % IA = double(imread('art-depth.png'));
  % if (IA >=2)
    V = zeros(1,2);
    
    [f1,f2] = freqspace(size(IA),'meshgrid');
    D = 100 / size(IA,1); % 100
    r = f1.^2 + f2.^2;
    Hd = zeros(size(IA));

  for i = 1 : size(IA,1)
    for j = 1 : size(IA,2)
        t = r(i,j) / (D * D);
        Hd(i,j) = exp(-t);
    end
  end

  Y = fft2(double(IA));
  Y = fftshift(Y);
  Ya = Y .* Hd;
  Ya = ifftshift(Ya);
  Ia = real(ifft2(Ya));
  
  Hf = (IA - Ia); % ��Ƶͼ
  V(:,1) = max(Hf(:));
  V(:,2) = min(Hf(:));
  Hf = (Hf-min(Hf(:))) / (max(Hf(:))-min(Hf(:))); % ��һ��
  Ia = (Ia-min(Ia(:))) / (max(Ia(:))-min(Ia(:))); % ��һ��
  % end
  
%   imshow(uint8(IA)); title('ԭͼ��')
%   figure
%   imshow(uint8(Ia)); title('��ͨͼ��')
%   figure
%   imshow(uint8(IA - Ia),[]); title('����ͼ')
  
end
