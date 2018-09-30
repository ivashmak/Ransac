close all;

fileID = fopen('../dataset/homography/graf_pts.txt','r');
formatSpec = '%f';
sizeA = [7 Inf];
A = fscanf(fileID, formatSpec, sizeA);
A = A';
sz = size(A, 1);

clear pts1 pts2;
for i = 1:sz
    pts1(i, 1) = A(i, 1);
    pts1(i, 2) = A(i, 2);
%     pts1(i, 3) = A(i, 3);
    
    pts2(i, 1) = A(i, 4);
    pts2(i, 2) = A(i, 5);
%     pts2(i, 3) = A(i, 6);
end

% img1 = imread('../dataset/img1.png');
% img2 = imread('../dataset/img2.png');
% 
% figure; imshow (img2);
% hold on
% plot(pts1(:,1), pts1(:,2), 'o', 'Color', 'white', 'LineWidth', 2);
% hold off
% 
% figure; imshow (img1);
% hold on
% plot(pts2(:,1), pts2(:,2), '+', 'Color', 'white', 'LineWidth', 2);
% hold off
% 

% F  = sevenpoint (pts1, pts2, 800)

% keyboard
% H_DLT = DLT(pts1, pts2);
[T_1, offset_1, s_1, s1_1, s2_1] = GetNormalizingTransformation(pts1);
[T_2, offset_2, s_2, s1_2, s2_2] = GetNormalizingTransformation(pts2);
    
pts1Tr=(T_1*[pts1';ones(1,sz)])';
pts2Tr=(T_2*[pts2';ones(1,sz)])';

pts1Tr=pts1Tr(:,1:2);
pts2Tr=pts2Tr(:,1:2);
    
Npts = [pts1Tr pts2Tr]

%H_NormalizedDLT = NormalizedDLT(pts1, pts2);
% 
% tform1 = projective2d(T_1');
% tform2 = projective2d(T_2');
% 
% out1 = transformPointsForward(tform1, pts1)
% out2 = transformPointsForward(tform2, pts1)
% 
% 
% outI1 = imwarp(img1, tform2);
% outI2 = imwarp(img2, tform2);

% figure; imshow(outI1);
% figure; imshow(outI2);


% keyboard


