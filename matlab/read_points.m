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
H_DLT = DLT(pts1, pts2);

% [T_1, offset_1, s_1, s1_1, s2_1] = GetNormalizingTransformation(pts1);
% [T_2, offset_2, s_2, s1_2, s2_2] = GetNormalizingTransformation(pts2);
%     
% pts1Tr=(T_1*[pts1';ones(1,sz)])';
% pts2Tr=(T_2*[pts2';ones(1,sz)])';
% 
% pts1Tr=pts1Tr(:,1:2);
% pts2Tr=pts2Tr(:,1:2);
%     
% Npts = [pts1Tr pts2Tr]

H_NormalizedDLT = NormalizedDLT(pts1, pts2);

% keyboard

figure; 
subplot (1,2,1); plot (pts1, '.');
subplot (1,2,2); plot (pts2, '.');

pts1 = [pts1 ones(sz, 1)]';
pts2 = [pts2 ones(sz, 1)]';

est_pts2 = H_NormalizedDLT*pts1;
est_pts2 = est_pts2./est_pts2(3,:);

est_pts1 = H_NormalizedDLT\pts2;
est_pts1 = est_pts1./est_pts1(3,:);

est_pts1_2d = est_pts1 (1:2, :)';
est_pts2_2d = est_pts2 (1:2, :)';

figure;
subplot (1,2,1); plot (est_pts1_2d, '.');
subplot (1,2,2); plot (est_pts2_2d, '.');

A = pts1';
b = pts2';
H = A'*A\A'*b

est_pts2 = H*pts1;
est_pts2 = est_pts2./est_pts2(3,:);

est_pts1 = H\pts2;
est_pts1 = est_pts1./est_pts1(3,:);

est_pts1_2d = est_pts1 (1:2, :)';
est_pts2_2d = est_pts2 (1:2, :)';

figure;
subplot (1,2,1); plot (est_pts1_2d, '.');
subplot (1,2,2); plot (est_pts2_2d, '.');


syms h11 h12 h13 h21 h22 h23 h31 h32 h33
syms x1 y1 x2 y2 x_est y_est
x_est = (h11 * x1 + h12 * y1 + h13)/(h31 * x1 + h32 * y1 + h33);
y_est = (h21 * x1 + h22 * y1 + h23)/(h31 * x1 + h32 * y1 + h33);
min_eqn = (x_est - x2)^2 + (y_est - y2)^2;
d_h11 = diff (min_eqn, h11);
d_h12 = diff (min_eqn, h12);
d_h13 = diff (min_eqn, h13);
d_h21 = diff (min_eqn, h21);
d_h22 = diff (min_eqn, h22);
d_h23 = diff (min_eqn, h23);
d_h31 = diff (min_eqn, h31);
d_h32 = diff (min_eqn, h32);
d_h33 = diff (min_eqn, h33);

d_h11_eq_0 = d_h11 == 0;
d_h12_eq_0 = d_h12 == 0;
d_h13_eq_0 = d_h13 == 0;
d_h21_eq_0 = d_h21 == 0;
d_h22_eq_0 = d_h22 == 0;
d_h23_eq_0 = d_h23 == 0;
d_h31_eq_0 = d_h31 == 0;
d_h32_eq_0 = d_h32 == 0;
d_h33_eq_0 = d_h33 == 0;

[A,B] = equationsToMatrix( ...
[d_h11_eq_0, d_h12_eq_0, d_h13_eq_0, ...
 d_h21_eq_0, d_h22_eq_0, d_h23_eq_0, ...
 d_h31_eq_0, d_h32_eq_0, d_h33_eq_0], ...
[h11, h12, h13, h21, h22, h23, h31, h32, h33]);

X = linsolve(A,B);

keyboard 
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


