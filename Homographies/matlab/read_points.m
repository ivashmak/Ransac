fileID = fopen('../points/graf_pts.txt','r');
formatSpec = '%f';
sizeA = [7 Inf];
A = fscanf(fileID, formatSpec, sizeA);
A = A';
sz = size(A, 1);

for i = 1:sz
    pts1(i, 1) = A(i, 1);
    pts1(i, 2) = A(i, 2);
%     pts1(i, 3) = A(i, 3);
    
    pts2(i, 1) = A(i, 4);
    pts2(i, 2) = A(i, 5);
%     pts2(i, 3) = A(i, 6);
end
% keyboard
H_DLT = DLT(pts1, pts2)
[T_1, offset_1, s_1, s1_1, s2_1] = GetNormalizingTransformation(pts1)
NormalizedDLT_res = NormalizedDLT(pts1, pts2)
% [T_2, offset_2, s_2, s1_2, s2_2] = GetNormalizingTransformation(pts2)
