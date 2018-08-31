fileID = fopen('../points/graf_pts.txt','r');
formatSpec = '%f';
sizeA = [7 Inf];
A = fscanf(fileID, formatSpec, sizeA);
A = A';
sz = size(A, 1);

clear pts1 pts2;
for i = 1:5
    pts1(i, 1) = A(i, 1);
    pts1(i, 2) = A(i, 2);
%     pts1(i, 3) = A(i, 3);
    
    pts2(i, 1) = A(i, 4);
    pts2(i, 2) = A(i, 5);
%     pts2(i, 3) = A(i, 6);
end

K1 = eye(3);
K2 = eye(3);

pts1 = pts1';
pts2 = pts2';

N = 5;
q1 = K1 \ [pts1; ones(1,N)];
q2 = K2 \ [pts2; ones(1,N)];


q = [q1(1,:)'.* q2(1,:)', q1(2,:)'.* q2(1,:)', q1(3,:)'.* q2(1,:)', ...
     q1(1,:)'.* q2(2,:)', q1(2,:)'.* q2(2,:)', q1(3,:)'.* q2(2,:)', ...
     q1(1,:)'.* q2(3,:)', q1(2,:)'.* q2(3,:)', q1(3,:)'.* q2(3,:)'];
 
%according to the author, the null space step can be further optimized, 
%following the efficiency considerations in section 3.2.1
% Can be further expand it to N > 5 by extracting the four singular vectors
% corresponding to the four smallest singular values.
nullSpace = null(q); 
X = nullSpace(:,1);
Y = nullSpace(:,2);
Z = nullSpace(:,3);
W = nullSpace(:,4);

% populating the equation system
mask = [1,2,3;4,5,6;7,8,9];
Xmat = X(mask);
Ymat = Y(mask);
Zmat = Z(mask);
Wmat = W(mask);
X_ = (K2') \ Xmat / K1;
Y_ = (K2') \ Ymat / K1;
Z_ = (K2') \ Zmat / K1;
W_ = (K2') \ Wmat / K1;



syms x y z w e E EQU ; 
E = x*Xmat +y*Ymat +z*Zmat + Wmat;

equ(1) = det(E);
EQU =  simplify(2*E*transpose(E)*E - trace(E*transpose(E))*E);
equ(2:10) = EQU ;
equ = transpose(equ);

keyboard