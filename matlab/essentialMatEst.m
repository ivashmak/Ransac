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


Xmat = sym ('x', [3 3]);
Ymat = sym ('y', [3 3]);
Zmat = sym ('z', [3 3]);
Wmat = sym ('w', [3 3]);

syms x y z; 
E = x*Xmat +y*Ymat +z*Zmat + Wmat;
subs (E, )

% Edet = x*Xmat +y*Ymat +z*Zmat + Wmat;
% E (1,1) = Xmat (1,1); E (1,2) = Ymat (1,1); E (1,3) = Zmat (1,1); E (1,4) = Wmat (1,1);
% E (2,1) = Xmat (2,1); E (2,2) = Ymat (2,1); E (2,3) = Zmat (2,1); E (2,4) = Wmat (2,1);
% E (3,1) = Xmat (3,1); E (3,2) = Ymat (3,1); E (3,3) = Zmat (3,1); E (3,4) = Wmat (3,1);
% E (4,1) = Xmat (1,2); E (4,2) = Ymat (1,2); E (4,3) = Zmat (1,2); E (4,4) = Wmat (1,2);
% E (5,1) = Xmat (2,2); E (5,2) = Ymat (2,2); E (5,3) = Zmat (2,2); E (5,4) = Wmat (2,2);
% E (6,1) = Xmat (3,2); E (6,2) = Ymat (3,2); E (6,3) = Zmat (3,2); E (6,4) = Wmat (3,1);
% E (7,1) = Xmat (1,3); E (7,2) = Ymat (1,3); E (7,3) = Zmat (1,3); E (7,4) = Wmat (1,3);
% E (8,1) = Xmat (2,3); E (8,2) = Ymat (2,3); E (8,3) = Zmat (2,3); E (8,4) = Wmat (2,3);
% E (9,1) = Xmat (3,3); E (9,2) = Ymat (3,3); E (9,3) = Zmat (3,3); E (9,4) = Wmat (3,3);


% keyboard

% equ(1) = det(reshape (E, 6,6));
equ (1) = det (E);
EQU =  simplify(2*E*transpose(E)*E - trace(E*transpose(E))*E);
equ (2:10) = EQU;
% syms x y z w;
% ls = [x; y; z; w];
% eq = EQU * ls;
% 
% RES = solve (eq);
% equ(2:10) = EQU ;
% equ = transpose(equ);

% 
% AA = [125930.1, 71924.281, 379.99866, 67194.797, 38377.934, 202.76274, 331.39618, 189.27509, 1;
%  88070.805, 116195.7, 303.34, 108162.12, 142703.08, 372.54001, 290.33694, 383.05435, 1;
%  129306.65, 83406.992, 380.97519, 78916.953, 50904, 232.51241, 339.40964, 218.93025, 1;
%  157281.47, 133575.66, 382.52612, 150040.06, 127425.68, 364.91418, 411.16531, 349.19354, 1;
%  148797.83, 106668.44, 386.60809, 112941.56, 80964.219, 293.44595, 384.88028, 275.90845, 1];

% for i = 1:9 
%     for j = 1:4 
%         fprintf ("%f ", EQU(i,j));
%     end
%     fprintf("\n");
% end

keyboard