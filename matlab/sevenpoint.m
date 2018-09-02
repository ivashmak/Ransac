function [ F ] = sevenpoint( pts1, pts2, M )
% sevenpoint:
%   pts1 - Nx2 matrix of (x,y) coordinates
%   pts2 - Nx2 matrix of (x,y) coordinates
%   M    - max (imwidth, imheight)

% Q2.2 - Todo:
%     Implement the eightpoint algorithm
%     Generate a matrix F from some '../data/some_corresp.mat'
%     Save recovered F (either 1 or 3 in cell), M, pts1, pts2 to q2_2.mat

%     Write recovered F and display the output of displayEpipolarF in your writeup
F=[];
N=size(pts1,1);
%scale matrix
T=eye(3)/M;
T(3,3)=1;
p_im1=[pts1,ones(N,1)];
p_im1=(T*p_im1')';
p_im2=[pts2,ones(N,1)];
p_im2=(T*p_im2')';
A=[p_im2(:,1).*p_im1(:,1),p_im2(:,1).*p_im1(:,2),p_im2(:,1).*p_im1(:,3),p_im2(:,2).*p_im1(:,1),p_im2(:,2).*p_im1(:,2),p_im2(:,2).*p_im1(:,3),p_im2(:,3).*p_im1(:,1),p_im2(:,3).*p_im1(:,2),p_im2(:,3).*p_im1(:,3)];
%finding F through Eigen decomposition
[V,D] = eig(A'*A);
[~, ind] = sort(diag(D));
f1 = V(:,ind(1));
f2 = V(:,ind(2));
F1= reshape(f1,3,3);
F1=F1';
F2= reshape(f2,3,3);
F2=F2';
syms a;
F=a*F1+(1-a)*F2;
D=det(F);
eqn=D==0;
sol_a=solve(eqn,a);
%disp(double(sol_a));
sol_a=max(double(real(sol_a)));
F= sol_a*F1+(1-sol_a)*F2;
F=refine(F,p_im1(:,1:2),p_im2(:,1:2));
F= T'*F*T;


%sol_a=real(double(sol_a));

% F=cell(1,3);
% for i=1: length(F)
%     F{i}=sol_a(i)*F1+(1-sol_a(i))*F2;
%     F{i}=T'*F{i}*T;
% end
save('q2_2.mat','F','M','pts1','pts2');
end