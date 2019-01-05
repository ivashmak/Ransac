function H=DLT(pts1,pts2)
    NUMP=size(pts1,1);
    
    A=[];
    for i=1:NUMP
        x1=pts1(i,1);
        y1=pts1(i,2);

        x2=pts2(i,1);
        y2=pts2(i,2);

        A(2*i-1,:)=[-x1,-y1,-1.0,0,0,0,x2*x1,x2*y1,x2];
        A(2*i,:)=[0,0,0,-x1,-y1,-1.0,y2*x1,y2*y1,y2];
    end

    
    
    
    % lambda1 >= lambda2 >= ... >= lambdaN
    [~,~,V]=svd(A);
    H=reshape (V(:,9), 3, 3)';
    H = H / H(3,3);
    
    % lambda1 <= lambda2 <= ... <= lambdaN (eigen values)
    [V_, ~] = eig (A'*A);
    H2 = reshape (-V_(:,1), 3, 3)';
    H2 = H2 / H2 (3,3);
    
%     A = zeros (2*NUMP, 8);
%     b = zeros (2*NUMP, 1);
%     for i=1:NUMP
%         x1=pts1(i,1);
%         y1=pts1(i,2);
% 
%         x2=pts2(i,1);
%         y2=pts2(i,2);
% 
%         A(2*i-1,:) = [x1,y1,1,0,0,0,-x2*x1,-x2*y1];
%         A(2*i,:)   = [0,0,0,x1,y1,1,-y2*x1,-y2*y1];
%         b(2*i-1) = x2;
%         b(2*i) = y2; 
%     end
% 
%     H3 = A \ b;
%     H3 = [H3(1) H3(2) H3(3); H3(4) H3(5) H3(6); H3(7) H3(8) 1];
%     
%     H
%     H2
%     H3
    
    
    
    AA1=zeros(9,9);
    AA2=zeros(9,9);
    AA=zeros(9,9);
    
    for i=1:NUMP
        x1=pts1(i,1);
        y1=pts1(i,2);

        x2=pts2(i,1);
        y2=pts2(i,2);

        a1=[-x1,-y1,-1.0,0,0,0,x2*x1,x2*y1,x2];
        AA1 = AA1 + a1' * a1;
        a2=[0,0,0,-x1,-y1,-1.0,y2*x1,y2*y1,y2];
        AA2 = AA2 + a2' * a2;
        AA = AA + a1'*a1 + a2'*a2;
    end
    keyboard
    V = eig (AA);
    H4 =  reshape (-V(:,1), 3, 3)'
end
