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
    % H = H / H(3,3);
    
    % lambda1 <= lambda2 <= ... <= lambdaN (eigen values)
    [VV, ~] = eig (A'*A);
    HH = reshape (-VV(:,1), 3, 3)';
    % HH = HH / HH (3,3);
    
    norm (H - HH)
end
