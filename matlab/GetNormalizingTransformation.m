function [T,offset,s,s1,s2]=GetNormalizingTransformation(pts)
    NUMP = size(pts,1);

    offset = mean(pts)';
    ptsOffseted = abs(pts-ones(NUMP,1)*offset');

    summa1 = sum (ptsOffseted (:,1));
    summa2 = sum (ptsOffseted (:,2));
    
    s1 = NUMP/summa1;
    s2 = NUMP/summa2;
%     s1 = summa1/NUMP;
%     s2 = summa2/NUMP;
    
     s=(s1+s2)/sqrt(2);

    T1 = eye(3);
    T2 = eye(3);
    T1(1:2,3) = -offset;
    T2(1,1) = s1;
    T2(2,2) = s2;
    T=T2*T1;

end
