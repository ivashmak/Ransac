function [T,offset,s,s1,s2]=GetNormalizingTransformation(pts)
    NUMP = size(pts,1);

    offset = mean(pts)';
    ptsOffseted = pts-ones(NUMP,1)*offset';

    
    summa1 = 0;
    summa2 = 0.0;
    for i=1:NUMP
        summa1 = summa1 + ptsOffseted(i,1)*ptsOffseted(i,1);
        summa2 = summa2 + ptsOffseted(i,2)*ptsOffseted(i,2);
    end;
    s1 = sqrt(summa1/NUMP);
    s2 = sqrt(summa2/NUMP);

    s=sqrt((summa1+summa2)/NUMP)/sqrt(2);
    keyboard
    T1 = eye(3);
    T2 = eye(3);
    T1(1:2,3) = -offset;
    T2(1,1) = 1.0/s;
    T2(2,2) = 1.0/s;
    T=T2*T1;
    
end
