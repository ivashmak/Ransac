function H=NormalizedDLT(pts1,pts2)
    NUMP=size(pts1,1);
    [T1,offset1,s1,s1_1,s1_2]=GetNormalizingTransformation(pts1);
    [T2,offset2,s2,s2_1,s2_2]=GetNormalizingTransformation(pts2);
    
    pts1Tr=(T1*[pts1';ones(1,NUMP)])';
    pts2Tr=(T2*[pts2';ones(1,NUMP)])';
        
    pts1Tr=pts1Tr(:,1:2);
    pts2Tr=pts2Tr(:,1:2);
    
    H=DLT(pts1Tr,pts2Tr);    

    H=T2\H*T1;    
end

