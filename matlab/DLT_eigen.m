clear all
N = 3;

X = sym ('x', [N 2]);
Y = sym ('y', [N 2]);


for i=1:N
    x1=X(i,1);
    y1=Y(i,1);

    x2=X(i,2);
    y2=Y(i,2);

    A(2*i-1,:)=[-x1,-y1,-1.0,0,0,0,x2*x1,x2*y1,x2];
    A(2*i,:)=[0,0,0,-x1,-y1,-1.0,y2*x1,y2*y1,y2];
end

C1 = transpose(A) * A;

C2 = zeros (9,9);
for i=1:N
    x1=X(i,1);
    y1=Y(i,1);

    x2=X(i,2);
    y2=Y(i,2);
    
    a1 = [-x1,-y1,-1.0,0,0,0,x2*x1,x2*y1,x2];
    a2 = [0,0,0,-x1,-y1,-1.0,y2*x1,y2*y1,y2];
    a = a1 + a2;
    C2 = C2 + transpose(a) * a;
end

for i = 1:9
    for j = i:9
%         i 
%         j
%         C1(i,j)
%         C2(i,j)
        if isequaln(C1(i,j), C2(i,j)) == 0
            
            disp ('not equal')
            i
            j
        end
        
        disp('------------------------')
    end
end



keyboard




