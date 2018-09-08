X = sym ('x', [3 3]);
Y = sym ('y', [3 3]);
Z = sym ('z', [3 3]);
W = sym ('w', [3 3]);

syms x y z; 
E = x*X +y*Y +z*Z + W;
equ (1) = det (E);
EQU = 2*E*transpose(E)*E - trace(E*transpose(E))*E;
equ (2:10) = EQU;

equ = expand(simplify(equ));

collects = {'x^3' 'y^3' '[x^2 y]' '[x y^2]' '[x^2 z]' '[y^2 z]' 'x^2' 'y^2' '[x y z]' '[x y]' '[x z^2]' '[x z]' 'x' '[y z^2]' '[y z]' 'y'};
patterns = {'(.*)*x\^3' '(.*)*y\^3' '(.*)*x\^2\*y' '(.*)*x\*y\^2' '(.*)*x\^2\*z' '(.*)*y\^2\*z' '(.*)*x\^2' '(.*)*y\^2' ...
            '(.*)*x\*y\*z' '(.*)*x\*y' '(.*)*x\*z\^2' '(.*)*x\*z' '(.*)*x' '(.*)*y\*z\^2' '(.*)*y\*z' '(.*)*y'};
replaces = {')*x^3' ')*y^3' ')*x^2*y' ')*x*y^2'  ')*x^2*z' ')*y^2*z' ')*x^2' ')*y^2' ')*x*y*z' ')*x*y' ')*x*z^2' ')*x*z' ')*x' ')*y*z^2' ')*y*z' ')*y'};

sz = size (collects);
sz = sz (2);

k = 1;
for c = 1:sz
    equ = expand(equ);
    equ = collect (equ, sym (collects(c)));
    pattern = patterns (c);

    for i = 1:9
       ii = string (i);
       cc = string (c);
        
       str = char (equ (i));
       match = regexp (str, pattern, 'match');
       if (cellfun(@isempty,match) == 1)
           RES (k) = { strcat('C(', ii{1}, ',',  cc{1}, ') = 0') };
           k = k + 1;
           continue;
       end
       str2 = match {1};    
       RES (k) = strcat ('C(', ii{1}, ',',  cc{1}, ') = ', strrep (str2, replaces(c), ')'));
       str = strrep (str, str2, '');
       equ (i) = sym (str);
       
       k = k + 1;
    end
end

for i = 1:9
    str = char (equ (i));
    str = strrep (str, '*z', '');
    str = strrep (str, '*z^2', '');
    str = strrep (str, '*z^3', '');
    ii = string (i);
    tmp =  strcat ('C(', ii{1}, ',17) = ', str) ;
    tmp = { tmp };
    RES (k) = tmp;
    k = k+1;
end    

vars = {'x' 'y' 'z' 'w'};
for v = 1:4
    for i = 1:3
        for j = 1:3
            ii = string (i);
            jj = string (j);
            pattern = strcat(vars(v), ii{1}, '_', jj{1});
            replace = strcat(vars(v), '(', ii{1}, ',', jj{1}, ')');
            RES = strrep (RES, pattern, replace);
            
            pattern1 = strcat (replace, '^2');
            pattern2 = strcat (replace, '^3');
            replace1 = strcat ('pow (', replace, ',2)');
            replace2 = strcat ('pow (', replace, ',3)');
          
            RES = strrep (RES, pattern1, replace1);
            RES = strrep (RES, pattern2, replace2);  
        end
    end
end

sz = size (RES);
sz = sz (2);

fileID = fopen('constraints.txt','w');
fprintf(fileID,'%s;\n',RES{:});
fclose(fileID);

% keyboard