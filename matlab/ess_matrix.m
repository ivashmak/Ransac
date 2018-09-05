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

collects = {'x^3' 'y^3' '[x^2 y]' '[x^2 z]' '[y^2 z]' 'x^2' 'y^2' '[x y z]' '[x y]' '[x z^2]' '[x z]' 'x' '[y z^2]' '[y z]' 'y'};
patterns = {'(.*)*x\^3' '(.*)*y\^3' '(.*)*x\^2\*y' '(.*)*x\^2\*z' '(.*)*y\^2\*z' '(.*)*x\^2' '(.*)*y\^2' ...
            '(.*)*x\*y\*z' '(.*)*x\*y' '(.*)*x\*z\^2' '(.*)*x\*z' '(.*)*x' '(.*)*y\*z\^2' '(.*)*y\*z' '(.*)*y'};
replaces = {')*x^3' ')*y^3' ')*x^2*y' ')*x^2*z' ')*y^2*z' ')*x^2' ')*y^2' ')*x*y*z' ')*x*y' ')*x*z^2' ')*x*z' ')*x' ')*y*z^2' ')*y*z' ')*y'};

sz = size (collects);
sz = sz (2);

for c = 1:sz
    equ = collect (equ, sym (collects(c)));
    pattern = patterns (c)
    collects (c)
    
    for i = 1:9
       str = char (equ (i));
       match = regexp (str, pattern, 'match')
       if (isempty (match) == 1)
           RES (i, c) = sym (0);
           continue;
       end
       str2 = match {1};    
       RES (i, c) = sym (strrep (str2, replaces(c), ')'));
       str = strrep (str, str2, '');
       equ (i) = sym (str);
    end    
end

keyboard