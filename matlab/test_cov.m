A = 1 * randn (1000, 2);

% almost linear O (n) complexity for dimension << number of points
sum_x_sq = sum (A(:,1).^2);
sum_y_sq = sum (A(:,2).^2);
sum_xy = sum (A(:,1).*A(:,2));
sum_x = sum(A(:,1));
sum_y = sum(A(:,2));

N = size (A, 1);

xm = sum_x / N;
ym = sum_y / N;

% (x-xm)(y-ym) = xy - y xm - x ym + xm ym
c = zeros (2,2);

covA1(1,1) = sum_x_sq - 2 * sum_x * xm + N * xm^2;
covA1(1,2) = sum_xy - sum_x * ym - sum_y * xm + N * xm * ym;
covA1(2,2) = sum_y_sq - 2 * sum_y * ym + N * ym^2;

covA1(2,1) = covA1(1,2);

covA2 = cov (A) * (N - 1);
covA3 = (A - mean (A))' * (A - mean (A));

covA1 
covA2
covA3

