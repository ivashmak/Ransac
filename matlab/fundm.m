syms t1_11 t1_13 t1_23 t1_22 t2_11 t2_22 t2_13 t2_23
F = sym ('f', [3 3]);

T1 = [t1_11 0 t1_13; 0 t1_22 t1_23; 0 0 1]
T2 = [t2_11 0 0; 0 t2_22 0; t2_13 t2_23 1]

FF = simplify (T2 * F * T1)