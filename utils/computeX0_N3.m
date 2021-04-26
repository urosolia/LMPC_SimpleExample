function Xout = computeX0_N3(A,B,X,U,Af,bf)
% Given:

Ax = X.A;
bx = X.b;
Au = U.A;
bu = U.b;

bigA = [Au zeros(2,4);
        zeros(2,1) Au zeros(2,3);
        zeros(2,2) Au zeros(2,2);
        Ax*B zeros(4,1) zeros(4,1) Ax*A;
        Ax*A*B Ax*B zeros(4,1) Ax*A^2;
        Af*A^2*B Af*A*B Af*B Af*A^3];
    
bigb = [bu;bu;bu;bx;bx;bf];

P = Polyhedron('A',bigA,'b',bigb);
X0 = projection(P,[4;5]);
Xout = X0.intersect(X);
end