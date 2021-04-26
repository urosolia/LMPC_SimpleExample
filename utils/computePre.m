function Xout = computePre(A,B,X,U,Af,bf)
% Given:

Au = U.A;
bu = U.b;

bigA = [Au zeros(2,2);
        Af*B Af*A];
    
bigb = [bu;bf];

P = Polyhedron('A',bigA,'b',bigb);
X0 = projection(P,[2;3]);
Xout = X0.intersect(X);
end