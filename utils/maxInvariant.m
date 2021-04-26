function Cinf = maxInvariant(A,B,X,U)
% Given:

preOld = X;
Af = preOld.A;
bf = preOld.b;
for i = [1:500]
    preNew = computePre(A,B,X,U,Af,bf);
    Af = preNew.A;
    bf = preNew.b;
    
    if preOld.contains(preNew) && preNew.contains(preOld)
        display(['Converged at iteration: ',num2str(i) ])
        break
    end
    preOld = preNew;
    
end
Cinf = preNew;
end