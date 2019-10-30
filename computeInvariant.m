function [ invariantSet ] = computeInvariant( Acl, W )
%computeInvariant Compute a positive invariant set using a fixed point
%iteration
%   Acl: closed-loop system matrix
%   W: disturbance support
X{1} = zeros(size(Acl,1),1);
for i = 1:10000
    X{i+1} = Acl*X{i} + W;
    X{i+1}.minVRep()
    X{i+1}.minHRep()
    
    if i > 1
        if (X{i+1}.contains(X{i})) && (X{i}.contains(X{i+1}))
            invariantSet = X{i+1};
            disp(['Invariant set computed in i = ',num2str(i),' iterations'])            
            break
        end
    end
end

end

