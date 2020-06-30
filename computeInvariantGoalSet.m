function [ invariantGoalSet ] = computeInvariantGoalSet( A, B, Q, R )

[K,~,~] = dlqr(A,B,Q,R);

Acl = A-B*K;
w_max = 0.05;
W = Polyhedron([w_max w_max; w_max -w_max; -w_max -w_max; -w_max w_max]);

X{1} = zeros(size(Acl,1),1); % Initialize the set
for i = 1:10000
    Set = Acl*X{i} + W; % Propagate the uncertanty
    Set.minHRep() % Compute minimal representation
    Set.minVRep() % Compute minimal representation
    X{i+1} = Set;
    % Check if the algorithm has covnerged 
    if i > 1
        if (X{i+1}.contains(X{i})) && (X{i}.contains(X{i+1}))
            invariantGoalSet = X{i+1}; % Set invaraint to the current iterate
            disp(['Invariant set computed in i = ',num2str(i),' iterations'])            
            break
        end
    end
end

end